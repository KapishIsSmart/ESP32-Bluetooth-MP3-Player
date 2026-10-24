// ESP32 DevKit V1 -> Bluetooth earbuds MP3 player
// SD card (SPI) + OLED 0.96" + 5 touch pads (\\
//
// TOUCH PADS (foil or copper pad on each pin, nothing else connected)
//   GPIO 4  : volume up   (+5%)
//   GPIO 13 : volume down (-5%)
//   GPIO 32 : scroll up
//   GPIO 27 : scroll down
//   GPIO 33 : select. Tap = play the highlighted song, hold (about 1 s) = shuffle the playlist
//   Hold a volume or scroll pad to repeat.
//   Do NOT touch the pads while the board starts: it measures their resting level then.
//
// WIRING
//   SD module : VCC -> VIN (5V) if it has a regulator, else 3V3 | GND | MISO 19 | MOSI 23 | SCK 18 | CS 5
//   OLED      : VCC -> 3V3 | GND | SDA 21 | SCL 22
//
// SCREEN
//   Top line   : time played / total time, and volume
//   Middle     : song list (inverted row = playing, ">" = cursor)
//   Bottom bar : progress
//
// LIBRARIES (Arduino IDE)
//   1. ESP32-A2DP        (Phil Schatzmann, GitHub ZIP)
//   2. arduino-libhelix  (Phil Schatzmann, GitHub ZIP)  <- MP3 decoder
//   3. Adafruit SSD1306 and Adafruit GFX (Library Manager)
//   SD, SPI and Wire come with the ESP32 board package.
//
// BOARD SETTINGS
//   Board: ESP32 Dev Module
//   Partition Scheme: Huge APP (3MB No OTA/1MB SPIFFS)
//   Upload Speed: 115200 or 460800 if uploads fail
//
// SD CARD: FAT32, .mp3 files in the root folder (not in subfolders).
// Best results: 44.1 kHz MP3s (other sample rates are converted).

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <vector>
#include <algorithm>
#include "BluetoothA2DPSource.h"
#include "MP3DecoderHelix.h"

using namespace libhelix;

// ------------------------- settings you may change -------------------------
#define EARBUDS_NAME "Device Name"
#define DEBUG_SERIAL 1               // set to 0 to turn off the status lines
const uint32_t RING_FRAMES = 2048;   // audio buffer (power of 2). 4096 did not leave enough memory for Bluetooth
const int START_VOLUME = 64;         // 0..127 at power on
const int VOL_STEP = 6;              // about 5% of the 0..127 scale
const float TOUCH_SENSITIVITY = 0.7; // pad counts as touched below baseline * this. Lower = needs a firmer touch
const uint32_t SHUFFLE_HOLD_MS = 800; // how long to hold the select pad to shuffle
// ---------------------------------------------------------------------------

// pins
const int SD_CS = 5;                          // SD module: MOSI 23, MISO 19, SCK 18, CS 5
const int TOUCH_PINS[5] = {4, 13, 32, 27, 33}; // vol up, vol down, scroll up, scroll down, select

const int LIST_ROWS = 5;
const int MAX_SONGS = 200;
const uint32_t RING_MASK = RING_FRAMES - 1;

BluetoothA2DPSource a2dp_source;
Adafruit_SSD1306 display(128, 64, &Wire, -1);
MP3DecoderHelix* decoderPtr = nullptr;

// ------------------------------ song list ---------------------------------
struct Song {
  String name;   // shown on screen (no ".mp3")
  String path;   // e.g. "/song1.mp3"
};
std::vector<Song> songs;
std::vector<uint16_t> order;   // display order: order[i] = index into songs
int cursor = 0;                // highlighted row (index into order)
int playingPos = -1;           // playing row (index into order), -1 = nothing
int topRow = 0;                // first visible row

// ------------------------- shared with audio task -------------------------
volatile int   reqTrack = -1;        // song index to open
volatile bool  gPlaying = false;
volatile bool  gEnded = false;
volatile uint32_t gFilePos = 0;
volatile uint32_t gAudioStart = 0;
volatile uint32_t gAudioBytes = 0;
volatile uint32_t gBitrate = 0;      // bits per second, learned from the first frame
volatile bool  announceTrack = false;

// ------------------------------ audio buffer ------------------------------
int16_t ring[RING_FRAMES][2];
volatile uint32_t ringHead = 0;      // written only by the audio task
volatile uint32_t ringTail = 0;      // written only by the Bluetooth callback
volatile uint32_t cbCount = 0;       // how many times Bluetooth asked for audio
volatile uint32_t underruns = 0;     // frames of silence sent because the buffer was empty
volatile int32_t  gGain = 0;         // software volume, 0..256 (256 = full)

// Returns false if we gave up because a new song was requested
bool pushFrame(int16_t l, int16_t r) {
  while ((uint32_t)(ringHead - ringTail) >= RING_FRAMES) {
    if (reqTrack >= 0) return false;
    vTaskDelay(1);
  }
  uint32_t h = ringHead;
  ring[h & RING_MASK][0] = l;
  ring[h & RING_MASK][1] = r;
  __sync_synchronize();
  ringHead = h + 1;
  return true;
}

// Bluetooth asks for audio here. Volume is applied here.
int32_t get_data_frames(Frame *frame, int32_t frame_count) {
  cbCount++;
  int32_t g = gGain;
  uint32_t t = ringTail;
  uint32_t h = ringHead;
  for (int i = 0; i < frame_count; i++) {
    if (t != h) {
      frame[i].channel1 = (int16_t)(((int32_t)ring[t & RING_MASK][0] * g) >> 8);
      frame[i].channel2 = (int16_t)(((int32_t)ring[t & RING_MASK][1] * g) >> 8);
      t++;
    } else {
      underruns++;
      frame[i].channel1 = 0;
      frame[i].channel2 = 0;
    }
  }
  ringTail = t;
  return frame_count;
}

// ------------------------------ MP3 decoding ------------------------------
// Linear-interpolation resampler state (used only when the MP3 is not 44.1 kHz)
int16_t prevL = 0, prevR = 0;
uint32_t resFrac = 0;   // 16.16 fixed point

// Called by the decoder with raw PCM; converts to 44.1 kHz stereo
void onPcm(MP3FrameInfo &info, short *pcm, size_t len, void *ref) {
  int br = info.bitrate;
  if (br > 0 && br < 1000) br *= 1000;
  if (br > 0) gBitrate = br;

  int ch = info.nChans < 1 ? 1 : info.nChans;
  int rate = info.samprate > 0 ? info.samprate : 44100;

#if DEBUG_SERIAL
  if (announceTrack) {
    announceTrack = false;
    Serial.printf("Track format: %d Hz, %d ch, %d bps\n", rate, ch, br);
  }
#endif

  size_t frames = len / ch;
  if (rate == 44100) {
    for (size_t i = 0; i < frames; i++) {
      int16_t l = pcm[i * ch];
      int16_t r = (ch > 1) ? pcm[i * ch + 1] : l;
      if (!pushFrame(l, r)) return;
    }
  } else {
    uint32_t step = (uint32_t)(((uint64_t)rate << 16) / 44100);
    for (size_t i = 0; i < frames; i++) {
      int16_t l = pcm[i * ch];
      int16_t r = (ch > 1) ? pcm[i * ch + 1] : l;
      while (resFrac < 65536) {
        int32_t f = (int32_t)(resFrac >> 4);   // 0..4095
        int16_t ol = prevL + (int16_t)((((int32_t)l - prevL) * f) >> 12);
        int16_t orr = prevR + (int16_t)((((int32_t)r - prevR) * f) >> 12);
        if (!pushFrame(ol, orr)) return;
        resFrac += step;
      }
      resFrac -= 65536;
      prevL = l;
      prevR = r;
    }
  }
}

File file;

void resetDecoder() {
  decoderPtr->end();
  decoderPtr->begin();
  prevL = 0;
  prevR = 0;
  resFrac = 0;
}

void openTrack(int id) {
  gPlaying = false;
  if (file) file.close();
  resetDecoder();
  gBitrate = 0;

  file = SD.open(songs[id].path.c_str());
  if (!file) {
    Serial.println("Cannot open file");
    gEnded = true;
    return;
  }
  uint32_t fileSize = file.size();
  uint32_t start = 0;

  // skip the ID3 tag at the start so time estimates are better
  uint8_t h[10];
  if (file.read(h, 10) == 10 && h[0] == 'I' && h[1] == 'D' && h[2] == '3') {
    start = ((uint32_t)(h[6] & 0x7F) << 21) | ((uint32_t)(h[7] & 0x7F) << 14) |
            ((uint32_t)(h[8] & 0x7F) << 7) | (uint32_t)(h[9] & 0x7F);
    start += 10;
    if (h[5] & 0x10) start += 10;
  }
  if (start >= fileSize) start = 0;
  file.seek(start);

  gAudioStart = start;
  gAudioBytes = fileSize - start;
  gFilePos = start;
  announceTrack = true;
  gPlaying = true;
  Serial.printf("Opened %s: %u bytes, audio starts at %u\n", songs[id].path.c_str(), (unsigned)fileSize, (unsigned)start);
}

void audioTask(void *param) {
  static uint8_t buf[512];
  for (;;) {
    if (reqTrack >= 0) {
      int id = reqTrack;
      reqTrack = -1;
      openTrack(id);
    }

    if (!gPlaying) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    int n = file.read(buf, sizeof(buf));
    if (n <= 0) {
      gPlaying = false;
      gEnded = true;
      file.close();
      continue;
    }
    gFilePos = gFilePos + n;
    decoderPtr->write(buf, n);
  }
}

// ------------------------------ UI helpers --------------------------------
int volume = START_VOLUME;
uint32_t lastDraw = 0;
bool wasConnected = false;

void showMessage(const char *msg) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 24);
  display.print(msg);
  display.display();
}

void startTrack(int pos) {
  playingPos = pos;
  gFilePos = 0;
  gAudioStart = 0;
  gAudioBytes = 0;
  gBitrate = 0;
  reqTrack = order[pos];
  Serial.printf("Playing: %s\n", songs[order[pos]].name.c_str());
  lastDraw = 0;
}

void autoNext() {
  if (order.empty()) return;
  bool follow = (cursor == playingPos);
  int next = (playingPos + 1) % (int)order.size();
  startTrack(next);
  if (follow) cursor = next;
}

// Shuffle the list; the playing song stays playing and moves to the top
void shuffleOrder() {
  int n = order.size();
  if (n < 2) return;
  int keepId = (playingPos >= 0) ? order[playingPos] : -1;
  for (int i = n - 1; i > 0; i--) {
    int j = random(i + 1);
    std::swap(order[i], order[j]);
  }
  if (keepId >= 0) {
    for (int i = 0; i < n; i++) {
      if (order[i] == keepId) {
        std::swap(order[0], order[i]);
        break;
      }
    }
    playingPos = 0;
  }
  cursor = 0;
  topRow = 0;
  lastDraw = 0;
  Serial.println("Shuffled");
}

// Software volume: quadratic curve so the steps feel even
void applyVolume() {
  gGain = ((int32_t)volume * volume * 256) / (127 * 127);
}

// ------------------------------ touch pads --------------------------------
uint16_t touchBase[5];
uint8_t  touchCnt[5]  = {0, 0, 0, 0, 0};
bool     touchHeld[5] = {false, false, false, false, false};
uint32_t touchNext[5] = {0, 0, 0, 0, 0};
bool     selDown = false, selLongDone = false;
uint32_t selStart = 0;

void calibrateTouch() {
  for (int i = 0; i < 5; i++) {
    uint32_t sum = 0;
    for (int k = 0; k < 20; k++) {
      sum += touchRead(TOUCH_PINS[i]);
      delay(5);
    }
    touchBase[i] = sum / 20;
    Serial.printf("Touch pad %d (GPIO %d) resting level: %u\n", i + 1, TOUCH_PINS[i], (unsigned)touchBase[i]);
  }
}

void readTouch(uint32_t now) {
  int n = order.size();
  for (int i = 0; i < 5; i++) {
    bool isTouched = touchRead(TOUCH_PINS[i]) < touchBase[i] * TOUCH_SENSITIVITY;

    // must stay touched for 2 checks in a row (about 20 ms) to count
    if (isTouched) {
      if (touchCnt[i] < 2) touchCnt[i]++;
    } else {
      touchCnt[i] = 0;
    }
    bool active = (touchCnt[i] >= 2);

    // select pad: tap = play (on release), hold = shuffle
    if (i == 4) {
      if (active && !selDown) {
        selDown = true;
        selStart = now;
        selLongDone = false;
      }
      if (active && selDown && !selLongDone && now - selStart > SHUFFLE_HOLD_MS) {
        selLongDone = true;
        shuffleOrder();
      }
      if (!active && selDown) {
        selDown = false;
        if (!selLongDone) startTrack(cursor);
      }
      continue;
    }

    if (!active) {
      touchHeld[i] = false;
      continue;
    }

    // first touch fires at once; holding repeats
    if (touchHeld[i] && now < touchNext[i]) continue;
    touchNext[i] = now + (touchHeld[i] ? 150 : 400);
    touchHeld[i] = true;

    switch (i) {
      case 0:
        volume = min(127, volume + VOL_STEP);
        applyVolume();
        lastDraw = 0;
        break;
      case 1:
        volume = max(0, volume - VOL_STEP);
        applyVolume();
        lastDraw = 0;
        break;
      case 2:
        cursor = (cursor + n - 1) % n;
        lastDraw = 0;
        break;
      case 3:
        cursor = (cursor + 1) % n;
        lastDraw = 0;
        break;
    }
  }
}

// Drawing uses plain char buffers (no String) so it does not churn the heap
void drawScreen() {
  int n = order.size();
  if (cursor < topRow) topRow = cursor;
  if (cursor >= topRow + LIST_ROWS) topRow = cursor - LIST_ROWS + 1;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  float frac = 0;
  uint32_t ab = gAudioBytes;
  if (ab > 0) frac = (float)((int32_t)gFilePos - (int32_t)gAudioStart) / (float)ab;
  frac = constrain(frac, 0.0f, 1.0f);

  // header
  display.setCursor(0, 0);
  if (!a2dp_source.is_connected()) {
    display.print("Connecting...");
  } else if (playingPos < 0) {
    display.print("Pick a song");
  } else {
    float dur = (gBitrate > 0 && ab > 0) ? ((float)ab * 8.0f / (float)gBitrate) : 0;
    int cur = (int)(frac * dur);
    if (cur < 0) cur = 0;
    int tot = (int)dur;
    char tb[24];
    if (dur > 0) snprintf(tb, sizeof(tb), "%d:%02d/%d:%02d", cur / 60, cur % 60, tot / 60, tot % 60);
    else snprintf(tb, sizeof(tb), "%d:%02d/--:--", cur / 60, cur % 60);
    display.print(tb);
  }
  char vb[12];
  snprintf(vb, sizeof(vb), "Vol %d%%", volume * 100 / 127);
  display.setCursor(128 - strlen(vb) * 6, 0);
  display.print(vb);

  // song list
  for (int r = 0; r < LIST_ROWS; r++) {
    int idx = topRow + r;
    if (idx >= n) break;
    int y = 11 + r * 9;
    bool isPlaying = (idx == playingPos);
    if (isPlaying) display.fillRect(0, y - 1, 128, 9, SSD1306_WHITE);
    display.setTextColor(isPlaying ? SSD1306_BLACK : SSD1306_WHITE);
    display.setCursor(0, y);
    display.print(idx == cursor ? ">" : " ");
    char nm[22];
    strncpy(nm, songs[order[idx]].name.c_str(), 20);
    nm[20] = 0;
    display.print(nm);
  }
  display.setTextColor(SSD1306_WHITE);

  // progress bar
  display.drawRect(0, 57, 128, 7, SSD1306_WHITE);
  display.fillRect(2, 59, (int)(frac * 124), 3, SSD1306_WHITE);

  display.display();
}

// ------------------------------- SD card ----------------------------------
void scanSD() {
  File root = SD.open("/");
  while (true) {
    File f = root.openNextFile();
    if (!f) break;
    if (!f.isDirectory()) {
      String n = f.name();
      int s = n.lastIndexOf('/');
      if (s >= 0) n = n.substring(s + 1);
      String low = n;
      low.toLowerCase();
      if (!n.startsWith(".") && low.endsWith(".mp3") && (int)songs.size() < MAX_SONGS) {
        songs.push_back({n.substring(0, n.length() - 4), "/" + n});
      }
    }
    f.close();
  }
  root.close();
}

// ------------------------------- setup/loop -------------------------------
void setup() {
  Serial.begin(115200);

  Wire.begin(21, 22);
  Wire.setClock(400000);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  showMessage("Starting...");

  calibrateTouch();   // keep your hands off the pads until the song list appears

  if (!SD.begin(SD_CS)) {
    showMessage("SD card error");
    Serial.println("SD card error: check wiring and FAT32");
    while (true) delay(1000);
  }
  scanSD();
  if (songs.empty()) {
    showMessage("No MP3 files found");
    Serial.println("No MP3 files in the root of the SD card");
    while (true) delay(1000);
  }
  std::sort(songs.begin(), songs.end(), [](const Song &a, const Song &b) {
    String x = a.name, y = b.name;
    x.toLowerCase();
    y.toLowerCase();
    return x.compareTo(y) < 0;
  });
  for (size_t i = 0; i < songs.size(); i++) order.push_back(i);
  Serial.printf("Found %u songs\n", (unsigned)songs.size());

  randomSeed(esp_random());
  applyVolume();

  a2dp_source.set_data_callback_in_frames(get_data_frames);
  a2dp_source.set_volume(127);   // library volume stays at maximum; the pads are applied in get_data_frames
  a2dp_source.start(EARBUDS_NAME);
  Serial.printf("free heap after Bluetooth start: %u\n", (unsigned)ESP.getFreeHeap());

  decoderPtr = new MP3DecoderHelix(onPcm);
  decoderPtr->begin();

  xTaskCreatePinnedToCore(audioTask, "audio", 8192, NULL, 3, NULL, 1);
}

void loop() {
  uint32_t now = millis();

  readTouch(now);

  if (gEnded) {
    gEnded = false;
    autoNext();
  }

  bool conn = a2dp_source.is_connected();
  if (conn != wasConnected) {
    wasConnected = conn;
    Serial.println(conn ? "Connected!" : "Disconnected, searching again...");
    lastDraw = 0;
  }

  if (now - lastDraw > 200) {
    drawScreen();
    lastDraw = now;
  }

#if DEBUG_SERIAL
  static uint32_t lastInfo = 0;
  if (now - lastInfo > 2000) {
    lastInfo = now;
    Serial.printf("buffer=%u, cb=%u, underruns=%u, heap=%u, maxblock=%u, vol=%d, conn=%d\n",
      (unsigned)(ringHead - ringTail), (unsigned)cbCount, (unsigned)underruns,
      (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMaxAllocHeap(), volume, (int)conn);
  }
#endif
  delay(10);
}
