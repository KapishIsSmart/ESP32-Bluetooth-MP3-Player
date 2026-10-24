// ESP32 DEV KIT V1
// Connects to the earbuds on power-up, then plays two beeps and says
// "hi, this is ESP32" in a robot voice.
//
// LIBRARIES (Arduino IDE)
//   1. ESP32-A2DP by Phil Schatzmann (install from GitHub as a ZIP)
//   2. Only if USE_SPEECH = 1: "ESP8266Audio" and "ESP8266SAM" by Earle Philhower
//      (both are in Library Manager)
//
// BOARD SETTINGS
//   Board:            ESP32 Dev Module
//   Partition Scheme: Huge APP (3MB No OTA/1MB SPIFFS)   <- needed, Bluetooth sketches are big
//
// If the speech part gives compile errors, set USE_SPEECH to 0.
// You will still get the connection + beeps, which proves Bluetooth works.

#define EARBUDS_NAME "Insert earbuds name"
#define USE_SPEECH 1

#include "BluetoothA2DPSource.h"

BluetoothA2DPSource a2dp_source;

const uint32_t SR = 44100;                       // Bluetooth sample rate
const uint32_t T_BEEP1  = SR*3;                    // 1.0 s of silence first (buds wake up)
const uint32_t L_BEEP   = SR / 5;                // each beep lasts 0.2 s
const uint32_t T_BEEP2  = T_BEEP1 + L_BEEP + SR / 10;
const uint32_t T_SPEECH = T_BEEP2 + L_BEEP + (SR * 3) / 10;

volatile uint32_t pos = 0;                       // current playback position in frames
bool wasConnected = false;

#if USE_SPEECH
#include <vector>
#include <AudioOutput.h>
#include <ESP8266SAM.h>

std::vector<int8_t> speech;                      // 8-bit speech, saves memory
int speechRate = 22050;

// Catches the samples from the speech synthesizer instead of playing them
class CaptureOutput : public AudioOutput {
 public:
  bool begin() { return true; }
  bool ConsumeSample(int16_t sample[2]) {
    if (speech.size() < 64000) speech.push_back((int8_t)(sample[0] >> 8));
    return true;
  }
  bool stop() { return true; }
  bool SetRate(int hz) { speechRate = hz; return true; }
};

void makeSpeech() {
  speech.reserve(64000);
  CaptureOutput out;
  ESP8266SAM *sam = new ESP8266SAM();
  sam->Say(&out, "Hi. This is E S P thirty two.");
  delete sam;
  Serial.printf("Speech ready: %u samples at %d Hz\n", (unsigned)speech.size(), speechRate);
}
#endif

int16_t beepSample(uint32_t n, float freq) {
  return (int16_t)(6000.0f * sinf(TWO_PI * freq * (float)n / (float)SR));
}

int16_t sampleAt(uint32_t p) {
  if (p >= T_BEEP1 && p < T_BEEP1 + L_BEEP) return beepSample(p - T_BEEP1, 440.0f);
  if (p >= T_BEEP2 && p < T_BEEP2 + L_BEEP) return beepSample(p - T_BEEP2, 880.0f);
#if USE_SPEECH
  if (p >= T_SPEECH) {
    uint32_t i = (uint32_t)(((uint64_t)(p - T_SPEECH) * speechRate) / SR);
    if (i < speech.size()) return (int16_t)speech[i] * 180;
  }
#endif
  return 0;
}

// Called by the Bluetooth library whenever the earbuds need more audio
int32_t get_data_frames(Frame *frame, int32_t frame_count) {
  for (int i = 0; i < frame_count; i++) {
    int16_t s = sampleAt(pos++);
    frame[i].channel1 = s;
    frame[i].channel2 = s;
  }
  return frame_count;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting...");

#if USE_SPEECH
  makeSpeech();                                  // do this before Bluetooth starts (memory)
#endif

  a2dp_source.set_data_callback_in_frames(get_data_frames);
  a2dp_source.set_volume(60);                    // 0 to 127, keep it low for earbuds
  a2dp_source.start(EARBUDS_NAME);
  Serial.println("Looking for " EARBUDS_NAME " ...");
}

void loop() {
  bool connected = a2dp_source.is_connected();
  if (connected && !wasConnected) {
    pos = 0;                                     // replay the greeting on every new connection
    Serial.println("Connected!");
  }
  if (!connected && wasConnected) {
    Serial.println("Disconnected, searching again...");
  }
  wasConnected = connected;
  delay(200);
}
