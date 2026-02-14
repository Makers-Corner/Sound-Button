// =======================================================
// XIAO ESP32-C3 + MAX98357A
// BUTTON + LIGHT SLEEP PLAYER: plays /clip.wav once per press (LittleFS)
// Power-optimized (Arduino-safe WiFi off, lower CPU, pins defined during sleep).
// =======================================================

#include <Arduino.h>
#include <LittleFS.h>
#include "driver/i2s.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include <WiFi.h>

// ---------------- PINS (XIAO ESP32-C3 labels) ----------------
#define PIN_BTN        D2   // momentary to GND (use INPUT_PULLUP)
#define PIN_AMP_SD     D8   // GPIO3  (amp shutdown)
#define PIN_I2S_BCLK   D9   // GPIO0
#define PIN_I2S_LRCLK  D10  // GPIO1
#define PIN_I2S_DATA   D7   // GPIO4

// ---------------- I2S SETTINGS ----------------
#define I2S_PORT       I2S_NUM_0
#define DMA_BUF_LEN    256
#define DMA_BUF_COUNT  4

// ---------------- WAV INFO ----------------
struct WavInfo {
  uint32_t sampleRate = 22050;
  uint16_t bitsPerSample = 16;
  uint16_t numChannels = 1;   // 1=mono, 2=stereo
  uint32_t dataOffset = 44;
  uint32_t dataSize   = 0;
};

// ---------------- AMP CONTROL ----------------
static inline void ampOn(bool on) {
  pinMode(PIN_AMP_SD, OUTPUT);
  digitalWrite(PIN_AMP_SD, on ? HIGH : LOW);
}

// ---------------- RADIOS OFF (Arduino-safe) ----------------
static void disableRadios() {
  WiFi.mode(WIFI_OFF);
  WiFi.disconnect(true);
  delay(50);
}

// ---------------- PIN HYGIENE FOR SLEEP ----------------
static void prepPinsForSleep() {
  // Ensure amp is off
  pinMode(PIN_AMP_SD, OUTPUT);
  digitalWrite(PIN_AMP_SD, LOW);

  // Button: keep stable using internal pullup (OK for now)
  // For even lower current later: add external 100k pullup and change to INPUT.
  pinMode(PIN_BTN, INPUT_PULLUP);

  // I2S pins: avoid floating while asleep
  pinMode(PIN_I2S_BCLK, INPUT_PULLDOWN);
  pinMode(PIN_I2S_LRCLK, INPUT_PULLDOWN);
  pinMode(PIN_I2S_DATA, INPUT_PULLDOWN);
}

static void restorePinsAfterSleep() {
  pinMode(PIN_BTN, INPUT_PULLUP);

  pinMode(PIN_AMP_SD, OUTPUT);
  digitalWrite(PIN_AMP_SD, LOW);

  // I2S pins will be configured by i2s_set_pin() during playback
  pinMode(PIN_I2S_BCLK, INPUT);
  pinMode(PIN_I2S_LRCLK, INPUT);
  pinMode(PIN_I2S_DATA, INPUT);
}

// ---------------- WAV PARSER (PCM only) ----------------
bool parseWav(File &f, WavInfo &info) {
  if (!f) return false;

  char hdr[12];
  if (f.read((uint8_t*)hdr, 12) != 12) return false;
  if (strncmp(hdr, "RIFF", 4) || strncmp(hdr + 8, "WAVE", 4)) return false;

  while (f.available()) {
    char id[4];
    uint32_t sz = 0;
    if (f.read((uint8_t*)id, 4) != 4) return false;
    if (f.read((uint8_t*)&sz, 4) != 4) return false;

    if (!strncmp(id, "fmt ", 4)) {
      uint16_t audioFormat = 0, channels = 0, bps = 0, blockAlign = 0;
      uint32_t sampleRate = 0, byteRate = 0;

      if (f.read((uint8_t*)&audioFormat, 2) != 2) return false;
      if (f.read((uint8_t*)&channels, 2)     != 2) return false;
      if (f.read((uint8_t*)&sampleRate, 4)   != 4) return false;
      if (f.read((uint8_t*)&byteRate, 4)     != 4) return false;
      if (f.read((uint8_t*)&blockAlign, 2)   != 2) return false;
      if (f.read((uint8_t*)&bps, 2)          != 2) return false;

      // Skip any extra fmt bytes
      if (sz > 16) f.seek(f.position() + (sz - 16));

      if (audioFormat != 1) return false;       // PCM only
      if (bps != 16) return false;              // 16-bit only
      if (channels != 1 && channels != 2) return false;

      info.sampleRate = sampleRate;
      info.numChannels = channels;
      info.bitsPerSample = bps;
    }
    else if (!strncmp(id, "data", 4)) {
      info.dataOffset = f.position();
      info.dataSize = sz;
      return true;
    }
    else {
      // Skip other chunks (LIST, fact, JUNK, etc.)
      f.seek(f.position() + sz);
    }
  }
  return false;
}

// ---------------- I2S INIT ----------------
bool i2sBegin(uint32_t rate) {
  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  cfg.sample_rate = (int)rate;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT; // stereo frames
#if defined(I2S_COMM_FORMAT_STAND_I2S)
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
#else
  cfg.communication_format = I2S_COMM_FORMAT_I2S;
#endif
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count = DMA_BUF_COUNT;
  cfg.dma_buf_len = DMA_BUF_LEN;
  cfg.use_apll = false;
  cfg.tx_desc_auto_clear = true;
  cfg.fixed_mclk = 0;

  i2s_pin_config_t pins = {};
  pins.bck_io_num = PIN_I2S_BCLK;
  pins.ws_io_num = PIN_I2S_LRCLK;
  pins.data_out_num = PIN_I2S_DATA;
  pins.data_in_num = I2S_PIN_NO_CHANGE;

  esp_err_t err;

  err = i2s_driver_install((i2s_port_t)I2S_PORT, &cfg, 0, nullptr);
  if (err != ESP_OK) {
    Serial.printf("I2S install failed: %d\n", (int)err);
    return false;
  }

  err = i2s_set_pin((i2s_port_t)I2S_PORT, &pins);
  if (err != ESP_OK) {
    Serial.printf("I2S set pin failed: %d\n", (int)err);
    i2s_driver_uninstall((i2s_port_t)I2S_PORT);
    return false;
  }

  // Force clock explicitly (this often fixes silent output)
  err = i2s_set_clk((i2s_port_t)I2S_PORT, rate, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);
  if (err != ESP_OK) {
    Serial.printf("I2S set clk failed: %d\n", (int)err);
    i2s_driver_uninstall((i2s_port_t)I2S_PORT);
    return false;
  }

  i2s_zero_dma_buffer((i2s_port_t)I2S_PORT);
  return true;
}

void i2sEnd() {
  i2s_driver_uninstall((i2s_port_t)I2S_PORT);
}

// ---------------- PLAY ONCE ----------------
bool playWavOnce(const char *path) {
  File f = LittleFS.open(path, "r");
  if (!f) {
    Serial.println("ERROR: open WAV failed");
    return false;
  }

  WavInfo info;
  if (!parseWav(f, info)) {
    Serial.println("ERROR: parseWav failed (needs PCM 16-bit mono/stereo)");
    f.close();
    return false;
  }

  Serial.printf("WAV: %lu Hz, %u ch, %u-bit, data=%lu bytes\n",
                (unsigned long)info.sampleRate,
                (unsigned)info.numChannels,
                (unsigned)info.bitsPerSample,
                (unsigned long)info.dataSize);

  if (!i2sBegin(info.sampleRate)) {
    Serial.println("ERROR: i2sBegin failed");
    f.close();
    return false;
  }

  ampOn(true);
  delay(5);

  // Buffers
  const size_t FRAMES = 256;               // frames = stereo sample pairs
  int16_t inBuf[FRAMES * 2];               // can hold stereo read
  int16_t outStereo[FRAMES * 2];           // always write stereo

  f.seek(info.dataOffset);
  uint32_t remaining = info.dataSize;

  while (remaining > 0) {
    size_t toRead = (info.numChannels == 1)
                  ? min<uint32_t>(FRAMES * 2, remaining)  // mono bytes
                  : min<uint32_t>(FRAMES * 4, remaining); // stereo bytes

    int got = f.read((uint8_t*)inBuf, toRead);
    if (got <= 0) break;

    const void* outPtr = nullptr;
    size_t outBytes = 0;

    if (info.numChannels == 1) {
      size_t samples = (size_t)got / 2;
      for (size_t i = 0; i < samples; i++) {
        int16_t s = inBuf[i];
        outStereo[2*i] = s;
        outStereo[2*i + 1] = s;
      }
      outPtr = outStereo;
      outBytes = samples * 4;
    } else {
      // Already interleaved stereo
      outPtr = inBuf;
      outBytes = (size_t)got;
    }

    size_t written = 0;
    esp_err_t err = i2s_write((i2s_port_t)I2S_PORT, outPtr, outBytes, &written, portMAX_DELAY);

    if (err != ESP_OK) {
      Serial.printf("I2S write err: %d\n", (int)err);
      break;
    }

    remaining -= (uint32_t)got;
  }

  f.close();
  delay(10);

  ampOn(false);
  i2sEnd();

  return true;
}

// ---------------- LIGHT SLEEP (wake on button LOW) ----------------
void enterLightSleepUntilButton() {
  prepPinsForSleep();

  // Wake when button is LOW
  gpio_wakeup_enable((gpio_num_t)PIN_BTN, GPIO_INTR_LOW_LEVEL);
  esp_sleep_enable_gpio_wakeup();

  // Enter light sleep
  esp_light_sleep_start();

  restorePinsAfterSleep();
}

// =======================================================
// SETUP / LOOP
// =======================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== Button + Light Sleep WAV Player (Power Optimized) ===");

  // Lower CPU frequency while awake (saves power)
  setCpuFrequencyMhz(80);

  disableRadios();

  pinMode(PIN_BTN, INPUT_PULLUP);
  ampOn(false);

  if (!LittleFS.begin()) {
    Serial.println("ERROR: LittleFS mount failed");
    while (true) delay(1000);
  }
  Serial.println("LittleFS mounted OK");

  if (!LittleFS.exists("/clip.wav")) {
    Serial.println("ERROR: /clip.wav not found in LittleFS");
    while (true) delay(1000);
  }
  Serial.println("Found /clip.wav");
  Serial.println("Ready: press button (D2) to play, then it will light-sleep.");
}

void loop() {
  // Sleep until the button is pressed (LOW)
  enterLightSleepUntilButton();

  // Debounce after wake
  delay(30);
  if (digitalRead(PIN_BTN) == LOW) {
    Serial.println("Playing /clip.wav...");
    bool ok = playWavOnce("/clip.wav");
    Serial.println(ok ? "Done.\n" : "Play failed.\n");

    // Wait for release so it doesn't re-trigger immediately
    while (digitalRead(PIN_BTN) == LOW) delay(5);
    delay(30);
  }
}
