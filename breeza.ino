/****
  BREEZA FINAL – MP3 embebido + Bluetooth A2DP
  ESP32 + PCM5102 + PAM8403
****/

#include <Arduino.h>
#include <driver/i2s_std.h>
#include <driver/gpio.h>
#include "freertos/FreeRTOS.h"
#include "AudioFileSourcePROGMEM.h"
#include "AudioGeneratorMP3.h"
#include "AudioOutput.h"
#include "BluetoothA2DPSink.h"
#include "aviso.h"

/* ================= PINES ================= */
const int relayPin = 32;
const int ledPin   = 33;

/* ================= TIMING ================= */
const unsigned long relayInterval   = 90000UL;
const unsigned long relayOnDuration = 12000UL;
const unsigned long mp3DurationMs   = 7800UL;

/* ================= I2S ================= */
const gpio_num_t I2S_BCK_PIN  = GPIO_NUM_26;
const gpio_num_t I2S_WS_PIN   = GPIO_NUM_25;
const gpio_num_t I2S_DATA_PIN = GPIO_NUM_22;

i2s_chan_handle_t tx_handle = NULL;

/* ================= AUDIO ================= */
AudioGeneratorMP3 *mp3 = nullptr;
AudioFileSourcePROGMEM *file = nullptr;
BluetoothA2DPSink a2dp_sink;

/* ================= ESTADOS ================= */
bool audioPlaying = false;
bool relayOn = false;
bool bluetoothConnected = false;
bool bluetoothPausedForCycle = false;

unsigned long previousMillis = 0;
unsigned long audioStartMillis = 0;

/* ================= AUDIO OUTPUT MP3 ================= */
class CustomAudioOutput : public AudioOutput {
  public:
    float gain = 0.12f;
    CustomAudioOutput(i2s_chan_handle_t handle) : tx(handle) {}

    bool SetRate(int hz) override { return hz == 44100; }
    bool SetChannels(int channels) override { return channels == 2; }
    bool begin() override { return true; }
    bool stop() override { return true; }

    bool ConsumeSample(int16_t sample[2]) override {
      sample[0] = (int16_t)(sample[0] * gain);
      sample[1] = (int16_t)(sample[1] * gain);

      size_t written;
      i2s_channel_write(tx, sample, sizeof(int16_t) * 2, &written, portMAX_DELAY);
      return written == sizeof(int16_t) * 2;
    }

  private:
    i2s_chan_handle_t tx;
};

CustomAudioOutput *out = nullptr;

/* ================= I2S INIT ================= */
void setupI2S() {
  i2s_chan_config_t chan_cfg = {
    .id = I2S_NUM_0,
    .role = I2S_ROLE_MASTER,
    .dma_desc_num = 8,
    .dma_frame_num = 64,
    .auto_clear = true
  };

  ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle, NULL));

  i2s_std_config_t std_cfg = {
    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(44100),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                  I2S_DATA_BIT_WIDTH_16BIT,
                  I2S_SLOT_MODE_STEREO),
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = I2S_BCK_PIN,
      .ws   = I2S_WS_PIN,
      .dout = I2S_DATA_PIN,
      .din  = I2S_GPIO_UNUSED
    }
  };

  ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &std_cfg));
  ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));

  Serial.println("✅ I2S listo (PCM5102)");
}

/* ================= BLUETOOTH PCM ================= */
void read_data_stream(const uint8_t *data, uint32_t len) {
  if (!bluetoothConnected || bluetoothPausedForCycle || !data) return;

  size_t written;
  i2s_channel_write(tx_handle, data, len, &written, portMAX_DELAY);
}

/* ================= BT CONNECTION ================= */
void connection_state_changed(esp_a2d_connection_state_t state, void *ptr) {
  if (state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
    Serial.println("🔵 Bluetooth conectado");
    bluetoothConnected = true;
  } 
  else if (state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
    Serial.println("🔴 Bluetooth desconectado");
    bluetoothConnected = false;
    bluetoothPausedForCycle = false;
  }
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);
  delay(800);

  pinMode(relayPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite(relayPin, LOW);
  digitalWrite(ledPin, LOW);

  setupI2S();

  out = new CustomAudioOutput(tx_handle);

  a2dp_sink.set_auto_reconnect(false);
  a2dp_sink.set_stream_reader(read_data_stream, false);
  a2dp_sink.set_on_connection_state_changed(connection_state_changed);
  a2dp_sink.start("BREEZA Audio");

  previousMillis = millis();
  Serial.println("🎧 Sistema listo");
}

/* ================= LOOP ================= */
void loop() {
  unsigned long now = millis();

  bool cycleTimeReached =
    (now - previousMillis >= (relayInterval - mp3DurationMs));

  /* === PRIORIDAD CICLO === */
  if (cycleTimeReached && !audioPlaying && !relayOn) {
    Serial.println("⏸️ Pausando Bluetooth (ciclo)");
    if (bluetoothConnected) {
      a2dp_sink.pause();
      delay(150);  
      bluetoothPausedForCycle = true;
    }

    file = new AudioFileSourcePROGMEM(rawData, sizeof(rawData));
    mp3  = new AudioGeneratorMP3();

    if (mp3->begin(file, out)) {
      audioPlaying = true;
      audioStartMillis = now;
      Serial.println("🔔 MP3 embebido ON");
    }
  }

  /* === MP3 LOOP === */
  if (audioPlaying && mp3 && mp3->isRunning()) {
    if (!mp3->loop()) {
      mp3->stop();
      delete mp3; mp3 = nullptr;
      delete file; file = nullptr;
      audioPlaying = false;

      digitalWrite(relayPin, HIGH);
      digitalWrite(ledPin, HIGH);
      relayOn = true;
      previousMillis = now;
      Serial.println("🔌 Relé ON");
    }
  }

  /* === TIMEOUT === */
  if (audioPlaying && (now - audioStartMillis > mp3DurationMs + 5000)) {
    Serial.println("⚠️ Timeout MP3");
    if (mp3) { mp3->stop(); delete mp3; mp3 = nullptr; }
    if (file) { delete file; file = nullptr; }
    audioPlaying = false;

    digitalWrite(relayPin, HIGH);
    digitalWrite(ledPin, HIGH);
    relayOn = true;
    previousMillis = now;
  }

  /* === RELÉ OFF + BT PLAY === */
  if (relayOn && (now - previousMillis >= relayOnDuration)) {
    digitalWrite(relayPin, LOW);
    digitalWrite(ledPin, LOW);
    relayOn = false;
    Serial.println("🔌 Relé OFF");

    if (bluetoothPausedForCycle && bluetoothConnected) {
      Serial.println("▶️ Reanudando Bluetooth");
      bluetoothPausedForCycle = false;
      a2dp_sink.play();
    }

  }

  delay(1);
}