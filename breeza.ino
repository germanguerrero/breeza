/****
  BREEZA - Versión con MP3 embebido + Bluetooth A2DP
  ESP32 + PCM5102 (I2S) + Relé + LED indicador
  - Reproduce audio embebido cada 90s + activa relé
  - Acepta audio Bluetooth sin interrumpir ciclos automáticos
****/

#include <Arduino.h>
#include <driver/i2s_std.h>  // Updated include for new I2S API
#include <driver/gpio.h>     // For gpio_num_t and GPIO_NUM_x
#include "freertos/FreeRTOS.h"  // For portMAX_DELAY
#include "AudioFileSourcePROGMEM.h"
#include "AudioGeneratorMP3.h"
#include "AudioOutputI2S.h"
#include "BluetoothA2DPSink.h"
#include "aviso.h"

// Pines
const int relayPin = 32;
const int ledPin   = 33;
const int touchPin = 27;

// Temporización
const unsigned long relayInterval    = 90000UL;  // 90 segundos
const unsigned long relayOnDuration  =  6000UL;  // 6 segundos encendido
const unsigned long mp3DurationMs    =  7800UL;  // duración del MP3

// Objetos de audio embebido (solo para ciclo automático)
AudioOutputI2S          *out = nullptr;
AudioFileSourcePROGMEM  *file = nullptr;
AudioGeneratorMP3       *mp3 = nullptr;

// Objeto Bluetooth
BluetoothA2DPSink a2dp_sink;

// Estados
bool audioPlaying = false;
bool relayOn      = false;
bool bluetoothConnected = false;
bool bluetoothPausedForCycle = false;  // Nuevo: indica si BT fue pausado por el ciclo

unsigned long previousMillis = 0;
unsigned long audioStartMillis = 0;
unsigned long lastBtLog = 0;
uint32_t totalBytesReceived = 0;

// Configuración I2S para Bluetooth
const gpio_num_t I2S_BCK_PIN  = GPIO_NUM_26;
const gpio_num_t I2S_WS_PIN   = GPIO_NUM_25;
const gpio_num_t I2S_DATA_PIN = GPIO_NUM_22;

i2s_chan_handle_t tx_handle = NULL;

// Callback para recibir y reproducir audio Bluetooth
void read_data_stream(const uint8_t *data, uint32_t length) {
  // Solo procesar si BT está conectado Y no fue pausado por el ciclo
  if (bluetoothConnected && !bluetoothPausedForCycle && data != nullptr && length > 0) {
    totalBytesReceived += length;
    
    // Escribir directamente al I2S
    size_t bytes_written = 0;
    i2s_channel_write(tx_handle, data, length, &bytes_written, portMAX_DELAY);
    
    // Log cada segundo
    unsigned long now = millis();
    if (now - lastBtLog > 1000) {
      Serial.print("📶 BT: ");
      Serial.print(length);
      Serial.print(" bytes | Escritos: ");
      Serial.print(bytes_written);
      Serial.print(" | Total: ");
      Serial.println(totalBytesReceived);
      lastBtLog = now;
    }
  }
}

// Callback de estado de audio
void audio_state_changed(esp_a2d_audio_state_t state, void *ptr) {
  Serial.print("🎵 Audio state: ");
  if (state == ESP_A2D_AUDIO_STATE_STARTED) {
    Serial.println("▶️  AUDIO BLUETOOTH INICIADO");
  } else if (state == ESP_A2D_AUDIO_STATE_STOPPED) {
    Serial.println("⏸️  AUDIO BLUETOOTH DETENIDO");
  } else if (state == ESP_A2D_AUDIO_STATE_REMOTE_SUSPEND) {
    Serial.println("⏸️  AUDIO BLUETOOTH EN PAUSA");
  }
}

// Configurar I2S manualmente para Bluetooth usando la nueva API
void setupI2SForBluetooth() {
  i2s_chan_config_t chan_cfg = {
    .id = I2S_NUM_0,
    .role = I2S_ROLE_MASTER,
    .dma_desc_num = 8,
    .dma_frame_num = 64,
    .auto_clear = true
  };

  esp_err_t err = i2s_new_channel(&chan_cfg, &tx_handle, NULL);
  if (err != ESP_OK) {
    Serial.printf("❌ Error creando canal I2S: %s\n", esp_err_to_name(err));
    return;
  }

  i2s_std_config_t std_cfg = {
    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(44100),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = I2S_BCK_PIN,
      .ws = I2S_WS_PIN,
      .dout = I2S_DATA_PIN,
      .din = I2S_GPIO_UNUSED,
      .invert_flags = {
        .mclk_inv = false,
        .bclk_inv = false,
        .ws_inv = false,
      },
    },
  };

  err = i2s_channel_init_std_mode(tx_handle, &std_cfg);
  if (err != ESP_OK) {
    Serial.printf("❌ Error inicializando modo std I2S: %s\n", esp_err_to_name(err));
    return;
  }

  err = i2s_channel_enable(tx_handle);
  if (err != ESP_OK) {
    Serial.printf("❌ Error habilitando canal I2S: %s\n", esp_err_to_name(err));
    return;
  }

  Serial.println("✅ I2S configurado para Bluetooth");
}

// Detener I2S para Bluetooth
void stopI2SForBluetooth() {
  if (tx_handle) {
    i2s_channel_disable(tx_handle);
    i2s_del_channel(tx_handle);
    tx_handle = NULL;
    Serial.println("⏹️  I2S liberado");
  }
}

// Callback de conexión Bluetooth
void connection_state_changed(esp_a2d_connection_state_t state, void *ptr) {
  if (state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
    Serial.println("🔵 Bluetooth CONECTADO");
    bluetoothConnected = true;
    totalBytesReceived = 0;
    
    // Detener audio embebido si estaba reproduciéndose
    if (audioPlaying) {
      Serial.println("Deteniendo audio embebido...");
      if (mp3) mp3->stop();
      audioPlaying = false;
    }
    
    // Liberar objetos de audio embebido
    if (file) {
      delete file;
      file = nullptr;
    }
    if (out) {
      delete out;
      out = nullptr;
    }
    if (mp3) {
      delete mp3;
      mp3 = nullptr;
    }
    
    delay(100);
    
    // Configurar I2S para Bluetooth
    setupI2SForBluetooth();
    
  } else if (state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
    Serial.println("🔴 Bluetooth DESCONECTADO");
    bluetoothConnected = false;
    bluetoothPausedForCycle = false;
    
    // Detener I2S de Bluetooth si estaba activo
    if (!audioPlaying && !relayOn) {
      stopI2SForBluetooth();
    }
    
    delay(200);
    Serial.println("Reinicializando audio embebido...");
    
    // Reinicializar sistema de audio embebido solo si no está en ciclo
    if (!audioPlaying && !relayOn) {
      out = new AudioOutputI2S();
      out->SetPinout((int)I2S_BCK_PIN, (int)I2S_WS_PIN, (int)I2S_DATA_PIN);
      out->SetGain(0.20);
      
      mp3 = new AudioGeneratorMP3();
      
      Serial.println("Audio embebido listo");
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== BREEZA v7 - MP3 embebido + Bluetooth ===");

  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  Serial.println("Iniciando Bluetooth A2DP...");
  
  // Configurar Bluetooth con stream reader personalizado
  a2dp_sink.set_auto_reconnect(false);
  a2dp_sink.set_on_connection_state_changed(connection_state_changed);
  a2dp_sink.set_stream_reader(read_data_stream, false);
  a2dp_sink.set_on_audio_state_changed(audio_state_changed);
  
  // Iniciar Bluetooth
  const char* deviceName = "BREEZA Audio";
  a2dp_sink.start(deviceName);
  
  Serial.println("✅ Bluetooth activado como: BREEZA Audio");
  Serial.println("📡 Esperando conexión...");
  
  delay(500);

  // Configuración inicial para audio embebido
  out = new AudioOutputI2S();
  out->SetPinout((int)I2S_BCK_PIN, (int)I2S_WS_PIN, (int)I2S_DATA_PIN);
  out->SetGain(0.20);
  
  mp3 = new AudioGeneratorMP3();

  Serial.print("Tamaño MP3 embebido: ");
  Serial.print(sizeof(rawData));
  Serial.println(" bytes");

  previousMillis = millis();
  Serial.println("Sistema listo. Ciclos automáticos + Bluetooth disponible");
}

void loop() {
  unsigned long currentMillis = millis();

  // === VERIFICAR SI ES MOMENTO DEL CICLO (PRIORIDAD MÁXIMA) ===
  bool cycleTimeReached = (currentMillis - previousMillis >= (relayInterval - mp3DurationMs));
  
  // Si es momento del ciclo Y Bluetooth está activo, pausarlo
  if (cycleTimeReached && bluetoothConnected && !bluetoothPausedForCycle && !audioPlaying && !relayOn) {
    Serial.println("⚠️  CICLO PRIORITARIO: Pausando Bluetooth (AVRCP)...");
    
    a2dp_sink.pause();          // ← ESTO ES LO IMPORTANTE
    delay(150);                 // Pequeño delay para que llegue el comando pause al teléfono
    
    bluetoothPausedForCycle = true;
    stopI2SForBluetooth();      // Descomentar esto para liberar I2S antes de audio embebido
    delay(100);
  }
  // === EJECUTAR CICLO AUTOMÁTICO (siempre tiene prioridad) ===
  if (cycleTimeReached && !audioPlaying && !relayOn) {
    
    Serial.println("🔔 Iniciando reproducción de audio embebido (7.8s antes)...");

    // Configurar audio embebido
    if (!out) {
      out = new AudioOutputI2S();
      out->SetPinout((int)I2S_BCK_PIN, (int)I2S_WS_PIN, (int)I2S_DATA_PIN);
      out->SetGain(0.20);
    }
    if (!mp3) {
      mp3 = new AudioGeneratorMP3();
    }

    file = new AudioFileSourcePROGMEM(rawData, sizeof(rawData));
    file->seek(0, SEEK_SET);
    
    if (mp3->begin(file, out)) {
      audioPlaying = true;
      audioStartMillis = currentMillis;
    } else {
      Serial.println("ERROR al iniciar MP3");
    }
  }

  // === REPRODUCIR AUDIO EMBEBIDO ===
  if (audioPlaying && mp3 && mp3->isRunning()) {
    if (!mp3->loop()) {
      Serial.println("Audio terminado. Activando relé...");
      audioPlaying = false;
      
      digitalWrite(relayPin, HIGH);
      digitalWrite(ledPin,   HIGH);
      relayOn = true;
      previousMillis = currentMillis;
    }
  }

  // === TIMEOUT DE SEGURIDAD ===
  if (audioPlaying && (currentMillis - audioStartMillis > mp3DurationMs + 5000)) {
    Serial.println("Timeout de audio → forzando activación del relé");
    
    if (file) { 
      delete file; 
      file = nullptr; 
    }

    if (mp3) mp3->stop();
    audioPlaying = false;
    
    digitalWrite(relayPin, HIGH);
    digitalWrite(ledPin,   HIGH);
    relayOn = true;
    previousMillis = currentMillis;
  }

  // === APAGAR RELÉ Y REANUDAR BLUETOOTH ===
  if (relayOn && (currentMillis - previousMillis >= relayOnDuration)) {
    digitalWrite(relayPin, LOW);
    digitalWrite(ledPin,   LOW);
    relayOn = false;
    Serial.println("Relé apagado.");

    if (bluetoothPausedForCycle && bluetoothConnected) {
      Serial.println("🔵 Reanudando Bluetooth (AVRCP play)...");
      
      // Liberar objetos embebidos
      if (file) { delete file; file = nullptr; }
      if (out)  { delete out;  out  = nullptr; }
      if (mp3)  { delete mp3;  mp3  = nullptr; }

      setupI2SForBluetooth();     // Reinstala I2S limpio
      delay(100);
      
      a2dp_sink.play();           // ← AQUÍ ESTÁ LA MAGIA: el teléfono vuelve a enviar audio
      bluetoothPausedForCycle = false;
      
      Serial.println("✅ Bluetooth reanudado - reproduciendo de nuevo");
    }
  }
  
  // Pequeño delay general
  delay(1);
}