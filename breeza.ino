#include <Wire.h>
#include "AudioTools.h"
#include "AudioTools/AudioCodecs/CodecMP3Helix.h"
#include "AudioTools/Disk/AudioSourceSDFAT.h"
#include <SD.h>

const char *mp3File = "/bre2.mp3";  // Archivo en raíz de SD
const int relayPin = 32;  // GPIO para el relé (bomba)
const int ledPin = 33;    // GPIO para el LED (enciende cuando el relé se enciende)
const int touchPin = 27;  // GPIO para el TTP223 (opcional, para futuros modos)
const unsigned long relayInterval = 180000;  // 20 segundos
const unsigned long relayOnDuration = 5000;  // 4 segundos
const unsigned long mp3DurationMs = 7800;   // 5 segundos (duración estimada de bre2.mp3)

AudioSourceSDFAT source("/", "mp3");  // Busca MP3 en raíz
MP3DecoderHelix decoder;
I2SStream out;
AudioPlayer player(source, out, decoder);

unsigned long previousMillis = 0;  // Temporizador para el ciclo completo
unsigned long audioStartMillis = 0;  // Temporizador para el audio
bool relayOn = false;  // Estado del relé
bool audioPlaying = false;  // Estado de reproducción de audio

void setup() {
  Serial.begin(115200);
  AudioLogger::instance().begin(Serial, AudioLogger::Info);  // Logs para depuración

  // Inicializa relé
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);  // Apaga relé al inicio (asumiendo low trigger)

// Inicializa LED
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);  // Apaga LED al inicio

  // Inicializa SD (CS en GPIO 5)
  if (!SD.begin(5)) {
    Serial.println("Error inicializando SD. Revisa conexiones.");
    while (true);
  }
  Serial.println("SD inicializada OK.");

  // Configuración de salida I2S para PCM5102
  auto cfg = out.defaultConfig(TX_MODE);
  cfg.sample_rate = 44100;  // Calidad máxima
  cfg.channels = 2;  // Estéreo
  cfg.bits_per_sample = 16;
  cfg.i2s_format = I2S_STD_FORMAT;
  cfg.pin_bck = 26;  // Bit Clock
  cfg.pin_ws = 25;   // Word Select (LCK)
  cfg.pin_data = 22; // Data In (DIN)
  out.begin(cfg);

  // Configura reproductor
  player.setVolume(0.5);  // Volumen moderado para evitar distorsión
  Serial.println("Sistema listo. Esperando ciclos...");
  previousMillis = millis();
}

void loop() {
  unsigned long currentMillis = millis();

  // Inicia reproducción de audio 5 segundos antes del intervalo
  unsigned long audioTriggerTime = relayInterval - mp3DurationMs;
  if (currentMillis - previousMillis >= audioTriggerTime && !audioPlaying && !relayOn) {
    Serial.println("Iniciando reproducción de bre2.mp3 (5s antes)...");
    player.setPath(mp3File);  // Establece la ruta
    player.begin();  // Inicia reproducción
    audioPlaying = true;
    audioStartMillis = currentMillis;  // Marca inicio del audio
  }

  // Procesa audio no bloqueante
  if (audioPlaying) {
    player.copy();  // Procesa el audio
    if (currentMillis - audioStartMillis >= mp3DurationMs) {
      Serial.println("Audio terminado. Activando relé...");
      player.stop();  // Detiene el audio
      audioPlaying = false;
      digitalWrite(relayPin, HIGH);  // Enciende relé
      digitalWrite(ledPin, HIGH);    // Enciende LED
      relayOn = true;
      previousMillis = currentMillis;  // Reinicia temporizador del ciclo
    }
  }

  // Apaga el relé después de 4 segundos
  if (relayOn && currentMillis - previousMillis >= relayOnDuration) {
    digitalWrite(relayPin, LOW);  // Apaga relé
    digitalWrite(ledPin, LOW);    // Enciende LED
    relayOn = false;
    Serial.println("Relé apagado. Esperando próximo ciclo...");
  }
}