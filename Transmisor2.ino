#include <driver/i2s.h>
#include <SPI.h>
#include <LoRa.h>
#include <Preferences.h>

// ==================== ID ARBITRO ====================
#define ARBITRO_ID  "CREW_CHIEF"

// ==================== PINES ====================
#define I2S_BCLK    1
#define I2S_WS      2
#define I2S_DATA    3
#define I2S_PORT    I2S_NUM_0

#define LORA_SCK    4
#define LORA_MISO   5
#define LORA_MOSI   6
#define LORA_CS     7
#define LORA_RST    10
#define LORA_DIO0   20

#define LORA_FREQ   915E6

#define BTN_START   8
#define BTN_STOP    9
#define DEBOUNCE_MS 50

// ==================== PREFERENCES (CALIBRACIÓN) ====================
Preferences prefs;
const char* PREF_KEY = "bat_cal";

// ==================== BATTERY ====================
// Ajusta BAT_PIN si mueves a otro pin ADC (actualmente usa GPIO0 según tu HW)
const int BAT_PIN = 0;               // GPIO donde está el divisor (actual: 0)
const float R1 = 47000.0;            // resistencia superior (a bateria)
const float R2 = 47000.0;            // resistencia inferior (a GND)
const float DIV_FACTOR = R2 / (R1 + R2); // 0.5 para 47k/47k

// ADC config
const int ADC_MAX = 4095;            // 12-bit (ESP32)
const float VREF_APPROX = 3.3;       // aproximación (usar calibración para mejor precisión)
float calib_factor = 1.0;            // factor de calibración (guardado en Preferences)

// Lectura/filtrado
const int MEDIAN_SAMPLES = 7;        // impar
const int SAMPLE_COUNT = 10;         // promedio externo: 10 lecturas
const int SAMPLE_DELAY_MS = 50;      // ms entre lecturas (10x50ms = 0.5 s por envío)

// Tabla voltaje -> porcentaje (LiPo 1S, más granular)
const int N_CURVE = 21;
const float voltages[N_CURVE] = {
  4.20,4.10,4.00,3.95,3.90,3.85,3.80,3.75,3.70,3.65,3.60,
  3.55,3.50,3.45,3.40,3.35,3.30,3.25,3.20,3.10,3.00
};
const int percents[N_CURVE] = {
  100,95,90,85,80,75,70,65,60,55,50,
   45,40,35,30,25,20,15,10,5,0
};

// ==================== AUDIO ====================
#define SAMPLE_RATE     16000
#define BUFFER_SIZE     512

int32_t buffer32[BUFFER_SIZE];
float samplesFloat[BUFFER_SIZE];

#define BAND_MIN         1500
#define BAND_MAX         5000
#define BAND_STEP        250
#define BAND_COUNT       ((BAND_MAX - BAND_MIN) / BAND_STEP + 1)  // 15
#define NOISE_BAND_COUNT 8

float bandCoeff[BAND_COUNT];
float filteredBand[BAND_COUNT];
float noiseBandCoeff[NOISE_BAND_COUNT];

#define FILTER_ALPHA    0.25f
#define NOISE_ALPHA     0.002f

float filteredBandEnergy  = 0.0f;
float filteredNoiseEnergy = 0.0f;

// ==================== PISO DE RUIDO ADAPTATIVO ====================
float noiseFloorBand = 0.0f;
float noiseFloorAmp  = 0.0f;

#define BAND_SNR_FACTOR  6.0f
#define RATIO_MIN        1.8f
#define AMP_SNR_FACTOR   2.5f
#define MIN_DUR_MS       200
#define MAX_GAP_MS       80
#define HOLD_MS          300

// ==================== DETECCION ====================
bool whistleDetected       = false;
unsigned long whistleStartTime = 0;
unsigned long lastGoodTime     = 0;
unsigned long holdUntil        = 0;
bool isCalibrating         = false;

float bandThreshold  = 0.0f;
float ratioThreshold = RATIO_MIN;
float ampThreshold   = 0.0f;

// ==================== PERFIL ESPECTRAL ====================
float whistleProfile[BAND_COUNT];
float whistleProfileAccum[BAND_COUNT];
float calibratedPeakBinSum = 0.0f;
int calibratedPeakBin = -1;

// Adaptativos: calculados en calibración
float adaptive_similarity_min = 0.80f;
int   adaptive_peak_tolerance = 1;

// ==================== BOTONES ====================
unsigned long lastStartPress = 0;
unsigned long lastStopPress  = 0;

// ==================== CALIBRACION ====================
#define MAX_PEAKS   5
float peakBandEnergy[MAX_PEAKS];
float peakRatio[MAX_PEAKS];
float peakAmp[MAX_PEAKS];
float peakProfiles[MAX_PEAKS][BAND_COUNT];  // Para calcular similitud interna
int   peakPeakBins[MAX_PEAKS];
int   peakCount = 0;

// ==================== PROTOTIPOS ====================
void initI2S();
void initLoRa();
void initBandFilters();
void initButtons();
void calculateThresholds();
void resetWhistleProfile();
void addWhistleProfileSample(float *samples, int n);
void finalizeWhistleProfile();
int findPeakBin(float *values, int n);
float computeProfileSimilarity(float *values, int n);
float processGoertzel(float *samples, int n, float coeff);
float computeBandEnergy(float *samples, int n);
float computeNoiseEnergy(float *samples, int n);
bool readSamples();
float readAmplitude();
void measureNoise();
void calibrateWhistles();
void updateAdaptiveNoise(float bandE, float amp);
void sendLoRa(const char* comando);
void checkButtons();
void checkLoRaCommands();
void executeCommand(char cmd);
void detect();

// --- funciones de bateria ---
int readRawMedianBattery();
float readBatteryVoltageOnceRaw();
float readBatteryVoltageOnce();
int percentFromVoltage(float v);
int getBatteryPercent();
void doSerialCalibration(String arg);
void checkSerialCommands();

// ==================== SETUP ====================

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println(F("\n=== SILBATO ARBITRO v4.7 ==="));
  Serial.printf("ID: %s\n", ARBITRO_ID);
  Serial.printf("Banda: %d-%d Hz | SNR: %.1f | RatioMin: %.1f | Dur: %dms | Gap: %dms\n",
                BAND_MIN, BAND_MAX, BAND_SNR_FACTOR, RATIO_MIN, MIN_DUR_MS, MAX_GAP_MS);
  Serial.println(F("Comandos: enviados por LoRa desde el receptor (CMD:c/s/+/-)"));

  initLoRa();
  initI2S();
  initBandFilters();
  initButtons();

  // Configuración ADC para batería
  analogReadResolution(12);
  analogSetPinAttenuation(BAT_PIN, ADC_11db);
  // cargar calib_factor guardado (si existe)
  prefs.begin("silbato", false);
  calib_factor = prefs.getFloat(PREF_KEY, 1.0f);
  prefs.end();
  Serial.printf("ADC battery configured (12-bit, 11db attenuation) | calib_factor loaded = %.6f\n", calib_factor);

  Serial.println(F("Calentando I2S..."));
  for (int i = 0; i < 10; i++) { readSamples(); delay(10); }

  Serial.println(F("PASO 1: Silencio 3 segundos..."));
  delay(500);
  measureNoise();
  Serial.printf("Ruido inicial -> Banda: %.2f | Amp: %.0f\n\n", noiseFloorBand, noiseFloorAmp);

  Serial.println(F("PASO 2: SILBA 3 veces..."));
  delay(500);
  calibrateWhistles();

  if (peakCount >= 2) {
    Serial.println(F("\n=== LISTO - Modo adaptativo activo ==="));
  } else {
    Serial.println(F("\n[AVISO] Pocos silbatos. Arrancando con umbrales por defecto."));
    calculateThresholds();
  }
}

// ==================== LOOP ====================

void loop() {
  checkSerialCommands();   // <-- modo calibración por Serial
  checkLoRaCommands();
  checkButtons();
  if (!isCalibrating) detect();
  delay(2);
}

// ==================== LORA ====================

void initLoRa() {
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  Serial.println(F("Iniciando LoRa..."));
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println(F("[ERROR] LoRa no encontrado."));
    while (1) delay(1000);
  }
  Serial.println(F("LoRa OK"));
}

void sendLoRa(const char* comando) {
  // Calcula porcentaje de bateria en el momento del envío (promedia SAMPLE_COUNT lecturas)
  int batPct = getBatteryPercent(); // 0..100

  char mensaje[80];
  // Formato: ID:comando:BAT  (ej. CREW_CHIEF:start:78)
  snprintf(mensaje, sizeof(mensaje), "%s:%s:%d", ARBITRO_ID, comando, batPct);
  LoRa.beginPacket();
  LoRa.print(mensaje);
  LoRa.endPacket();
  Serial.printf("[TX] %s\n", mensaje);
}

// ==================== checkLoRaCommands / executeCommand ====================

void checkLoRaCommands() {
  int packetSize = LoRa.parsePacket();
  if (packetSize == 0) return;

  String pkt = "";
  while (LoRa.available()) pkt += (char)LoRa.read();
  pkt.trim();

  Serial.printf("[RX] '%s' | RSSI:%d\n", pkt.c_str(), LoRa.packetRssi());

  if (!pkt.startsWith("CMD:")) return;

  String payload = pkt.substring(4);
  int sep = payload.indexOf(':');
  char cmd = 0;

  if (sep != -1) {
    String destID = payload.substring(0, sep);
    if (destID != ARBITRO_ID) return;
    cmd = payload.charAt(sep + 1);
  } else {
    cmd = payload.charAt(0);
  }

  if (cmd != 0) executeCommand(cmd);
}

void executeCommand(char cmd) {
  switch (cmd) {
    case 'c': case 'C':
      Serial.println(F("\n=== RECALIBRANDO (autorizado por receptor) ==="));
      measureNoise();
      Serial.printf("Ruido banda:%.2f Amp:%.0f\n", noiseFloorBand, noiseFloorAmp);
      Serial.println(F("Silba 3 veces..."));
      calibrateWhistles();
      Serial.println(F("Listo."));
      break;

    case 's': case 'S': {
      Serial.println(F("\n--- ESTADO ---"));
      Serial.printf("ID:      %s\n", ARBITRO_ID);
      Serial.printf("BandE:   %.1f (thr=%.1f)\n", filteredBandEnergy, bandThreshold);
      Serial.printf("NoiseE:  %.1f\n", filteredNoiseEnergy);
      float ratio = (filteredNoiseEnergy > 0.1f) ? filteredBandEnergy / filteredNoiseEnergy : 0.0f;
      Serial.printf("Ratio:   %.2f (thr=%.2f)\n", ratio, ratioThreshold);
      Serial.printf("Amp:     %.0f (thr=%.0f)\n", readAmplitude(), ampThreshold);
      Serial.printf("NoiseFl: Banda=%.2f Amp=%.0f\n", noiseFloorBand, noiseFloorAmp);
      Serial.printf("Estado:  %s\n", whistleDetected ? "SILBATO" : "quieto");
      Serial.printf("Perfil:  sim=%.2f (thr=%.2f) bin=%d (tol=%d)\n",
                    computeProfileSimilarity(filteredBand, BAND_COUNT),
                    adaptive_similarity_min,
                    calibratedPeakBin,
                    adaptive_peak_tolerance);
      Serial.println(F("--------------"));
      break;
    }

    case '+':
      bandThreshold *= 0.9f;
      ampThreshold  *= 0.9f;
      Serial.printf("[SENS +] BandThr=%.2f AmpThr=%.0f\n", bandThreshold, ampThreshold);
      break;

    case '-':
      bandThreshold *= 1.1f;
      ampThreshold  *= 1.1f;
      Serial.printf("[SENS -] BandThr=%.2f AmpThr=%.0f\n", bandThreshold, ampThreshold);
      break;

    default:
      Serial.printf("[RX] Comando desconocido: '%c'\n", cmd);
      break;
  }
}

// ==================== BOTONES ====================

void initButtons() {
  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_STOP,  INPUT_PULLUP);
  Serial.printf("Botones -> START: GPIO%d | STOP: GPIO%d\n", BTN_START, BTN_STOP);
}

void checkButtons() {
  unsigned long now = millis();
  if (digitalRead(BTN_START) == LOW && now - lastStartPress > DEBOUNCE_MS) {
    lastStartPress = now;
    sendLoRa("start");
  }
  if (digitalRead(BTN_STOP) == LOW && now - lastStopPress > DEBOUNCE_MS) {
    lastStopPress = now;
    sendLoRa("stop");
  }
}

// ==================== I2S ====================

void initI2S() {
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 256,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  i2s_pin_config_t pins = {
    .bck_io_num   = I2S_BCLK,
    .ws_io_num    = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num  = I2S_DATA
  };
  i2s_driver_install(I2S_PORT, &cfg, 0, NULL);
  i2s_set_pin(I2S_PORT, &pins);
  i2s_set_clk(I2S_PORT, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_MONO);
}

// ==================== BANCO DE FILTROS ====================

void initBandFilters() {
  for (int i = 0; i < BAND_COUNT; i++) {
    float freq = BAND_MIN + i * BAND_STEP;
    float k = (float)BUFFER_SIZE * freq / SAMPLE_RATE;
    bandCoeff[i] = 2.0f * cosf(2.0f * PI * k / BUFFER_SIZE);
    filteredBand[i] = 0.0f;
  }
  int noiseFreqs[NOISE_BAND_COUNT] = {300, 500, 750, 1000, 1250, 5500, 6000, 6500};
  for (int i = 0; i < NOISE_BAND_COUNT; i++) {
    float k = (float)BUFFER_SIZE * noiseFreqs[i] / SAMPLE_RATE;
    noiseBandCoeff[i] = 2.0f * cosf(2.0f * PI * k / BUFFER_SIZE);
  }
}

float processGoertzel(float *samples, int n, float coeff) {
  float q0 = 0.0f, q1 = 0.0f, q2 = 0.0f;
  for (int i = 0; i < n; i++) {
    q0 = coeff * q1 - q2 + samples[i];
    q2 = q1; q1 = q0;
  }
  float magSq = q1*q1 + q2*q2 - coeff*q1*q2;
  if (magSq < 0.0f) magSq = 0.0f;
  return sqrtf(magSq);
}

float computeBandEnergy(float *samples, int n) {
  float total = 0.0f;
  for (int i = 0; i < BAND_COUNT; i++)
    total += processGoertzel(samples, n, bandCoeff[i]);
  return total;
}

float computeNoiseEnergy(float *samples, int n) {
  float total = 0.0f;
  for (int i = 0; i < NOISE_BAND_COUNT; i++)
    total += processGoertzel(samples, n, noiseBandCoeff[i]);
  return total;
}

// ==================== PERFIL ESPECTRAL ====================

void resetWhistleProfile() {
  for (int i = 0; i < BAND_COUNT; i++) {
    whistleProfile[i] = 0.0f;
    whistleProfileAccum[i] = 0.0f;
  }
  calibratedPeakBinSum = 0.0f;
  calibratedPeakBin = -1;
  adaptive_similarity_min = 0.80f;
  adaptive_peak_tolerance = 1;
  peakCount = 0;
}

int findPeakBin(float *values, int n) {
  int idx = 0;
  float maxV = values[0];
  for (int i = 1; i < n; i++) {
    if (values[i] > maxV) {
      maxV = values[i];
      idx = i;
    }
  }
  return idx;
}

void addWhistleProfileSample(float *samples, int n) {
  float mags[BAND_COUNT];
  float sum = 0.0f;

  for (int i = 0; i < BAND_COUNT; i++) {
    mags[i] = processGoertzel(samples, n, bandCoeff[i]);
    if (mags[i] < 0.0f) mags[i] = 0.0f;
    sum += mags[i];
  }

  if (sum < 0.001f) return;

  int peakBin = findPeakBin(mags, BAND_COUNT);
  calibratedPeakBinSum += peakBin;
  peakPeakBins[peakCount] = peakBin;

  for (int i = 0; i < BAND_COUNT; i++) {
    float normalized = mags[i] / sum;
    whistleProfileAccum[i] += normalized;
    peakProfiles[peakCount][i] = normalized;
  }
}

void finalizeWhistleProfile() {
  if (peakCount <= 0) return;

  float sum = 0.0f;
  for (int i = 0; i < BAND_COUNT; i++) {
    whistleProfile[i] = whistleProfileAccum[i] / peakCount;
    sum += whistleProfile[i];
  }

  if (sum > 0.001f) {
    for (int i = 0; i < BAND_COUNT; i++) {
      whistleProfile[i] /= sum;
    }
  }

  calibratedPeakBin = (int)roundf(calibratedPeakBinSum / peakCount);

  // Calcular similitud interna entre silbatos para adaptar el umbral
  float minSim = 1.0f;
  for (int i = 0; i < peakCount; i++) {
    for (int j = i+1; j < peakCount; j++) {
      float dot = 0.0f, normA = 0.0f, normB = 0.0f;
      for (int k = 0; k < BAND_COUNT; k++) {
        float a = peakProfiles[i][k];
        float b = peakProfiles[j][k];
        dot += a * b;
        normA += a * a;
        normB += b * b;
      }
      if (normA > 0.000001f && normB > 0.000001f) {
        float sim = dot / sqrtf(normA * normB);
        if (sim < minSim) minSim = sim;
      }
    }
  }

  // RELAJADO: usar 0.70 en vez de 0.85
  adaptive_similarity_min = minSim * 0.70f;

  // RELAJADO: mínimo absoluto más bajo
  if (adaptive_similarity_min < 0.60f) adaptive_similarity_min = 0.60f;
  if (adaptive_similarity_min > 0.95f) adaptive_similarity_min = 0.95f;

  // Calcular variación del bin dominante
  int minBin = peakPeakBins[0], maxBin = peakPeakBins[0];
  for (int i = 1; i < peakCount; i++) {
    if (peakPeakBins[i] < minBin) minBin = peakPeakBins[i];
    if (peakPeakBins[i] > maxBin) maxBin = peakPeakBins[i];
  }
  adaptive_peak_tolerance = maxBin - minBin;
  if (adaptive_peak_tolerance < 0) adaptive_peak_tolerance = 0;
  if (adaptive_peak_tolerance > 2) adaptive_peak_tolerance = 2;

  Serial.printf("[CAL] PeakBin=%d (±%d) | SimMin=%.2f\n",
                calibratedPeakBin, adaptive_peak_tolerance, adaptive_similarity_min);
}

float computeProfileSimilarity(float *values, int n) {
  float sum = 0.0f;
  for (int i = 0; i < n; i++) {
    if (values[i] > 0.0f) sum += values[i];
  }
  if (sum < 0.001f) return 0.0f;

  float dot = 0.0f;
  float normA = 0.0f;
  float normB = 0.0f;

  for (int i = 0; i < n; i++) {
    float a = (values[i] > 0.0f) ? values[i] / sum : 0.0f;
    float b = whistleProfile[i];
    dot += a * b;
    normA += a * a;
    normB += b * b;
  }

  if (normA < 0.000001f || normB < 0.000001f) return 0.0f;
  return dot / sqrtf(normA * normB);
}

// ==================== LECTURA ====================

bool readSamples() {
  size_t bytesRead = 0;
  esp_err_t err = i2s_read(I2S_PORT, &buffer32, sizeof(buffer32), &bytesRead, portMAX_DELAY);
  if (err != ESP_OK || bytesRead == 0) return false;
  int n = bytesRead / 4;
  if (n > BUFFER_SIZE) n = BUFFER_SIZE;
  for (int i = 0; i < n; i++)
    samplesFloat[i] = (float)(buffer32[i] >> 8) / 8388608.0f;
  for (int i = n; i < BUFFER_SIZE; i++)
    samplesFloat[i] = 0.0f;
  return true;
}

float readAmplitude() {
  if (!readSamples()) return -1.0f;
  int64_t sum = 0; int32_t maxVal = 0;
  for (int i = 0; i < BUFFER_SIZE; i++) {
    int32_t v = abs((int32_t)(samplesFloat[i] * 8388608.0f));
    sum += v; if (v > maxVal) maxVal = v;
  }
  return ((float)sum / BUFFER_SIZE * 0.7f) + (maxVal * 0.3f);
}

// ==================== UMBRALES ====================

void calculateThresholds() {
  if (peakCount > 0) {
    float avgBand = 0.0f;
    float avgRatio = 0.0f;
    float avgAmp = 0.0f;

    for (int i = 0; i < peakCount; i++) {
      avgBand += peakBandEnergy[i];
      avgRatio += peakRatio[i];
      avgAmp += peakAmp[i];
    }

    avgBand /= peakCount;
    avgRatio /= peakCount;
    avgAmp /= peakCount;

    bandThreshold  = max(noiseFloorBand * 5.0f, avgBand * 0.45f);
    ampThreshold   = max(noiseFloorAmp  * 2.2f, avgAmp  * 0.35f);
    ratioThreshold = max(RATIO_MIN, avgRatio * 0.75f);
  } else {
    bandThreshold  = noiseFloorBand * BAND_SNR_FACTOR;
    ampThreshold   = noiseFloorAmp  * AMP_SNR_FACTOR;
    ratioThreshold = RATIO_MIN;
  }

  Serial.printf("[THR] BandThr=%.2f | AmpThr=%.0f | RatioThr=%.2f\n",
                bandThreshold, ampThreshold, ratioThreshold);
}

// ==================== RUIDO ADAPTATIVO ====================

void updateAdaptiveNoise(float bandE, float amp) {
  if (bandE < noiseFloorBand * 2.0f && amp < noiseFloorAmp * 2.0f) {
    noiseFloorBand = NOISE_ALPHA * bandE + (1.0f - NOISE_ALPHA) * noiseFloorBand;
    noiseFloorAmp  = NOISE_ALPHA * amp   + (1.0f - NOISE_ALPHA) * noiseFloorAmp;
    bandThreshold = max(noiseFloorBand * 5.0f, bandThreshold);
    ampThreshold  = max(noiseFloorAmp  * 2.2f, ampThreshold);
  }
}

// ==================== MEDIR RUIDO INICIAL ====================

void measureNoise() {
  float sumBand = 0.0f, sumAmp = 0.0f;
  int count = 0;
  for (int i = 0; i < 10; i++) { readSamples(); delay(10); }
  unsigned long start = millis();
  while (millis() - start < 3000) {
    if (readSamples()) {
      sumBand += computeBandEnergy(samplesFloat, BUFFER_SIZE);
      int64_t s = 0;
      for (int i = 0; i < BUFFER_SIZE; i++)
        s += abs((int32_t)(samplesFloat[i] * 8388608.0f));
      sumAmp += (float)s / BUFFER_SIZE;
      count++;
    }
    delay(5);
  }
  noiseFloorBand = (count > 0) ? (sumBand / count) : 1.0f;
  noiseFloorAmp  = (count > 0) ? (sumAmp  / count) : 100.0f;
  calculateThresholds();
}

// ==================== CALIBRACION ====================

void calibrateWhistles() {
  resetWhistleProfile();
  peakCount = 0;
  unsigned long start = millis(), lastPeak = 0;
  Serial.println(F("Escuchando... silba 3 veces"));

  while (millis() - start < 8000 && peakCount < MAX_PEAKS) {
    if (!readSamples()) { delay(2); continue; }

    int64_t s = 0; int32_t maxVal = 0;
    for (int i = 0; i < BUFFER_SIZE; i++) {
      int32_t v = abs((int32_t)(samplesFloat[i] * 8388608.0f));
      s += v; if (v > maxVal) maxVal = v;
    }
    float amp    = ((float)s / BUFFER_SIZE * 0.7f) + (maxVal * 0.3f);
    float bandE  = computeBandEnergy(samplesFloat, BUFFER_SIZE);
    float noiseE = computeNoiseEnergy(samplesFloat, BUFFER_SIZE);
    float ratio  = (noiseE > 0.1f) ? (bandE / noiseE) : 0.0f;
    unsigned long now = millis();

    static unsigned long lastDbg = 0;
    if (amp > noiseFloorAmp * 2.0f && now - lastDbg > 200) {
      lastDbg = now;
      Serial.printf("  [CAL] BandE:%.1f NoiseE:%.1f Ratio:%.2f Amp:%.0f\n",
                    bandE, noiseE, ratio, amp);
    }

    if (bandE > noiseFloorBand * 4.0f &&
        ratio > RATIO_MIN &&
        amp   > noiseFloorAmp * 2.0f &&
        now - lastPeak > 500) {
      peakBandEnergy[peakCount] = bandE;
      peakRatio[peakCount]      = ratio;
      peakAmp[peakCount]        = amp;

      addWhistleProfileSample(samplesFloat, BUFFER_SIZE);

      peakCount++;
      lastPeak = now;
      Serial.printf("  SILBATO #%d: BandE=%.1f | Ratio=%.2f | Amp=%.0f\n",
                    peakCount, bandE, ratio, amp);
    }
    delay(2);
  }
  Serial.printf("\nDetectados: %d silbatos\n", peakCount);
  isCalibrating = false;

  if (peakCount > 0) finalizeWhistleProfile();

  calculateThresholds();
}

// ==================== DETECCION ====================

void detect() {
  if (!readSamples()) return;

  int64_t s = 0; int32_t maxVal = 0;
  for (int i = 0; i < BUFFER_SIZE; i++) {
    int32_t v = abs((int32_t)(samplesFloat[i] * 8388608.0f));
    s += v; if (v > maxVal) maxVal = v;
  }
  float amp = ((float)s / BUFFER_SIZE * 0.7f) + (maxVal * 0.3f);

  float rawBandE = 0.0f;
  for (int i = 0; i < BAND_COUNT; i++) {
    float mag = processGoertzel(samplesFloat, BUFFER_SIZE, bandCoeff[i]);
    filteredBand[i] = FILTER_ALPHA * mag + (1.0f - FILTER_ALPHA) * filteredBand[i];
    rawBandE += filteredBand[i];
  }
  filteredBandEnergy = rawBandE;

  float noiseE = computeNoiseEnergy(samplesFloat, BUFFER_SIZE);
  filteredNoiseEnergy = FILTER_ALPHA * noiseE + (1.0f - FILTER_ALPHA) * filteredNoiseEnergy;

  float ratio = (filteredNoiseEnergy > 0.1f) ? (filteredBandEnergy / filteredNoiseEnergy) : 0.0f;

  int currentPeakBin = findPeakBin(filteredBand, BAND_COUNT);
  float similarity = computeProfileSimilarity(filteredBand, BAND_COUNT);

  bool bandOK    = filteredBandEnergy > bandThreshold;
  bool ratioOK   = ratio > ratioThreshold;
  bool ampOK     = amp   > ampThreshold;
  bool peakBinOK = (calibratedPeakBin >= 0) &&
                   (abs(currentPeakBin - calibratedPeakBin) <= adaptive_peak_tolerance);
  bool profileWarning = (similarity < adaptive_similarity_min); // avisar si perfil bajo

  // Permitir detección aunque perfil sea bajo
  bool conditionsMet = bandOK && ratioOK && ampOK && peakBinOK;

  unsigned long now = millis();

  if (!whistleDetected && !conditionsMet)
    updateAdaptiveNoise(rawBandE, amp);

  static unsigned long lastDebug = 0;
  if (amp > noiseFloorAmp * 2.0f && now - lastDebug > 300) {
    lastDebug = now;
    if (!bandOK)
      Serial.printf("[DBG] Banda BAJA: %.1f (thr=%.1f) | NoiseFl=%.1f | Amp=%.0f\n",
                    filteredBandEnergy, bandThreshold, noiseFloorBand, amp);
    else if (!ratioOK)
      Serial.printf("[DBG] Ratio BAJO: BandE=%.1f NoiseE=%.1f Ratio=%.2f (thr=%.2f)\n",
                    filteredBandEnergy, filteredNoiseEnergy, ratio, ratioThreshold);
    else if (!ampOK)
      Serial.printf("[DBG] Amp BAJA: %.0f (thr=%.0f)\n", amp, ampThreshold);
    else if (!peakBinOK)
      Serial.printf("[DBG] BIN NO COINCIDE: actual=%d calibrado=%d (±%d)\n",
                    currentPeakBin, calibratedPeakBin, adaptive_peak_tolerance);
    else if (profileWarning)
      Serial.printf("[DBG] ⚠️  PERFIL BAJO: sim=%.2f (thr=%.2f)\n",
                    similarity, adaptive_similarity_min);
    else {
      unsigned long dur = (whistleStartTime > 0) ? (now - whistleStartTime) : 0;
      Serial.printf("[DBG] CANDIDATO! BandE=%.1f Ratio=%.2f Amp=%.0f Sim=%.2f dur=%lums/%dms\n",
                    filteredBandEnergy, ratio, amp, similarity, dur, MIN_DUR_MS);
    }
  }

  bool prevDetected = whistleDetected;

  if (!whistleDetected) {
    if (conditionsMet) {
      if (whistleStartTime == 0) whistleStartTime = now;
      lastGoodTime = now;
      if (now - whistleStartTime >= MIN_DUR_MS) {
        whistleDetected = true;
        holdUntil = now + HOLD_MS;
        Serial.printf("[OK] SILBATO CONFIRMADO! BandE=%.1f Ratio=%.2f Amp=%.0f Sim=%.2f\n",
                      filteredBandEnergy, ratio, amp, similarity);
      }
    } else {
      if (whistleStartTime > 0 && lastGoodTime == 0) lastGoodTime = now;
      if (whistleStartTime > 0 && now - lastGoodTime > MAX_GAP_MS) {
        whistleStartTime = 0;
        lastGoodTime     = 0;
      }
    }
  } else {
    if (filteredBandEnergy < bandThreshold * 0.5f ||
        ratio              < ratioThreshold * 0.6f ||
        amp                < ampThreshold   * 0.5f) {
      if (now > holdUntil) {
        whistleDetected  = false;
        whistleStartTime = 0;
        lastGoodTime     = 0;
      }
    } else {
      holdUntil = now + HOLD_MS;
    }
  }

  if (whistleDetected && !prevDetected)
    sendLoRa("silbatazo");
}

// ==================== FUNCIONES DE BATERIA ====================

// Lee MEDIAN_SAMPLES valores y devuelve la mediana del raw ADC
int readRawMedianBattery() {
  int s[MEDIAN_SAMPLES];
  for (int i = 0; i < MEDIAN_SAMPLES; ++i) {
    s[i] = analogRead(BAT_PIN);
    delay(6);
  }
  // insertion sort
  for (int i = 1; i < MEDIAN_SAMPLES; ++i) {
    int key = s[i];
    int j = i - 1;
    while (j >= 0 && s[j] > key) {
      s[j + 1] = s[j];
      j--;
    }
    s[j + 1] = key;
  }
  return s[MEDIAN_SAMPLES / 2];
}

// Devuelve la tension calculada SIN aplicar calib_factor (V raw estimada)
float readBatteryVoltageOnceRaw() {
  int raw = readRawMedianBattery();
  float vDiv = (raw / (float)ADC_MAX) * VREF_APPROX; // voltaje en punto medio del divisor (aprox)
  float vBatRaw = (vDiv / DIV_FACTOR);              // sin aplicar calib_factor
  return vBatRaw;
}

// Devuelve la tension ya calibrada/aplicando calib_factor
float readBatteryVoltageOnce() {
  return readBatteryVoltageOnceRaw() * calib_factor;
}

// Interpola la tabla voltaje->% y devuelve 0..100
int percentFromVoltage(float v) {
  if (v >= voltages[0]) return 100;
  if (v <= voltages[N_CURVE - 1]) return 0;
  for (int i = 0; i < N_CURVE - 1; ++i) {
    if (v <= voltages[i] && v >= voltages[i + 1]) {
      float t = (v - voltages[i + 1]) / (voltages[i] - voltages[i + 1]);
      float p = percents[i + 1] + t * (percents[i] - percents[i + 1]);
      return constrain((int)round(p), 0, 100);
    }
  }
  return 0;
}

// Promedia SAMPLE_COUNT lecturas (cada lectura usa mediana); devuelve porcentaje entero
int getBatteryPercent() {
  float sum = 0.0f;
  for (int i = 0; i < SAMPLE_COUNT; ++i) {
    float v = readBatteryVoltageOnce(); // ya aplica calib_factor
    sum += v;
    delay(SAMPLE_DELAY_MS);
  }
  float avgV = sum / SAMPLE_COUNT;
  int pct = percentFromVoltage(avgV);
  Serial.printf("[BAT] Vavg=%.3f V -> %d%% (factor=%.4f)\n", avgV, pct, calib_factor);
  return pct;
}

// --- Calibración por Serial ---
// Usa la lectura RAW (sin factor) para calcular factor = V_medido / V_raw
void doSerialCalibration(String arg) {
  arg.trim();
  if (arg.length() == 0) {
    Serial.println("Uso: cal <voltaje_en_V>  (ej: cal 3.98)");
    return;
  }
  float vMeasured = arg.toFloat();
  if (vMeasured <= 0.0f) {
    Serial.println("Valor invalido. Introduce voltaje en V (ej: 3.98)");
    return;
  }
  Serial.println("Iniciando lectura promedio RAW para calibracion...");
  // hacemos el mismo promedio que en getBatteryPercent pero con RAW
  float sum = 0.0f;
  for (int i = 0; i < SAMPLE_COUNT; ++i) {
    float vRaw = readBatteryVoltageOnceRaw();
    sum += vRaw;
    delay(SAMPLE_DELAY_MS);
  }
  float avgRaw = sum / SAMPLE_COUNT;
  float newFactor = vMeasured / avgRaw;
  calib_factor = newFactor;
  // guardar en Preferences
  prefs.begin("silbato", false);
  prefs.putFloat(PREF_KEY, calib_factor);
  prefs.end();
  Serial.printf("Calibracion guardada: Vmed=%.3fV | Vraw=%.3fV | factor=%.4f\n",
                vMeasured, avgRaw, calib_factor);
}

// Leer comando simple por Serial (llamar desde loop)
void checkSerialCommands() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;
  String low = line;
  low.toLowerCase();
  if (low.startsWith("cal ")) {
    String arg = line.substring(4);
    doSerialCalibration(arg);
  } else if (low == "cal") {
    Serial.println("Uso: cal <voltaje_en_V>  (ej: cal 3.98)");
  } else if (low == "showcal") {
    Serial.printf("calib_factor = %.6f\n", calib_factor);
  } else {
    Serial.println("Comando desconocido. 'cal <V>' o 'showcal'");
  }
}