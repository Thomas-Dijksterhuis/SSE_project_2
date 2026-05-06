#include "driver/i2s.h"
#include <WiFi.h>
#include <SD.h>
#include <ArduinoJson.h>
#include <SPI.h>

#define FRAME_SIZE 1024
#define QUEUE_SIZE 20
volatile uint32_t queueDrops = 0;
volatile uint32_t queueUnderruns = 0;

uint32_t fileIdx = 0;

#define CHUNK_FRAMES 10
int16_t chunkBuffer[FRAME_SIZE * CHUNK_FRAMES];
int chunkPos = 0;
int16_t audioQueue[QUEUE_SIZE][FRAME_SIZE];
volatile int writeIdx = 0;
volatile int readIdx = 0;
volatile int count = 0;

File audioFile;
bool sdLogging = false;
uint32_t sdFrameCounter = 0;

#define I2S_PORT I2S_NUM_0
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

WiFiClient client;

#define I2S_BCK  26
#define I2S_WS   25
#define I2S_DATA 22

#define CS 5
#define MOSI 23
#define MISO 19
#define CLK 18

char ssid[64];
char password[64];
char targetIP[64];
char deviceId[64];

struct __attribute__((packed)) tcpPackage {
  char deviceId[16];
  uint32_t timestamp;
  uint32_t length;
};

#define SAMPLE_RATE 10000

#define FILTER_LEN 8
#define MU 0.001f

int16_t i2sBuffer[FRAME_SIZE * 2];

float x[FILTER_LEN] = {0};
float w[FILTER_LEN] = {0};
int idx = 0;

int16_t frameOut[FRAME_SIZE];
int framePos = 0;

uint16_t targetPort = 50005;

bool sdReady = false;
bool wifiLoaded = false;
unsigned long lastCheck = 0;

bool InitSD() {
  if (!SD.begin(CS)) return false;

  File test = SD.open("/");
  if (!test) return false;

  test.close();
  return true;
}

bool getNetworkInfo() {
  File file = SD.open("/Network/Network.json");
  if (!file) {
    Serial.println("could not find network file");
    return false;
  }

  String data = file.readString();
  file.close();

  JsonDocument doc;
  if (deserializeJson(doc, data)) return false;

  strlcpy(ssid, doc["SSID"] | "", sizeof(ssid));
  strlcpy(password, doc["Password"] | "", sizeof(password));

  // reuse your field but now as UDP target
  strlcpy(targetIP, doc["ReceiverIpAddress"] | "", sizeof(targetIP));

  strlcpy(deviceId, doc["DeviceId"] | "", sizeof(deviceId));

  return true;
}

void setupI2S() {

  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 6,
    .dma_buf_len = 256,
    .use_apll = false
  };

  i2s_pin_config_t pins = {
    .bck_io_num = I2S_BCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_DATA
  };

  i2s_driver_install(I2S_PORT, &cfg, 0, NULL);
  i2s_set_pin(I2S_PORT, &pins);
}

inline float nlms_step(float u, float d) {

  x[idx] = u;

  float y = 0;
  float power = 0;
  int j = idx;

  for (int i = 0; i < FILTER_LEN; i++) {
    y += w[i] * x[j];
    power += x[j] * x[j];
    if (--j < 0) j = FILTER_LEN - 1;
  }

  power += 1e-6f;

  float e = d - y;
  float step = MU / power;

  j = idx;
  for (int i = 0; i < FILTER_LEN; i++) {
    w[i] += step * e * x[j];
    if (--j < 0) j = FILTER_LEN - 1;
  }

  idx = (idx + 1) % FILTER_LEN;

  return e;
}

void pushFrame(int16_t *frame) {

  portENTER_CRITICAL(&mux);

  if (count >= QUEUE_SIZE) {
    queueDrops++;
    portEXIT_CRITICAL(&mux);
    return;
  }

  memcpy(audioQueue[writeIdx], frame, FRAME_SIZE * sizeof(int16_t));

  writeIdx = (writeIdx + 1) % QUEUE_SIZE;
  count++;

  portEXIT_CRITICAL(&mux);
}

bool popFrame(int16_t *frame) {
  portENTER_CRITICAL(&mux);

  if (count == 0) {
    queueUnderruns++;
    portEXIT_CRITICAL(&mux);
    return false;
  }

  memcpy(frame, audioQueue[readIdx], FRAME_SIZE * sizeof(int16_t));

  readIdx = (readIdx + 1) % QUEUE_SIZE;
  count--;

  portEXIT_CRITICAL(&mux);
  return true;
}

void audioProcesTask(void *pv) {
  while (true) {
    if (sdReady) {
      size_t bytesRead = 0;
      i2s_read(I2S_PORT, i2sBuffer, sizeof(i2sBuffer),
              &bytesRead, portMAX_DELAY);

      int samples = bytesRead / sizeof(int16_t);

      for (int i = 0; i < samples; i += 2) {

        float u = i2sBuffer[i] / 32768.0f;
        float d = i2sBuffer[i + 1] / 32768.0f;

        float cleaned = nlms_step(u, d);

        frameOut[framePos++] = (int16_t)cleaned;

        if (framePos >= FRAME_SIZE) {
          pushFrame(frameOut);
          framePos = 0;
        }
      }
    } else { 
      vTaskDelay(10);
    }
    vTaskDelay(1);
  }
}

void openNewLogFile() {

  char fileName[64];
  snprintf(fileName, sizeof(fileName),
           "/missed_transmissions/audio_%d.raw",
           fileIdx);

  fileIdx++;

  audioFile = SD.open(fileName, FILE_WRITE);
  sdLogging = true;

  sdFrameCounter = 0;
}

void sendTask(void *pv) {
  while (true) {
    if (sdReady) {
      int16_t frame[FRAME_SIZE];

      if (!popFrame(frame)) {
        vTaskDelay(1);
        continue;
      }

      memcpy(&chunkBuffer[chunkPos * FRAME_SIZE], frame, FRAME_SIZE * sizeof(int16_t));
      chunkPos++;

      if (chunkPos >= CHUNK_FRAMES) {
        if (wifiLoaded) {
          if (!client.connected()) {
            client.stop();
            client.connect(targetIP, targetPort);
          }
          tcpPackage header;
          strncpy(header.deviceId, deviceId, sizeof(header.deviceId));
          header.timestamp = millis();
          header.length = FRAME_SIZE * CHUNK_FRAMES * sizeof(int16_t);

          client.write((uint8_t*)&header, sizeof(header));
          client.write((uint8_t*)chunkBuffer, header.length);
        } else {
          if (!sdLogging) {
            openNewLogFile();
          }

          audioFile.write((uint8_t*)chunkBuffer, sizeof(chunkBuffer));

          sdFrameCounter++;

          if (sdFrameCounter >= 200) {
            audioFile.flush();
            audioFile.close();
            sdLogging = false;
          }
        }
        chunkPos = 0;
      } else {
        vTaskDelay(10);
      }
    }
  vTaskDelay(1);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(CS, OUTPUT);
  SPI.begin(CLK, MISO, MOSI, CS);
  setupI2S();

  xTaskCreatePinnedToCore(audioProcesTask, "audio", 8192, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(sendTask, "send", 8192, NULL, 1, NULL, 1);
}

void loop() {
  if (sdReady && !SD.exists("/")) {
    Serial.println("SD card removed!");
    sdReady = false;
    wifiLoaded = false;
  }

  if (!sdReady && sdLogging) {
    audioFile.close();
    sdLogging = false;
  }

  if (!sdReady && (millis() - lastCheck >= 2000)) {
    lastCheck = millis();

    if (InitSD()) {
      Serial.println("SD card connected!");
      sdReady = true;
    } else {
      Serial.println("Waiting for SD card...");
    }
  }

  if (sdReady && !wifiLoaded) {
    getNetworkInfo();

    WiFi.begin(ssid, password);

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
      delay(200);
      Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi failed");
      wifiLoaded = false;

    } else {
      Serial.println("Wifi connected");
      client.connect(targetIP, 50005);
      wifiLoaded = true;
    }
  }

  if (wifiLoaded && WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi lost, reconnecting...");
    wifiLoaded = false;
    client.stop();
    WiFi.reconnect();
  }

  static uint32_t lastPrint = 0;

  if (sdReady && millis() - lastPrint > 1000) {
    lastPrint = millis();

    Serial.print("Queue fill: ");
    Serial.print(count);

    Serial.print(" | drops: ");
    Serial.print(queueDrops);

    Serial.print(" | underruns: ");
    Serial.println(queueUnderruns);
  }
}