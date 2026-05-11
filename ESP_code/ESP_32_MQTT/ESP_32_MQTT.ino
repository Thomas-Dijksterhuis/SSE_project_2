#include "driver/i2s.h"
#include <WiFi.h>
#include <SD.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <PubSubClient.h>

//SD card parameters
uint16_t fileIdx = 0;
File audioFile;
volatile bool sdLogging = false;
uint32_t sdFrameCounter = 0;
volatile bool sdReady = false;
unsigned long sdLastCheck = 0;
#define SD_FRAMES_PER_FILE 1000

#define CS 5
#define MOSI 23
#define MISO 19
#define CLK 18

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
SemaphoreHandle_t sdFileMutex;

//Audio parameter
#define I2S_BCK  26
#define I2S_WS   25
#define I2S_DATA 22
#define I2S_PORT I2S_NUM_0

#define FRAME_SIZE 512
#define QUEUE_SIZE 20
#define SAMPLE_RATE 10000

int32_t* audioQueue[QUEUE_SIZE];
int32_t framePool[QUEUE_SIZE][FRAME_SIZE];
bool frameUsed[QUEUE_SIZE] = {0};
volatile uint16_t writeIdx = 0;
volatile uint16_t readIdx = 0;
volatile uint16_t count = 0;

struct __attribute__((packed)) AudioPacket {
  char deviceId[16];
  uint16_t length;
  int32_t samples[FRAME_SIZE];
};

int32_t frameOut[FRAME_SIZE];
uint16_t framePos = 0;
volatile uint32_t queueDrops = 0;
volatile uint32_t queueUnderruns = 0;

//WiFi/MQTT parameters
WiFiClient wifiClient;
PubSubClient client(wifiClient);
char ssid[64];
char password[64];
char targetIP[64];
char deviceId[16];
volatile bool wifiLoaded = false;

//I2S parameters
int32_t i2sBuffer[FRAME_SIZE * 2];
i2s_config_t cfg = {
  .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
  .sample_rate = SAMPLE_RATE,
  .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
  .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
  .communication_format = I2S_COMM_FORMAT_I2S,
  .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
  .dma_buf_count = 6,
  .dma_buf_len = 256,
  .use_apll = true
};

i2s_pin_config_t pins = {
  .bck_io_num = I2S_BCK,
  .ws_io_num = I2S_WS,
  .data_out_num = -1,
  .data_in_num = I2S_DATA
};

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

  strlcpy(targetIP, doc["ReceiverIpAddress"] | "", sizeof(targetIP));

  strlcpy(deviceId, doc["DeviceId"] | "", sizeof(deviceId));

  return true;
}

int32_t* getFreeFrame() {
    portENTER_CRITICAL(&mux);
    for (int i = 0; i < QUEUE_SIZE; i++) {
        if (!frameUsed[i]) {
            frameUsed[i] = true;
            portEXIT_CRITICAL(&mux);
            return framePool[i];
        }
    }
    portEXIT_CRITICAL(&mux);
    return NULL;
}

void releaseFrame(int32_t* ptr) {
  portENTER_CRITICAL(&mux);
  for (int i = 0; i < QUEUE_SIZE; i++) {
    if (framePool[i] == ptr) {
      frameUsed[i] = false;
      break;
    }
  }
  portEXIT_CRITICAL(&mux);
}

void pushFrame(int32_t *frame) {
  portENTER_CRITICAL(&mux);

  if (count >= QUEUE_SIZE) {
    int32_t* dropped = audioQueue[readIdx];
    readIdx = (readIdx + 1) % QUEUE_SIZE;
    count--;
    queueDrops++;

    for (int i = 0; i < QUEUE_SIZE; i++) {
      if (framePool[i] == dropped) {
        frameUsed[i] = false;
        break;
      }
    }
  }

  audioQueue[writeIdx] = frame;
  writeIdx = (writeIdx + 1) % QUEUE_SIZE;
  count++;

  portEXIT_CRITICAL(&mux);
}

bool popFrame(int32_t **frame) {
  portENTER_CRITICAL(&mux);

  if (count == 0) {
    queueUnderruns++;
    portEXIT_CRITICAL(&mux);
    return false;
  }

  *frame = audioQueue[readIdx];

  readIdx = (readIdx + 1) % QUEUE_SIZE;
  count--;

  portEXIT_CRITICAL(&mux);
  return true;
}

float simPhase = 0.0f;

void sim_i2s_read(int32_t* buffer, int samples) {
    for (int i = 0; i < samples; i += 2) {

        int32_t sample = (int32_t)(sinf(simPhase) * 2147483647.0f);

        simPhase += 2.0f * M_PI * 440.0f / SAMPLE_RATE;
        if (simPhase > 2.0f * M_PI) simPhase -= 2.0f * M_PI;

        int32_t noise = (rand() - RAND_MAX / 2) * 2000;

        int32_t left = sample;
        int32_t right = sample + noise;

        buffer[i]     = left;
        buffer[i + 1] = right;
    }
}

void audioProcessTask(void *pv) {
  int32_t* frame = NULL;
  while (!(frame = getFreeFrame())) {
    vTaskDelay(1);
  }
  
  uint16_t pos = 0;

  while (true) {
    if (!sdReady) {
      vTaskDelay(10);
      continue;
    }

    size_t bytesRead = 0;
    // i2s_read(I2S_PORT, i2sBuffer, sizeof(i2sBuffer), &bytesRead, portMAX_DELAY);
    // int samples = (bytesRead / sizeof(int32_t)) & ~1;
    int samples = FRAME_SIZE * 2;
    sim_i2s_read(i2sBuffer, samples);
    vTaskDelay(pdMS_TO_TICKS((FRAME_SIZE * 1000) / SAMPLE_RATE));  // pace it realistically


    for (int i = 0; i < samples; i++) {
      frame[pos++] = i2sBuffer[i];

      if (pos >= FRAME_SIZE) {
        pushFrame(frame);
        frame = NULL;

        // Spin until a frame is available — never proceed with NULL
        while (!(frame = getFreeFrame())) {
          vTaskDelay(1);
        }

        pos = 0;
      }
    }
    vTaskDelay(1);
  }
}

void sendTask(void *pv) {
  AudioPacket payload;
  while (true) {
    if (sdReady) {
      int32_t* frame;

      if (!popFrame(&frame)) {
        vTaskDelay(1);
        continue;
      }

      if (wifiLoaded) {
        if (!client.connected()) {
        Serial.println("MQTT connecting...");
        if (!client.connect(deviceId)) {
          Serial.print("MQTT failed, state=");
          Serial.println(client.state());
          releaseFrame(frame);
          vTaskDelay(pdMS_TO_TICKS(5000));
          continue;
        }
        Serial.println("MQTT connected");
        }
        client.loop();
        strncpy(payload.deviceId, deviceId, sizeof(payload.deviceId) - 1);
        payload.deviceId[15] = '\0';
        payload.length = FRAME_SIZE * sizeof(int32_t);
        memcpy(payload.samples, frame, sizeof(payload.samples));

        bool code = client.publish(
            "Readings",
            (uint8_t*)&payload,
            sizeof(payload)
        );
        if (code == false) {
          Serial.println("Failed to send package");
        }
      } else {
        if (xSemaphoreTake(sdFileMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          if (!sdLogging) {
              char fileName[64];
              snprintf(fileName, sizeof(fileName),
                      "/missed_transmissions/audio_%d.raw",
                      fileIdx);

              fileIdx++;

              SD.mkdir("/missed_transmissions");
              audioFile = SD.open(fileName, FILE_WRITE);
              if (!audioFile) {
                  Serial.println("Failed to open SD file");
                  sdLogging = false;
              } else {
                  sdLogging = true;
                  sdFrameCounter = 0;
              }
          }    

          if (sdLogging) { 
            audioFile.write((uint8_t*)frame, FRAME_SIZE * sizeof(int32_t));

            sdFrameCounter++;

            if (sdFrameCounter >= SD_FRAMES_PER_FILE) {
              audioFile.flush();
              audioFile.close();
              sdLogging = false;
            }
          }
          xSemaphoreGive(sdFileMutex);
        }
      }
      releaseFrame(frame);
    } else {
      vTaskDelay(10);
    }
  vTaskDelay(1);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(CS, OUTPUT);
  SPI.begin(CLK, MISO, MOSI, CS);
  i2s_driver_install(I2S_PORT, &cfg, 0, NULL);
  i2s_set_pin(I2S_PORT, &pins);
  sdFileMutex = xSemaphoreCreateMutex();
  client.setBufferSize(4096);
  xTaskCreatePinnedToCore(audioProcessTask, "audio", 16384, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(sendTask, "send", 16384, NULL, 2, NULL, 0);
}

void loop() {
  if (sdReady && !SD.exists("/")) {
    Serial.println("SD card removed!");
    sdReady = false;
    if (xSemaphoreTake(sdFileMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      if (sdLogging) {
        audioFile.close();
        sdLogging = false;
      }
      xSemaphoreGive(sdFileMutex);
    }
    if (wifiLoaded) {
      WiFi.disconnect();
      wifiLoaded = false;
    }
  }

  if (!sdReady && (millis() - sdLastCheck >= 2000)) {
    sdLastCheck = millis();

    if (InitSD()) {
      Serial.println("SD card connected!");
      sdReady = true;
    } else {
      Serial.println("Waiting for SD card...");
    }
  }

  if (sdReady && (!wifiLoaded || WiFi.status() != WL_CONNECTED)) {
    getNetworkInfo();
    client.setServer(targetIP, 1883);
    WiFi.begin(ssid, password);

    unsigned long start = millis();
    Serial.print("WiFi connecting .");
    while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
      delay(200);
      Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("Wifi connected");
      wifiLoaded = true;
    } else {
      Serial.println("WiFi failed");
      wifiLoaded = false;
    }
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