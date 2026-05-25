#include "driver/i2s.h"
#include <WiFi.h>
#include <SD.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <PubSubClient.h>

//SD card parameters
uint16_t fileIdx = 0;
volatile bool sdReady = false;
unsigned long sdLastCheck = 0;
volatile bool systemActive = true;

#define SD_FRAMES_PER_FILE 1000

#define CS 5
#define MOSI 23
#define MISO 19
#define CLK 18

SemaphoreHandle_t sdFileMutex;

//Audio parameter
#define I2S_BCK  26
#define I2S_WS   25
#define I2S_DATA 22
#define I2S_PORT I2S_NUM_0

#define FRAME_SIZE 512
#define QUEUE_SIZE 32
#define SAMPLE_RATE 10000

struct __attribute__((packed)) AudioPacket {
  char deviceId[16];
  uint16_t length;
  int32_t samples[FRAME_SIZE];
};

volatile uint32_t queueDrops = 0;

static int32_t ring[QUEUE_SIZE][FRAME_SIZE];

static volatile uint16_t writeIdx = 0;
static volatile uint16_t readIdx = 0;
static volatile uint16_t count = 0;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

//WiFi/MQTT parameters
WiFiClient wifiClient;
PubSubClient client(wifiClient);
char ssid[64];
char password[64];
char targetIP[64];
char deviceId[16];
volatile bool wifiLoaded = false;

//Scheduling parametes
char schedulingMode[64];
uint16_t durationMinutes = 0;
uint16_t periodMinutes = 0;

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
  if (!SD.begin(CS)) {
    return false;
  }
  File f = SD.open("/");
  if (!f) return false;
  f.close();
  return true;
}

bool getScheduleInfo() {
  File file = SD.open("/Schedule/Schedule.json");
  if (!file) {
    Serial.println("could not find schedule file");
    return false;
  }

  String data = file.readString();
  file.close();

  JsonDocument doc;
  if (deserializeJson(doc, data)) return false;

  strlcpy(schedulingMode, doc["mode"] | "", sizeof(schedulingMode));
  if (strcmp(schedulingMode, "sampling") == 0) {
    JsonArray schedule = doc["schedule"].as<JsonArray>();
    if (schedule.isNull() || schedule.size() == 0) {
      strlcpy(schedulingMode, "constant", sizeof(schedulingMode));
    } else {
        durationMinutes = schedule[0]["durationMinutes"];
        periodMinutes   = schedule[0]["periodMinutes"];
      if (durationMinutes == 0 || periodMinutes == 0) {
        strlcpy(schedulingMode, "constant", sizeof(schedulingMode));
      }
    }
  }

  Serial.printf("Mode: %s\n", schedulingMode);
  Serial.printf("durationMinutes: %d\n", durationMinutes);
  Serial.printf("periodMinutes: %d\n", periodMinutes);

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

void pushFrame(int32_t *frame)
{
    int idx;

    portENTER_CRITICAL(&mux);
    if (count == QUEUE_SIZE) {
        readIdx = (readIdx + 1) % QUEUE_SIZE;
        count--;
        queueDrops++;
    }

    idx = writeIdx;
    writeIdx = (writeIdx + 1) % QUEUE_SIZE;
    count++;

    portEXIT_CRITICAL(&mux);

    memcpy(ring[idx], frame, FRAME_SIZE * sizeof(int32_t));
}

bool popFrame(int32_t *dst)
{
    int idx;

    portENTER_CRITICAL(&mux);

    if (count == 0) {
        portEXIT_CRITICAL(&mux);
        return false;
    }

    idx = readIdx;
    readIdx = (readIdx + 1) % QUEUE_SIZE;
    count--;

    portEXIT_CRITICAL(&mux);

    memcpy(dst, ring[idx], FRAME_SIZE * sizeof(int32_t));
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

void audioProcessTask(void *pv)
{
    int32_t local[FRAME_SIZE * 2];
    int32_t frame[FRAME_SIZE];
    uint16_t pos = 0;

    while (true) {

        if (!sdReady || !systemActive) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        sim_i2s_read(local, FRAME_SIZE * 2);
        vTaskDelay(pdMS_TO_TICKS((FRAME_SIZE * 1000) / SAMPLE_RATE));

        for (int i = 0; i < FRAME_SIZE * 2; i ++) {

            frame[pos++] = local[i];

            if (pos >= FRAME_SIZE) {
                pushFrame(frame);
                pos = 0;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

void sendTask(void *pv)
{
    int32_t frame[FRAME_SIZE];
    AudioPacket payload;

    while (true) {

        if (!sdReady || !systemActive) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (!popFrame(frame)) {
            vTaskDelay(1);
            continue;
        }

        if (WiFi.status() != WL_CONNECTED) {
            vTaskDelay(10);
            continue;
        }

        if (!client.connected()) {
            if (!client.connect(deviceId)) {
                vTaskDelay(2000);
                continue;
            }
        }

        client.loop();
        vTaskDelay(pdMS_TO_TICKS(5));

        memset(payload.deviceId, 0, sizeof(payload.deviceId));
        strncpy(payload.deviceId, deviceId, sizeof(payload.deviceId) - 1);
        
        payload.length = FRAME_SIZE * sizeof(int32_t);
        memcpy(payload.samples, frame, sizeof(payload.samples));

        client.publish("Readings", (uint8_t*)&payload, sizeof(payload));

        vTaskDelay(1);
    }
}

void sdTask(void *pv)
{
    int32_t frame[FRAME_SIZE];
    File    localFile;
    uint32_t frameCounter = 0;

    while (true) {

        if (!sdReady || client.connected() || !systemActive) {
            if (localFile) {
                localFile.flush();
                localFile.close();
                frameCounter = 0;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (!popFrame(frame)) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        if (!localFile) {
            char fileName[64];
            if (xSemaphoreTake(sdFileMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
                continue; 
            }

            SD.mkdir("/missed_transmissions");
            snprintf(fileName, sizeof(fileName),
                     "/missed_transmissions/audio_%u.raw", fileIdx++);
            localFile = SD.open(fileName, FILE_WRITE);

            xSemaphoreGive(sdFileMutex);

            if (!localFile) {
                Serial.println("SD: failed to open file");
                vTaskDelay(pdMS_TO_TICKS(500));
                continue;
            }

            frameCounter = 0;
            Serial.printf("SD: writing to %s\n", fileName);
        }

        localFile.write((uint8_t*)frame, FRAME_SIZE * sizeof(int32_t));
        frameCounter++;

        if (frameCounter >= SD_FRAMES_PER_FILE) {
            localFile.flush();
            localFile.close();
            frameCounter = 0;
            Serial.println("SD: file rotated");
        }

        vTaskDelay(pdMS_TO_TICKS(2));
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
  xTaskCreatePinnedToCore(audioProcessTask, "audio", 12288, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(sendTask, "send", 8192, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(sdTask , "sd", 8192, NULL, 1, NULL, 1);
}

void loop() {
  // --- SD card hotplug ---
  if (sdReady && (!SD.exists("/"))) {
      Serial.println("SD card removed!");
      sdReady = false;
      if (wifiLoaded) { 
        WiFi.disconnect(); 
        wifiLoaded = false; }
  }

  if (!sdReady && (millis() - sdLastCheck >= 2000)) {
    sdLastCheck = millis();
    if (InitSD()) {
      Serial.println("SD card connected!");
      sdReady = true;
      getNetworkInfo();
      getScheduleInfo();
    } else {
      Serial.println("Waiting for SD card...");
    }
  }
  if (sdReady) {
    // --- WiFi ---
    if (systemActive && (!wifiLoaded || WiFi.status() != WL_CONNECTED)) {
      client.setServer(targetIP, 1883);
      WiFi.begin(ssid, password);
      unsigned long start = millis();
      Serial.print("WiFi connecting .");
      while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
        delay(200); Serial.print(".");
      }
      Serial.println();
      wifiLoaded = (WiFi.status() == WL_CONNECTED);
      Serial.println(wifiLoaded ? "Wifi connected" : "WiFi failed");
    }

    // --- Sampling schedule ---
    if (strcmp(schedulingMode, "sampling") == 0) {
      static uint32_t cycleStart = 0;
      uint32_t now = millis();
      uint32_t periodMs   = (uint32_t)periodMinutes   * 60000UL;
      uint32_t durationMs = (uint32_t)durationMinutes * 60000UL;

      if (cycleStart == 0) cycleStart = now;  // first run init

      uint32_t elapsed = now - cycleStart;

      if (elapsed < durationMs) {
        // Within the active sampling window
        if (!systemActive) {
          Serial.println("Schedule: waking up");
          systemActive = true;
        }
      } else if (elapsed < periodMs) {
        // Past the sampling window, within the sleep portion
        if (systemActive) {
          Serial.println("Schedule: going to sleep");
          systemActive = false;
          if (wifiLoaded) { WiFi.disconnect(); wifiLoaded = false; }
        }
      } else {
        // Full period elapsed, start new cycle
        cycleStart = now;
        systemActive = true;
        Serial.println("Schedule: new cycle");
      }
    }

    // --- Status print ---
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint > 1000) {
      lastPrint = millis();
      Serial.printf("Mode: %s | Active: %s | Queue: %d | Drops: %d\n",
                    schedulingMode,
                    systemActive ? "yes" : "sleeping",
                    count, queueDrops);
    }
  }
}