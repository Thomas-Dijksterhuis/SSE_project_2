#include "driver/i2s.h"
#include <WiFi.h>
#include <SD.h>
#include <ArduinoJson.h>
#include <SPI.h>

//SD card parameters
int32_t fileIdx = 0;
volatile bool sdReady = false;
uint32_t sdLastCheck = 0;
const char* idxFileName = "/missed_transmissions/file_index";
uint32_t sleepMs = 0;
uint32_t cycleStart = 0;
bool cycleStarted = false;
bool soundFileOpen = false;
bool tcpConnected = false;

QueueHandle_t tcpQueue;
SemaphoreHandle_t sdMutex;

#define TARGET_PORT 50000

#define CS 5
#define MOSI 23
#define MISO 19
#define CLK 18

//Audio parameter
#define I2S_BCK  26
#define I2S_WS   25
#define I2S_DATA 22
#define I2S_PORT I2S_NUM_0

#define FRAME_FORMAT int16_t
#define MONO_SIZE 256
#define QUEUE_SIZE 64
#define FRAME_SIZE (MONO_SIZE * 2)
#define SAMPLE_RATE 20000
#define SD_SECONDS_PER_FILE 20

struct __attribute__((packed)) AudioPacket {
    char     deviceId[16];
    uint16_t length;
    FRAME_FORMAT samples[FRAME_SIZE]; 
};

AudioPacket payload;

volatile uint32_t queueDrops = 0;

//WiFi/UDP parameters
WiFiClient client;
IPAddress serverIp;
char ssid[64];
char password[64];
char targetIP[64];
char deviceId[16];

//Scheduling parametes
char schedulingMode[64];
uint32_t durationMs = 0;
uint32_t periodMs = 0;

//I2S parameters
int32_t i2sBuffer[FRAME_SIZE];
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

//Deep sleep parameters
#define WAKE_GPIO GPIO_NUM_0

bool InitSD() {

  bool ok = false;
  if (SD.begin(CS)) {
    File f = SD.open("/");
    if (f) { f.close(); ok = true; }
  }

  return ok;
}

bool getScheduleInfo() {
  String data;

  File file = SD.open("/Schedule/Schedule.json");
  if (!file) {
    Serial.println("could not find schedule file");
    return false;
  }
  data = file.readString();
  file.close();

  JsonDocument doc;
  if (deserializeJson(doc, data)) return false;

  strlcpy(schedulingMode, doc["mode"] | "", sizeof(schedulingMode));
  if (strcmp(schedulingMode, "sampling") == 0) {
    JsonArray schedule = doc["schedule"].as<JsonArray>();
    if (schedule.isNull() || schedule.size() == 0) {
      strlcpy(schedulingMode, "constant", sizeof(schedulingMode));
    } else {
      durationMs = ((uint32_t)schedule[0]["durationMinutes"]) * 60000UL;
      periodMs   = ((uint32_t)schedule[0]["periodMinutes"]) * 60000UL;
      if (durationMs == 0 || periodMs == 0) {
        strlcpy(schedulingMode, "constant", sizeof(schedulingMode));
      }
      else if (durationMs >= periodMs) {
        strlcpy(schedulingMode, "constant", sizeof(schedulingMode));
      } else {
        sleepMs = (periodMs - durationMs);
      }
    }
  }
  return true;
}

bool getNetworkInfo() {
  String data;

  File file = SD.open("/Network/Network.json");
  if (!file) {
    Serial.println("could not find network file");
    return false;
  }
  data = file.readString();
  file.close();

  JsonDocument doc;
  if (deserializeJson(doc, data)) return false;

  strlcpy(ssid,     doc["SSID"]               | "", sizeof(ssid));
  strlcpy(password, doc["Password"]           | "", sizeof(password));
  strlcpy(targetIP, doc["ReceiverIpAddress"]  | "", sizeof(targetIP));
  strlcpy(deviceId, doc["DeviceId"]           | "", sizeof(deviceId));
  serverIp.fromString(targetIP);

  return true;

}


float phase = 0.0f;
float modPhase = 0.0f;

void generateWail(FRAME_FORMAT* buffer, int samples)
{
    for (int i = 0; i < samples; i += 2)
    {
        float lfo = 0.5f + 0.5f * sinf(modPhase);

        float freq = 300.0f + lfo * 1200.0f;

        float amp = 0.3f + 0.7f * lfo;

        float val = sinf(phase) * amp;

        phase += 2.0f * M_PI * freq / SAMPLE_RATE;
        modPhase += 2.0f * M_PI * 0.5f / SAMPLE_RATE;

        if (phase > 2.0f * M_PI) phase -= 2.0f * M_PI;
        if (modPhase > 2.0f * M_PI) modPhase -= 2.0f * M_PI;

        buffer[i] = (FRAME_FORMAT)(val * 32767.0f);
        buffer[i + 1] = buffer[i];
    }
}

void audioProcessTask(void *pv)
{
    static FRAME_FORMAT local32[FRAME_SIZE];

    const TickType_t periodTicks = pdMS_TO_TICKS(1000.0f * FRAME_SIZE / SAMPLE_RATE);

    TickType_t wakeTime = xTaskGetTickCount();

    while (true) {

        if (!sdReady) {
            vTaskDelay(pdMS_TO_TICKS(100));
            wakeTime = xTaskGetTickCount();
            continue;
        }

        generateWail(local32, FRAME_SIZE);

        if (!xQueueSend(tcpQueue, local32, 0)) {
          queueDrops++;
        }

        vTaskDelayUntil(&wakeTime, periodTicks);
    }
}
void connectTCP()
{
    if (tcpConnected && client.connected()) return;

    client.stop();
    if (client.connect(serverIp, TARGET_PORT))
    {
        tcpConnected = true;
        Serial.println("TCP connected");
    }
    else
    {
        tcpConnected = false;
        Serial.println("TCP connect failed");
    }
}

void sinkTCP(FRAME_FORMAT* frame) {
    payload.length = FRAME_SIZE * sizeof(FRAME_FORMAT);
    memset(payload.deviceId, 0, sizeof(payload.deviceId));
    strncpy(payload.deviceId, deviceId, sizeof(payload.deviceId) - 1);
    memcpy(payload.samples, frame, sizeof(payload.samples));
    client.write((uint8_t*)&payload, sizeof(payload));
}

void sinkSD(FRAME_FORMAT* frame) {
  static uint32_t frameCounter = 0;
  static File soundFile;
  static File idxFile;
  if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(10)) != pdTRUE) return;
  if (!soundFileOpen) {
      char fileName[64];
      idxFile = SD.open(idxFileName, FILE_READ);
      if (idxFile) {
        if (idxFile.size() == sizeof(uint32_t)) {
          idxFile.read((uint8_t*)&fileIdx, sizeof(uint32_t));
        }
        idxFile.close();
      } else {
        fileIdx = 0;
      }

      snprintf(fileName, sizeof(fileName),
                "/missed_transmissions/audio_%u.raw", fileIdx++); 
      soundFile = SD.open(fileName, FILE_WRITE);
      soundFileOpen = soundFile;
      if (!soundFileOpen) {
          Serial.println("SD: failed to open file");
          xSemaphoreGive(sdMutex);
          return;
      }

      frameCounter = 0;
      Serial.printf("SD: writing to %s\n", fileName);
    }

    soundFile.write((uint8_t*)frame, FRAME_SIZE * sizeof(FRAME_FORMAT));
    frameCounter++;

    if (frameCounter >= (SD_SECONDS_PER_FILE * SAMPLE_RATE * 2) / FRAME_SIZE) {
      soundFile.flush();
      soundFile.close();
      soundFileOpen = false;
      idxFile = SD.open(idxFileName, FILE_WRITE);
      idxFile.seek(0);
      idxFile.write((uint8_t*)&fileIdx, sizeof(uint32_t));
      idxFile.flush();
      idxFile.close();
      frameCounter = 0;
      Serial.println("SD: file rotated");
    }
    xSemaphoreGive(sdMutex);
}


void sendTask(void *pv)
{
  static FRAME_FORMAT frame[FRAME_SIZE];
  while (true)
  {
      if (!sdReady) {
        vTaskDelay(1);
        continue;
      }

      if (xQueueReceive(tcpQueue, frame, portMAX_DELAY) == pdTRUE)
      {
        if (client.connected()) {
          sinkTCP(frame);
        } else { 
          sinkSD(frame);
        }
      }
    vTaskDelay(1);
  }
}

void setup()
{
    Serial.begin(115200);
    pinMode(CS, OUTPUT);
    SPI.begin(CLK, MISO, MOSI, CS);
    i2s_driver_install(I2S_PORT, &cfg, 0, NULL);
    i2s_set_pin(I2S_PORT, &pins);
    pinMode(WAKE_GPIO, INPUT_PULLUP);

    cycleStart = 0;
    sdReady = false;
    sdLastCheck = 0;
    tcpQueue = xQueueCreate(QUEUE_SIZE, sizeof(FRAME_FORMAT) * FRAME_SIZE);

    sdMutex = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(
        audioProcessTask,
        "audio",
        12288 ,
        NULL,
        2,
        NULL,
        0
    );

    xTaskCreatePinnedToCore(
        sendTask,
        "send",
        12288,
        NULL,
        3,
        NULL,
        1
    );
}

void loop() {
  if (sdReady) {
      if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          if (!SD.exists("/")) {
              sdReady = false;
              xSemaphoreGive(sdMutex);
              if (WiFi.status() == WL_CONNECTED) WiFi.disconnect(false, false);
          } else {
              xSemaphoreGive(sdMutex);
          }
      }
  }

  if (!sdReady && (millis() - sdLastCheck >= 2000)) {
    sdLastCheck = millis();
    if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (InitSD()) {
            sdReady = true;
            getNetworkInfo();
            getScheduleInfo();
            if (!SD.exists("/missed_transmissions")) SD.mkdir("/missed_transmissions");
        } else {
            Serial.println("Waiting for SD card...");
        }
        xSemaphoreGive(sdMutex);
    }
  }

  if (sdReady) {
    if (WiFi.status() != WL_CONNECTED) {

        static uint32_t lastAttempt = 0;

        if (millis() - lastAttempt > 10000) {

            lastAttempt = millis();

            Serial.printf("Connecting WiFi to %s\n", ssid);

            WiFi.begin(ssid, password);
        }
    }

    if (WiFi.isConnected() && !client.connected())
    {
      tcpConnected = false;
      connectTCP();
    }

    // --- Sampling schedule ---
    if (strcmp(schedulingMode, "sampling") == 0) {
      uint32_t now = millis();

      if (!cycleStarted) {
        cycleStart = now;  // first run init
        cycleStarted = true;
      }

      uint32_t elapsed = now - cycleStart;

      if (elapsed >= durationMs) {
          if (client.connected()) {
            client.flush();
            client.stop();
            tcpConnected = false;
          }
          if (WiFi.status() == WL_CONNECTED) { 
            WiFi.disconnect(true); 
          }
          WiFi.mode(WIFI_OFF);
          esp_sleep_enable_timer_wakeup((uint64_t)sleepMs * 1000ULL);

        esp_sleep_enable_ext0_wakeup(WAKE_GPIO, 0);
        Serial.flush();
        esp_deep_sleep_start();
      }
    }

    // --- Status print ---
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint > 1000) {
      lastPrint = millis();
      Serial.printf("Mode: %s | WiFi %s | Queue: %d | Drops: %d, RSSI %d\n",
                    schedulingMode,
                    WiFi.isConnected() ? "Connected" : "Disconnected",
                    uxQueueMessagesWaiting(tcpQueue),
                    queueDrops,
                    WiFi.RSSI()
      );
    }
  }
}