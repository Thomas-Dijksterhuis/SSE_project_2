#include "driver/i2s.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <SD.h>
#include <ArduinoJson.h>
#include <SPI.h>

//SD card parameters
int32_t fileIdx = 0;
volatile bool sdReady = false;
unsigned long sdLastCheck = 0;

#define SD_FRAMES_PER_FILE 10000
#define TARGET_PORT 50000

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

#define FRAME_SIZE 64
#define QUEUE_SIZE 128
#define SAMPLE_RATE 20000

struct __attribute__((packed)) AudioPacket {
    char     deviceId[16];
    uint16_t length;
    int32_t  samples[FRAME_SIZE]; 
};

volatile uint32_t queueDrops = 0;
static int32_t ring[QUEUE_SIZE][FRAME_SIZE];

static volatile uint16_t writeIdx = 0;
static volatile uint16_t readIdx = 0;
static volatile uint16_t count = 0;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

//WiFi/UDP parameters
WiFiClient client;
IPAddress serverIp;
char ssid[64];
char password[64];
char targetIP[64];
char deviceId[16];

//Scheduling parametes
char schedulingMode[64];
uint16_t durationMinutes = 0;
uint16_t periodMinutes = 0;

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
  if (!xSemaphoreTake(sdFileMutex, pdMS_TO_TICKS(100))) return false;

  bool ok = false;
  if (SD.begin(CS)) {
    File f = SD.open("/");
    if (f) { f.close(); ok = true; }
  }

  xSemaphoreGive(sdFileMutex);
  return ok;
}

bool getScheduleInfo() {
  String data;

  if (!xSemaphoreTake(sdFileMutex, pdMS_TO_TICKS(100))) return false;

  File file = SD.open("/Schedule/Schedule.json");
  if (!file) {
    Serial.println("could not find schedule file");
    xSemaphoreGive(sdFileMutex);
    return false;
  }
  data = file.readString();
  file.close();
  xSemaphoreGive(sdFileMutex);

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
  String data;

  if (!xSemaphoreTake(sdFileMutex, pdMS_TO_TICKS(100))) return false;

  File file = SD.open("/Network/Network.json");
  if (!file) {
    Serial.println("could not find network file");
    xSemaphoreGive(sdFileMutex);
    return false;
  }
  data = file.readString();
  file.close();
  xSemaphoreGive(sdFileMutex);

  JsonDocument doc;
  if (deserializeJson(doc, data)) return false;

  strlcpy(ssid,     doc["SSID"]               | "", sizeof(ssid));
  strlcpy(password, doc["Password"]           | "", sizeof(password));
  strlcpy(targetIP, doc["ReceiverIpAddress"]  | "", sizeof(targetIP));
  strlcpy(deviceId, doc["DeviceId"]           | "", sizeof(deviceId));
  serverIp.fromString(targetIP);

  return true;
}

void pushFrame(int32_t *frame) {
  portENTER_CRITICAL(&mux);
  if (count == QUEUE_SIZE) {
    readIdx = (readIdx + 1) % QUEUE_SIZE;
    count--;
    queueDrops++;
  }
  int idx = writeIdx;
  writeIdx = (writeIdx + 1) % QUEUE_SIZE;
  count++;
  portEXIT_CRITICAL(&mux);

  memcpy(ring[idx], frame, (FRAME_SIZE) * sizeof(int32_t));
}

bool popFrame(int32_t *dst) {
  portENTER_CRITICAL(&mux);
  if (count == 0) { portEXIT_CRITICAL(&mux); return false; }
  int idx = readIdx;
  readIdx = (readIdx + 1) % QUEUE_SIZE;
  count--;
  portEXIT_CRITICAL(&mux);

  memcpy(dst, ring[idx], (FRAME_SIZE) * sizeof(int32_t));
  return true;
}

float phase = 0.0f;
float modPhase = 0.0f;

void generateWail(int32_t* buffer, int samples)
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

        buffer[i] = (int32_t)(val * 60000000.0f);
        buffer[i + 1] = buffer[i];
    }
}

void audioProcessTask(void *pv)
{
    int32_t local32[FRAME_SIZE];

    const TickType_t periodTicks =
    // FRAME_SIZE is total int32 values, but generateWail() fills stereo pairs.
    // So one buffer contains FRAME_SIZE / 2 samples per channel.
    pdMS_TO_TICKS(1000.0f * FRAME_SIZE / SAMPLE_RATE);

    TickType_t wakeTime = xTaskGetTickCount();

    while (true) {

        if (!sdReady) {
            vTaskDelay(pdMS_TO_TICKS(100));
            wakeTime = xTaskGetTickCount();
            continue;
        }

        generateWail(local32, FRAME_SIZE);

        pushFrame(local32);

        vTaskDelayUntil(&wakeTime, periodTicks);
    }
}
bool tcpConnected = false;
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

void sendTask(void *pv)
{
  int32_t frame[FRAME_SIZE];
  File    localFile;
  uint32_t frameCounter = 0;

  AudioPacket payload;
  payload.length = FRAME_SIZE * sizeof(int32_t);

  while (true)
  {
      if (!sdReady)
      {
          vTaskDelay(pdMS_TO_TICKS(100));
          continue;
      }

      if (!popFrame(frame))
      {
          vTaskDelay(pdMS_TO_TICKS(1));
          continue;
      }

      if (!client.connected())
      {
        memset(payload.deviceId, 0, sizeof(payload.deviceId));
        strncpy(payload.deviceId, deviceId, sizeof(payload.deviceId) - 1);


        memcpy(payload.samples, frame, sizeof(payload.samples));

        size_t written = client.write((uint8_t*)&payload, sizeof(payload));

        if (written != sizeof(payload))
        {
            Serial.println("TCP write failed");
            client.stop();
            tcpConnected = false;
        }
      } else {
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
    }
    vTaskDelay(pdMS_TO_TICKS(2));
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

    sdFileMutex = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(
        audioProcessTask,
        "audio",
        12288,
        NULL,
        3,
        NULL,
        0
    );

    xTaskCreatePinnedToCore(
        sendTask,
        "send",
        8192,
        NULL,
        2,
        NULL,
        1
    );
}

void loop() {
  // --- SD card hotplug ---
  if (xSemaphoreTake(sdFileMutex, pdMS_TO_TICKS(100))) {
    if (sdReady && (!SD.exists("/"))) {
      Serial.println("SD card removed!");
      sdReady = false;
      if (WiFi.status() == WL_CONNECTED) { 
        WiFi.disconnect(false, false); 
      }
    }
    xSemaphoreGive(sdFileMutex);
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
      static uint32_t cycleStart = 0;
      uint32_t now = millis();
      uint32_t periodMs   = (uint32_t)periodMinutes   * 60000UL;
      uint32_t durationMs = (uint32_t)durationMinutes * 60000UL;

      if (cycleStart == 0) cycleStart = now;  // first run init

      uint32_t elapsed = now - cycleStart;

      if (elapsed >= durationMs) {
        uint32_t sleepMs = periodMs - durationMs;
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
      Serial.printf("Mode: %s | WiFi %s | Queue: %d | Drops: %d\n",
                    schedulingMode,
                    WiFi.isConnected() ? "Connected" : "Disconnected",
                    count, queueDrops);
    }
  }
}