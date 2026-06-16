#include <Arduino.h>
#include "driver/i2s.h"
#include <WiFi.h>
#include <SD.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <WiFiUdp.h>
#include <atomic>

#define DEBUG

//Pin define
#define RED_GPIO 4
#define BLUE_GPIO 15
#define GREEN_GPIO 2
#define WAKE_GPIO GPIO_NUM_13

#define SPI_SD_CS 18
#define SPI_SD_MOSI 16
#define SPI_SD_MISO 17
#define SPI_SD_CLK 19

#define I2S_RX_BCK_PIN   26
#define I2S_RX_WS_PIN    25
#define I2S_RX_DATA_PIN  27
#define I2S_RX_PORT   I2S_NUM_1

//Frame info
#define FRAME_FORMAT int16_t
#define SAMPLES_PER_CHANNEL 256
#define UDP_QUEUE_SIZE 32
#define CHANNELS 2
#define FRAME_SAMPLES (SAMPLES_PER_CHANNEL * CHANNELS)
#define FRAME_BYTES (sizeof(FRAME_FORMAT) * FRAME_SAMPLES)
#define SAMPLE_RATE 20000
#define SD_SECONDS_PER_FILE 20

i2s_config_t rx_config = {
  .mode                 = (i2s_mode_t)(I2S_MODE_SLAVE | I2S_MODE_RX),
  .sample_rate          = SAMPLE_RATE,
  .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
  .channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT,
  .communication_format = I2S_COMM_FORMAT_STAND_I2S,
  .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
  .dma_buf_count        = 4,
  .dma_buf_len          = SAMPLES_PER_CHANNEL,
  .use_apll             = false,
  .fixed_mclk           = -1,
};

i2s_pin_config_t rx_pins = {
  .bck_io_num   = I2S_RX_BCK_PIN,
  .ws_io_num    = I2S_RX_WS_PIN,
  .data_out_num = I2S_PIN_NO_CHANGE,
  .data_in_num  = I2S_RX_DATA_PIN
};


#define PORT 50000

struct __attribute__((packed)) WavHeader {
  char riff[4] = { 'R', 'I', 'F', 'F' };
  uint32_t chunkSize;
  char wave[4] = { 'W', 'A', 'V', 'E' };

  char fmt[4] = { 'f', 'm', 't', ' ' };
  uint32_t subchunk1Size = 16;
  uint16_t audioFormat = 1;
  uint16_t numChannels = 2;
  uint32_t sampleRate;
  uint32_t byteRate;
  uint16_t blockAlign;
  uint16_t bitsPerSample;

  char data[4] = { 'd', 'a', 't', 'a' };
  uint32_t subchunk2Size;
};

struct __attribute__((packed)) AudioPacket {
  char deviceId[16];
  char timestamp[24];
  uint16_t length;
  uint32_t sequence = 0;
};

enum deviceMode {
  DEVICEMODE_OFF,
  DEVICEMODE_STARTING,
  DEVICEMODE_NO_SD,
  DEVICEMODE_SD_ERROR,
  DEVICEMODE_SD_FULL,
  DEVICEMODE_SD,
  DEVICEMODE_WIFI,
};

//SD card parameters
volatile uint32_t fileIdx = 0;
AudioPacket payload;
uint32_t sdLastCheck = 0;
uint32_t sleepMs = 0;
volatile uint32_t cycleStart = 0;
volatile bool cycleStarted = false;
File soundFile;
File idxFile;
uint64_t total = 0;

uint32_t setupFinished = 0;

const char* transmissionFolder = "/missed_transmissions";
std::atomic<bool> sdReady = false;
std::atomic<deviceMode> currentDevice = DEVICEMODE_STARTING;
static deviceMode lastMode = DEVICEMODE_STARTING;
std::atomic<bool> systemActive = false;

//WiFi/UDP parameters
WiFiUDP client;
IPAddress serverIp;
char ssid[64];
char password[64];
char targetIP[64];
char deviceId[16];
volatile bool udpStarted = false;

static volatile bool timeSynced = false;
volatile uint32_t lastNtpAttempt = 0;

QueueHandle_t udpQueue;
volatile uint32_t queueDrops = 0;
TaskHandle_t mainTaskHandle = NULL;

//Scheduling parametes
char schedulingMode[64];
uint32_t durationMs = 0;
uint32_t periodMs = 0;

void setDevice(deviceMode mode) {
  switch (mode) {
    case DEVICEMODE_OFF:
      {
        digitalWrite(RED_GPIO, 0);
        digitalWrite(GREEN_GPIO, 0);
        digitalWrite(BLUE_GPIO, 0);
        break;
      }
    case DEVICEMODE_STARTING:
      {
        digitalWrite(RED_GPIO, 1);
        digitalWrite(GREEN_GPIO, 1);
        digitalWrite(BLUE_GPIO, 0);
        break;
      }
    case DEVICEMODE_NO_SD:
      {
        digitalWrite(RED_GPIO, 1);
        digitalWrite(GREEN_GPIO, 0);
        digitalWrite(BLUE_GPIO, 0);
        break;
      }
    case DEVICEMODE_SD_ERROR:
      {
        digitalWrite(RED_GPIO, 1);
        digitalWrite(GREEN_GPIO, 0);
        digitalWrite(BLUE_GPIO, 1);
        break;
      }
    case DEVICEMODE_SD_FULL:
      {
        digitalWrite(RED_GPIO, 1);
        digitalWrite(GREEN_GPIO, 1);
        digitalWrite(BLUE_GPIO, 1);
        break;
      }
    case DEVICEMODE_SD:
      {
        digitalWrite(RED_GPIO, 0);
        digitalWrite(GREEN_GPIO, 0);
        digitalWrite(BLUE_GPIO, 1);
        break;
      }
    case DEVICEMODE_WIFI:
      {
        digitalWrite(RED_GPIO, 0);
        digitalWrite(GREEN_GPIO, 1);
        digitalWrite(BLUE_GPIO, 0);
        break;
      }
  }
}

void wifiConnect() {
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);
  delay(50);

  WiFi.mode(WIFI_MODE_STA);

  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);

  WiFi.begin(ssid, password);
}

bool InitSD() {
    SD.end();
    if (SD.begin(SPI_SD_CS, SPI)) {
        return SD.exists("/.connected");
    }
    return false;
}

bool getScheduleInfo() {
  String data;

  File file = SD.open("/Schedule.json");
  if (!file) {
    return false;
  }
  data = file.readString();
  file.close();

  StaticJsonDocument<2048> doc;
  if (deserializeJson(doc, data)) return false;

  strlcpy(schedulingMode, doc["mode"] | "", sizeof(schedulingMode));
  if (strcmp(schedulingMode, "sampling") == 0) {
    JsonArray schedule = doc["schedule"].as<JsonArray>();
    if (schedule.isNull() || schedule.size() == 0) {
      strlcpy(schedulingMode, "constant", sizeof(schedulingMode));
    } else {
      durationMs = ((uint32_t)schedule[0]["durationMinutes"]) * 60000UL;
      periodMs = ((uint32_t)schedule[0]["periodMinutes"]) * 60000UL;
      if (durationMs == 0 || periodMs == 0) {
        strlcpy(schedulingMode, "constant", sizeof(schedulingMode));
      } else if (durationMs >= periodMs) {
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

  File file = SD.open("/Network.json");
  if (!file) {
    return false;
  }
  data = file.readString();
  file.close();

  StaticJsonDocument<2048> doc;
  if (deserializeJson(doc, data)) return false;

  strlcpy(ssid,     doc["SSID"]               | "", sizeof(ssid));
  strlcpy(password, doc["Password"]           | "", sizeof(password));
  strlcpy(targetIP, doc["ReceiverIpAddress"] | "", sizeof(targetIP));
  strlcpy(deviceId, doc["DeviceId"] | "", sizeof(deviceId));
  serverIp.fromString(targetIP);

  return true;
}

void writeWavHeader(File& file, uint32_t sampleRate, uint16_t bitsPerSample, uint16_t channels, uint32_t dataSize) {
  WavHeader h;

  h.sampleRate = sampleRate;
  h.bitsPerSample = bitsPerSample;
  h.numChannels = channels;

  h.byteRate = sampleRate * channels * bitsPerSample / 8;
  h.blockAlign = channels * bitsPerSample / 8;

  h.subchunk2Size = dataSize;
  h.chunkSize = 36 + dataSize;

  file.seek(0);
  file.write((uint8_t*)&h, sizeof(h));
  file.flush();
}

void enableDeepSleep() {
  systemActive = false;
  setDevice(DEVICEMODE_OFF);
  ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5000));
  if (WiFi.status() == WL_CONNECTED) {
    payload.sequence++;
    payload.length = 0;
    memset(payload.deviceId, 0, sizeof(payload.deviceId));
    strncpy(payload.deviceId, deviceId, sizeof(payload.deviceId) - 1);
    time_t now;
    time(&now);

    struct tm timeinfo;
    localtime_r(&now, &timeinfo);

    snprintf(payload.timestamp, sizeof(payload.timestamp), "%d-%d-%d_%d-%d-%d",
             timeinfo.tm_year + 1900,
             timeinfo.tm_mon + 1,
             timeinfo.tm_mday,
             timeinfo.tm_hour,
             timeinfo.tm_min,
             timeinfo.tm_sec);
    client.beginPacket(serverIp, PORT);
    client.write((uint8_t*)&payload, sizeof(payload));
    client.endPacket();
    delay(1000);
    WiFi.disconnect(true, true);
    WiFi.mode(WIFI_OFF);
  }

  if (soundFile) {
    soundFile.flush();
    uint32_t dataSize = 0;
    if (soundFile.size() > sizeof(WavHeader)) {
      dataSize = soundFile.size() - sizeof(WavHeader);
    }
    writeWavHeader(soundFile, SAMPLE_RATE, 16, 2, dataSize);
    soundFile.close();
  }
  xQueueReset(udpQueue);
  if (strcmp(schedulingMode, "sampling") == 0) {
    esp_sleep_enable_timer_wakeup((uint64_t)sleepMs * 1000ULL);
  }
  esp_sleep_enable_ext0_wakeup(WAKE_GPIO, 0);
  Serial.println("Going to sleep");
  Serial.flush();
  esp_deep_sleep_start();
}

void sinkUDP(FRAME_FORMAT* frame) {
  if (soundFile) {
    soundFile.flush();
    uint32_t dataSize = 0;
    if (soundFile.size() > sizeof(WavHeader)) {
      dataSize = soundFile.size() - sizeof(WavHeader);
    }

    writeWavHeader(soundFile, SAMPLE_RATE, 16, 2, dataSize);
    soundFile.close();
  }
  payload.length = FRAME_BYTES;
  payload.sequence++;
  memset(payload.deviceId, 0, sizeof(payload.deviceId));
  strncpy(payload.deviceId, deviceId, sizeof(payload.deviceId) - 1);
  time_t now;
  time(&now);

  struct tm timeinfo;
  localtime_r(&now, &timeinfo);

  snprintf(payload.timestamp, sizeof(payload.timestamp), "%d-%d-%d_%d-%d-%d",
           timeinfo.tm_year + 1900,
           timeinfo.tm_mon + 1,
           timeinfo.tm_mday,
           timeinfo.tm_hour,
           timeinfo.tm_min,
           timeinfo.tm_sec);

  client.beginPacket(serverIp, PORT);
  client.write((uint8_t*)&payload, sizeof(payload));  // header only
  client.write((uint8_t*)frame, FRAME_SAMPLES * sizeof(int16_t));
  client.endPacket();
}

bool sdAlmostFull(float threshold = 0.95f) {
  uint64_t used = SD.usedBytes();

  if (total == 0) return false;

  float ratio = (float)used / (float)total;
  #ifdef DEBUG
  Serial.printf("SD card is %.02f%% full\n", ratio * 100);
  #endif
  return ratio >= threshold;
}

void sinkSD(FRAME_FORMAT* frame) {
  static uint32_t frameCounter = 0;
  if (!sdReady || (currentDevice == DEVICEMODE_SD_FULL)) {
    if (soundFile) {
      soundFile.close();
    }
    return;
  }

  if (!soundFile) {
    char folderName[64];
    char fileName[64];
    char idxFileName[64];

    snprintf(idxFileName, sizeof(idxFileName), "%s/no_date/fileIdx", transmissionFolder);

    if (timeSynced) {

      time_t now;
      time(&now);

      struct tm timeinfo;
      localtime_r(&now, &timeinfo);

      snprintf(folderName, sizeof(folderName),
               "%s/%d-%d-%d",
               transmissionFolder,
               timeinfo.tm_year + 1900,
               timeinfo.tm_mon + 1,
               timeinfo.tm_mday);

      if (!SD.exists(folderName)) {
        SD.mkdir(folderName);
      }

      snprintf(fileName, sizeof(fileName),
               "%s/%02d-%02d-%02d.wav",
               folderName,
               timeinfo.tm_hour,
               timeinfo.tm_min,
               timeinfo.tm_sec);
    } else {
      if (!SD.exists("/missed_transmissions/no_date")) {
        SD.mkdir("/missed_transmissions/no_date");
      }
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
               "%s/%s/audio_%d.wav", transmissionFolder, "no_date", fileIdx);
    }
    SD.remove(fileName);
    soundFile = SD.open(fileName, FILE_WRITE);
    if (!soundFile) {
      return;
    }

    soundFile.seek(sizeof(WavHeader));

    if (!timeSynced) {
      fileIdx++;
      SD.remove(idxFileName);
      idxFile = SD.open(idxFileName, FILE_WRITE);
      if (idxFile) {
        idxFile.seek(0);
        idxFile.write((uint8_t*)&fileIdx, sizeof(uint32_t));
        idxFile.flush();
        idxFile.close();
      }
    }
    frameCounter = 0;
    #ifdef DEBUG
    Serial.printf("SD: writing to %s\n", fileName);
    #endif
  }
  
  size_t n = soundFile.write((uint8_t*)frame, FRAME_BYTES);
  if (n != FRAME_BYTES) {
    Serial.println("SD write failed");
    soundFile.close();
    sdReady = false;
    return;
  }
  frameCounter++;

  if (frameCounter >= (SD_SECONDS_PER_FILE * SAMPLE_RATE * CHANNELS) / FRAME_SAMPLES) {
    uint32_t dataSize = 0;
    soundFile.flush();
    if (soundFile.size() > sizeof(WavHeader)) {
      dataSize = soundFile.size() - sizeof(WavHeader);
    }
    writeWavHeader(soundFile, SAMPLE_RATE, 16, 2, dataSize);
    soundFile.close();
    frameCounter = 0;
  }
}

void syncTime() {
  if (millis() - lastNtpAttempt > 30000) {
    lastNtpAttempt = millis();
    configTzTime("CET-1CEST,M3.5.0,M10.5.0/3", "pool.ntp.org");
    struct tm timeinfo;
    timeSynced = getLocalTime(&timeinfo, 100);
  }
}

void audioProcessTask(void *pv) {
  static FRAME_FORMAT frame[FRAME_SAMPLES];

  while (true) {
    size_t bytesRead = 0;
    if (i2s_read(I2S_RX_PORT, frame, FRAME_BYTES, &bytesRead, pdMS_TO_TICKS(10)) != ESP_OK) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }
    if (!sdReady || !systemActive) {
        vTaskDelay(pdMS_TO_TICKS(1));
        continue;
    } 
    if (!xQueueSend(udpQueue, frame, 0)) {
      queueDrops++;
    }
  }
}

void sendTask(void* pv) {
  static FRAME_FORMAT frame[FRAME_SAMPLES];
  while (true) {
    if (!systemActive) {
      while (xQueueReceive(udpQueue, frame, 0) == pdTRUE) {
        if (WiFi.isConnected()) {
          sinkUDP(frame);
        } else {
          sinkSD(frame);
        }
      }
      xTaskNotifyGive(mainTaskHandle);
      vTaskSuspend(NULL);
      continue;
    }

    if (!sdReady) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    if (xQueueReceive(udpQueue, frame, 0) == pdTRUE) {
      if (WiFi.isConnected()) {
        sinkUDP(frame);
      } else {
        sinkSD(frame);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void setup() {
  Serial.begin(115200);

  i2s_driver_install(I2S_RX_PORT, &rx_config, 0, NULL);
  i2s_set_pin(I2S_RX_PORT, &rx_pins);
  i2s_zero_dma_buffer(I2S_RX_PORT);

  pinMode(SPI_SD_CS, OUTPUT);
  if (!SPI.begin(SPI_SD_CLK, SPI_SD_MISO, SPI_SD_MOSI, SPI_SD_CS)){
    Serial.println("could not start spi");
  }

  pinMode(WAKE_GPIO, INPUT_PULLUP);
  pinMode(RED_GPIO, OUTPUT);
  pinMode(GREEN_GPIO, OUTPUT);
  pinMode(BLUE_GPIO, OUTPUT);

  digitalWrite(RED_GPIO, 0);
  digitalWrite(GREEN_GPIO, 0);
  digitalWrite(BLUE_GPIO, 0);

  setDevice(DEVICEMODE_STARTING);

  timeSynced = false;
  cycleStarted = false;
  cycleStart = 0;
  sdReady = false;
  sdLastCheck = 0;
  udpQueue = xQueueCreate(UDP_QUEUE_SIZE, FRAME_BYTES);
  mainTaskHandle = xTaskGetCurrentTaskHandle();

  #ifdef DEBUG
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0: Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_TIMER: Serial.println("Wakeup caused by timer"); break;
    default: Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
  #endif

  systemActive = true;
  xTaskCreatePinnedToCore(audioProcessTask, "audio", 8192, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(sendTask, "send", 12288, NULL, 2, NULL, 1);

  setupFinished = millis();
}

void loop() {
  if (digitalRead(WAKE_GPIO) == 0 && millis() > (setupFinished + 2000)) {
    while (digitalRead(WAKE_GPIO) == 0) {
      delay(10);
    }
    enableDeepSleep();
  }
  if (sdReady) {
    if (!SD.exists("/.connected")) {
      sdReady = false;
      timeSynced = false;
      if (WiFi.status() == WL_CONNECTED) {
        WiFi.disconnect(true, true);
      }
      if (soundFile) {
        soundFile.close();
      }
    }
  }
  if (!sdReady && (millis() - sdLastCheck >= 2000)) {
    sdLastCheck = millis();

    if (InitSD()) {
      if (!getNetworkInfo()) {
        currentDevice = DEVICEMODE_SD_ERROR;
      } else if (!getScheduleInfo()) {
        currentDevice = DEVICEMODE_SD_ERROR;
      } else {
        sdReady = true;
      }
      if (!SD.exists("/missed_transmissions")) SD.mkdir("/missed_transmissions");
      total = SD.totalBytes();
      if (sdAlmostFull()) {
        currentDevice = DEVICEMODE_SD_FULL;
      }
    } else {
      #ifdef DEBUG
      Serial.println("Waiting for SD card...");
      #endif
      currentDevice = DEVICEMODE_NO_SD;
    }
  }
  if (lastMode != currentDevice) {
    setDevice(currentDevice);
    lastMode = currentDevice;
  }
  if (sdReady) {
    if (!WiFi.isConnected()) {
      udpStarted = false;

      static uint32_t lastAttempt = 0;
      if (millis() - lastAttempt > 10000) {

        lastAttempt = millis();
        #ifdef DEBUG
        Serial.printf("Connecting WiFi to %s\n", ssid);
        #endif
        wifiConnect();
      }
    }
    if (WiFi.isConnected()) {
      if (!udpStarted) {
        client.begin(PORT);
        if (currentDevice != DEVICEMODE_SD_FULL) {
          currentDevice = DEVICEMODE_WIFI;
        }
        udpStarted = true;
      }
      if (!timeSynced) {
        syncTime();   
      }
    } else {
      if (currentDevice != DEVICEMODE_SD_FULL) {
        currentDevice = DEVICEMODE_SD;
      }
    }

    if (strcmp(schedulingMode, "sampling") == 0) {
      uint32_t now = millis();

      if (!cycleStarted) {
        cycleStart = now;  // first run init
        cycleStarted = true;
      }

      uint32_t elapsed = now - cycleStart;

      if (elapsed >= durationMs) {
        enableDeepSleep();
      }
    }
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint > 1000) {
      lastPrint = millis();
      #ifdef DEBUG
      Serial.printf("WiFi %s | Queue: %d | Drops: %d, RSSI %d\n",
                    WiFi.isConnected() ? "Connected" : "Disconnected",
                    uxQueueMessagesWaiting(udpQueue),
                    queueDrops,
                    WiFi.RSSI());
      #endif
    }
    static uint32_t lastSizeCheck = 0;
    if (millis() - lastSizeCheck > 1800000) {
      lastSizeCheck = millis();
      if (sdAlmostFull()) {
        currentDevice = DEVICEMODE_SD_FULL;
      }
    }
  }
}