#include <Arduino.h>
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include <WiFi.h>
#include <SD.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <WiFiUdp.h>
#include <atomic>

//Pin define
#define RED_GPIO 4
#define BLUE_GPIO 15
#define GREEN_GPIO 2
#define WAKE_GPIO GPIO_NUM_13

#define SPI_SD_CS 18
#define SPI_SD_MOSI 16
#define SPI_SD_MISO 17
#define SPI_SD_CLK 19

#define SPI_AUDIO_CS 32
#define SPI_AUDIO_MOSI 25
#define SPI_AUDIO_MISO 27
#define SPI_AUDIO_CLK 26

//Frame info
#define FRAME_FORMAT int16_t
#define SAMPLES_PER_CHANNEL 128
#define UDP_QUEUE_SIZE 64
#define CHANNELS 2
#define FRAME_SAMPLES (SAMPLES_PER_CHANNEL * 2)
#define FRAME_BYTES (sizeof(FRAME_FORMAT) * FRAME_SAMPLES)
#define SAMPLE_RATE 20000
#define SD_SECONDS_PER_FILE 20

int16_t* rxBuffer = nullptr;

spi_bus_config_t buscfg = {
  .mosi_io_num     = SPI_AUDIO_MOSI,
  .miso_io_num     = SPI_AUDIO_MISO,
  .sclk_io_num     = SPI_AUDIO_CLK,
  .quadwp_io_num   = -1,
  .quadhd_io_num   = -1,
  .max_transfer_sz = FRAME_BYTES,
};

spi_slave_interface_config_t slvcfg = {
  .spics_io_num   = SPI_AUDIO_CS,
  .flags          = 0,
  .queue_size     = 3,
  .mode           = 0,
  .post_setup_cb  = NULL,
  .post_trans_cb  = NULL,
};

#define PORT 50000

struct __attribute__((packed)) WavHeader {
  char riff[4] = { 'R', 'I', 'F', 'F' };
  uint32_t chunkSize;
  char wave[4] = { 'W', 'A', 'V', 'E' };

  char fmt[4] = { 'f', 'm', 't', ' ' };
  uint32_t subchunk1Size = 16;
  uint16_t audioFormat = 1;
  uint16_t numChannels;
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

uint16_t setupFinished = 0;

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

SPIClass spiSd(VSPI);

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

bool InitSD() {
    SD.end();
    bool begun = SD.begin(SPI_SD_CS, spiSd);
    Serial.printf("SD.begin: %s\n", begun ? "OK" : "FAIL");
    if (begun) {
        return SD.exists("/.connected");
    }
    return false;
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

  File file = SD.open("/Network/Network.json");
  if (!file) {
    Serial.println("could not find network file");
    return false;
  }
  data = file.readString();
  file.close();

  JsonDocument doc;
  if (deserializeJson(doc, data)) return false;

  //strlcpy(ssid,     doc["SSID"]               | "", sizeof(ssid));
  //strlcpy(password, doc["Password"]           | "", sizeof(password));

  strlcpy(ssid, "CiscoE1200", sizeof(ssid));
  strlcpy(password, "CiscoE1200", sizeof(password));
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
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
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
    WiFi.disconnect(true);
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

void sinkSD(FRAME_FORMAT* frame) {
  static uint32_t frameCounter = 0;
  if (!sdReady) {
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
      Serial.println("SD: failed to open file");
      return;
    }

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
    Serial.printf("SD: writing to %s\n", fileName);
  }

  size_t n = soundFile.write((uint8_t*)frame, FRAME_BYTES);
  if (n != FRAME_BYTES) {
    Serial.println("SD write failed");
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
    Serial.println("SD: file rotated");
  }
}

void syncTime() {
  if (millis() - lastNtpAttempt > 10000) {
    lastNtpAttempt = millis();

    configTzTime("CET-1CEST,M3.5.0,M10.5.0/3", "pool.ntp.org");

    time_t now = time(nullptr);
    if (now > 1700000000) {
      timeSynced = true;
      Serial.println("Time synced");
    }
  }
}

float phase = 0.0f;
float modPhase = 0.0f;

void generateWail(FRAME_FORMAT* buffer, int samples) {
  for (int i = 0; i < samples; i += 2) {
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

void audioProcessTask(void *pv) {
  static FRAME_FORMAT local32[FRAME_SAMPLES];

  const TickType_t periodTicks = pdMS_TO_TICKS(1000.0f * FRAME_SAMPLES / (SAMPLE_RATE * CHANNELS));

  TickType_t wakeTime = xTaskGetTickCount();

  while (true) {
    if (!sdReady || !systemActive) {
        vTaskDelay(pdMS_TO_TICKS(1));
        wakeTime = xTaskGetTickCount();
        continue;
    } 

    generateWail(local32, FRAME_SAMPLES);

    if (!xQueueSend(udpQueue, local32, 0)) {
      queueDrops++;
    }

    vTaskDelayUntil(&wakeTime, periodTicks);
  }
}

void sendTask(void* pv) {
  static FRAME_FORMAT frame[FRAME_SAMPLES];
  while (true) {
    if (!systemActive) {
      if (!sdReady) {
        vTaskDelay(pdMS_TO_TICKS(1));
        continue;
      }
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

    if (xQueueReceive(udpQueue, frame, pdMS_TO_TICKS(6)) == pdTRUE) {
      if (WiFi.isConnected()) {
        sinkUDP(frame);
      } else {
        sinkSD(frame);
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  
  esp_err_t ret = spi_slave_initialize(SPI2_HOST, &buscfg, &slvcfg, SPI_DMA_CH1);
  if (ret != ESP_OK) {
    Serial.print("SPI slave init failed: ");
    Serial.println(ret);
    return;
  }

  pinMode(SPI_SD_CS, OUTPUT);
  spiSd.begin(SPI_SD_CLK, SPI_SD_MISO, SPI_SD_MOSI, SPI_SD_CS);

  pinMode(WAKE_GPIO, INPUT_PULLUP);
  pinMode(RED_GPIO, OUTPUT);
  pinMode(GREEN_GPIO, OUTPUT);
  pinMode(BLUE_GPIO, OUTPUT);

  digitalWrite(RED_GPIO, 0);
  digitalWrite(GREEN_GPIO, 0);
  digitalWrite(BLUE_GPIO, 0);

  setDevice(DEVICEMODE_STARTING);

  cycleStart = 0;
  sdReady = false;
  sdLastCheck = 0;
  udpQueue = xQueueCreate(UDP_QUEUE_SIZE, FRAME_BYTES);
  mainTaskHandle = xTaskGetCurrentTaskHandle();

  WiFi.setSleep(false);

  gpio_set_pull_mode((gpio_num_t)SPI_AUDIO_MOSI, GPIO_PULLUP_ONLY);
  gpio_set_pull_mode((gpio_num_t)SPI_AUDIO_CLK, GPIO_PULLUP_ONLY);
  gpio_set_pull_mode((gpio_num_t)SPI_AUDIO_CS, GPIO_PULLUP_ONLY);

  rxBuffer = (int16_t*)heap_caps_malloc(FRAME_BYTES, MALLOC_CAP_DMA);

  if (!rxBuffer) {
    Serial.println("Failed to allocate DMA buffers");
    return;
  }

  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0: Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_TIMER: Serial.println("Wakeup caused by timer"); break;
    default: Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }

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
    bool connected = SD.exists("/.connected");

    if (!connected) {
      sdReady = false;
      timeSynced = false;
      if (WiFi.status() == WL_CONNECTED) {
        WiFi.disconnect(false, false);
      }
      if (soundFile) {
        soundFile.close();
      }
    }
  }
  if (!sdReady && (millis() - sdLastCheck >= 2000)) {
    sdLastCheck = millis();
    if (InitSD()) {
      getNetworkInfo();
      getScheduleInfo();
      sdReady = true;
      if (!SD.exists("/missed_transmissions")) SD.mkdir("/missed_transmissions");
    } else {
      Serial.println("Waiting for SD card...");
      currentDevice = DEVICEMODE_NO_SD;
    }
  }
  if (lastMode != currentDevice) {
    setDevice(currentDevice);
    lastMode = currentDevice;
  }
  if (sdReady) {
    if (!WiFi.isConnected()) {
      timeSynced = false;
      udpStarted = false;

      static uint32_t lastAttempt = 0;

      if (millis() - lastAttempt > 10000) {

        lastAttempt = millis();

        Serial.printf("Connecting WiFi to %s\n", ssid);

        WiFi.disconnect(false, false);
        WiFi.begin(ssid, password);
      }
    }
    if (WiFi.isConnected()) {
      currentDevice = DEVICEMODE_WIFI;
      if (!udpStarted) {
        client.begin(PORT);
        udpStarted = true;
      }
      if (!timeSynced) {
        syncTime();
        timeSynced = true;
      }
    } else {
      currentDevice = DEVICEMODE_SD;
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
      Serial.printf("WiFi %s | Queue: %d | Drops: %d, RSSI %d\n",
                    WiFi.isConnected() ? "Connected" : "Disconnected",
                    uxQueueMessagesWaiting(udpQueue),
                    queueDrops,
                    WiFi.RSSI());
    }
  }
}