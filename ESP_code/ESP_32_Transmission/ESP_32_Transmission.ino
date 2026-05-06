#include <WiFi.h>
#include <SD.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <HTTPClient.h>

#define CS 5
#define MOSI 23
#define MISO 19
#define CLK 18
#define CHUNK_SIZE 5000

char ssid[64];
char password[64];
char targetIP[64];
char deviceId[64];

bool fileSent = false;

struct PacketHeader {
  uint16_t index;
  uint16_t total;
  uint16_t len;
};

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

void setup() {
  Serial.begin(115200);
  pinMode(CS, OUTPUT);
  SPI.begin(CLK, MISO, MOSI, CS);
}

void loop() {

  if (sdReady && !SD.exists("/")) {
    Serial.println("SD card removed!");
    sdReady = false;
    wifiLoaded = false;
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
      wifiLoaded = true;
    }
  }

  if (wifiLoaded && WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi lost, reconnecting...");
    wifiLoaded = false;
    fileSent = false;
  }

  if (sdReady && wifiLoaded && !fileSent) {

    File file = SD.open("/Missed_transmissions/output.raw");
    if (!file) {
      Serial.println("File open failed");
      return;
    }

    WiFiClient client;
    HTTPClient http;

    String url = "http://";
    url += targetIP;
    url += ":5000/upload_chunk";

    size_t fileSize = file.size();
    uint16_t totalChunks = (fileSize + CHUNK_SIZE - 1) / CHUNK_SIZE;

    uint16_t index = 0;
    uint8_t buffer[CHUNK_SIZE];


    while (file.available()) {

      size_t len = file.read(buffer, CHUNK_SIZE);
      http.begin(client, url);
      http.addHeader("Device-Id", deviceId);
      http.addHeader("Content-Type", "application/octet-stream");
      http.addHeader("Chunk-Index", String(index));
      http.addHeader("Total-Chunks", String(totalChunks));

      int code = http.POST(buffer, len);
      http.end();

      int retries = 10;

      while (code <= 0 && retries > 0) {
        delay(10);

        Serial.println("Retrying chunk...");
        http.begin(client, url);
        http.addHeader("Device-Id", deviceId);
        http.addHeader("Content-Type", "application/octet-stream");
        http.addHeader("Chunk-Index", String(index));
        http.addHeader("Total-Chunks", String(totalChunks));

        int code = http.POST(buffer, len);
        http.end();

        retries--;
      }

      if (code > 0) {
        Serial.print("Chunk ");
        Serial.print(index);
        Serial.print("/");
        Serial.print(totalChunks);
        Serial.println(" sent");
      } else {
        Serial.print("Chunk permanently failed: ");
        Serial.println(index);
      }

      index++;
    }
    file.close();
    fileSent = true;

    Serial.println("All chunks sent");
  }
  delay(1);
}