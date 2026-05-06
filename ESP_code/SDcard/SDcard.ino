#include <SD.h>
#include <ArduinoJson.h>
#include <SPI.h>

#define CS 5
#define MOSI 23
#define MISO 19
#define CLK 18

char* ssid = nullptr;
char* password = nullptr;

bool sdReady = false;
bool wifiLoaded = false;

unsigned long lastCheck = 0;

void freeWifi() {
  if (ssid) {
    free(ssid);
    ssid = nullptr;
  }
  if (password) {
    free(password);
    password = nullptr;
  }
}

bool tryInitSD() {
  SPI.end();
  SPI.begin(CLK, MISO, MOSI, CS);

  if (!SD.begin(CS)) return false;

  File test = SD.open("/");
  if (!test) return false;

  test.close();
  return true;
}

bool isSdStillPresent() {
  File root = SD.open("/");
  if (!root) return false;
  root.close();
  return true;
}

bool getWifiInfo() {
  File wifiFile = SD.open("/WiFi/WiFi.json");
  if (!wifiFile) {
    Serial.println("Error opening WiFi.json");
    return false;
  }

  String data = wifiFile.readString();
  wifiFile.close();

  JsonDocument wifiInfo;
  DeserializationError err = deserializeJson(wifiInfo, data);

  if (err) {
    Serial.println("JSON parse failed");
    return false;
  }

  if (!wifiInfo["SSID"] || !wifiInfo["Password"]) {
    Serial.println("Missing fields in JSON");
    return false;
  }

  freeWifi();

  ssid = strdup(wifiInfo["SSID"].as<const char*>());
  password = strdup(wifiInfo["Password"].as<const char*>());

  return true;
}
void setup() {
  Serial.begin(115200);
  SPI.begin(CLK, MISO, MOSI, CS);
  pinMode(CS, OUTPUT);
}

void loop() {
  if (sdReady && !isSdStillPresent()) {
    Serial.println("SD card removed!");
    
    sdReady = false;
    wifiLoaded = false;
    freeWifi();
  }

  if (!sdReady && (millis() - lastCheck >= 2000)) {
    lastCheck = millis();

    if (tryInitSD()) {
      Serial.println("SD card connected!");
      sdReady = true;
    } else {
      Serial.println("Waiting for SD card...");
    }
  }
  
  if (sdReady && !wifiLoaded) {
    getWifiInfo();

    Serial.println(ssid ? ssid : "SSID null");
    Serial.println(password ? password : "PASS null");

    wifiLoaded = true;
  }
}