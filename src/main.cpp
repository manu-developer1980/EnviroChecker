#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <WiFiManager.h>
#include <ESP8266HTTPClient.h>
#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ArduinoJson.h>

#include "secrets.h"

// Blynk
#define BLYNK_PRINT Serial
#include <BlynkSimpleEsp8266.h>
#include <LittleFS.h>

// ---------- Config ----------
static const uint64_t SLEEP_US = 5ULL * 60ULL * 1000000ULL; // 5 min

// Wemos D1 ESP32 I2C pins
static const int I2C_SDA =  4;
static const int I2C_SCL = 5;

static const int STATUS_LED_PIN = 2;

// BME280
Adafruit_BME280 bme;
bool bmeOk = false;

// Sensor values
float tC = NAN, h = NAN, pHpa = NAN;

// ---------- Helpers ----------
struct WiFiCred {
  String ssid;
  String pass;
};

bool loadWiFiCreds(WiFiCred& cred) {
  if (!LittleFS.begin()) return false;
  if (!LittleFS.exists("/wifi.json")) return false;
  File f = LittleFS.open("/wifi.json", "r");
  if (!f) return false;
  StaticJsonDocument<128> doc;
  DeserializationError e = deserializeJson(doc, f);
  f.close();
  if (e) return false;
  const char* s = doc["ssid"] | "";
  const char* p = doc["pass"] | "";
  cred.ssid = String(s);
  cred.pass = String(p);
  return cred.ssid.length() > 0;
}

bool saveWiFiCreds(const String& ssid, const String& pass) {
  if (!LittleFS.begin()) return false;
  File f = LittleFS.open("/wifi.json", "w");
  if (!f) return false;
  StaticJsonDocument<128> doc;
  doc["ssid"] = ssid;
  doc["pass"] = pass;
  bool ok = (serializeJson(doc, f) > 0);
  f.close();
  return ok;
}
void goToSleep() {
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);

  delay(50);
  ESP.deepSleep(SLEEP_US);
}

bool initBME280() {
  Wire.begin(I2C_SDA, I2C_SCL);

  // Common addresses
  if (bme.begin(0x76)) return true;
  if (bme.begin(0x77)) return true;
  return false;
}

bool readBME() {
  if (!bmeOk) return false;

  tC   = bme.readTemperature();
  h    = bme.readHumidity();
  pHpa = bme.readPressure() / 100.0F; // Pa -> hPa

  return !(isnan(tC) || isnan(h) || isnan(pHpa));
}

bool connectWiFiWithPortal() {
  WiFi.mode(WIFI_STA);
  WiFi.persistent(true);
  WiFi.setAutoReconnect(true);

  WiFiCred cred;
  if (loadWiFiCreds(cred)) {
    WiFi.begin(cred.ssid.c_str(), cred.pass.length() ? cred.pass.c_str() : nullptr);
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 12000) {
      delay(100);
    }
    if (WiFi.status() == WL_CONNECTED) {
      return true;
    }
  }

  // Intento 1: reconectar usando credenciales guardadas por el SDK
  String ssid = WiFi.SSID();
  String pass = WiFi.psk();
  if (ssid.length() > 0) {
    Serial.print("Reconnecting to saved WiFi: ");
    Serial.println(ssid);
    WiFi.begin(ssid.c_str(), pass.length() ? pass.c_str() : nullptr);
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 12000) {
      delay(100);
    }
    if (WiFi.status() == WL_CONNECTED) {
      saveWiFiCreds(WiFi.SSID(), WiFi.psk());
      return true;
    }
    Serial.println("Saved WiFi reconnect failed, opening portal...");
  }

  WiFiManager wm;
  wm.setConfigPortalTimeout(180); // max 3 min
  wm.setConnectTimeout(15);       // 15s trying router

  // If it cannot connect, it creates AP portal:
  // SSID: ESP32-BME280-SETUP
  bool ok = wm.autoConnect("ESP32-BME280-SETUP");
  if (!ok) return false;
  delay(200);
  saveWiFiCreds(WiFi.SSID(), WiFi.psk());

  return (WiFi.status() == WL_CONNECTED);
}

bool sendToBlynk() {
  // Push mode: connect -> send -> disconnect
  Blynk.config(BLYNK_AUTH_TOKEN, "blynk.cloud", 80);

  delay(300);
  bool connected = Blynk.connect(30000);
  if (!connected) {
    return false;
  }

  // Virtual Pins:
  // V0: Temperature, V1: Humidity, V2: Pressure
  Blynk.virtualWrite(V0, tC);
  Blynk.virtualWrite(V1, h);
  Blynk.virtualWrite(V2, pHpa);

  // Give time to flush packets
  unsigned long t0 = millis();
  while (millis() - t0 < 600) {
    Blynk.run();
    delay(5);
  }

  Blynk.disconnect();
  return true;
}
/*
bool sendToSupabase() {
  if (WiFi.status() != WL_CONNECTED) return false;

  HTTPClient http;
  String endpoint = String(SUPABASE_URL) + "/rest/v1/" + SUPABASE_TABLE;

  http.begin(endpoint);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("apikey", SUPABASE_KEY);
  http.addHeader("Prefer", "return=minimal");

  StaticJsonDocument<256> doc;
  doc["device_id"] = DEVICE_UUID;
  doc["temperature"] = tC;
  doc["humidity"] = h;
  doc["pressure"] = pHpa;

  String body;
  serializeJson(doc, body);

  int code = http.POST(body);
  String resp = http.getString();
  http.end();

  Serial.print("Supabase HTTP code: ");
  Serial.println(code);
  if (code < 200 || code >= 300) {
    Serial.print("Supabase resp: ");
    Serial.println(resp);
  }

  return (code >= 200 && code < 300);
}
*/
// ---------- Arduino ----------
void setup() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  Serial.begin(115200);
  delay(200);

  // Init BME
  bmeOk = initBME280();
  if (!bmeOk) {
    Serial.println("ERROR: BME280 not found at 0x76/0x77. Sleeping.");
    goToSleep();
  }

  // WiFi with portal fallback
  if (!connectWiFiWithPortal()) {
    Serial.println("ERROR: WiFi not connected. Sleeping.");
    goToSleep();
  }

  Serial.print("WiFi OK. IP: ");
  Serial.println(WiFi.localIP());

  // Read sensor
  if (!readBME()) {
    Serial.println("ERROR: Failed reading BME280. Sleeping.");
    goToSleep();
  }

  Serial.printf("T=%.2fC  H=%.2f%%  P=%.2fhPa\n", tC, h, pHpa);

  // Send
  bool bOk = sendToBlynk();
  //bool sOk = sendToSupabase();

  Serial.print("Blynk: ");    Serial.println(bOk ? "OK" : "FAIL");
  //Serial.print("Supabase: "); Serial.println(sOk ? "OK" : "FAIL");

  // Sleep
  Serial.println("Deep sleep 5 minutes...");
  delay(100);
  goToSleep();
}

void loop() {
  // Not used
}
