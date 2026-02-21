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
#include <BlynkSimpleEsp8266.h>

// ---------- Config ----------
static const uint64_t SLEEP_US = 5ULL * 60ULL * 1000000ULL; // 5 min

// Wemos D1 ESP32 I2C pins
static const int I2C_SDA = 21;
static const int I2C_SCL = 22;

static const int STATUS_LED_PIN = 2;

// BME280
Adafruit_BME280 bme;
bool bmeOk = false;

// Sensor values
float tC = NAN, h = NAN, pHpa = NAN;

// ---------- Helpers ----------
void goToSleep() {
  WiFi.disconnect(true);
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

  WiFiManager wm;
  wm.setConfigPortalTimeout(180); // max 3 min
  wm.setConnectTimeout(15);       // 15s trying router

  // If it cannot connect, it creates AP portal:
  // SSID: ESP32-BME280-SETUP
  bool ok = wm.autoConnect("ESP32-BME280-SETUP");
  if (!ok) return false;

  return (WiFi.status() == WL_CONNECTED);
}

bool sendToBlynk() {
  // Push mode: connect -> send -> disconnect
  Blynk.config(BLYNK_AUTH_TOKEN);

  unsigned long start = millis();
  while (!Blynk.connected() && millis() - start < 8000) {
    Blynk.run();
    delay(10);
  }
  if (!Blynk.connected()) return false;

  // Virtual Pins:
  // V0: Temperature, V1: Humidity, V2: Pressure
  Blynk.virtualWrite(V0, tC);
  Blynk.virtualWrite(V1, h);
  Blynk.virtualWrite(V2, pHpa);

  // Give time to flush packets
  unsigned long t0 = millis();
  while (millis() - t0 < 300) {
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
