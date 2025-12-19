#include <Arduino.h>
#include <NimBLEDevice.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <TAMC_GT911.h>
#include <Preferences.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Update.h>
#include "esp_ota_ops.h"
#include <functional>

/**********************
 * Firmware Metadata  *
 **********************/
#define FW_VERSION "0.0.9"

/**********************
 * Hardware Constants *
 **********************/
#define PIN_NEOPIXEL   15
#define NUM_LEDS        8
#define TOUCH_SDA      21
#define TOUCH_SCL      22
#define TOUCH_INT      19
#define TOUCH_RST      18
#define TOUCH_WIDTH  1024
#define TOUCH_HEIGHT  800
#define GT911_ADDR   0x5D

static const int OUTPUT_PINS[7] = { 32, 33, 25, 26, 27, 14, 12};

/**********************
 * BLE UUIDs          *
 **********************/
#define SERVICE_UUID        "3f542309-50a0-4edb-aa41-c4d068dc72f5"
#define CHARACTERISTIC_UUID "39861dbb-c278-4e78-a542-17468828adb9"

/**********************
 * Network Endpoints  *
 **********************/
static const char* WEBSOCKET_HOST = "reflekt.onrender.com";
static const uint16_t WEBSOCKET_PORT = 443;
static const char* MANIFEST_URL = "https://reflekt.onrender.com/presence/firmware";

/**********************
 * NVS Keys           *
 **********************/
static const char* PREF_NAMESPACE   = "wifi";
static const char* PREF_SSID        = "ssid";
static const char* PREF_PASS        = "pass";
static const char* PREF_PIN_PREFIX  = "pin_"; // + pin number

/**********************
 * Forward Decl (ptr) *
 **********************/
class CommandRouter; // full definition appears before any use

/**********************
 * LED Controller     *
 **********************/
class LEDController {
public:
  LEDController(uint8_t pin, uint16_t count)
  : strip(count, pin, NEO_GRB + NEO_KHZ800) {}

  void begin(uint8_t brightness = 128) {
    Serial.println("LED started Init");
    strip.begin();
    strip.setBrightness(brightness);
    strip.show();
    Serial.println("Done Setup");
    setAll(253,244,220,128);
  }

  void setAll(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness) {
    Serial.println("Change color");
    strip.setBrightness(constrain(brightness, 0, 255));
    for (uint16_t i = 0; i < strip.numPixels(); ++i) {
      strip.setPixelColor(i, strip.Color(r, g, b));
    }
    strip.show();
  }

private:
  Adafruit_NeoPixel strip;
};

/**********************
 * Pin Manager (NVS)  *
 **********************/
class PinManager {
public:
  PinManager(Preferences& prefs, const int* pins, size_t count)
  : preferences(prefs), pinArray(pins), pinCount(count) {}

  void begin() {
    for (size_t i = 0; i < pinCount; ++i) {
      restorePin(pinArray[i]);
    }
  }

  int currentLevel(int pin) {
    pinMode(pin, OUTPUT);
    return digitalRead(pin);
  }

  int savedLevel(int pin) {
    uint8_t saved = preferences.getUChar(pinKey(pin).c_str(), 0);
    return saved ? HIGH : LOW;
  }

  void toggle(int pin) {
    pinMode(pin, OUTPUT);
    int curr = digitalRead(pin);
    setPinAndSave(pin, curr == HIGH ? LOW : HIGH);
  }

  void setPinAndSave(int pin, int level) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, level);
    String key = pinKey(pin);
    preferences.putUChar(key.c_str(), level == HIGH ? 1 : 0);
    Serial.printf(" Pin %d set to %s (saved)", pin, level == HIGH ? "HIGH" : "LOW");
  }

  void restorePin(int pin) {
    pinMode(pin, OUTPUT);
    uint8_t saved = preferences.getUChar(pinKey(pin).c_str(), 0); // default LOW
    digitalWrite(pin, saved ? HIGH : LOW);
    Serial.printf("Pin %d restored to %s", pin, saved ? "HIGH" : "LOW");
  }

  void buildAllStatuses(JsonObject out, bool useSaved=false) {
    for (size_t i = 0; i < pinCount; ++i) {
      int pin = pinArray[i];
      int lvl = useSaved ? savedLevel(pin) : currentLevel(pin);
      out[String(pin)] = (lvl == HIGH) ? 1 : 0;
    }
  }

  int pinStatus(int pin) {                 
    return currentLevel(pin);
  }

  String pinsCsv(bool useSaved = false) {
    String out;
    for (size_t i = 0; i < pinCount; ++i) {
      int pin = pinArray[i];
      int lvl = useSaved
        ? (preferences.getUChar(pinKey(pin).c_str(), 0) ? HIGH : LOW)
        : digitalRead(pin);
      if (i) out += ",";
      out += String(pin) + ":" + (lvl == HIGH ? "1" : "0");
    }
    return out;
  }

  static bool isManagedPin(int pin) {
    for (size_t i = 0; i < 7; ++i) if (OUTPUT_PINS[i] == pin) return true;
    return false;
  }

private:
  String pinKey(int pin) { return String(PREF_PIN_PREFIX) + String(pin); }

private:
  Preferences& preferences;
  const int* pinArray;
  size_t pinCount;
};

/**********************
 * Touch Panel (GT911)*
 **********************/
class TouchPanel {
public:
  TouchPanel() : ts(TOUCH_SDA, TOUCH_SCL, TOUCH_INT, TOUCH_RST, TOUCH_WIDTH, TOUCH_HEIGHT) {}

  void begin(bool enable = false) {
    enabled = enable;
    if (!enabled) return;

    Wire.begin(TOUCH_SDA, TOUCH_SCL);
    Wire.setTimeOut(50);
    Wire.setClock(100000);  // start slow; speed up after detect

    // INFO: see if INT is actually high at idle (GPIO4 is a strap pin)
    pinMode(TOUCH_INT, INPUT_PULLUP);
    delay(2);
    Serial.printf("GT911 INT idle level: %d (1=HIGH)\n", digitalRead(TOUCH_INT));

    // Passive wait: some modules appear late after power-up
    if (!passiveDetect()) {
      // Force 0x5D first (INT=HIGH), then 0x14 (INT=LOW)
      resetAndWake(/*want5D=*/true);
      if (!waitForAddr(0x5D, 2500)) {
        resetAndWake(/*want5D=*/false);
        if (!waitForAddr(0x14, 2500)) {
          Serial.println("GT911 not found at 0x5D or 0x14. Touch disabled.");
          enabled = false;
          return;
        } else {
          detectedAddr = 0x14;
          Serial.println("GT911 detected at 0x14 (forced).");
        }
      } else {
        detectedAddr = 0x5D;
        Serial.println("GT911 detected at 0x5D (forced).");
      }
    }

    Wire.setClock(400000);   // OK to speed up now

    // NOTE: TAMC_GT911 typically works at 0x5D by default.
    // If your board ended up at 0x14 and the lib doesn't auto-detect,
    // we may need to patch the lib; try as-is first.
    ts.begin();
    // ts.setRotation(0);
    Serial.printf("GT911 ready (addr=0x%02X).\n", detectedAddr ? detectedAddr : (uint8_t)GT911_ADDR);
  }

  // Returns 1..4 for bulbs, or 0 if no new touch
  int readBulb() {
    // Serial.println("Reading");
    // Serial.println(enabled);
    if (!enabled) return 0;

    ts.read();
    if (ts.touches <= 0) { lastTouchedBulb = 0; return 0; }

    TP_Point p = ts.points[0];
    int rawX = p.x, rawY = p.y;


    // Target screen size
    const int SCR_W = 800;
    const int SCR_H = 480;


     int screenX = map(rawY, 800, 200, 0, 800);
    int screenY = map(rawX, 1020, 0, 0, 480);

    // Clamp
    screenX = constrain(screenX, 0, SCR_W - 1);
    screenY = constrain(screenY, 0, SCR_H - 1);
    // ---- Debug prints ----
    Serial.printf("RawX=%d, RawY=%d  -->  ScreenX=%d, ScreenY=%d\n",
                rawX, rawY, screenX, screenY);
    int currentBulb = 0;


    if      (screenX >= 180 && screenX <= 360 && screenY >= 60 && screenY <= 110) currentBulb = 1;
    else if (screenX >= 170 && screenX <= 360 && screenY >= 120 && screenY <= 240) currentBulb = 2;
    else if (screenX >= 170 && screenX <= 360 && screenY >= 360 && screenY <= 420) currentBulb = 3;
    else if (screenX >= 490 && screenX <= 680 && screenY >= 60 && screenY <= 110) currentBulb = 4;
    else if (screenX >= 490 && screenX <= 680 && screenY >= 200 && screenY <= 270) currentBulb = 5;
    else if (screenX >= 490 && screenX <= 680 && screenY >= 300 && screenY <= 420) currentBulb = 6;


    if (currentBulb != 0 && currentBulb != lastTouchedBulb) {
      lastTouchedBulb = currentBulb;
      return currentBulb;
    }
    return 0;
  }

private:
  // --- tiny helpers (self-contained) ---
  bool waitForAddr(uint8_t addr, uint32_t timeout_ms) {
    uint32_t t0 = millis();
    while (millis() - t0 < timeout_ms) {
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() == 0) return true;
      delay(10);
    }
    return false;
  }

  // Try 0x5D/0x14 without resetting (handles slow wake)
  bool passiveDetect() {
    // try up to ~2s
    uint32_t t0 = millis();
    while (millis() - t0 < 2000) {
      if (isConnected(0x5D)) { detectedAddr = 0x5D; Serial.println("GT911 detected at 0x5D (passive)."); return true; }
      if (isConnected(0x14)) { detectedAddr = 0x14; Serial.println("GT911 detected at 0x14 (passive)."); return true; }
      delay(50);
    }
    return false;
  }

  bool isConnected(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
  }

  // Hold INT during reset to select address
  void resetAndWake(bool want5D = true) {
    pinMode(TOUCH_INT, OUTPUT);
    pinMode(TOUCH_RST, OUTPUT);

    digitalWrite(TOUCH_INT, want5D ? HIGH : LOW);
    digitalWrite(TOUCH_RST, LOW);    delay(30);
    digitalWrite(TOUCH_RST, HIGH);   delay(300);   // internal firmware boot
    pinMode(TOUCH_INT, INPUT_PULLUP); 
    delay(200); // settle before first I2C
  }

private:
  TAMC_GT911 ts;
  bool enabled = false;
  int  lastTouchedBulb = 0;
  uint8_t detectedAddr = 0;   // 0x5D or 0x14 when found
};


/**********************
 * OTA Updater        *
 **********************/
class OTAUpdater {
public:
  bool fetchManifest(String& version, String& binUrl) {
    WiFiClientSecure client; client.setInsecure();
    HTTPClient http;
    if (!http.begin(client, MANIFEST_URL)) return false;
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

    int code = http.GET();
    if (code != HTTP_CODE_OK) { http.end(); return false; }
    String body = http.getString();
    http.end();

    StaticJsonDocument<512> doc;
    DeserializationError err = deserializeJson(doc, body);
    if (err) return false;

    version = doc["version"].as<String>();
    binUrl  = doc["file"].as<String>();
    return version.length() && binUrl.length();
  }

  bool doHttpUpdate(const String& binUrl) {
    WiFiClientSecure client; client.setInsecure();
    HTTPClient http;
    if (!http.begin(client, binUrl)) return false;

    int code = http.GET();
    if (code != HTTP_CODE_OK) { http.end(); return false; }

    int contentLen = http.getSize();
    if (contentLen <= 0) { http.end(); return false; }

    const esp_partition_t* part = esp_ota_get_next_update_partition(NULL);
    Serial.printf("OTA slot '%s' size=%d bytes, new image size=%d", part ? part->label : "(null)", part ? part->size : -1, contentLen);

    if (!Update.begin(contentLen)) { http.end(); return false; }

    WiFiClient* stream = http.getStreamPtr();
    size_t written = Update.writeStream(*stream);
    bool ok = (written == (size_t)contentLen) && Update.end();

    http.end();

    if (!ok || !Update.isFinished()) {
      Serial.printf("Update failed err=%d", Update.getError());
      return false;
    }

    Serial.println("Update success, rebooting...");
    delay(500);
    ESP.restart();
    return true; // not reached
  }

  void checkAndUpdate() {
    String latestVer, binUrl;
    if (!fetchManifest(latestVer, binUrl)) { Serial.println("Manifest fetch failed"); return; }
    Serial.printf("Current: %s | Latest: %s ", FW_VERSION, latestVer.c_str());
    if (latestVer == FW_VERSION) { Serial.println("Already up to date"); return; }
    Serial.printf("Updating from %s -> %s", FW_VERSION, latestVer.c_str());
    if (!doHttpUpdate(binUrl)) Serial.println("OTA failed");
  }
};

/**********************
 * WiFi Helper        *
 **********************/
class WifiHelper {
public:
  WifiHelper(Preferences& prefs) : preferences(prefs) {}

  bool autoConnect(uint32_t timeoutMs = 10000) {
    String ssid = preferences.getString(PREF_SSID, "");
    String pass = preferences.getString(PREF_PASS, "");
    if (ssid.isEmpty()) {
      Serial.println("No stored WiFi credentials.");
      return false;
    }
    Serial.printf("Found stored SSID='%s'", ssid.c_str());
    return connect(ssid, pass, timeoutMs, /*save*/false);
  }

  bool connectAndSave(const String& ssid, const String& pass, uint32_t timeoutMs = 10000) {
    bool ok = connect(ssid, pass, timeoutMs, /*save*/true);
    return ok;
  }

  bool isConnected() const { return WiFi.status() == WL_CONNECTED; }

private:
  bool connect(const String& ssid, const String& pass, uint32_t timeoutMs, bool save) {
    WiFi.begin(ssid.c_str(), pass.c_str());
    Serial.printf("Connecting to WiFi SSID='%s'", ssid.c_str());

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
      delay(500); Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("✅ WiFi connected! IP: %s", WiFi.localIP().toString().c_str());
      if (save) {
        preferences.putString(PREF_SSID, ssid);
        preferences.putString(PREF_PASS, pass);
        Serial.println("⚙️ Credentials saved to flash");
      }
      return true;
    }

    Serial.println("❌ WiFi connection failed");
    return false;
  }

  Preferences& preferences;
};

/**********************
 * Command Router     *
 **********************/
class CommandRouter {
public:
  CommandRouter(PinManager& pins, LEDController& leds, WifiHelper& wifi, OTAUpdater& ota)
  : pinMgr(pins), led(leds), wifi(wifi), ota(ota) {}

  void setBleNotifier(std::function<void(const String&)> fn) { bleNotify = fn; }
  void setWsStarter(std::function<void()> fn) { startWebSocket = fn; }

  // Handle raw BLE ASCII commands
  void handleRawCommand(String value) {
    value.trim();

    // Provision WiFi: WIFI:<SSID>;<PASSWORD>
    if (value.startsWith("WIFI:")) {
      String creds = value.substring(5);
      int sep = creds.indexOf(';');
      if (sep > 0) {
        String ssid = creds.substring(0, sep);
        String pass = creds.substring(sep + 1);
        Serial.printf("⏳ Connecting to WiFi SSID='%s'", ssid.c_str());
        if (wifi.connectAndSave(ssid, pass)) {
          if (bleNotify) bleNotify("WIFI_OK");
          if (startWebSocket) startWebSocket();
          ota.checkAndUpdate();
        } else {
          if (bleNotify) bleNotify("WIFI_FAIL");
        }
      } else {
        Serial.println("❗ Malformed WIFI command. Use WIFI:<SSID>;<PASSWORD>");
      }
      return;
    }

    // COLOR:r,g,b;BRIGHTNESS:x
    int cIdx = value.indexOf("COLOR:");
    int bIdx = value.indexOf("BRIGHTNESS:");
    if (cIdx >= 0 && bIdx >= 0) {
      String cStr = value.substring(cIdx + 6, value.indexOf(';', cIdx));
      int r = 0, g = 0, b = 0; sscanf(cStr.c_str(), "%d,%d,%d", &r, &g, &b);
      int brightness = value.substring(bIdx + 11).toInt();
      handleColor(r, g, b, brightness);
      return;
    }

    // PIN:<pin>;STATUS:toggle
    int pIdx = value.indexOf("PIN:");
    int sIdx = value.indexOf("STATUS:");
    if (pIdx >= 0 && sIdx > pIdx) {
      int pin = value.substring(pIdx + 4, value.indexOf(';', pIdx)).toInt();
      String status = value.substring(sIdx + 7); status.trim();
      if (PinManager::isManagedPin(pin) && status == "toggle") { handleToggle(pin); return; }
      Serial.println("Invalid pin or command.");
      return;
    }

    if (value.startsWith("REST:")) {
      const String csv = pinMgr.pinsCsv(false);
      if (bleNotify) {
        bleNotify(csv);
      }
      Serial.println(csv);
      return;
    }

    Serial.println("Malformed command.");
  }

  // From WebSocket JSON or parsed BLE
  void handleToggle(int pin) {
    pinMgr.toggle(pin);
  }

  void handleColor(int r, int g, int b, int brightness) {
    led.setAll(r, g, b, brightness);
    Serial.printf("LEDs: color(%d,%d,%d), brightness %d", r, g, b, brightness);
  }

private:
  PinManager& pinMgr;
  LEDController& led;
  WifiHelper& wifi;
  OTAUpdater& ota;
  std::function<void(const String&)> bleNotify = nullptr;
  std::function<void()> startWebSocket = nullptr;
};

/**********************
 * WebSocket Wrapper  *
 **********************/
class WSClientWrapper {
public:
  WSClientWrapper(CommandRouter* router) : router(router) {}

  void begin(const String& macQuery) {
    String path = "/?mac=" + macQuery;
    ws.beginSSL(WEBSOCKET_HOST, WEBSOCKET_PORT, path.c_str());
    ws.onEvent([this](WStype_t t, uint8_t* p, size_t l){ this->onEvent(t, p, l); });
    ws.setReconnectInterval(5000);
    lastPongTime = millis();
  }

  void loop() {
    ws.loop();

    // Periodic ping
    if (millis() - lastPing >= 20000) {
      lastPing = millis();
      ws.sendTXT("ping");
    }

    // Timeout handling
    if (!pongTimeoutFired && millis() - lastPongTime > 60000) {
      pongTimeoutFired = true;
      Serial.println("Haven't received pong in over 60 seconds!");
      digitalWrite(5, HIGH); // visible indicator
    }
  }

private:
  void onEvent(WStype_t type, uint8_t* payload, size_t length) {
    switch (type) {
      case WStype_CONNECTED:
        Serial.println("WebSocket connected!");
        break;
      case WStype_DISCONNECTED:
        Serial.println("WebSocket disconnected.");
        break;
      case WStype_TEXT: {
        String msg = String((const char*)payload);
        msg.trim();
        if (msg == "pong") {
          lastPongTime = millis();
          pongTimeoutFired = false;
          break;
        }

        Serial.println("Raw WebSocket message: " + msg);

        StaticJsonDocument<384> doc;
        DeserializationError err = deserializeJson(doc, msg);
        if (err) { Serial.printf("JSON parse error: %s", err.c_str()); return; }

        JsonObject cmd = doc.containsKey("data") ? doc["data"].as<JsonObject>() : doc.as<JsonObject>();
        handleJson(cmd);
        break;
      }
      default:
        break;
    }
  }

  void handleJson(JsonObject cmd) {
    if (!cmd.containsKey("cmd")) return;

    const char* c = cmd["cmd"] | "";
    if (!strcmp(c, "toggle")) {
      int pin = cmd["pin"] | -1;
      if (router) router->handleToggle(pin);
    } else if (!strcmp(c, "color")) {
      int r = cmd["color"][0] | 0;
      int g = cmd["color"][1] | 0;
      int b = cmd["color"][2] | 0;
      int brightness = cmd["brightness"] | 128;
      if (router) router->handleColor(r, g, b, brightness);
    }
  }

private:
  WebSocketsClient ws;
  CommandRouter* router;
  unsigned long lastPing = 0;
  unsigned long lastPongTime = 0;
  bool pongTimeoutFired = false;
};

/**********************
 * BLE Server         *
 **********************/
class BLEServerHandler : public NimBLECharacteristicCallbacks, public NimBLEServerCallbacks {
public:
  BLEServerHandler(CommandRouter* router) : router(router) {}

  void begin() {
    NimBLEDevice::init("ESP32_BLE_WS");
    bleMac = String(NimBLEDevice::getAddress().toString().c_str());

    NimBLEServer* server = NimBLEDevice::createServer();
    server->setCallbacks(this);
    server->advertiseOnDisconnect(true);

    NimBLEService* svc = server->createService(SERVICE_UUID);
    pCharacteristic = svc->createCharacteristic(CHARACTERISTIC_UUID,
                    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE |
                    NIMBLE_PROPERTY::WRITE_NR | NIMBLE_PROPERTY::NOTIFY);
    pCharacteristic->setCallbacks(this);
    pCharacteristic->setValue("Ready");

    svc->start();

    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID(SERVICE_UUID);
    adv->start();

    Serial.println("BLE advertising as ESP32_BLE_WS.");
  }

  String mac() const { return bleMac; }

  // --- NimBLECharacteristicCallbacks ---
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& info) override {
    String value = String(c->getValue().c_str());
    Serial.print("BLE Received: "); Serial.println(value);
    if (router) router->handleRawCommand(value);
  }

  // --- NimBLEServerCallbacks ---
  void onConnect(NimBLEServer* server, NimBLEConnInfo& connInfo) override {
    server->updateConnParams(connInfo.getConnHandle(), 12, 12, 0, 20);
  }

  void onDisconnect(NimBLEServer* server, NimBLEConnInfo& connInfo, int reason) override {
    Serial.printf("Client disconnected (reason %d)", reason);
    server->getAdvertising()->start();
  }

  void notify(const String& msg) {
    if (!pCharacteristic) return;
    pCharacteristic->setValue(msg.c_str());
    pCharacteristic->notify();
  }

private:
  NimBLECharacteristic* pCharacteristic = nullptr;
  String bleMac;
  CommandRouter* router;
};

/**********************
 * App Orchestrator   *
 **********************/
class ReflektApp {
public:
  ReflektApp()
    : pins(prefs, OUTPUT_PINS, 7),
      router(pins, leds, wifi, ota),
      ws(&router),
      ble(&router) {}

  void setup() {
    Serial.begin(115200);

    prefs.begin(PREF_NAMESPACE, false);
    Serial.println("PIN setup");
    // Pins state
    pins.begin();
    Serial.println("LED Setup");
    // LEDs
    leds.begin(128);

    Serial.println("Touch Setup");
    // Touch (disabled by default; pass true to enable)
    touch.begin(false);


    // BLE (must precede WebSocket so we have MAC)
    ble.begin();


    // Wire router -> BLE notify
    router.setBleNotifier([this](const String& s){ ble.notify(s); });

    // Wire router -> WebSocket starter
    router.setWsStarter([this]() { startWebSocket(); });

    // Attempt auto WiFi, then WS + OTA
    if (wifi.autoConnect()) {
      startWebSocket();
      ota.checkAndUpdate();
    }
  }

  void loop() {
    ws.loop();

    // Optional touch → toggle
    int touched = touch.readBulb();
    if (touched >= 1 && touched <= 7) {
      int pin = OUTPUT_PINS[touched - 1];
      router.handleToggle(pin);
      Serial.printf("Toggled Bulb %d (pin %d)", touched, pin);
      delay(300); // debounce
    }

    delay(50);
  }

private:
  void startWebSocket() {
    if (wsStarted) return;
    ws.begin(ble.mac());
    wsStarted = true;
  }

private:
  Preferences prefs;
  LEDController leds{PIN_NEOPIXEL, NUM_LEDS};
  PinManager pins;
  TouchPanel touch;
  OTAUpdater ota;
  WifiHelper wifi{prefs};
  CommandRouter router;
  WSClientWrapper ws;
  BLEServerHandler ble;
  bool wsStarted = false;
};

/**********************
 * Global App         *
 **********************/
ReflektApp APP;

void setup() { APP.setup(); }
void loop()  { APP.loop();  }
