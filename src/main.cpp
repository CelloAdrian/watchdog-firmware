#include <Arduino.h>
#include <ArduinoJson.h>
#include "arduinoFFT.h"
#include <BLEAdvertising.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <Preferences.h>
#include <WiFi.h>

#define SAMPLES 512 // has to be a power of 2
#define SAMPLING_FREQ 20000 // should be atleast twice the max freq we're going to observe
#define AMPLITUDE 200

unsigned int sampling_period_us;
unsigned long microseconds;

byte peak[] = {0, 0, 0, 0, 0, 0, 0};

double vReal[SAMPLES];
double vImaginary[SAMPLES];
unsigned long newTime, oldTime;

arduinoFFT FFT = arduinoFFT();

const int pirPin = 33;
const int buzzer = 32;
const int mic = 35;

const int freq = 500;

int val = 0;
bool motionState = false;

#define SERVICE_UUID "3881cf3d-0bc9-41fa-8fb3-86606bbbfac7"
#define WIFI_UUID "ed3f271a-d44e-42fe-87b7-32547071fe1d"

char apName[] = "WDALRM-xxxxxxxxxxxx";

bool hasCredentials = false;
bool isConnected = false;
bool connectionStatusChanged = false;

String SSID;
String Password;

BLECharacteristic *pCharacteristicWifi;
BLEAdvertising *pAdvertising;
BLEService *pService;
BLEServer *pServer;

// Max size is 51 bytes for frame : {"SSID":"","Password":""} + 64 bytes (115 bytes)
StaticJsonBuffer<150> jsonBuffer;

class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *pServer) {
        Serial.println("BLE client connected");
    };

    void onDisconnect(BLEServer *pServer) {
        Serial.println("BLE client disconnected");
        pAdvertising->start();
    }
};

class CallbackHandler : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        if (value.length() == 0) {
            return;
        }
        Serial.println("Received over BLE: " + String((char *)&value[0]));

        // Decode data
        int keyIndex = 0;
        for (int index = 0; index < value.length(); index++) {
            value[index] = (char)value[index] ^ (char)apName[keyIndex];
            keyIndex++;
            if (keyIndex >= strlen(apName)) keyIndex = 0;
        }

        /** Json object for incoming data */
        JsonObject &jsonIn = jsonBuffer.parseObject((char *)&value[0]);
        if (jsonIn.success()) {
            if (jsonIn.containsKey("SSID") &&
                jsonIn.containsKey("Password")) {
                SSID = jsonIn["SSID"].as<String>();
                Password = jsonIn["Password"].as<String>();

                Preferences preferences;
                preferences.begin("WiFiCred", false);
                preferences.putString("SSID", SSID);
                preferences.putString("Password", Password);
                preferences.putBool("valid", true);
                preferences.end();

                Serial.println("Received over bluetooth:");
                Serial.println("primary SSID: " + SSID + " password: " + Password);
                connectionStatusChanged = true;
                hasCredentials = true;
            } else if (jsonIn.containsKey("erase")) {
                Serial.println("Received erase command");
                Preferences preferences;
                preferences.begin("WiFiCred", false);
                preferences.clear();
                preferences.end();
                connectionStatusChanged = true;
                hasCredentials = false;
                SSID = "";
                Password = "";

                // int err;
                // err = nvs_flash_init();
                // Serial.println("nvs_flash_init: " + err);
                // err = nvs_flash_erase();
                // Serial.println("nvs_flash_erase: " + err);
            } else if (jsonIn.containsKey("reset")) {
                WiFi.disconnect();
                esp_restart();
            }
        } else {
            Serial.println("Received invalid JSON");
        }
        jsonBuffer.clear();
    };

    void onRead(BLECharacteristic *pCharacteristic) {
        Serial.println("BLE onRead request");
        String wifiCredentials;

        /** Json object for outgoing data */
        JsonObject &jsonOut = jsonBuffer.createObject();
        jsonOut["SSID"] = SSID;
        jsonOut["Password"] = Password;
        // Convert JSON object into a string
        jsonOut.printTo(wifiCredentials);

        // encode the data
        int keyIndex = 0;
        Serial.println("Stored settings: " + wifiCredentials);
        for (int index = 0; index < wifiCredentials.length(); index++) {
            wifiCredentials[index] = (char)wifiCredentials[index] ^ (char)apName[keyIndex];
            keyIndex++;
            if (keyIndex >= strlen(apName)) keyIndex = 0;
        }
        pCharacteristicWifi->setValue((uint8_t *)&wifiCredentials[0], wifiCredentials.length());
        jsonBuffer.clear();
    }
};

void initBLE() {
    BLEDevice::init(apName);
    BLEDevice::setPower(ESP_PWR_LVL_P7);

    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());
    pService = pServer->createService(BLEUUID(SERVICE_UUID), 20);

    pCharacteristicWifi = pService->createCharacteristic(
        BLEUUID(WIFI_UUID),
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
    pCharacteristicWifi->setCallbacks(new CallbackHandler());
    pService->start();

    pAdvertising = pServer->getAdvertising();
    pAdvertising->start();
}

void gotIP(system_event_id_t event) {
    isConnected = true;
    connectionStatusChanged = true;
}

void lostConnection(system_event_id_t event) {
    isConnected = false;
    connectionStatusChanged = true;
}

bool scanWifi() {
    int8_t RSSI_PRIM;

    Serial.println("Scanning for networks");

    WiFi.disconnect(true);
    WiFi.enableSTA(true);
    WiFi.mode(WIFI_STA);

    int apNum = WiFi.scanNetworks(false, true, false, 1000);

    if (apNum == 0) {
        Serial.println("Found no networks.");
        return false;
    }

    for (int i = 0; i < apNum; i++) {
        String SSID = WiFi.SSID(i);
        Serial.println("Found AP: " + SSID + " RSSI: " + WiFi.RSSI(i));
        if (!strcmp((const char *)&SSID[0], (const char *)&SSID[0])) {
            RSSI_PRIM = WiFi.RSSI(i);
        }
    }
}

void connectWifi() {
    WiFi.onEvent(gotIP, SYSTEM_EVENT_STA_GOT_IP);
    WiFi.onEvent(lostConnection, SYSTEM_EVENT_STA_DISCONNECTED);

    WiFi.disconnect(true);
    WiFi.enableSTA(true);
    WiFi.mode(WIFI_STA);

    Serial.println();
    WiFi.begin(SSID.c_str(), Password.c_str());
}

void setup() {
    uint8_t baseMac[6];
    esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
    sprintf(apName, "watchdog-%02X%02X%02X%02X%02X%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);

    setCpuFrequencyMhz(80);
    Serial.begin(115200);
    Serial.println("Booting...");

    pinMode(pirPin, INPUT);

    ledcSetup(0, freq, 8);
    ledcAttachPin(buzzer, 0);

    Preferences pref;
    pref.begin("WiFiCred", false);
    bool hasPref = pref.getBool("valid", false);
    if (hasPref) {
        SSID = pref.getString("SSID", "");
        Password = pref.getString("Password", "");

        if (SSID.equals("") || Password.equals("")) {
            Serial.println("Found preferences but credentials are invalid");
        } else {
            Serial.println("Read from preferences: ");
            Serial.println("SSID: " + SSID + "Password: " + Password);
            hasCredentials = true;
        }
    } else {
        Serial.println("Couldn't find preferences");
    }
    pref.end();

    initBLE();

    if (hasCredentials) {
        if (!scanWifi) {
            Serial.println("Couldn't find any AP");
        } else {
            connectWifi();
        }
    }
}

void loop() {
    val = digitalRead(pirPin);
    bool isArmed = false;

    if (connectionStatusChanged) {
        if (isConnected) {
            Serial.print("Connected to AP: ");
            Serial.print(WiFi.SSID());
            Serial.print(WiFi.localIP());
            Serial.print("RSSI: ");
            Serial.println(WiFi.RSSI());
        } else {
            if (hasCredentials) {
                Serial.println("Lost connection");

                if (!scanWifi) {
                    Serial.println("Could not find any AP");
                } else {
                    connectWifi();
                }
            }
        }
        connectionStatusChanged = false;
    }

    if (isArmed) {
      // MOTION SENSOR
      if (val == HIGH) {
        if (motionState == false) {
          Serial.println("Motion detected!");
          ledcWriteTone(0, freq);
          motionState = true;
        }
      } else {
        if (motionState == true) {
          Serial.println("Motion ended");
          motionState = false;
        }
      }

      // GLASSBREAK SENSOR
      for (int i = 0; i < SAMPLES; i++) {
        newTime = micros() - oldTime;
        oldTime = newTime;

        vReal[i] = analogRead(mic);
        vImaginary[i] = 0;

        while (micros() < (newTime + sampling_period_us)) {
          // do nothing to wait
        }
      }

      FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
      FFT.Compute(vReal, vImaginary, SAMPLES, FFT_FORWARD); 
      FFT.ComplexToMagnitude(vReal, vImaginary, SAMPLES);

      for (int i = 0; i < (SAMPLES / 2); i++) {
        if (vReal[i] > 2000) {
          /*
            SAMPLING_FREQ / SAMPLES
            20000 / 512 = 39.0625 Hz / bin

            4 kHz - 7 kHz is the most common frequency that glass produces when it breaks
            To get to 4 kHz:
            4000 / 39.0625 = 102.4

            To get to 7 kHz:
            7000 / 39.0625 = 179.2
          */
         if (i >= 102 && i <= 179) {
           ledcWriteTone(0, freq);
         }
        }
      }

    } else {

    }
}