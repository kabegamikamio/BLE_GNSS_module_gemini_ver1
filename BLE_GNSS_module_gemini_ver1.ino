#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <EEPROM.h> // ★★★ 修正点①：EEPROMライブラリをインクルード ★★★

// --- UUIDs ---
#define SERVICE_UUID_LNS                   "00001819-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID_LN_FEATURE       "00002a6a-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID_LOCATION_SPEED   "00002a67-0000-1000-8000-00805f9b34fb"
#define SERVICE_UUID_CONFIG                "c48e6067-5295-48d3-8d5c-0395f61792b1"
#define CHARACTERISTIC_UUID_SMA_WINDOW     "c48e6068-5295-48d3-8d5c-0395f61792b1"

// --- EEPROM Settings ---
#define EEPROM_SIZE 4 // 保存に必要なサイズ
#define ADDR_SMA_SIZE 0 // 保存先アドレス

// --- グローバル設定変数 ---
bool useGpsSpeedSource = true; // true: gps.speed, false: 手動計算
bool enableSpeedSmoothing = true;
int smaWindowSize = 5; // 移動平均のサンプル数（デフォルト値）

// --- 速度計算用のグローバル変数 ---
double lastLat = 0.0, lastLng = 0.0;
unsigned long lastSpeedCalcMillis = 0;
float calculatedSpeedMps = 0.0;
float smoothedSpeedMps = 0.0;
float* speedSamples = nullptr;
int currentSampleIndex = 0;
const int MAX_SMA_WINDOW_SIZE = 50;

BLECharacteristic *pLocationSpeedCharacteristic;
BLECharacteristic *pSmaWindowCharacteristic; // キャラクタリスティックへのポインタをグローバル化
bool deviceConnected = false;
unsigned long lastGpsCheck = 0;

// --- 関数プロトタイプ宣言 ---
void setSmaWindowSize(int newSize);

// --- Callbacks ---
class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { deviceConnected = true; Serial.println("\n✅ Central device connected!"); }
    void onDisconnect(BLEServer* pServer) { deviceConnected = false; Serial.println("❌ Central device disconnected."); pServer->getAdvertising()->start(); }
};

class SmaConfigCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        String value = pCharacteristic->getValue();
        if (value.length() == 1) {
            int newSize = (int)value[0];
            Serial.printf("Received new SMA size via BLE: %d\n", newSize);
            setSmaWindowSize(newSize);
            // BLEからの書き込みをキャラクタリスティックのRead値にも反映
            pCharacteristic->setValue(String(smaWindowSize).c_str());
        }
    }
};

// ★★★ 修正点②：構造体の定義を正しく記述 ★★★
#pragma pack(push, 1)
struct LocationAndSpeed_19Byte {
    uint16_t flags;
    uint16_t instantaneousSpeed;
    int32_t  latitude;
    int32_t  longitude;
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  second;
};
#pragma pack(pop)

// --- GPS Settings ---
String nameBLE = "KawaiiMyGNSS";
#define RX_PIN 23
#define TX_PIN 22
#define GPS_BAUD 57600
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

// --- 関数定義 ---
void setSmaWindowSize(int newSize) {
    if (newSize < 0) return;

    if (newSize > MAX_SMA_WINDOW_SIZE) {
        newSize = MAX_SMA_WINDOW_SIZE;
        Serial.printf("Requested size is too large. Capping at %d\n", MAX_SMA_WINDOW_SIZE);
    }

    if ((newSize == 0 && !enableSpeedSmoothing) || (newSize == smaWindowSize && enableSpeedSmoothing && newSize != 0)) {
        return;
    }

    if (newSize == 0) {
        enableSpeedSmoothing = false;
        if (speedSamples != nullptr) {
            delete[] speedSamples;
            speedSamples = nullptr;
        }
        Serial.println("Speed smoothing DISABLED.");
    } else {
        enableSpeedSmoothing = true;
        if (newSize != smaWindowSize || speedSamples == nullptr) {
            if (speedSamples != nullptr) {
                delete[] speedSamples;
            }
            speedSamples = new float[newSize];
            for (int i = 0; i < newSize; i++) {
                speedSamples[i] = 0.0;
            }
            currentSampleIndex = 0;
            smoothedSpeedMps = 0.0;
        }
        Serial.printf("Speed smoothing ENABLED with window size: %d\n", newSize);
    }
    
    smaWindowSize = newSize;

    EEPROM.write(ADDR_SMA_SIZE, smaWindowSize);
    EEPROM.commit();
    Serial.println("Saved new setting to EEPROM.");
}

void updateCalculatedSpeed() {
    if (gps.location.isValid() && gps.location.isUpdated()) {
        if (lastLat != 0.0 && lastLng != 0.0) {
            double distanceMeters = TinyGPSPlus::distanceBetween(
                gps.location.lat(), gps.location.lng(), lastLat, lastLng);
            unsigned long currentMillis = millis();
            double timeSeconds = (double)(currentMillis - lastSpeedCalcMillis) / 1000.0;
            if (timeSeconds > 0) {
                calculatedSpeedMps = distanceMeters / timeSeconds;
            }
        }
        lastLat = gps.location.lat();
        lastLng = gps.location.lng();
        lastSpeedCalcMillis = millis();

        if(enableSpeedSmoothing && speedSamples != nullptr && smaWindowSize > 0) {
            speedSamples[currentSampleIndex] = calculatedSpeedMps;
            currentSampleIndex = (currentSampleIndex + 1) % smaWindowSize;

            float sum = 0.0;
            for (int i = 0; i < smaWindowSize; i++) {
                sum += speedSamples[i];
            }
            smoothedSpeedMps = sum / smaWindowSize;
        }
    }
}

// --- setup() ---
void setup() {
    Serial.begin(115200);
    
    EEPROM.begin(EEPROM_SIZE);
    int storedSize = EEPROM.read(ADDR_SMA_SIZE);
    
    if (storedSize < 0 || storedSize > MAX_SMA_WINDOW_SIZE) {
        Serial.println("No valid SMA size in EEPROM. Using default.");
        smaWindowSize = 7;
    } else {
        smaWindowSize = storedSize;
        Serial.printf("Loaded SMA size from EEPROM: %d\n", smaWindowSize);
    }
    
    setSmaWindowSize(smaWindowSize); 

    Serial.println("\n🚀 Starting BLE LNS GPS (Persistent Config)...");
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
    Serial.printf("🛰️  GPS Serial started at %d baud on RX:%d, TX:%d.\n", GPS_BAUD, RX_PIN, TX_PIN);

    BLEDevice::init(nameBLE.c_str());
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    
    // LNS Service
    BLEService *pLnsService = pServer->createService(SERVICE_UUID_LNS);
    BLECharacteristic *pLnFeatureCharacteristic = pLnsService->createCharacteristic(CHARACTERISTIC_UUID_LN_FEATURE, BLECharacteristic::PROPERTY_READ);
    const uint32_t ln_feature = 0b0000000001000101;
    pLnFeatureCharacteristic->setValue((uint8_t*)&ln_feature, 4);
    pLocationSpeedCharacteristic = pLnsService->createCharacteristic(CHARACTERISTIC_UUID_LOCATION_SPEED, BLECharacteristic::PROPERTY_NOTIFY);
    pLocationSpeedCharacteristic->addDescriptor(new BLE2902());
    pLnsService->start();
    
    // Config Service
    BLEService *pConfigService = pServer->createService(SERVICE_UUID_CONFIG);
    pSmaWindowCharacteristic = pConfigService->createCharacteristic(
        CHARACTERISTIC_UUID_SMA_WINDOW,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
    );
    pSmaWindowCharacteristic->setCallbacks(new SmaConfigCallbacks());
    pSmaWindowCharacteristic->setValue(String(smaWindowSize).c_str());
    pConfigService->start();
    
    // アドバタイジングの設定
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID_LNS);
    pAdvertising->addServiceUUID(SERVICE_UUID_CONFIG);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMaxPreferred(0x0C);
    BLEDevice::startAdvertising();
    Serial.printf("📡 BLE advertising started as '%s'.\n", nameBLE.c_str());
}

// --- loop() ---
void loop() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        int newSize = input.toInt();
        if (String(newSize) == input) {
            Serial.printf("Received new SMA size via Serial: %d\n", newSize);
            setSmaWindowSize(newSize);
            if (pSmaWindowCharacteristic) {
                pSmaWindowCharacteristic->setValue(String(smaWindowSize).c_str());
            }
        }
    }

    while (gpsSerial.available() > 0) {
        if (gps.encode(gpsSerial.read())) {
            if (!useGpsSpeedSource) {
                updateCalculatedSpeed();
            }
        }
    }

    if (millis() - lastGpsCheck > 100) { // 10Hz
        lastGpsCheck = millis();

        if (gps.location.isValid() && gps.time.isValid() && deviceConnected) {
            LocationAndSpeed_19Byte loc_data;
            memset(&loc_data, 0, sizeof(loc_data));

            loc_data.flags = 0b0000000001000101; 

            float speedKmph = 0.0;
            if (useGpsSpeedSource) {
                speedKmph = gps.speed.kmph();
            } else {
                if (enableSpeedSmoothing) {
                    speedKmph = smoothedSpeedMps * 3.6; // 3.6倍する必要があるのか…?
                } else {
                    speedKmph = calculatedSpeedMps * 3.6;
                }
            }
            
            loc_data.instantaneousSpeed = (uint16_t)(speedKmph * 100.0);
            loc_data.latitude = (int32_t)(gps.location.lat() * 10000000);
            loc_data.longitude = (int32_t)(gps.location.lng() * 10000000);
            
            loc_data.year   = gps.date.year();
            loc_data.month  = gps.date.month();
            loc_data.day    = gps.date.day();
            loc_data.hour   = gps.time.hour();
            loc_data.minute = gps.time.minute();
            loc_data.second = gps.time.second();

            pLocationSpeedCharacteristic->setValue((uint8_t*)&loc_data, sizeof(loc_data));
            pLocationSpeedCharacteristic->notify();
            
            Serial.printf("0,80,%.02f\n", speedKmph);
        }
    }
}

