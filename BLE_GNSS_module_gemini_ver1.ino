#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>

// --- LNSの基本UUID + Control Point ---
#define SERVICE_UUID_LNS                    "00001819-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID_LN_FEATURE      "00002a6a-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID_LOCATION_SPEED  "00002a67-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID_POSITION_QUALITY "00002a69-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID_LN_CONTROL_POINT "00002a6b-0000-1000-8000-00805f9b34fb"

// --- キャラクタリスティックのポインタ ---
BLECharacteristic *pLnFeatureCharacteristic;
BLECharacteristic *pLocationSpeedCharacteristic;
BLECharacteristic *pPositionQualityCharacteristic;
BLECharacteristic *pLnControlPointCharacteristic;
bool deviceConnected = false;
unsigned long lastGpsCheck = 0;

// --- 接続・切断コールバック ---
class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { deviceConnected = true; Serial.println("\n✅ Central device connected!"); }
    void onDisconnect(BLEServer* pServer) { deviceConnected = false; Serial.println("❌ Central device disconnected."); pServer->getAdvertising()->start(); }
};

// --- Control Point 書き込みコールバック ---
class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string value = pCharacteristic->getValue().c_str();
        if (value.length() > 0) {
            Serial.print("LN Control Point received: ");
            for (int i = 0; i < value.length(); i++) {
                Serial.printf("%02X ", value[i]);
            }
            Serial.println();
        }
    }
};

// --- データ構造体 (LNS基本仕様) ---
#pragma pack(push, 1)
struct LocationAndSpeed_Simple {
    uint16_t flags;
    uint16_t instantaneousSpeed;
    int32_t  latitude;
    int32_t  longitude;
};
struct PositionQuality_Simple {
    uint16_t flags;
    uint8_t  numberOfSatellites;
    uint16_t horizontalDOP;
};
#pragma pack(pop)

// --- GPS設定 ---
#define RX_PIN 10
#define TX_PIN 5
#define GPS_BAUD 115200 // ユーザー指定のボーレート
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

// -----------------------------------------------------------------------------
// setup()
// -----------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    Serial.println("\n🚀 Starting BLE LNS GPS (Name & Speed Test)...");
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
    Serial.printf("🛰️  GPS Serial started at %d baud.\n", GPS_BAUD);

    // ★★★ 対策①：デバイス名を「GL-770」に変更 ★★★
    BLEDevice::init("GL-770");
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    BLEService *pLnsService = pServer->createService(SERVICE_UUID_LNS);

    // 1. LN Feature (最小構成)
    pLnFeatureCharacteristic = pLnsService->createCharacteristic(CHARACTERISTIC_UUID_LN_FEATURE, BLECharacteristic::PROPERTY_READ);
    const uint16_t ln_feature = 0b0000000000000101;
    pLnFeatureCharacteristic->setValue((uint8_t*)&ln_feature, 2);

    // 2. Location and Speed (Notify)
    pLocationSpeedCharacteristic = pLnsService->createCharacteristic(CHARACTERISTIC_UUID_LOCATION_SPEED, BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
    BLE2902* p2902_loc = new BLE2902();
    p2902_loc->setNotifications(true);
    pLocationSpeedCharacteristic->addDescriptor(p2902_loc);

    // 3. Position Quality (Notify)
    pPositionQualityCharacteristic = pLnsService->createCharacteristic(CHARACTERISTIC_UUID_POSITION_QUALITY, BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
    BLE2902* p2902_pos = new BLE2902();
    p2902_pos->setNotifications(true);
    pPositionQualityCharacteristic->addDescriptor(p2902_pos);

    // 4. LN Control Point (Indicate)
    pLnControlPointCharacteristic = pLnsService->createCharacteristic(
                                      CHARACTERISTIC_UUID_LN_CONTROL_POINT,
                                      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_INDICATE
                                    );
    pLnControlPointCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
    BLE2902* p2902_cp = new BLE2902();
    p2902_cp->setIndications(true);
    pLnControlPointCharacteristic->addDescriptor(p2902_cp);

    pLnsService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID_LNS);
    pAdvertising->setScanResponse(true);
    BLEDevice::startAdvertising();
    Serial.println("📡 BLE advertising started as 'GL-770'.");
}

// -----------------------------------------------------------------------------
// loop()
// -----------------------------------------------------------------------------
void loop() {
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }

    // ★★★ 対策②：データ送信間隔を200ms (5Hz) に変更 ★★★
    if (millis() - lastGpsCheck > 200) {
        lastGpsCheck = millis();

        if (gps.location.isValid() && deviceConnected) {
            Serial.printf("\n🛰️  GPS Fix! LAT: %f, LNG: %f", gps.location.lat(), gps.location.lng());

            LocationAndSpeed_Simple loc_data = {0};
            loc_data.flags = 0b0000000000000101;
            loc_data.instantaneousSpeed = (uint16_t)(gps.speed.mps() * 100);
            loc_data.latitude = (int32_t)(gps.location.lat() * 10000000);
            loc_data.longitude = (int32_t)(gps.location.lng() * 10000000);
            pLocationSpeedCharacteristic->setValue((uint8_t*)&loc_data, sizeof(loc_data));
            pLocationSpeedCharacteristic->notify();

            PositionQuality_Simple quality_data = {0};
            quality_data.flags = 0b000000000000011;
            quality_data.numberOfSatellites = gps.satellites.value();
            quality_data.horizontalDOP = (uint16_t)(gps.hdop.value() / 10.0);
            pPositionQualityCharacteristic->setValue((uint8_t*)&quality_data, sizeof(quality_data));
            pPositionQualityCharacteristic->notify();

            Serial.println(" -> ✅ Notified Minimal Data!");
        } else if (!gps.location.isValid()) {
            Serial.print(".");
        }
    }
}