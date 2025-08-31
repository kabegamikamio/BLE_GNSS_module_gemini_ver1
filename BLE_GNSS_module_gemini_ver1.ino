#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>

// --- LNSの基本UUID + Control Point ---
#define SERVICE_UUID_LNS                   "00001819-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID_LN_FEATURE       "00002a6a-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID_LOCATION_SPEED   "00002a67-0000-1000-8000-00805f9b34fb"
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
// ★★★ 変更点①：UTC時刻フィールドを追加した新しい構造体を定義 ★★★
struct LocationAndSpeed_WithTime {
    uint16_t flags;
    uint16_t instantaneousSpeed;
    int32_t  latitude;
    int32_t  longitude;
    uint16_t  year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  second;
};
struct PositionQuality_Simple {
    uint16_t flags;
    uint8_t  numberOfSatellites;
    uint16_t horizontalDOP;
};
#pragma pack(pop)

// --- GPS設定 (ユーザー設定を反映) ---
String nameBLE = "KawaiiMyGNSS";
#define RX_PIN 23
#define TX_PIN 22
#define GPS_BAUD 57600 // ユーザー指定のボーレート
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

// -----------------------------------------------------------------------------
// setup()
// -----------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    Serial.println("\n🚀 Starting BLE LNS GPS (UTC Time Enabled)...");
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
    Serial.printf("🛰️  GPS Serial started at %d baud on RX:%d, TX:%d.\n", GPS_BAUD, RX_PIN, TX_PIN);

    BLEDevice::init(nameBLE);
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    BLEService *pLnsService = pServer->createService(SERVICE_UUID_LNS);

    // 1. LN Feature (最小構成)
    pLnFeatureCharacteristic = pLnsService->createCharacteristic(CHARACTERISTIC_UUID_LN_FEATURE, BLECharacteristic::PROPERTY_READ);
    // UTC Time Supported (bit 6) を有効にする
    const uint32_t ln_feature = 0b0000000001000101;
    pLnFeatureCharacteristic->setValue((uint8_t*)&ln_feature, 4);

    // 2. Location and Speed (Notify)
    pLocationSpeedCharacteristic = pLnsService->createCharacteristic(CHARACTERISTIC_UUID_LOCATION_SPEED, BLECharacteristic::PROPERTY_NOTIFY);
    BLE2902* p2902_loc = new BLE2902();
    p2902_loc->setNotifications(true);
    pLocationSpeedCharacteristic->addDescriptor(p2902_loc);

    // 3. Position Quality (Notify) - 今回は使用しないためコメントアウト
    // pPositionQualityCharacteristic = pLnsService->createCharacteristic(CHARACTERISTIC_UUID_POSITION_QUALITY, BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
    // BLE2902* p2902_pos = new BLE2902();
    // p2902_pos->setNotifications(true);
    // pPositionQualityCharacteristic->addDescriptor(p2902_pos);

    // BLEアドバタイズを開始    
    pLnsService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID_LNS);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  // 希望の最小通信間隔を1.25 * 6 = 7.5msに設定
    pAdvertising->setMinPreferred(0x0C);  // 希望の最小通信間隔を1.25 * 12 = 15msに設定
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

    // ユーザー設定に合わせて10Hz (100ms) に変更
    if (millis() - lastGpsCheck > 50) {
        lastGpsCheck = millis();

        // 接続中で、かつ位置情報と時刻情報の両方が有効な場合のみ送信
        if (gps.location.isValid() && gps.time.isValid() && deviceConnected) {
            Serial.printf("\n🛰️  GPS Fix! LAT: %f, LNG: %f, DATE: %04d/%02d/%02d TIME: %02d:%02d:%02d", 
                gps.location.lat(), 
                gps.location.lng(),
                gps.date.year(),
                gps.date.month(),
                gps.date.day(),
                gps.time.hour(),
                gps.time.minute(),
                gps.time.second()
            );

            // ★★★ 変更点②：新しい構造体を使ってデータを準備 ★★★
            LocationAndSpeed_WithTime loc_data = {0};

            // ★★★ 変更点③：FlagsにUTC Time Present (bit 6) を追加 ★★★
            // Bit 0: Instantaneous Speed Present
            // Bit 2: Location Present
            // Bit 6: UTC Time Present
            loc_data.flags = 0b0000000001000101; // 0x0045

            loc_data.instantaneousSpeed = (uint16_t)(gps.speed.mps() * 100);
            loc_data.latitude = (int32_t)(gps.location.lat() * 10000000);
            loc_data.longitude = (int32_t)(gps.location.lng() * 10000000);
            
            // GPSから取得した月、日、時、分、秒を格納
            loc_data.year   = gps.date.year();
            loc_data.month  = gps.date.month();
            loc_data.day    = gps.date.day();
            loc_data.hour   = gps.time.hour();   // TinyGPS++の時刻はUTC
            loc_data.minute = gps.time.minute();
            loc_data.second = gps.time.second();

            Serial.printf("RAW: %x\n", (uint8_t*)&loc_data);

            // 新しい構造体のサイズでデータをセットしてNotify
            pLocationSpeedCharacteristic->setValue((uint8_t*)&loc_data, sizeof(loc_data));
            pLocationSpeedCharacteristic->notify();
            
            Serial.println(" -> ✅ Notified Location with UTC Time!");

        } else if (!gps.location.isValid()) {
            Serial.print(".");
        }
    }
}
