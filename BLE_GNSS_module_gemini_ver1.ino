#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <EEPROM.h>

// --- UUIDs ---
#define SERVICE_UUID_LNS "00001819-0000-1000-8000-00805f9b34fb"                    // 位置情報を送るのに使うサービス
#define CHARACTERISTIC_UUID_LN_FEATURE "00002a6a-0000-1000-8000-00805f9b34fb"      // LNSが持つフィールドの種類を表すキャラ
#define CHARACTERISTIC_UUID_LOCATION_SPEED "00002a67-0000-1000-8000-00805f9b34fb"  // 位置情報と速度(あと時刻)のキャラ
#define SERVICE_UUID_CONFIG "c48e6067-5295-48d3-8d5c-0395f61792b1"                 // ESP32やGNSSモジュールの設定に使うサービス
#define CHARACTERISTIC_UUID_SMA_WINDOW "c48e6068-5295-48d3-8d5c-0395f61792b1"      // 移動平均区間の設定用キャラ
#define CHARACTERISTIC_UUID_GNSS_RATE "c48e6069-5295-48d3-8d5c-0395f61792b1"       // GNSSの測位頻度設定用キャラ
#define CHARACTERISTIC_UUID_GNSS_BAUD "c48e606a-5295-48d3-8d5c-0395f61792b1"       // GNSSとの通信に使うボーレートの設定用キャラ

// --- EEPROM Settings ---
#define EEPROM_SIZE 16  // SMA(1) + Rate(1) + Baud(4) + reserved
#define ADDR_SMA_SIZE 0
#define ADDR_GNSS_RATE 1
#define ADDR_GNSS_BAUD 2  // Baud rate (uint32_t) starts at address 2

// --- グローバル設定変数 ---
bool useGpsSpeedSource = true;            // 速度のデータソース: 初期値はNMEAから取得する方式(推奨)
bool enableSpeedSmoothing = true;         // 速度の平滑化: 初期値は有効
int smaWindowSize = 5;                    // 平滑化の移動平均区間: 初期値は5
uint32_t currentGpsBaud = 57600;          // GNSSとの通信で使うボーレート: 初期値は57600
int currentGnssRateHz = 10;               // GNSSの測位頻度: 初期値は10Hz
unsigned long bleNotifyIntervalMs = 100;  // 位置情報の通知頻度(初期値は 1/10 s = 100ms)
const uint32_t validBaudRates[] = {       // 有効なボーレートの集合
  9600, 19200, 31250, 38400, 57600, 75880,
  115200, 230400, 250000, 460800
};

// --- 速度計算用のグローバル変数 ---
double lastLat = 0.0, lastLng = 0.0;
unsigned long lastSpeedCalcMillis = 0;
float calculatedSpeedKph = 0.0;
float smoothedSpeedKph = 0.0;
float *speedSamples = nullptr;
int currentSampleIndex = 0;
const int MAX_SMA_WINDOW_SIZE = 50;

// --- BLE ---
BLECharacteristic *pLocationSpeedCharacteristic;
BLECharacteristic *pSmaWindowCharacteristic;
BLECharacteristic *pGnssRateCharacteristic;
BLECharacteristic *pGnssBaudCharacteristic;
bool deviceConnected = false;
unsigned long lastGpsCheck = 0;

// LNSで送るパケットの構造体
#pragma pack(push, 1)
struct LocationAndSpeed_19Byte {
  uint16_t flags;
  uint16_t instantaneousSpeed;
  int32_t latitude;
  int32_t longitude;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
};
#pragma pack(pop)

// --- GPS Settings ---
String nameBLE = "KawaiiMyGNSS";
#define RX_PIN 23
#define TX_PIN 22
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);


// --- 関数プロトタイプ宣言 ---
void setSmaWindowSize(int newSize);
void sendUb(byte *ubloxCommand, size_t len);
bool setGnssRate(int newRateHz);
bool setGpsBaudRate(uint32_t newBaudRate);
void saveGnssConfig();

// --- BLE Callbacks ---
// 接続/切断時のコールバック
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
    Serial.println("\n✅ Central device connected!");
  }
  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    Serial.println("❌ Central device disconnected.");
    pServer->getAdvertising()->start();
  }
};

// 移動平均更新時のコールバック
class SmaConfigCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String value = pCharacteristic->getValue();
    if (value.length() == 1) {
      int newSize = (int)value[0];
      Serial.printf("Received new SMA size via BLE (raw byte): %d\n", newSize);
      setSmaWindowSize(newSize);
      pCharacteristic->setValue(String(smaWindowSize).c_str());
    }
  }
};

// 測位頻度とボーレート更新時のコールバック
class GnssConfigCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String value = pCharacteristic->getValue();
    uint32_t receivedValue = 0;

    if (value.length() > 0 && value.length() <= 4) {
      for (int i = 0; i < value.length(); i++) {
        receivedValue = (receivedValue << 8) | (uint8_t)value[i];
      }
    } else {
      Serial.println("Invalid data length received via BLE.");
      return;
    }

    if (pCharacteristic == pGnssRateCharacteristic) {
      if (setGnssRate((int)receivedValue)) {
        saveGnssConfig();
        pGnssRateCharacteristic->setValue(String(currentGnssRateHz).c_str());
      }
    } else if (pCharacteristic == pGnssBaudCharacteristic) {
      if (setGpsBaudRate(receivedValue)) {
        saveGnssConfig();
        pGnssBaudCharacteristic->setValue(String(currentGpsBaud).c_str());
      }
    }
  }
};

// --- 関数定義 ---
// GxRMC, GxVTGの速度を使わず、ESP32自前で速度を計算する関数(非推奨)
void updateCalculatedSpeed() {
  if (gps.location.isValid() && gps.location.isUpdated()) {
    if (lastLat != 0.0 && lastLng != 0.0) {
      double distanceMeters = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), lastLat, lastLng);
      unsigned long currentMillis = millis();
      double timeSeconds = (double)(currentMillis - lastSpeedCalcMillis) / 1000.0;
      if (timeSeconds > 0) calculatedSpeedKph = distanceMeters / timeSeconds * 3.6;  // 3.6をかけて[m/s]から[km/h]に変換
    }
    lastLat = gps.location.lat();
    lastLng = gps.location.lng();
    lastSpeedCalcMillis = millis();
  }
}

// 移動平均を使って速度を平滑化
void updateSmoothedSpeed(float rawSpeedKph) {
  if (enableSpeedSmoothing && speedSamples != nullptr && smaWindowSize > 0) {
    speedSamples[currentSampleIndex] = rawSpeedKph;
    currentSampleIndex = (currentSampleIndex + 1) % smaWindowSize;
    float sum = 0.0;
    for (int i = 0; i < smaWindowSize; i++) {
      sum += speedSamples[i];
    }
    smoothedSpeedKph = sum / smaWindowSize;
  }
}

// 移動平均の区間の設定関数
void setSmaWindowSize(int newSize) {
  if (newSize < 0) return;
  if (newSize > MAX_SMA_WINDOW_SIZE) newSize = MAX_SMA_WINDOW_SIZE;
  if ((newSize == 0 && !enableSpeedSmoothing) || (newSize == smaWindowSize && enableSpeedSmoothing && newSize != 0)) return;

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
      if (speedSamples != nullptr) delete[] speedSamples;
      speedSamples = new float[newSize];
      for (int i = 0; i < newSize; i++) speedSamples[i] = 0.0;
      currentSampleIndex = 0;
      smoothedSpeedKph = 0.0;
    }
    Serial.printf("Speed smoothing ENABLED with window size: %d\n", newSize);
  }
  smaWindowSize = newSize;
  EEPROM.write(ADDR_SMA_SIZE, smaWindowSize);
  EEPROM.commit();
  Serial.println("Saved new SMA setting to EEPROM.");
}

// GNSSモジュールに設定コマンドを送る
void sendUb(byte *ubloxCommand, size_t len) {
  byte ck_a = 0, ck_b = 0;
  for (size_t i = 2; i < len - 2; i++) {
    ck_a = ck_a + ubloxCommand[i];
    ck_b = ck_b + ck_a;
  }
  ubloxCommand[len - 2] = ck_a;
  ubloxCommand[len - 1] = ck_b;

  Serial.print("Sending UBX Command: ");
  for (size_t i = 0; i < len; i++) {
    gpsSerial.write(ubloxCommand[i]);
    Serial.printf("%02X ", ubloxCommand[i]);
  }
  Serial.println();
}

// 測位頻度を変える
bool setGnssRate(int newRateHz) {
  // 有効な測位頻度は 1 ~ 25Hz
  if (newRateHz < 1 || newRateHz > 25) {
    Serial.printf("Invalid rate: %d Hz. Must be between 1 and 25.\n", newRateHz);
    return false;
  }

  // 新しい測位周期を計算
  bleNotifyIntervalMs = 1000 / newRateHz;
  Serial.printf("Setting measurement rate to %d Hz (%d ms)...\n", newRateHz, bleNotifyIntervalMs);

  // 測定頻度、周期を更新、EEPROMにも格納
  currentGnssRateHz = newRateHz;
  EEPROM.write(ADDR_GNSS_RATE, (uint8_t)currentGnssRateHz);
  EEPROM.commit();

  // UBXフォーマットに従ってモジュールに命令を送信
  byte ubxCfgRate[] = { 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00 };
  ubxCfgRate[6] = (uint16_t)bleNotifyIntervalMs & 0xFF;
  ubxCfgRate[7] = ((uint16_t)bleNotifyIntervalMs >> 8) & 0xFF;
  sendUb(ubxCfgRate, sizeof(ubxCfgRate));

  Serial.printf("BLE notify interval updated to %lu ms.\n", bleNotifyIntervalMs);
  Serial.println("Saved new Rate setting to EEPROM.");

  return true;
}

// モジュールとのボーレートを変更する
bool setGpsBaudRate(uint32_t newBaudRate) {
  bool isValid = false;

  // 入力されたボーレートが有効か検証する
  for (uint32_t rate : validBaudRates) {
    if (newBaudRate == rate) {
      isValid = true;
      break;
    }
  }

  // 有効でない場合
  if (!isValid) {
    Serial.printf("Invalid baud rate: %u bps.\n", newBaudRate);
    return false;
  }

  // UBXパケットの設定
  Serial.printf("Attempting to change baud rate to %u bps...\n", newBaudRate);
  byte ubxCfgPrt[] = { 0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00,
                       0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00 };
  ubxCfgPrt[14] = newBaudRate & 0xFF;
  ubxCfgPrt[15] = (newBaudRate >> 8) & 0xFF;
  ubxCfgPrt[16] = (newBaudRate >> 16) & 0xFF;
  ubxCfgPrt[17] = (newBaudRate >> 24) & 0xFF;

  // ボーレート変更命令を送り、新しいボーレートで通信を確立
  sendUb(ubxCfgPrt, sizeof(ubxCfgPrt));
  gpsSerial.flush();
  delay(100);
  gpsSerial.updateBaudRate(newBaudRate);

  // ESP32側でもボーレートを変更
  currentGpsBaud = newBaudRate;
  EEPROM.put(ADDR_GNSS_BAUD, currentGpsBaud);
  EEPROM.commit();
  Serial.println("Saved new Baud Rate setting to EEPROM.");

  return true;
}

// CFG命令を送る
void saveGnssConfig() {
  Serial.println("Saving configuration to GNSS receiver's BBR/Flash...");
  byte ubxCfgCfg[] = { 0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x00, 0x00 };
  sendUb(ubxCfgCfg, sizeof(ubxCfgCfg));
  delay(100);
}

// --- setup() ---
void setup() {
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);

  // --- EEPROMから設定を読み込み ---
  // 移動平均の設定
  int storedSize = EEPROM.read(ADDR_SMA_SIZE);
  smaWindowSize = (storedSize < 0 || storedSize > MAX_SMA_WINDOW_SIZE) ? 7 : storedSize;
  Serial.printf("Loaded SMA size from EEPROM: %d\n", smaWindowSize);
  setSmaWindowSize(smaWindowSize);

  // 測位頻度の設定
  int storedRate = EEPROM.read(ADDR_GNSS_RATE);
  currentGnssRateHz = (storedRate < 1 || storedRate > 25) ? 10 : storedRate;
  Serial.printf("Loaded GNSS Rate from EEPROM: %d Hz\n", currentGnssRateHz);
  bleNotifyIntervalMs = 1000 / currentGnssRateHz;

  // ボーレートの設定
  EEPROM.get(ADDR_GNSS_BAUD, currentGpsBaud);
  bool isValidBaud = false;
  for (uint32_t rate : validBaudRates) {  // 有効なボーレートか検証
    if (currentGpsBaud == rate) {
      isValidBaud = true;
      break;
    }
  }
  if (!isValidBaud) {
    currentGpsBaud = 57600;
    Serial.println("No valid Baud Rate in EEPROM. Using default.");
  }
  Serial.printf("Loaded Baud Rate from EEPROM: %u bps\n", currentGpsBaud);

  // GNSSモジュールとの通信開始
  Serial.println("\n🚀 Starting BLE GNSS module v2...");
  gpsSerial.begin(currentGpsBaud, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial.printf("🛰️  GPS Serial started at %u baud on RX:%d, TX:%d.\n", currentGpsBaud, RX_PIN, TX_PIN);

  // BLEデバイスの設定を開始
  BLEDevice::init(nameBLE.c_str());
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // LNS Service
  BLEService *pLnsService = pServer->createService(SERVICE_UUID_LNS);
  BLECharacteristic *pLnFeatureCharacteristic = pLnsService->createCharacteristic(CHARACTERISTIC_UUID_LN_FEATURE, BLECharacteristic::PROPERTY_READ);
  const uint32_t ln_feature = 0b0000000001000101;
  pLnFeatureCharacteristic->setValue((uint8_t *)&ln_feature, 4);
  pLocationSpeedCharacteristic = pLnsService->createCharacteristic(CHARACTERISTIC_UUID_LOCATION_SPEED, BLECharacteristic::PROPERTY_NOTIFY);
  pLocationSpeedCharacteristic->addDescriptor(new BLE2902());
  pLnsService->start();

  // --- Config Service ---
  BLEService *pConfigService = pServer->createService(SERVICE_UUID_CONFIG);

  // 移動区間設定キャラの初期化
  pSmaWindowCharacteristic = pConfigService->createCharacteristic(CHARACTERISTIC_UUID_SMA_WINDOW, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  pSmaWindowCharacteristic->setCallbacks(new SmaConfigCallbacks());
  pSmaWindowCharacteristic->setValue(String(smaWindowSize).c_str());

  // 測位頻度設定キャラの初期化
  pGnssRateCharacteristic = pConfigService->createCharacteristic(CHARACTERISTIC_UUID_GNSS_RATE, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  pGnssRateCharacteristic->setCallbacks(new GnssConfigCallbacks());
  pGnssRateCharacteristic->setValue(String(currentGnssRateHz).c_str());

  // ボーレート設定キャラの初期化
  pGnssBaudCharacteristic = pConfigService->createCharacteristic(CHARACTERISTIC_UUID_GNSS_BAUD, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  pGnssBaudCharacteristic->setCallbacks(new GnssConfigCallbacks());
  pGnssBaudCharacteristic->setValue(String(currentGpsBaud).c_str());

  pConfigService->start();

  // アドバタイジングの設定
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID_LNS);
  pAdvertising->addServiceUUID(SERVICE_UUID_CONFIG);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMaxPreferred(0x0C);

  // BLEアドバタイジング開始
  BLEDevice::startAdvertising();
  Serial.printf("📡 BLE advertising started as '%s'.\n", nameBLE.c_str());
}

// --- loop() ---
void loop() {
  // --- シリアル通信を受けた際の処理 ---
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.endsWith("Hz")) {  // 測位頻度設定
      int newRate = input.substring(0, input.length() - 2).toInt();
      if (setGnssRate(newRate)) {
        saveGnssConfig();
        pGnssRateCharacteristic->setValue(String(currentGnssRateHz).c_str());
      }
    } else if (input.endsWith("bps")) {  // ボーレート設定
      uint32_t newBaud = input.substring(0, input.length() - 3).toInt();
      if (setGpsBaudRate(newBaud)) {
        saveGnssConfig();
        pGnssBaudCharacteristic->setValue(String(currentGpsBaud).c_str());
      }
    } else {  // 移動平均区間設定
      int newSize = input.toInt();
      if (String(newSize) == input) {
        setSmaWindowSize(newSize);
        pSmaWindowCharacteristic->setValue(String(smaWindowSize).c_str());
      }
    }
  }

  // 位置情報を受信するまで待機
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      // まず、どちらかのソースから生の速度(km/h)を取得する
      float rawSpeedKph = 0.0;
      if (useGpsSpeedSource) {
        rawSpeedKph = gps.speed.kmph();
      } else {
        updateCalculatedSpeed();  // 手動計算を実行
        rawSpeedKph = calculatedSpeedKph;
      }
      // 次に、平滑化が有効なら、生の速度を元に平滑化を行う
      updateSmoothedSpeed(rawSpeedKph);
    }
  }

  // 設定された位置情報更新間隔以上経過したら位置情報の送信開始
  if (millis() - lastGpsCheck > bleNotifyIntervalMs) {
    lastGpsCheck = millis();

    if (gps.location.isValid() && gps.time.isValid() && deviceConnected) {
      // LNSパケットを作成
      LocationAndSpeed_19Byte loc_data;
      memset(&loc_data, 0, sizeof(loc_data));

      loc_data.flags = 0b0000000001000101;

      float speedKmph = 0.0;
      if (enableSpeedSmoothing) {
        speedKmph = smoothedSpeedKph;
      } else {
        // 平滑化が無効なときは、選択されたソースの生速度を使用
        speedKmph = useGpsSpeedSource ? gps.speed.kmph() : calculatedSpeedKph;
      }

      loc_data.instantaneousSpeed = (uint16_t)(speedKmph * 100.0);
      loc_data.latitude = (int32_t)(gps.location.lat() * 10000000);
      loc_data.longitude = (int32_t)(gps.location.lng() * 10000000);

      loc_data.year = gps.date.year();
      loc_data.month = gps.date.month();
      loc_data.day = gps.date.day();
      loc_data.hour = gps.time.hour();
      loc_data.minute = gps.time.minute();
      loc_data.second = gps.time.second();

      pLocationSpeedCharacteristic->setValue((uint8_t *)&loc_data, sizeof(loc_data));
      pLocationSpeedCharacteristic->notify();

      Serial.printf("0,80,%.02f\n", speedKmph);
    }
  }
}
