# BLE_GNSS_module_gemini_ver1
GNSSモジュールで取得した位置情報をBLE-LNSでiPhone等のデバイスに送信するプログラム。
GnssCircuitLoggerとの互換性を確認しています。

## 動作確認済み環境
- ESP32 DevKitC WROOM-32U
- GNSSモジュール: Quescan G10B-F30
- iPhone 15 Pro, iPhone SE2
- GnssCircuitLogger v1.6.5

## 基本的な使い方
- [こちら](https://minkara.carview.co.jp/userid/3475803/car/3322507/8352285/note.aspx)のみんカラを参考に、回路、GNSSモジュールの準備をします。
- このリポジトリの `BLE_GNSS_gemini_ver1.ino` をArduino IDEなどでESP32に焼きます。
- あとはPlug & Playです。GnssCircuitLoggerの設定からBLE GNSSデバイス `KawaiiMyGNSS` を選択し、ラップタイマーを開始します。

## 初期設定
- 測位頻度: 10Hz (変更するにはプログラムだけでなく、GNSSモジュール側設定も変更が必要です)
- 速度データソース: GNSSから送られるデータ (GxRMCの速度フィールド)
- 速度の平滑化: 有効、直近5件の移動平均

## 応用的な使い方
- Arduino IDEなどのシリアルモニタで数値を送ると、その数値分の移動平均を取るようになります。
  - e.g., シリアルモニタのテキストボックスから `10` を送信 &rightarrow; 直近10件の移動平均を現在の速度とする
- LightBlueなどのBLE評価アプリを使うと、iPhoneからも設定変更が可能です。
  - LightBlueから `KawaiiMyGNSS` に接続し、 `0xC48E6067` サービスの `0xC48E6068` キャラクタリスティックを開きます。「Write value」ボタンから希望の移動平均区間を16進数で入力して送信します。
  - e.g., 直近10件の移動平均を取りたい &rightarrow; `0A` を送ります。
  - 「Read」ボタンを押すと、現在の設定を見ることができます。例えば `0x35` なら区間は5、 `0x3135` なら区間は10というように設定されていることがわかります。
- いずれの方法でも、 `0` を送ると移動平均による平滑化が無効になります。安定して速度情報が取れる環境でしたら、平滑化は不要だと思います。

## 謝辞
GnssCircuitLoggerの仕様に関する情報提供をしてくださった Yuji Kanda 様にこの場を借りて謝意を表します。