# GPS Speed Meter

M5Stackを使ったGPSの車速計(10 Hz更新)。耐久レースでの利用を想定した、
ピットレーン制限速度の超過防止機能を搭載しています。

M5Stack Basic V2.7とGPS Module V2.1で動作し、ESP-IDF v5.5.1と
FreeRTOSで実装しています。起動時は黒基調の専用画面に
「M5Stack GPS Speed Meter」と「by kurumario」を表示し、
GNSS初期化後に速度画面へ移行します。

正本となる要求は [docs/requirements.md](docs/requirements.md) を参照してください。

## ハードウェア

| 機器 | 備考 |
| --- | --- |
| M5Stack Basic V2.7 | ESP32、320×240 LCD、内蔵スピーカー、物理ボタン3個 |
| M5Stack GPS Module V2.1 | AT6668搭載、NMEA 0183、最大10 Hz更新 |
| M5Stack Battery Bottom 110 mAh V1.1 | 補助電源 |
| 外部GNSSアンテナ | GPS Module V2.1付属品 |
| USB電源 | レース中の主電源。車両電源から安定化した5 Vを供給 |

### GPS ModuleのDIPスイッチ設定

GPS Module V2.1のDIPスイッチは、Basic向けに次のUARTピンを選択します。

- GPS TX → ESP32 G16(RX)
- GPS RX ← ESP32 G17(TX)
- PPSはすべてOFF

GPS Module基板上の表示では、`TXD → G17`と`RXD → G16`だけをONにし、
ほかのTXD/RXDおよびPPSをOFFにします。G12はESP32のストラッピングピン、
G25は内蔵スピーカーとの共用ピンなので選択しません。

## GPSモジュールとの通信

起動時にESP32 UARTのエッジ計測でボーレートを推定し、推定値でNMEA受信を
検証します。推定できない場合はCASICの製品情報照会を460800~4800 bpsで
順に送信し、チェックサムが正常な応答を受信したボーレートを自動選択します。
通信確立後はRMC/GGA出力をその電源サイクル中だけ要求し、設定をFLASHへ
保存しません。未検出の場合は走査を繰り返します。

通信確立後、CASICコマンドでGPSを車載動作モデル、2D/3D自動測位、
推測航法0秒、100 ms/10 Hz更新に初期化します。各設定はACKに加えて
再読出し値が要求値と一致することを確認し、確認できるまで速度受信を開始しません。
この初期化もGPSモジュールのFLASHには保存せず、起動ごとに適用します。

## 開発環境のセットアップ

ビルドには [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/) v5.5.1が必要です。
[公式手順](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html#installation)に従ってインストールしてください。

```sh
git clone -b v5.5.1 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32
```

依存コンポーネント(M5Unified)は、初回ビルド時にESP-IDF Component Managerが
[main/idf_component.yml](main/idf_component.yml) と
[dependencies.lock](dependencies.lock) に基づいて自動取得します。

## ビルドと書き込み

```sh
# ESP-IDF環境の読み込み(インストール先に合わせてパスを変更)
. <ESP-IDFのインストール先>/export.sh

cd GPS_SpeedMeter
idf.py build
idf.py -p <シリアルポート> flash monitor
```

シリアルポートは、macOSでは`/dev/cu.usbserial-XXXX`、Linuxでは
`/dev/ttyUSB0`などです。`idf.py -p`を省略すると自動検出を試みます。

## ホスト単体テスト

NMEAパーサーなどのロジックは、ESP-IDF環境なしでホスト上でテストできます。

```sh
cmake -S tests -B build_host
cmake --build build_host
ctest --test-dir build_host --output-on-failure
```

## 操作

通常画面:

- A長押し: 5 km/h以下でピットレーンモードへ手動復帰
- B長押し: 設定画面
- Cクリック: 警告音テスト

設定画面:

- A: 値を下げる
- B: 次の項目
- C: 値を上げる
- B長押し: NVSへ保存して終了

初期設定は、ピット制限40 km/h、60 km/h以上を3秒でレースモード、
40 km/h以下を3秒でピットレーンモードです。
速度警告は35 km/hから黄色の注意、40 km/hから赤色の警告を開始します。
注意開始速度と警告開始速度は、設定画面で1 km/h刻みに直接変更できます。
ピットレーンモードで丸め前の速度がピット制限速度以上になると、
制限速度未満へ戻るまで連続したNG音を鳴らします。レースモード、GPS無効、
またはピット警告音オフではNG音を停止します。

## 画面表示

走行画面は黒基調のダークテーマです。速度を画面高の大半を使って最大表示し、
通常時は小さな走行モード、ピット制限速度、単位だけを併記します。
ピット警告時は赤または黄、レースモードは青、GNSS無効時は灰色の外枠で表示します。

設定画面は、レース参加者がマイコンの知識なしに操作できるよう、全項目に
日本語の名称と説明を表示します。音量と明るさは百分率、切替項目はオン/オフで
表示し、Aで減らす、Bで次へ、Cで増やす、B長押しで保存して戻ります。

## ライセンス

このプロジェクトは [Apache License 2.0](LICENSE) で公開しています。

表示用フォント `main/speed_font.vlw` はRoboto Bold(Copyright Google 2014、
Apache License 2.0)から生成しています。詳細は
[main/speed_font.NOTICE](main/speed_font.NOTICE) を参照してください。
