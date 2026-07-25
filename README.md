# GPS Speed Meter

M5Stackを使ったGPSの車速計(10 Hz更新)。耐久レースでの利用を想定した、
ピットレーン制限速度の超過防止機能を搭載しています。

M5Stack Basic V2.7とGPS Module V2.1で動作し、ESP-IDF v5.5.1と
FreeRTOSで実装しています。起動時は黒基調の専用画面に
「M5Stack GPS Speed Meter」と「by kurumario」を表示し、
2秒後に速度画面へ移行します。GNSSの測位完了を待たずに画面操作できます。

正本となる要求は [docs/requirements.md](docs/requirements.md) を参照してください。

本プロジェクトはM5Stack Technology Co., Ltd.の公式プロジェクトではありません。
M5Stackおよび関連する製品名は、対応ハードウェアを説明する目的で使用しています。

## 主な機能

- GPS車速を10 Hzで取得し、大きな整数表示でドライバーへ提示
- ピットレーンモードとレースモードを車速・継続時間から自動切替
- 注意速度では黄色、警告速度では赤色の表示
- ピット制限速度以上では、制限速度未満へ戻るまで連続したNG音
- 本体の3ボタンだけで速度、切替時間、音量、明るさなどを設定
- 設定値をNVSへ保存し、再起動後も保持
- GPSモジュールのボーレートを自動検出
- 起動ごとにCASIC設定を送信し、車載モデル・10 Hz動作を読戻し確認

## ハードウェア

| 機器 | 備考 |
| --- | --- |
| M5Stack Basic V2.7 | ESP32、320×240 LCD、内蔵スピーカー、物理ボタン3個 |
| M5Stack GPS Module V2.1 | AT6668搭載、NMEA 0183、最大10 Hz更新 |
| M5Stack Battery Bottom 110 mAh V1.1 | 補助電源 |
| 外部GNSSアンテナ | GPS Module V2.1付属品 |
| USB電源 | レース中の主電源。車両電源から安定化した5 Vを供給 |

開発と書き込みには、データ通信対応のUSB Type-Cケーブルと、macOSまたは
Linuxを搭載したPCも必要です。USB給電だけで走行試験する場合は、振動で
コネクターが抜けないよう固定してください。

### GPS ModuleのDIPスイッチ設定

GPS Module V2.1のDIPスイッチは、Basic向けに次のUARTピンを選択します。

- GPS TX → ESP32 G16(RX)
- GPS RX ← ESP32 G17(TX)
- PPSはすべてOFF

GPS Module基板上の表示では、`TXD → G17`と`RXD → G16`だけをONにし、
ほかのTXD/RXDおよびPPSをOFFにします。G12はESP32のストラッピングピン、
G25は内蔵スピーカーとの共用ピンなので選択しません。

組み立て前にM5StackとGPS Moduleの電源を切り、DIPスイッチを設定してから
モジュールを積み重ねてください。GNSSアンテナは空が見える位置へ設置します。

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

必要なホスト側ツール:

- Git
- ESP-IDF v5.5.1がサポートするPython環境
- CMakeとC++コンパイラー（ホスト単体テストを実行する場合）
- USBシリアルドライバー（PCがM5Stackを認識しない場合）

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
# ソースコードの取得
git clone https://github.com/kurario/GPS_SpeedMeter.git
cd GPS_SpeedMeter

# ESP-IDF環境の読み込み（インストール先に合わせてパスを変更）
. <ESP-IDFのインストール先>/export.sh

# 初回だけターゲットを指定
idf.py set-target esp32

# ビルド
idf.py build

# 書き込みとシリアルログ表示
idf.py -p <シリアルポート> flash monitor
```

シリアルポートは、macOSでは`/dev/cu.usbserial-XXXX`、Linuxでは
`/dev/ttyUSB0`などです。`idf.py -p`を省略すると自動検出を試みます。
シリアルモニターは`Ctrl+]`で終了できます。

書き込み後は、起動画面が2秒間表示されてから速度画面へ移行します。
初回測位には屋外で数分かかる場合があります。測位前の速度表示が無効でも、
NMEA受信とCASIC設定が正常なら故障ではありません。

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

### 初回起動時の確認

1. 屋外で電源を入れ、起動画面と起動音を確認します。
2. GPS測位後、停止中に速度が0 km/h付近になることを確認します。
3. Cボタンで警告音をテストします。
4. Bボタン長押しで設定画面を開き、ピット制限速度を確認します。
5. 実車走行前に、安全な場所で表示速度とモード切替を確認します。

## 画面表示

走行画面は黒基調のダークテーマです。速度を画面高の大半を使って最大表示し、
通常時は小さな走行モード、ピット制限速度、単位だけを併記します。
ピット警告時は赤または黄、レースモードは青、GNSS無効時は灰色の外枠で表示します。

設定画面は、レース参加者がマイコンの知識なしに操作できるよう、全項目に
日本語の名称と説明を表示します。音量と明るさは百分率、切替項目はオン/オフで
表示し、Aで減らす、Bで次へ、Cで増やす、B長押しで保存して戻ります。

## トラブルシュート

### 速度が表示されない

- GNSSアンテナを接続し、屋外の空が広く見える場所で数分待ってください。
- GPS ModuleのG16/G17用DIPスイッチと、ほかのスイッチがOFFか確認してください。
- シリアルログで`GNSS baud detected`、`GNSS receive started`、
  `CASIC vehicle configuration verified`が出ているか確認してください。

### 書き込みできない

- データ通信対応USBケーブルへ交換してください。
- シリアルモニターなど、ポートを使用中のアプリを終了してください。
- M5Stackを書き込みモードで起動してから、もう一度`idf.py flash`を実行してください。
- Linuxではユーザーがシリアルポートを使用できる権限を持つか確認してください。

### 速度の反応が遅い

CASIC設定の読戻し検証が成功していることと、ログ上で100 ms/10 Hzになって
いることを確認してください。GNSSの受信環境が悪い場合は、更新周期が正常でも
測位結果が遅れたり無効になったりします。

## ディレクトリ構成

| パス | 内容 |
| --- | --- |
| `main/` | ESP-IDFファームウェア、設定保存、NMEA解析、表示用フォント |
| `tests/` | PC上で実行できる単体テスト |
| `docs/requirements.md` | 要求仕様と受入条件 |
| `tools/` | 表示用フォント生成ツール |
| `sdkconfig.defaults` | ESP32向けの共有ビルド設定 |

## 安全上の注意

本機はドライバー支援用の試作機であり、競技規則への適合や速度計としての精度を
保証するものではありません。GPS速度には受信環境による遅延や誤差があります。
車両の純正速度計、ピットクルーの指示、主催者の規則を優先し、走行前に十分な
動作確認を行ってください。運転中に設定操作をしないでください。

## ライセンス

このプロジェクトは [Apache License 2.0](LICENSE) で公開しています。

M5Unified、M5GFX、ESP-IDFおよび内蔵フォントなど、利用している第三者著作物は
それぞれのライセンスに従います。バージョン、著作権表示、利用箇所および
ライセンス全文は [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) を
参照してください。

表示用フォント `main/speed_font.vlw` はRoboto Bold(Copyright Google 2014、
Apache License 2.0)から生成しています。詳細は
[main/speed_font.NOTICE](main/speed_font.NOTICE) を参照してください。
