# Third-Party Notices

GPS Speed Meterは、以下のオープンソースソフトウェアおよびフォントを利用します。
このファイルは第三者著作物の帰属とライセンスを明確にするためのものであり、
各ライセンスの内容を変更するものではありません。

本リポジトリはソースコードのみを公開し、ビルド済みファームウェアは配布しません。
ESP-IDF Component Managerが取得するコンポーネントは
`dependencies.lock`に記録されたバージョンで使用します。

## ビルド時に取得するコンポーネント

### ESP-IDF 5.5.1

- Copyright: Espressif Systemsおよび各コントリビューター
- License: Apache License 2.0
- Upstream: <https://github.com/espressif/esp-idf>
- 用途: ESP32向けSDK、FreeRTOS、ドライバー、NVSなど

Apache License 2.0の全文は、本リポジトリの[LICENSE](LICENSE)にも収録しています。
ESP-IDF内の個別コンポーネントに追加の表示がある場合は、ESP-IDFの配布物に
含まれる各ライセンス表示が優先されます。

### M5Unified 0.2.13

- Copyright (c) 2021 M5Stack
- License: MIT License
- Upstream: <https://github.com/m5stack/M5Unified>
- 用途: M5Stack Basicの画面、スピーカー、ボタンなどの制御

ライセンス全文: [licenses/M5Stack-MIT.txt](licenses/M5Stack-MIT.txt)

### M5GFX 0.2.26

- Copyright (c) 2021 M5Stack
- License: MIT License
- Upstream: <https://github.com/m5stack/M5GFX>
- 用途: 画面描画およびフォント表示

ライセンス全文: [licenses/M5Stack-MIT.txt](licenses/M5Stack-MIT.txt)

## フォント

### Adafruit GFX FreeFont

M5GFXを通じて、設定画面と起動画面でFreeSansおよびFreeMonoの
ビットマップフォントを使用します。

- Copyright (c) 2012 Adafruit Industries
- License: BSD License
- Upstream: <https://github.com/adafruit/Adafruit-GFX-Library>
- License source in M5GFX:
  <https://github.com/m5stack/M5GFX/blob/729297d6e3d657ddc1ec5189bac2f2ea68828085/src/lgfx/Fonts/GFXFF/license.txt>

ライセンス全文:
[licenses/Adafruit-GFX-BSD.txt](licenses/Adafruit-GFX-BSD.txt)

### IPAフォント

M5GFXを通じて、日本語設定画面でIPAフォント由来の
`lgfxJapanGothicP_16`を使用します。

- License: IPAフォントライセンスv1.0
- Upstream: <https://moji.or.jp/ipafont/>
- License source in M5GFX:
  <https://github.com/m5stack/M5GFX/blob/729297d6e3d657ddc1ec5189bac2f2ea68828085/src/lgfx/Fonts/IPA/IPA_Font_License_Agreement_v1.0.txt>

ライセンス全文:
[licenses/IPA-Font-License-1.0.txt](licenses/IPA-Font-License-1.0.txt)

### Roboto Bold

`main/speed_font.vlw`はRoboto Boldから生成した速度表示用フォントデータです。

- Font data copyright Google 2014
- Roboto is a trademark of Google
- License: Apache License 2.0
- Notice: [main/speed_font.NOTICE](main/speed_font.NOTICE)

Apache License 2.0の全文は、本リポジトリの[LICENSE](LICENSE)に収録しています。

## 商標と非公式プロジェクト

M5Stackおよび関連する製品名は、各権利者の商標または登録商標です。
GoogleおよびRobotoも各権利者の商標です。

本プロジェクトはM5Stack Technology Co., Ltd.、Google、Adafruit Industries、
Espressif SystemsまたはIPA／文字情報技術促進協議会が運営、保証、推奨する
公式プロジェクトではありません。製品名は対応ハードウェアと第三者著作物の
出所を説明する目的でのみ使用しています。
