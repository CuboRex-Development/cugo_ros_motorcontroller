# cugo-ros-arduinodriver
CuGoをROSで制御する際、ROS開発キットに付属するArduinoのリファレンススケッチです。セットでROSアプリと同時に使用します。  
ROSpkgのリポジトリはこちら：https://github.com/CuboRex-Development/cugo-ros-controller.git  
English Documents here：   
正式リリースするまでは、beta branchで管理しますので、そちらをご参照ください。
 
# Features
CuGo-ROS-Controller-pkgと使うと/cmd_velのベクトルでCuGoの制御を行います。  
また、遠隔操作ロボットキット付属のMC-8のプロポを用いて、RC操作と自律走行の切り替えを実施できます。RC-MODEに切り替えることで
無線コントローラで緊急停止と同じ操作ができます。

切り替え説明の画像

# Requirement
ハードウェア
* ArduinoUno / Uno準拠ボード
* Ethernet Shield2 / 準拠ボード

Arduino標準ライブラリ 
* Servo.h
* SPI.h
* Ethernet2.h
* EthernetUdp2.h
 
# Installation
 Arduino標準IDEを使用していれば、依存ライブラリのインストールは必要ありません。
 
# Usage
 
cugo-ros-arduinodriver.inoのヘッダ部分にある設定係数を入力して、ArduinoIDEで書き込みを行ってください。

設定できる項目は以下の通りです。
~~~
// シリアル通信での情報の表示有無
bool UDP_CONNECTION_DISPLAY = false;
bool ENCODER_DISPLAY = true;
bool PID_CONTROLL_DISPLAY = false;
bool FAIL_SAFE_DISPLAY = true;

// Ethernet Shield に印刷されている6桁の番号を入れてください。なお、ロボット内ローカル環境動作なので、そのままでもOK。
byte mac[] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x00};  // お持ちのArduinoShield相当の端末のアドレスを記入

// ROSアプリケーションと同じ値にしてください。
IPAddress ip(192, 168, 8, 216);     // Arduinoのアドレス。LAN内でかぶらない値にすること。
unsigned int localPort = 8888;      // 8888番ポートを聞いて待つ

// PID ゲイン調整
// L側
//const float L_KP = 1.5;  // CuGoV3
//const float L_KI = 0.02; // CuGoV3
//const float L_KD = 0.1;  // CuGoV3
const float L_KP = 1.0;    // MEGA
const float L_KI = 0.06;   // MEGA
const float L_KD = 0.1;    // MEGA

// R側
const float R_KP = 1.0;    // MEGA
const float R_KI = 0.06;   // MEGA
const float R_KD = 0.1;    // MEGA
//const float R_KP = 1.5;  // CuGoV3
//const float R_KI = 0.02; // CuGoV3
//const float R_KD = 0.1;  // CuGoV3

// ローパスフィルタ
const float L_LPF = 0.95;
const float R_LPF = 0.95;

// 回転方向ソフトウェア切り替え
const bool L_reverse = true;
const bool R_reverse = false;
~~~
 
# Note
 
クローラ走行の振動が非常に大きいので、RJ45端子のEthernetケーブルでの通信 / WiFi接続による通信をお勧めします。　　
シリアル通信ものちに対応予定です。
 
 
# License
このプロジェクトはApache License 2.0のもと、公開されています。詳細はLICENSEをご覧ください。
 
