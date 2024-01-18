![_DSC0521](https://user-images.githubusercontent.com/22425319/234768864-03dacbd2-171a-4932-8552-271770513bb8.JPG)

# cugo_ros_motorcontroller
CuGoをROSで制御する際、ROS開発キットに付属するArduinoのリファレンススケッチです。セットでROSアプリと同時に使用します。  
ROSpkgのリポジトリはこちら：https://github.com/CuboRex-Development/cugo-ros-controller  
### このArduinoスケッチはROS開発キット専用です。クローラロボット開発プラットフォームの方はこちらを参照してください：https://github.com/CuboRex-Development/cugo_ros_motorcontroller/tree/pico-usb
 
# Features
CuGo-ROS-Controller-pkgと使うと/cmd_velのベクトルをそれぞれL/Rの回転数に変換しrpmとしてArduinoに送られてきます。その必要rpmを達成する制御をこのCuGO-ROS-ArduinoDriverで実施します。  
また、CuGoROS開発キット付属のMC-8のプロポを用いて、RC操作と自律走行の切り替えを実施できます。左スティックを左に倒すことでラジコン操作を受け付けるRC-MODEに、右に倒すことでROSの速度ベクトルをCuGoに伝えるROS-MODEに切り替えることができます。自律走行中に、誤った判断をし障害物に衝突しそうなシーンでは、コントローラよりRC-MODEに変更することでラジコン操作に即時に切り替えることができ、緊急停止と同じ操作ができます。  
![image](https://user-images.githubusercontent.com/22425319/234765585-23458585-ea44-40d5-b71f-395c93509fc8.png)

# System
ROSを使用したロボットシステムでは、以下の図のように、LinuxPCとの通信はEthernetを使用したUDPプロトコルで行います。ROS開発キット以外でご活用の際には、PC-Arduinoの間にIPを割り振るDHCPサーバの機能があるルータを挟むか、PC-Arduino間で名前解決する構造にしてください。  
![system](https://user-images.githubusercontent.com/22425319/234768207-93dd7840-2d6f-4fbc-96f8-6960f93b9192.png)

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
 
cugo-ros-arduinodriver.inoのコード上部にある設定係数を入力して、ArduinoIDEで書き込みを行ってください。

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
 
