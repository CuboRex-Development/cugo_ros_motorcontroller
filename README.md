![V4メイン](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/e1b76ade-498c-49db-9c62-2013c0201fa4)

# cugo_ros_motorcontroller
CuGoをROSで制御する際、クローラロボット開発プラットフォームのRaspberryPiPicoのリファレンススケッチです。セットでROSアプリと同時に使用します。  
ROSpkgのリポジトリはこちら：https://github.com/CuboRex-Development/cugo-ros-controller  
### このArduinoスケッチはクローラロボット開発プラットフォーム専用です。ROS開発キットの方はこちらを参照してください：https://github.com/CuboRex-Development/cugo_ros_motorcontroller/tree/uno-udp

# Features
cugo_ros_controlと使うと/cmd_velのベクトルをそれぞれL/Rの回転数に変換しrpmとしてRaspberryPiPicoに送られてきます。その必要rpmを達成する制御をこのcugo_ros_motorcontrollerで実施します。  
また、クローラロボット開発プラットフォーム付属のMC-8のプロポを用いて、RC操作と自律走行の切り替えを実施できます。左スティックを左に倒すことでラジコン操作を受け付けるRC-MODEに、右に倒すことでROSの速度ベクトルをCuGoに伝えるROS-MODEに切り替えることができます。自律走行中に、誤った判断をし障害物に衝突しそうなシーンでは、コントローラよりRC-MODEに変更することでラジコン操作に即時に切り替えることで、ロボットをその場で止め、正しい位置に戻すなどの操作ができます。  
![image](https://user-images.githubusercontent.com/22425319/234765585-23458585-ea44-40d5-b71f-395c93509fc8.png)

# System
ROSを使用したロボットシステムでは、以下の図のように、LinuxPCとの通信はUSBケーブルを使用したUSB-Serialで行います。ROSを入れる上位のPCはご自身でご用意ください。  

![system](https://github.com/CuboRex-Development/cugo_ros_motorcontroller/assets/22425319/2b20c7a0-7947-4b92-96dc-3e4d41865eea)

![system](https://github.com/CuboRex-Development/cugo_ros_motorcontroller/assets/22425319/8da5af96-69a2-4591-a654-4b4bc1e0abde)

# Requirement
ハードウェア
* RaspberryPiPico

Arduino標準ライブラリ 
* Servo.h
* PacketSerial.h
* RPi_Pico_TimerInterrupt.h
* hardware/watchdog.h
 
# Installation
 Arduino標準IDEでこのスケッチを開き、クローラロボット開発プラットフォームのRaspberryPiPicoに書き込んでください。

 ArduinoIDEのインストール方法や書き込み方法などの基本的な操作方法はこちらの　2.準備　をご覧ください：https://github.com/CuboRex-Development/cugo-beginner-programming/tree/pico

 
# Usage
 
cugo-ros-arduinodriver.inoのコード上部にある設定係数を入力して、ArduinoIDEで書き込みを行ってください。RaspberryPiPicoに書き込みさえしてあれば、ROS PCのcugo_ros_control側で通信を開始すると、自動的にROSと通信します。

 
# License
このプロジェクトはApache License 2.0のもと、公開されています。詳細はLICENSEをご覧ください。
 
