![image](https://github.com/CuboRex-Development/cugo_ros_control/assets/97714660/a2525198-fa61-4c4d-9a0f-7dd6824fa625)

# cugo_ros_motorcontroller
クローラロボット開発プラットフォームでROSの制御をするRaspberryPiPicoのリファレンススケッチです。セットでROSパッケージの[cugo_ros_control](https://github.com/CuboRex-Development/cugo-ros-controller)と使用します。


### このArduinoスケッチはクローラロボット開発プラットフォーム専用です。
ROS開発キットの方は[こちら](https://github.com/CuboRex-Development/cugo_ros_motorcontroller/tree/uno-udp)を参照してください。

    

# Features
cugo_ros_motorcontrollerでは、ROSパッケージの [cugo_ros_control](https://github.com/CuboRex-Development/cugo_ros_control)から受信した目標回転数になるようにモータ制御し、実際の回転数を返信します。

また、付属のプロポ（MR-8）を用いて、RC操作と自律走行の切り替えを実施できます。左スティックを左に倒すことでラジコン操作を受け付けるRC-MODEに、右に倒すことでROSの速度ベクトル受け付けるROS-MODEに切り替えることができます。自律走行中に、誤った判断をし障害物に衝突しそうなシーンでは、コントローラよりRC-MODEに変更することでラジコン操作に即時に切り替え、ロボットを止めることができます。  

RaspberryPiPicoに書き込みさえしてあれば、PCのROSパッケージ[cugo_ros_control](https://github.com/CuboRex-Development/cugo_ros_control)側で通信を開始すると、自動的にROSと通信します。

![image](https://user-images.githubusercontent.com/22425319/234765585-23458585-ea44-40d5-b71f-395c93509fc8.png)

#### 対応製品
CuboRex製品では、“ROS開発キット CuGo V3”、“クローラロボット開発プラットフォーム CuGo V4”、“クローラロボット開発プラットフォーム CuGo V3i”でお使いいただけます。それぞれ使用するコードが異なることがありますので、下記表からご参照ください。

ここでは、“クローラロボット開発プラットフォーム CuGo V4”と“クローラロボット開発プラットフォーム CuGo V3i”は“クローラロボット開発プラットフォーム”と総称します。


製品名|ROSパッケージ|マイコンスケッチ
-----------|-----------------|-----------------------------
ROS開発キット| [cugo_ros_control](https://github.com/CuboRex-Development/cugo_ros_control)|[ArduinoUNO用](https://github.com/CuboRex-Development/cugo_ros_motorcontroller/tree/uno-udp)                            
クローラロボット開発プラットフォーム| [cugo_ros_control](https://github.com/CuboRex-Development/cugo_ros_control)|このページ   


# System
クローラロボット開発プラットフォームでは、以下の図のように、LinuxPCとの通信はUSBケーブルで行います。ROSを実行する上位のPCはご自身でご用意ください。  

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
 Arduino標準IDEでCuGo-ROS-PicoDriver.inoを開き、クローラロボット開発プラットフォームのRaspberryPiPicoに書き込んでください。

 ArduinoIDEのインストール方法や書き込み方法などの基本的な操作方法はこちらの[2.準備](https://github.com/CuboRex-Development/cugo-beginner-programming/tree/pico)をご覧ください。

 
# Usage
ArduinoIDEでRaspberryPiPicoにスケッチを書き込んでください。
RaspberryPiPicoに書き込みさえしてあれば、PCのROSパッケージ[cugo_ros_control](https://github.com/CuboRex-Development/cugo_ros_control)側で通信を開始すると、自動的にROSと通信します。

# Note
ご不明点がございましたら、[issues](https://github.com/CuboRex-Development/cugo_ros_control/issues)にてお問い合わせください。回答いたします。


# License
このプロジェクトはApache License 2.0のもと、公開されています。詳細はLICENSEをご覧ください。
 
