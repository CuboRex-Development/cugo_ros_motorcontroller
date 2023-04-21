#include <Arduino.h>
#include <Servo.h>
#include "MotorController.h"
#include <SPI.h>
#include <Ethernet2.h>
#include <EthernetUdp2.h>

/***** ↓各ユーザーごとに設定してください↓ *****/

// シリアル通信での情報の表示有無
bool UDP_CONNECTION_DISPLAY = false;
bool ENCODER_DISPLAY = true;
bool PID_CONTROLL_DISPLAY = false;
bool FAIL_SAFE_DISPLAY = true;

// Ethernet Shield に印刷されている6桁の番号を入れてください。なお、ロボット内ローカル環境動作なので、そのままでもOK。
byte mac[] = {0xA8, 0x61, 0x0A, 0xAE, 0x73, 0x1A};  // お持ちのArduinoShield相当の端末のアドレスを記入

// ROSアプリケーションと同じ値にしてください。
IPAddress ip(192, 168, 8, 216);     // Arduinoのアドレス。LAN内でかぶらない値にすること。
unsigned int localPort = 8888;      // 8888番ポートを聞いて待つ

// PID ゲイン調整
// L側
//const float L_KP = 1.5;  CuGoV3
//const float L_KI = 0.02; CuGoV3
//const float L_KD = 0.1;  CuGoV3
const float L_KP = 1.0;
const float L_KI = 0.06;
const float L_KD = 0.1;

// R側
const float R_KP = 1.0;
const float R_KI = 0.06;
const float R_KD = 0.1;
//const float R_KP = 1.5;  CuGoV3
//const float R_KI = 0.02; CuGoV3
//const float R_KD = 0.1;  CuGoV3

// ローパスフィルタ
const float L_LPF = 0.95;
const float R_LPF = 0.95;

// 回転方向ソフトウェア切り替え
const bool L_reverse = false;
const bool R_reverse = true;

/***** ↑各ユーザーごとに設定してください↑ *****/

#define UDP_BUFF_SIZE 256 // UDP string用のバッファサイズ
#define UDP_BIN_BUFF_SIZE 64 // UDP binary用のバッファ(body)サイズ
#define UDP_HEADER_SIZE 8 // UDP binary用のheaderサイズ
//char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; // 来たメッセージのサイズだけ確保。送る側で大きすぎるものは送らないものとする
char packetBuffer[UDP_BUFF_SIZE]; // UDP string用のバッファ
uint8_t packetBinaryBuffer[UDP_HEADER_SIZE + UDP_BIN_BUFF_SIZE]; // UDP binary用のバッファ
char ReplyBuffer[] = "Initial Buffer Value!";      // 初期値。これが通常通信できたらバグ。
EthernetUDP Udp;

// 受信バッファ
#define RECV_HEADER_CHECKSUM_PTR 6 // ヘッダ チェックサム
#define TARGET_RPM_L_PTR 0 // 左モータ目標RPM
#define TARGET_RPM_R_PTR 4 // 右モータ目標RPM

// 送信バッファ
#define SEND_ENCODER_L_PTR 0 // 左エンコーダ回転数
#define SEND_ENCODER_R_PTR 4 // 右エンコーダ回転数

#define PIN_MOTOR_L A0  // モータ出力ピン(L)
#define PIN_MOTOR_R A1  // モータ出力ピン(R)
#define PIN_ENCODER_L_A 2  // エンコーダ割り込み入力ピン(L)
#define PIN_ENCODER_L_B 8  // エンコーダ回転方向入力ピン(L)
#define PIN_ENCODER_R_A 3  // エンコーダ割り込み入力ピン(R)
#define PIN_ENCODER_R_B 9  // エンコーダ回転方向入力ピン(R)

// プロポ信号の読み取りピン（L/R/MODE_CAHNGE）
#define PWM_IN_PIN0   5   // プロポスティック入力ピン(L)
#define PWM_IN_PIN1   6   // プロポスティック入力ピン(MODE)
#define PWM_IN_PIN2   7   // プロポスティック入力ピン(R)

unsigned long long current_time = 0, prev_time_10ms = 0, prev_time_100ms, prev_time_1000ms; // オーバーフローしても問題ないが64bit確保

#define MOTOR_NUM 2 // モータ接続数（最大4の予定）
#define MOTOR_LEFT 0
#define MOTOR_RIGHT 1
MotorController motor_controllers[2];

#define PIN_UP(no)    upTime[no] = micros();
#define PIN_DOWN(no)  time[no] = micros() - upTime[no]

#define PWM_IN_MAX    3

#define ROS_MODE_IN   1700  // ROSモードに入るときの閾値(us) (1100~1900/中央1500)
#define ROS_MODE_OUT  1300  // ROMモードから抜けるときの閾値(us) (1100~1900/中央1500)

// 動作モード定義
typedef enum {
  RC_MODE = 0,  // RCで動くモード
  ROS_MODE    // ROSで動くモード
} RUN_MODE;

volatile unsigned long upTime[PWM_IN_MAX];
volatile unsigned long rcTime[PWM_IN_MAX];
volatile unsigned long time[PWM_IN_MAX];
int OLD_PWM_IN_PIN0_VALUE;   // プロポスティック入力値(L)
int OLD_PWM_IN_PIN1_VALUE;   // プロポスティック入力値(MODE)
int OLD_PWM_IN_PIN2_VALUE;   // プロポスティック入力値(R)
RUN_MODE runMode = RC_MODE;  // 初回起動時はRC_MODE（無意識な暴走を防ぐため）

// FAIL SAFE
int UDP_FAIL_COUNT = 0;


// ピン変化割り込みの割り込み #TODO まだコードを畳められていない
ISR(PCINT2_vect)
{
  if (OLD_PWM_IN_PIN0_VALUE != digitalRead(PWM_IN_PIN0))
  {
    if (LOW == OLD_PWM_IN_PIN0_VALUE)
    { // 立ち上がり時の処理
      PIN_UP(0);
    }
    else
    { // 立下り時の処理
      PIN_DOWN(0);
    }
    OLD_PWM_IN_PIN0_VALUE = OLD_PWM_IN_PIN0_VALUE ? LOW : HIGH;
  }

  if (OLD_PWM_IN_PIN1_VALUE != digitalRead(PWM_IN_PIN1))
  {
    if (LOW == OLD_PWM_IN_PIN1_VALUE)
    { // 立ち上がり時の処理
      PIN_UP(1);
    }
    else
    { // 立下り時の処理
      PIN_DOWN(1);
    }
    OLD_PWM_IN_PIN1_VALUE = OLD_PWM_IN_PIN1_VALUE ? LOW : HIGH;
  }

  if (OLD_PWM_IN_PIN2_VALUE != digitalRead(PWM_IN_PIN2))
  {
    if (LOW == OLD_PWM_IN_PIN2_VALUE)
    { // 立ち上がり時の処理
      PIN_UP(2);
    }
    else
    { // 立下り時の処理
      PIN_DOWN(2);
    }
    OLD_PWM_IN_PIN2_VALUE = OLD_PWM_IN_PIN2_VALUE ? LOW : HIGH;
  }
}


int split(String data, char delimiter, String *dst)
{
  int index = 0;
  int arraySize = (sizeof(data) / sizeof((data)[0]));
  int datalength = data.length();
  for (int i = 0; i < datalength; i++)
  {
    char tmp = data.charAt(i);
    if (tmp == delimiter)
    {
      index++;
      if (index > (arraySize - 1)) return -1;
    }
    else dst[index] += tmp;
  }
  return (index + 1);
}


void init_PID()
{
  pinMode(PIN_ENCODER_L_A, INPUT_PULLUP);     //A相用信号入力　入力割り込みpinを使用　内蔵プルアップ有効
  pinMode(PIN_ENCODER_L_B, INPUT_PULLUP);     //B相用信号入力　内蔵プルアップ有効
  pinMode(PIN_ENCODER_R_A, INPUT_PULLUP);     //A相用信号入力　入力割り込みpinを使用　内蔵プルアップ有効
  pinMode(PIN_ENCODER_R_B, INPUT_PULLUP);     //B相用信号入力　内蔵プルアップ有効

  // TODO: 初期値入力をコンフィグファイルかROSLAUNCHで入力
  // LEFTインスタンス有効化
  motor_controllers[MOTOR_LEFT] = MotorController(PIN_ENCODER_L_A, PIN_ENCODER_L_B, PIN_MOTOR_L, 2048, 600, 100, L_LPF, L_KP, L_KI, L_KD, L_reverse);
  // RIGHTインスタンス有効化
  motor_controllers[MOTOR_RIGHT] = MotorController(PIN_ENCODER_R_A, PIN_ENCODER_R_B, PIN_MOTOR_R, 2048, 600, 100, R_LPF, R_KP, R_KI, R_KD, R_reverse);

  // エンコーダカウンタは純正のハードウェア割り込みピンを使用
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_L_A), leftEncHandler, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_R_A), rightEncHandler, RISING);

  // 初期値でモータ指示。起動時に停止を入力しないと保護機能が働き、回りません。
  motor_direct_instructions(1500, 1500); //直接停止命令を出す
  delay(100); // すぐに別の値でモータを回そうとするとガクガクするので落ち着くまで待つ。10ms程度でも問題なし。
}

void init_KOPROPO()
{
  // ピン変化割り込みの初期状態保存
  OLD_PWM_IN_PIN0_VALUE = digitalRead(PWM_IN_PIN0);
  OLD_PWM_IN_PIN1_VALUE = digitalRead(PWM_IN_PIN1);
  OLD_PWM_IN_PIN2_VALUE = digitalRead(PWM_IN_PIN2);

  // ピン変化割り込みの設定（D5,D6,D7をレジスタ直接読み取りで割り込み処理）
  pinMode(PWM_IN_PIN0, INPUT);
  pinMode(PWM_IN_PIN1, INPUT);
  pinMode(PWM_IN_PIN2, INPUT);
  PCMSK2 |= B11100000;  // D5,6,7を有効
  PCICR  |= B00000100;  // PCIE2を有効

  pinMode(LED_BUILTIN, OUTPUT); // ROS/RC MODEの表示
  delay(100);
}


void init_UDP()
{
  Ethernet.begin(mac, ip);
  Udp.begin(localPort);
}


void leftEncHandler()
{
  motor_controllers[MOTOR_LEFT].updateEnc();
}


void rightEncHandler()
{
  motor_controllers[MOTOR_RIGHT].updateEnc();
}


void display_speed()
{
  if (ENCODER_DISPLAY == true)
  {
    Serial.println("DISPLAY MOTOR COUNTER & SPEED");
    Serial.print("Mode:");
    Serial.println(runMode);

    Serial.print("Encoder count (L/R):");
    //Serial.print(motor_controllers[MOTOR_LEFT].getRpm());   // 制御量を見るため。開発用
    //Serial.print(motor_controllers[MOTOR_LEFT].getSpeed()); // 制御量を見るため。開発用
    Serial.print(String(motor_controllers[MOTOR_LEFT].getCount()));
    Serial.print(",");
    //Serial.println(motor_controllers[MOTOR_RIGHT].getRpm());    //制御量を見るため。デバッグ用
    //Serial.println(motor_controllers[MOTOR_RIGHT].getSpeed());  //制御量を見るため。デバッグ用
    Serial.println(String(motor_controllers[MOTOR_RIGHT].getCount()));


    Serial.print("PID CONTROL RPM(L/R):");
    Serial.print(motor_controllers[MOTOR_LEFT].getRpm()); // 制御量を見るため。デバッグ用
    //Serial.print(motor_controllers[MOTOR_LEFT].getSpeed()); // 制御量を見るため。デバッグ用
    Serial.print(",");
    Serial.println(motor_controllers[MOTOR_RIGHT].getRpm());    //制御量を見るため。デバッグ用
    //Serial.println(motor_controllers[MOTOR_RIGHT].getSpeed());    //制御量を見るため。デバッグ用

    Serial.println(""); // 改行
  }
}


void display_UDP(int packetSize, char send_msg[])
{
  if (UDP_CONNECTION_DISPLAY == true)
  {
    Serial.println("DISPLAY UDP MESSAGES");
    Serial.print("Packet recieved! size of ");
    Serial.println(packetSize);
    Serial.print("From: ");
    IPAddress remote = Udp.remoteIP();
    for (int i = 0; i < 4; i++)
    {
      Serial.print(remote[i], DEC);
      if (i < 3)
      {
        Serial.print(".");
      }
    }
    Serial.print(", port ");
    Serial.println(Udp.remotePort());

    Serial.print("Recieve msg: ");
    Serial.println(packetBuffer);
    Serial.print("Send msg: ");
    Serial.println(send_msg);
    Serial.println("");
  }
}


void display_target_rpm()
{
  Serial.println("target_rpm[L]:" + String(motor_controllers[0].getTargetRpm()));
  Serial.println("target_rpm[R]:" + String(motor_controllers[1].getTargetRpm()));
}


void display_failsafe()
{
  if (FAIL_SAFE_DISPLAY == true)
  {
    Serial.println("DISPLAY FAIL SAFE PARAM");

    Serial.print("Mode(ROS/RC): ");
    Serial.println(runMode);

    Serial.print("UDP recieve fail count: ");
    Serial.println(UDP_FAIL_COUNT);

    Serial.println("");
  }
}

void display_PID()
{
  if (PID_CONTROLL_DISPLAY == true)
  {
    Serial.println("DISPLAY PID PRAMETER");

    Serial.print("Encoder count (L/R): " + String(motor_controllers[MOTOR_LEFT].getCount()) + ",");
    //Serial.print(String(motor_controllers[MOTOR_LEFT].getCount()));
    //Serial.print(",");
    Serial.println(String(motor_controllers[MOTOR_RIGHT].getCount()));

    Serial.print("Target RPM (L/R): " + String(motor_controllers[MOTOR_LEFT].getTargetRpm()) + ",");
    //Serial.print(String(motor_controllers[MOTOR_LEFT].getTargetRpm()));
    //Serial.print(",");
    Serial.println(String(motor_controllers[MOTOR_RIGHT].getTargetRpm()));

    Serial.print("PID CONTROL RPM(L/R):" + String(motor_controllers[MOTOR_LEFT].getRpm()) + ",");
    //Serial.print(motor_controllers[MOTOR_LEFT].getRpm()); // 制御量を見るため。デバッグ用
    //Serial.print(motor_controllers[MOTOR_LEFT].getSpeed()); // 制御量を見るため。デバッグ用
    //Serial.print(",");
    Serial.println(motor_controllers[MOTOR_RIGHT].getRpm());    //制御量を見るため。デバッグ用
    //Serial.println(motor_controllers[MOTOR_RIGHT].getSpeed());    //制御量を見るため。デバッグ用

    Serial.println("PID controll gain = P x kp + I x ki + D x kd");


    Serial.print("[L]: " + String(motor_controllers[MOTOR_LEFT].getSpeed()) + " = ");
    Serial.print(String(motor_controllers[MOTOR_LEFT].getPID_P()) + " x " + String(L_KP) + " + ");
    Serial.print(String(motor_controllers[MOTOR_LEFT].getPID_I()) + " x " + String(L_KI) + " + ");
    Serial.println(String(motor_controllers[MOTOR_LEFT].getPID_D()) + " x " + String(L_KD) + " + ");
    Serial.print("[R]: " + String(motor_controllers[MOTOR_RIGHT].getSpeed()) + " = ");
    Serial.print(String(motor_controllers[MOTOR_RIGHT].getPID_P()) + " x " + String(L_KP) + " + ");
    Serial.print(String(motor_controllers[MOTOR_RIGHT].getPID_I()) + " x " + String(L_KI) + " + ");
    Serial.println(String(motor_controllers[MOTOR_RIGHT].getPID_D()) + " x " + String(L_KD) + " + ");

    Serial.println("");
  }
}

void display_nothing()
{
  if (UDP_CONNECTION_DISPLAY == false && ENCODER_DISPLAY == false && PID_CONTROLL_DISPLAY == false)
  {
    Serial.println("Display item not set");
    Serial.println("Arduino is working...");
    Serial.println("");
  }
}


void ros_mode()
{
  digitalWrite(LED_BUILTIN, HIGH);  // ROS_MODEでLED点灯

  for (int i = 0; i < MOTOR_NUM; i++) { // 4輪でも使えるように
    motor_controllers[i].driveMotor();
  }
}


void rc_mode()
{
  digitalWrite(LED_BUILTIN, LOW); // RC_MODEでLED消灯
  // 値をそのままへESCへ出力する
  motor_direct_instructions(rcTime[0], rcTime[2]);
 // Serial.println("input cmd:" + String(rcTime[0]) + ", " + String(rcTime[2]));
}


void check_mode_change()
{
  noInterrupts();      //割り込み停止
  rcTime[0] = time[0];
  rcTime[1] = time[1];
  rcTime[2] = time[2];
  interrupts();     //割り込み開始

  // chBでモードの切り替え
  if (ROS_MODE_IN < rcTime[1])
  {
    if (runMode != ROS_MODE)
    { // モードが変わった時(RC→ROS)
      motor_direct_instructions(1500, 1500); //直接停止命令を出す
    }
    reset_pid_gain();
    runMode = ROS_MODE;
  }
  else if (ROS_MODE_OUT > rcTime[1])
  {
    if (runMode != RC_MODE)
    { // モードが変わった時(ROS→RC)
    }
    runMode = RC_MODE;
  }

  // モードごとの処理
  if (ROS_MODE == runMode)
  {
    ros_mode();
  }
  else
  {
    rc_mode();
  }
}


void motor_direct_instructions(int left, int right)
{
  motor_controllers[MOTOR_LEFT].servo_.writeMicroseconds(left);
  motor_controllers[MOTOR_RIGHT].servo_.writeMicroseconds(right);
  Serial.println("Left: " + String(left) + ", Right: " + String(right));
}

void stop_motor_immediately()
{
  //set_motorにしないのはセットすることでUDP受け取れないコマンドがリセットされてしまう。
  motor_controllers[0].setTargetRpm(0.0);
  motor_controllers[1].setTargetRpm(0.0);
  motor_direct_instructions(1500, 1500);
}

void check_failsafe()
{
  // UDP 通信失敗チェック
  UDP_FAIL_COUNT++;
  if (UDP_FAIL_COUNT > 5)
  {
    stop_motor_immediately();
  }
}


void job_10ms()
{
  check_mode_change();
}


void job_100ms()
{
  check_failsafe();
  display_speed();
  display_PID();
  display_failsafe();
  display_target_rpm(); // デバッグ用
}


void job_1000ms()
{
  display_nothing();
}


void write_float_to_buf(uint8_t* buf, const int TARGET, float val)
{
  uint8_t* val_ptr = reinterpret_cast<uint8_t*>(&val);
  memmove(buf + TARGET, val_ptr, sizeof(float));
}


void write_int_to_buf(uint8_t* buf, const int TARGET, int val)
{
  uint8_t* val_ptr = reinterpret_cast<uint8_t*>(&val);
  memmove(buf + TARGET, val_ptr, sizeof(int));
}


void write_bool_to_buf(uint8_t* buf, const int TARGET, bool val)
{
  uint8_t* val_ptr = reinterpret_cast<uint8_t*>(&val);
  memmove(buf + TARGET, val_ptr, sizeof(bool));
}


float read_float_from_buf(uint8_t* buf, const int TARGET)
{
  float val = *reinterpret_cast<float*>(buf + UDP_HEADER_SIZE + TARGET);
  return val;
}


int read_int_from_buf(uint8_t* buf, const int TARGET)
{
  int val = *reinterpret_cast<int*>(buf + UDP_HEADER_SIZE + TARGET);
  return val;
}


bool read_bool_from_buf(uint8_t* buf, const int TARGET)
{
  bool val = *reinterpret_cast<bool*>(buf + UDP_HEADER_SIZE + TARGET);
  return val;
}

uint16_t read_uint16_t_from_header(uint8_t* buf, const int TARGET)
{
  if(TARGET >= UDP_HEADER_SIZE-1) return 0;
  uint16_t val = *reinterpret_cast<uint16_t*>(buf + TARGET);
  return val;
}


/* 工事中
  void recieve_serial_cmd()
  {
  reciev_str = Serial.readStringUntil('\n');
  }
*/

void set_motor_cmd(String reciev_str)
{
  if (reciev_str.length() > 0)
  {
    // 2輪の場合
    String sp_reciev_str[2];
    split(reciev_str, ',', sp_reciev_str);

    for (int i = 0; i < MOTOR_NUM; i++) {
      motor_controllers[i].setTargetRpm(sp_reciev_str[i].toFloat());
    }
    /*  モータに指令値を無事セットできたら、通信失敗カウンタをリセット
        毎回リセットすることで通常通信できる。
        10Hzで通信しているので、100msJOBでカウンタアップ。
    */
    UDP_FAIL_COUNT = 0;
  }
  else
  {
    for (int i = 0; i < MOTOR_NUM; i++) {
      motor_controllers[i].setTargetRpm(0.0);
    }
  }
}


void set_motor_cmd_binary(uint8_t* reciev_buf, int size)
{
  if (size > 0)
  {
    // 2輪の場合
    float sp_reciev_float[2];
    sp_reciev_float[0] = read_float_from_buf(reciev_buf, TARGET_RPM_L_PTR);
    sp_reciev_float[1] = read_float_from_buf(reciev_buf, TARGET_RPM_R_PTR);
    //split(reciev_str, ',', sp_reciev_str);

    for (int i = 0; i < MOTOR_NUM; i++) {
      motor_controllers[i].setTargetRpm(sp_reciev_float[i]);
    }
    /*  モータに指令値を無事セットできたら、通信失敗カウンタをリセット
        毎回リセットすることで通常通信できる。
        10Hzで通信しているので、100msJOBでカウンタアップ。
    */
    UDP_FAIL_COUNT = 0;
  }
  else
  {
    for (int i = 0; i < MOTOR_NUM; i++) {
      motor_controllers[i].setTargetRpm(0.0);
    }
  }
}

String get_send_cmd_string()
{
  String send_msg = String(motor_controllers[MOTOR_LEFT].getCount()) +
                    "," +
                    String(motor_controllers[MOTOR_RIGHT].getCount());
  //Serial.println(send_msg);
  return send_msg;
}


void reset_pid_gain()
{
  for (int i = 0; i < MOTOR_NUM; i++)
  {
    motor_controllers[i].reset_PID_param();
  }
}


uint16_t calculate_checksum(const void* data, size_t size, size_t start = 0)
{
  uint16_t checksum = 0;
  const uint8_t* bytes = static_cast<const uint8_t*>(data);
  // バイト列を2バイトずつ加算
  for (size_t i = start; i < size; i += 2)
  {
    checksum += (bytes[i] << 8) | bytes[i+1];
  }
  // 桁あふれがあった場合は回収
  checksum = (checksum & 0xFFFF) + (checksum >> 16);
  // チェックサムを反転
  return ~checksum;
}


void create_UDP_packet(uint8_t* packet, uint16_t* header, uint8_t* body)
{
  size_t offset = 0;
  memmove(packet, header, sizeof(uint8_t)*UDP_HEADER_SIZE);
  offset += sizeof(uint8_t)*UDP_HEADER_SIZE;
  memmove(packet + offset, body, sizeof(uint8_t)*UDP_BIN_BUFF_SIZE);
}


void UDP_read_write(int packetSize)
{
  // 送信用のデータを整理
  char send_buff[UDP_BUFF_SIZE];
  String send_str = get_send_cmd_string();  // Stringクラスを使いたかったもので
  send_str.toCharArray(send_buff, UDP_BUFF_SIZE);
  display_UDP(packetSize, send_buff);
  //Serial.println(send_buff); // 確認用

  // バッファにたまったデータを抜き出して制御に適用
  Udp.read(packetBuffer, UDP_BUFF_SIZE);

  set_motor_cmd(packetBuffer);

  // 送信された相手に対して制御結果を投げ返す。したがって相手のIPアドレスの指定などは不要
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  Udp.write(send_buff);
  Udp.endPacket();
}


void UDP_read_write_binary(int packetSize)
{
  // バッファにたまったデータを抜き出して制御に適用
  Udp.read(packetBinaryBuffer, UDP_HEADER_SIZE + UDP_BIN_BUFF_SIZE);
  int size = sizeof(packetBinaryBuffer) / sizeof(uint8_t);
  // チェックサムの確認
  uint16_t recv_checksum = read_uint16_t_from_header(packetBinaryBuffer, RECV_HEADER_CHECKSUM_PTR);
  uint16_t calc_checksum = calculate_checksum(packetBinaryBuffer, UDP_HEADER_SIZE + UDP_BIN_BUFF_SIZE, UDP_HEADER_SIZE);
  if(recv_checksum != calc_checksum)
  {
    Serial.println("Packet integrity check failed");
  }
  else
  {
    set_motor_cmd_binary(packetBinaryBuffer, size);
  }

  // 送信用のデータを整理
  // 送信ボディの作成
  uint8_t send_body[UDP_BIN_BUFF_SIZE];
  // 送信ボディの初期化
  memset(send_body, 0, sizeof(send_body));
  // ボディへ送信データの書き込み
  write_float_to_buf(send_body, SEND_ENCODER_L_PTR, motor_controllers[MOTOR_LEFT].getCount());
  write_float_to_buf(send_body, SEND_ENCODER_R_PTR, motor_controllers[MOTOR_RIGHT].getCount());

  // チェックサムの計算
  uint16_t checksum = calculate_checksum(send_body, UDP_BIN_BUFF_SIZE);
  uint16_t send_len = UDP_HEADER_SIZE + UDP_BIN_BUFF_SIZE;
  // 送信ヘッダの作成
  uint16_t send_header[4] = {localPort, Udp.remotePort(), send_len, checksum};

  // 送信パケットの作成
  uint8_t send_packet[send_len];
  create_UDP_packet(send_packet, send_header, send_body);
  display_UDP(send_len, send_packet);

  // 送信された相手に対して制御結果を投げ返す。したがって相手のIPアドレスの指定などは不要
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  Udp.write(send_packet, send_len);
  Udp.endPacket();
}


void UDP_FAIL_CHECK()
{

}

void setup()
{
  Serial.begin(115200);
  init_PID();
  init_KOPROPO();
  init_UDP();
}

void loop()
{
  current_time = micros();  // オーバーフローまで約40分

  if (current_time - prev_time_10ms > 10000) // TODO 10秒で1msくらいズレる
  {
    job_10ms();
    prev_time_10ms = current_time;
  }

  if (current_time - prev_time_100ms > 100000) // TODO 1秒で1msくらいズレる
  {
    job_100ms();
    prev_time_100ms = current_time;
  }

  if (current_time - prev_time_1000ms > 1000000)
  {
    job_1000ms();
    prev_time_1000ms = current_time;
  }

  /*  // シリアル通信でコマンドを受信するとき #工事中(フラグで22年度中に切り替え可の予定)
    if (Serial.available() > 0)
    {
      recieve_serial_cmd();
    }
  */

  // UDP通信でコマンドを受信するとき（標準）
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    //UDP_read_write(packetSize); // string
    UDP_read_write_binary(packetSize); // binary
  }
}
