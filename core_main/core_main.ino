/* 2020/11/04 2018b24 竹中翔子
/* 2024/01/13 2021a21 髙田陸生
   モータ、サーボ、ソレノイドのテスト用プログラム
   ArduinoDue用
   2022/9/23 2021a21 髙田陸生　　LtoH関数の不具合を修正

   Esp32用

   データナンバー　 アクチュエータ   　操作
   send_data[1]　 モータ　　　   Rスティック
   send_data[2]　 モータ　　　   Lスティック
   send_data[3]　 サーボ　　　   UP　DOWN　R1　L1
   send_data[4]　 ソレノイド4　  〇△□×

   モータのpwmやサーボの角度をフィードバックできるが、やりすぎると動作が重くなるので注意
*/
#include <PS4Controller.h>
#include <stdio.h>
#include<math.h>//数学関数

//シリアル送受信用定義
#define DATA_SEND_NUMBER 17//送信データ数
#define PWM_MAX 127//pwmの絶対値の最大値がこの値になるように調整

//シリアル送受信用グローバル変数
int send_data[DATA_SEND_NUMBER] = {0}; //MDなどへの送信データ数
//切り替え変数
int SWM = 0;
typedef union {
  int8_t signed_data;
  uint8_t unsigned_data;
} analog_data; //符号付き、符号なしを同様に扱うための共用体

typedef union {
  struct {
    uint8_t bit_0 : 1; //right
    uint8_t bit_1 : 1; //down
    uint8_t bit_2 : 1; //up
    uint8_t bit_3 : 1; //left
    uint8_t bit_4 : 1; //square
    uint8_t bit_5 : 1; //Cross
    uint8_t bit_6 : 1; //Circle
    uint8_t bit_7 : 1; //Triangle
    uint8_t bit_8 : 1; //L1
    uint8_t bit_9 : 1; //R1
    uint8_t bit_A : 1; //L3
    uint8_t bit_B : 1; //R3
    uint8_t bit_C : 1; //Share
    uint8_t bit_D : 1; //Options
    uint8_t bit_E : 1; //none
    uint8_t bit_F : 1; //none
  };
  uint32_t all_data : 16;
} digital_data; //スイッチの数は14個なので14bitsで状態を表す

void setup() {
  /*ここで入力ピン設定，割り込み設定，シリアル設定などを行う*/
  Serial.begin(115200); //シリアル1通信開始
  Serial2.begin(9600); //シリアル2通信開始
  // put your setup code here, to run once:

  uint8_t ESP_bt_mac[6] = {0};
  char DS4_bt_mac[30];

  esp_read_mac(ESP_bt_mac, ESP_MAC_BT);

  sprintf(DS4_bt_mac, "%x:%x:%x:%x:%x:%x",
          ESP_bt_mac[0],
          ESP_bt_mac[1],
          ESP_bt_mac[2],
          ESP_bt_mac[3],
          ESP_bt_mac[4],
          ESP_bt_mac[5]); //esp32のマックアドレスを文字列に変換してる。

  PS4.begin(DS4_bt_mac);
}

void loop() {
  int i, j;
  static int angle = 0;
  int LStickY, LStickX, RStickY, RStickX, L1, L2, UP, DOWN, SQUARE, CIRCLE, TRIANGLE, LEFT, RIGHT, CROSS, R1, R2;
  int lY, lX, rX, rY;
  double pwm[5], x, y, Fire = 1.0;
  static double pwmLast[5];
  /*センサ情報を読み取る(ここではコントローラの状態)*/
  // put your main code here, to run repeatedly:

  digital_data DS4_switch_stat;
  analog_data DS4_analog_stat[6];

  DS4_switch_stat.all_data = 0;

  if (PS4.isConnected()) {

    DS4_switch_stat.bit_0 = PS4.Right();
    DS4_switch_stat.bit_1 = PS4.Down();
    DS4_switch_stat.bit_2 = PS4.Up();
    DS4_switch_stat.bit_3 = PS4.Left();

    DS4_switch_stat.bit_4 = PS4.Square();
    DS4_switch_stat.bit_5 = PS4.Cross();
    DS4_switch_stat.bit_6 = PS4.Circle();
    DS4_switch_stat.bit_7 = PS4.Triangle();

    DS4_switch_stat.bit_8 = PS4.L1();
    DS4_switch_stat.bit_9 = PS4.R1();

    DS4_switch_stat.bit_A = PS4.L3();
    DS4_switch_stat.bit_B = PS4.R3();

    DS4_switch_stat.bit_C = PS4.Share();
    DS4_switch_stat.bit_D = PS4.Options();

    DS4_analog_stat[0].unsigned_data = PS4.L2Value(); //符号なし1Byte
    DS4_analog_stat[1].unsigned_data = PS4.R2Value(); //符号なし1Byte
    DS4_analog_stat[2].signed_data = PS4.LStickX(); //符号あり1Byte
    DS4_analog_stat[3].signed_data = PS4.LStickY(); //符号あり1Byte
    DS4_analog_stat[4].signed_data = PS4.RStickX(); //符号あり1Byte
    DS4_analog_stat[5].signed_data = PS4.RStickY(); //符号あり1Byte

    LStickX  = PS4.LStickX();
    LStickY  = PS4.LStickY();
    RStickX  = PS4.RStickX();
    RStickY = PS4.RStickY();
    L1 = PS4.L1();
    L2 = PS4.L2Value();
    R1 = PS4.R1();
    R2 = PS4.R2Value();
    UP = PS4.Up();
    DOWN = PS4.Down();
    SQUARE = PS4.Square();
    CIRCLE = PS4.Circle();
    TRIANGLE = PS4.Triangle();
    CROSS = PS4.Cross();
    RIGHT = PS4.Right();
    LEFT = PS4.Left();
    if (-20 < LStickX && LStickX < 20) {
      LStickX = 0;
    }
    if (-20 < LStickY && LStickY < 20) {
      LStickY = 0;
    }
    if (-20 < RStickX && RStickX < 20) {
      RStickX = 0;
    }
    if (-20 < RStickY && RStickY < 20) {
      RStickY = 0;
    }
    /*****主にいじる所ここから*****/
    //モーター

    lY = LStickY * 0.13;
    lX = LStickX * 0.13;
    rX = RStickX * 0.13;
    send_data[5] = 30 + (-lY - lX + rX);
    send_data[6] = 30 + (lY - lX + rX);
    send_data[7] = 30 + (lY + lX + rX);
    send_data[8] = 30 + (-lY + lX + rX);

    /*if ( && SWM == 0) {
        lY*=-1;
        lX*=-1;
        rX*=-1;
        SWM = 1;
      }
    else if ( && SWM == 1) {
        lY*=-1;
        lX*=-1;
        rX*=-1;
        SWM = 0;
      }*/

    //アームの移動
    if (L1)pwm[1] = 100; //上
    else if (L2 > 0)pwm[1] = -100; //下
    else pwm[1] = 0;
    if (UP && send_data[9] == 0b00001000)pwm[2] = -100; //前
    else if (DOWN && send_data[9] == 0b00001000)pwm[2] = 100; //後
    else pwm[2] = 0;

    //アームのつかみ
    if (SQUARE)send_data[9] |= 0b00000001;
    else send_data[9] &= 0b11111110;
    if (CIRCLE)send_data[9] |= 0b00000010;
    else send_data[9] &= 0b11111101;
    if (TRIANGLE)send_data[9] |= 0b00000100;
    else send_data[9] &= 0b11111011;
    if (R1)send_data[9] |= 0b00001000;//横
    else send_data[9] &= 0b11110111;
    //射出
    if (RIGHT)Fire += 0.1;
    if (LEFT)Fire -= 0.1;
    if (CROSS) {
      pwm[3] = 74 * Fire;
      pwm[4] = -74 * Fire;//-73 good
    } else {
      pwm[3] = 0;
      pwm[4] = 0;
    //装填
    if (PS4.L1()) {
      send_data[9] |= 0b00000011;
    }
    else {
      send_data[9] &= 0b00001100;
    }

    //シリアルモニタに表示

    /*****主にいじる所ここまで*****/
    for (i = 1; i < 5; i++) {//各pwmの比を保ちつつ最大値を超えないように修正してsend_data[1~2]に格納
      double pwm_abs = 0.;//絶対値
      if (pwm[i] > 0)pwm_abs = pwm[i];//pwm[i]の絶対値をpwm_absに代入．
      else pwm_abs = -pwm[i];
      //絶対値が最大値より大きければすべてのpwmに(絶対値/最大値)をかける．
      if (PWM_MAX < pwm_abs)for (j = 1; j < 5; j++)pwm[j] *= (PWM_MAX / pwm_abs);
      send_data[i] = (int)(pwm[i] + 128);//送信データに代入．
    }
    send_data[0] = '<';
    send_data[DATA_SEND_NUMBER - 1] = '>';
    //send_data[0 : (DATA_SEND_NUMBER-1)]をTX2から送信
    for (i = 0; i < DATA_SEND_NUMBER; i++)Serial2.write(send_data[i]);
    Serial2.write(0b10100000 + DATA_SEND_NUMBER);//送信データ数を送信
  }
}
