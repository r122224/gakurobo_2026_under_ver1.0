#include "AutoControl.h"
// #include "AStar.h"
// #include "Controller.h"
#include "Controller_gakurobo2023.h"
#include "Filter.h"
#include "LpmsMe1Peach.h"
#include "PIDclass.h"
#include "PathTracking.h"
#include "Platform.h"
#include "STS_Servo.h"
#include "SDclass.h"
#include "define.h"
#include "mbed.h"
#include "phaseCounterPeach.h"
#include "BNO085_SPI.h"
#include "RoboClaw_Mbed_Studio.h"
#include "RoboClawRegisters.h"
#include "platform/mbed_thread.h"
#include "SensorComm.h"
#include "CommTalk.h"
#include "Robomas.h"
#include "AMT22.h"
#include "AMT222C.h"
#include "LimitComm.h"
#include "UDPSocketComm.h"
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <math.h>
#include <string>

#define DIR1 1  //ホイールの回転方向-1で逆回転
#define DIR2 1
#define DIR3 1
#define DIR4 1

//初期位置-------------
#define INIT_X  0.350//0.325//0.4//1.15//2.8//0.37
#define INIT_Y  1.3525//1.4//0.55//3.0//4.605
#define INIT_Z  (M_PI/2)//0.000//(M_PI/2)
//リトライ位置 ゾーン3
#define RETRY_X  11.4
#define RETRY_Y  5.50
#define RETRY_Z  (-M_PI/2)
//リトライ位置 槍回収しないとき
#define RETRY_F_X  0.350
#define RETRY_F_Y  1.3525
#define RETRY_F_Z  0.000

#define LIMIT_VEL_X 3.9   // 速度の制限．超えると速度0になる
#define LIMIT_VEL_Y 3.9   // m/s
#define LIMIT_VEL_Z 3.0   // m/s
#define NORMALSPEED_X 2.0 //通常走行速度の設定 //このプログラムでは2.0以上にしないでください．PWMの設定をしているところが正常に動かなくなります
#define NORMALSPEED_Y 2.0 // m/s
#define NORMALSPEED_Z 2.0 // m/s

//接地エンコーダ
#define ENC_X_X (0.146865)//x,yの位置　後ろについているもの
#define ENC_X_Y (0.00)
#define ENC_Y_X (0.00)//右についているもの
#define ENC_Y_Y (0.16725)
#define ENC_X_R (0.146865 * 1000) //[mm]　中心からの距離
#define ENC_Y_R (0.16725 * 1000) //[mm]
#define ENC_X_THETA (0.0 / 180 * M_PI) //[rad] 角度
#define ENC_Y_THETA (90.0 / 180 * M_PI) //[rad]
#define ENC_X_WHEEL_D (RADIUS_X * 2 * 1000)// [mm]　接地エンコーダの直径
#define ENC_Y_WHEEL_D (RADIUS_Y * 2 * 1000)// [mm]
#define ENC_X_RES4 (500.0 * 4)//分解能
#define ENC_Y_RES4 (500.0 * 4)

//LRTB
//lrtbの位置(中心から出力面まで)
#define LRTB0_POSI_Y -0.264300 //m front                 /|
#define LRTB0_POSI_X 0.107349  //                       / |
#define LRTB1_POSI_Y 0.291000  // left                 /  | x
#define LRTB1_POSI_X -0.00030  //                     /   |      -> atan2(x, y) x,yの順番はexelと逆
#define LRTB2_POSI_Y -0.289800 // right              /    |
#define LRTB2_POSI_X -0.010900 //    (中心)theta -> /\_y__| ↑x+, →y+
#define LRTB3_POSI_Y 0.007500  //back
#define LRTB3_POSI_X 0.265200  //

#define LRTB0_THETA atan2( LRTB0_POSI_X, LRTB0_POSI_Y)// theta [rad] 
#define LRTB1_THETA atan2( LRTB1_POSI_X, LRTB1_POSI_Y)    
#define LRTB2_THETA atan2( LRTB2_POSI_X, LRTB2_POSI_Y)
#define LRTB3_THETA atan2( LRTB3_POSI_X, LRTB3_POSI_Y)

#define LRTB0_DIST sqrt(pow(LRTB0_POSI_X, 2) + pow(LRTB0_POSI_Y, 2))// dist [m]
#define LRTB1_DIST sqrt(pow(LRTB1_POSI_X, 2) + pow(LRTB1_POSI_Y, 2))
#define LRTB2_DIST sqrt(pow(LRTB2_POSI_X, 2) + pow(LRTB2_POSI_Y, 2))
#define LRTB3_DIST sqrt(pow(LRTB3_POSI_X, 2) + pow(LRTB3_POSI_Y, 2))

//サーボID
#define SERVO_ID0 0 //cam1_pitch(right)
#define SERVO_ID1 1 //cam1_yaw
#define SERVO_ID2 2 //cam2_pitch(left)
#define SERVO_ID3 3 //cam2_yaw

//サーボ方向
#define SERVO_DIR0 1 //cam1_pitch(right)
#define SERVO_DIR1 1 //cam1_yaw
#define SERVO_DIR2 1 //cam2_pitch(left)
#define SERVO_DIR3 1 //cam2_yaw

#define CONCOM_INTERVAL INT_TIME*1000 // ms
#define CONCOM_AVAILABLETIME 850 // ms

#define LEDBLINKING_TIME 2      //回数
#define LEDBLINKING_INTERVAL 30 // ms

#define JOYSTICK_DEAD_BAND 0.03
#define NOINPUT_TIME 1.0 / INT_TIME //回数

int flag_Area;

#define Area1 1
#define Area2 2
#define Area3 3

#define dip_on 0

const char* A_ADDRESS = "192.168.10.2"; // 
const uint16_t A_PORT = 22222;
const char* B_ADDRESS = "192.168.10.1"; //
const uint16_t B_PORT = 44444;
UDP client(A_ADDRESS,A_PORT);

// |-----|-----|-----|
// | 12  | 13- | 14  | 出口
// |-----|-----|-----|
// | -9  | 10  | 11  |
// |-----|-----|-----|
// |  6  |  7  |  8  |
// |-----|-----|-----|
// |  3  |  4  |  5  |
// |-----|-----|-----|
// |  0  |  1  |  2  | 入口
// |-----|-----|-----|

// #define 
//1:回収 2:通過 3:回収通過
forest_route route[7] = {
    {0,2},
    {3,3},
    {6,3},
    {9,2},
    {7,1},
    {9,3},
    {12,2}
};

//SDカード
mySDclass mySD;
int SDcount = 0;
int a[30000],b[30000],c[30000],d[30000],e[30000],f[30000],g[30000],h[30000],i2[30000],j[30000],k[30000],l[30000],n[30000],m[30000],o[30000],p[30000],q[30000],r[30000],s[30000],t[30000],u[30000],v[30000],w[30000],x[30000],y[30000],z[30000],aa[30000],ab[30000],ac[30000],ad[30000],ae[30000],af[30000],ag[30000],ah[30000],ai[30000],aj[30000];
float A[30000], B[30000], C[30000], D[30000], E[30000], F[30000],G[30000],H1[30000],I[30000],J[30000],K[30000],L4[30000],N[30000],M[30000],O[30000],P[30000],Q[30000],R[30000],S[30000],T[30000],U[30000],V[30000];//,W[30000],X[30000],Y[30000],Z[30000];//,AA[30000],AB[30000],AC[30000],AD[30000],AE[30000];//,AF[30000],AG[30000];
char str[350];
bool flag_SDwrite = false, LED_SDwrite = false;


//通信
Serial pc(USBTX, USBRX, 115200*2);
Controller con(P7_4, P7_5, 115200); // xbee
Serial com(P5_3,P5_4,115200);//マイコン間通信
CommTalk talk(&com,115200);//マイコン間通信　
//limitスイッチ
LimitComm lim(P5_6, P5_7, 115200); //tx,rx,BaudRateP5_6, P5_7

//lpms
Serial lpmsSerial(P2_5, P2_6); //ジャイロ
LpmsMe1 lpms(&lpmsSerial);//lpmsの初期化

//ホイールの回転方向設定
Platform platform(DIR1, DIR2, DIR3, DIR4);
AutoControl autonomous;
//位相係数．接地エンコーダ
PhaseCounter encX(1);
PhaseCounter encY(2);

//昇降
RoboClaw roboclaw1(131, 115200*2, P8_14, P8_15);//M1:昇降（前）
RoboClaw roboclaw2(128, 115200*2, P8_14, P8_15);//M1:昇降（後），M2:後輪（1個）

//エンコーダ----------------------------------
//spi通信
SPI spi(P10_14, P10_15, P10_12);
//AMT203(アブソリュート)
//カメラ
AMT203V enc_cam1_pitch(&spi, P7_15);//right
AMT203V enc_cam1_yaw(&spi, P2_9);
AMT203V enc_cam2_pitch(&spi, P3_9);//left
AMT203V enc_cam2_yaw(&spi, P8_1);
//昇降
AMT203V enc_lift_front_angle(&spi, P3_8);
AMT203V enc_lift_back_angle(&spi, P2_10);

//サーボ----------------------------------------------
Serial servoSerial(P8_13, P8_11, 115200);//使用するポートをインスタンス化P8_13, P8_11
STS Servo(&servoSerial, 115200);//クラスのインスタンス化

// LowPassFilter
Filter filter(INT_TIME * 2);
Filter joyLX_filter(INT_TIME * 2);
Filter joyLY_filter(INT_TIME * 2);
Filter joyRX_filter(INT_TIME * 2);
Filter joyRY_filter(INT_TIME * 2);
Filter lrtb_right_filter(INT_TIME);
Filter lrtb_left_filter(INT_TIME);
Filter lrtb_front_filter(INT_TIME);
Filter lrtb_back_filter(INT_TIME);
Filter cam_right_pitch_filter(INT_TIME);
Filter cam_right_yaw_filter(INT_TIME);
Filter cam_left_pitch_filter(INT_TIME);
Filter cam_left_yaw_filter(INT_TIME);

//PID-----------------------------------------
//足
pidGain posiGain_x = {3.00,0.0,1.0};
pidGain posiGain_y = {3.00,0.0,1.0};
pidGain posiGain_z = {4.00,0.0,1.0};
PID posiPID_x(posiGain_x.Kp, posiGain_x.Ki, posiGain_x.Kd, INT_TIME);
PID posiPID_y(posiGain_y.Kp, posiGain_y.Ki, posiGain_y.Kd, INT_TIME);
PID posiPID_z(posiGain_z.Kp, posiGain_z.Ki, posiGain_z.Kd, INT_TIME);

//段越え--------------------------------------
// pidGain liftfrontupGain = {3.00, 0.0, 1.0};//昇降前上がる
pidGain liftfrontupGain = {0.02, 0.0, 0.005};//昇降前上がる
pidGain liftfrontdownGain = {3.00, 0.0, 1.0};//昇降前下がる
// pidGain liftbackupGain = {3.00, 0.0, 1.0};//昇降前上がる
pidGain liftbackupGain = {0.02, 0.0, 0.005};
pidGain liftbackdownGain = {3.00, 0.0, 1.0};//昇降前下がる
PID PID_lift_front_up(liftfrontupGain.Kp, liftfrontupGain.Ki, liftfrontupGain.Kd, INT_TIME);
PID PID_lift_front_down(liftfrontdownGain.Kp, liftfrontdownGain.Ki, liftfrontdownGain.Kd, INT_TIME);
PID PID_lift_back_up(liftbackupGain.Kp, liftbackupGain.Ki, liftbackupGain.Kd, INT_TIME);
PID PID_lift_back_down(liftbackdownGain.Kp, liftbackdownGain.Ki, liftbackdownGain.Kd, INT_TIME);



//ON -> 0, OFF -> 1
//dipスイッチ 通信  [0]lrtb [1] 上半身通信  [2]pc（カメラ）通信 [3]lidar
DigitalIn dip[4] = {P5_2,P5_5,P2_13,P4_0};
DigitalIn userButton(P6_0); // USER BUTTON

//拡張ボタン（1 -> 0で押した判定）--------------------------
DigitalIn btn[5] = {P2_4,P2_7,P4_5,P4_6,P4_7};
bool btn0read = false, btn1read = false, btn2read = false, btn3read = false, btn4read = false;
bool pre_btn0read = false, pre_btn1read = false, pre_btn2read = false, pre_btn3read  = false, pre_btn4read = false;

//led
PinName pin_led1(P10_0); // void ledBilnk(PinName,int,int) 用
PinName pin_led2(P2_0);  // PinName で宣言しないと使えなかった
PinName pin_led3(P2_1);
PinName pin_led4(P2_2);
PinName pin_USER_LED(P6_12);
PinName pin_LED_R(P6_13);
PinName pin_LED_G(P6_14);
PinName pin_LED_B(P6_15);
DigitalOut led1(P10_0);
DigitalOut led2(P2_0);
DigitalOut led3(P2_1);
DigitalOut led4(P2_2);
DigitalOut USER_LED(P6_12); //赤色に光る．
DigitalOut LED_R(P6_13);
DigitalOut LED_G(P6_14);
DigitalOut LED_B(P6_15);

coords gPosi = {INIT_X, INIT_Y, INIT_Z};//初期の位置を入れる
//↑ static 使うと autoControl class の extern が使えなくなるので注意
//参考: http://www.picfun.com/pic18/mcc06.html
coords corVal = {0.0, 0.0, 0.0};//自己位置補正医用

Ticker flipper;            // void timer_warikomi() 用
Timer timer;               //経過時間確認用
bool flag_inttime = false; // 15msごとにtrueになる
bool flag_print = false;   // 105msごとにtrueになる
bool flag_free = false; //ホイールをpwm出力0にするためのフラグ


bool set_print = true;
bool flaga_use_controller = true;

//**********************↓関数↓*************************************************//

void timer_warikomi()
//タイマ割り込み．　flag_inttime と flag_printを立てる
//を立てる．RGBledも動かしている．
{
    static uint8_t count4LED = 0;  // LED用
    static uint8_t count4Flag = 0; // Flag用
    count4Flag++;

    if (!(count4Flag % 1))
        flag_inttime = true;
    if (!(count4Flag % int(PRINT_TIME / INT_TIME))) {
        flag_print = true;
        count4Flag = 0;
        count4LED++; 
    }

    if (count4LED == 0) {
        LED_R = true;
        LED_G = false;
        LED_B = false;
    } else if (count4LED == 3) {
        LED_G = true;
    } else if (count4LED == 6) {
        LED_R = false;
    } else if (count4LED == 9) {
        LED_B = true;
    } else if (count4LED == 12) {
        LED_G = false;
    } else if (count4LED == 15) {
        LED_R = true;
    } else if (count4LED == 18) {
        LED_B = false;
        count4LED = 1;
    }
}

void LEDblink(PinName ppin, int times, int interval)
// led点滅用プログラム　セットアップ時に使用
{
    DigitalOut pin(ppin);
    pin = 0;
    for (int i = 0; i < times; i++) {
        ThisThread::sleep_for(interval);
        pin = 1;
        ThisThread::sleep_for(interval);
        pin = 0;
    }
}

void print(char str[]){
    if(set_print) pc.printf("%s",str);
}

double deg2rad(double deg)
// 16384を-PI ~ PIに変換
{
  return M_PI * deg / 180;
}

double pipi(double angle)
// -PI ~ PIに変換
{
    int cor_rotate = int(angle / M_PI);
    return angle - cor_rotate * 2 * M_PI;
}

int convert_qpps(int vel, int ppr, double d) { //rps,分解能,半径
    // roboclawでの速度指令をQPPSで
    int qpps = 0;
    //   qpps = double(vel * ppr) * 4.0 / (d * M_PI); //ローラーの1sごとに進む距離
    qpps = double(vel * ppr) * 4.0 ;
    #define _2RES_PI  ( 2.0 * 500 / M_PI ) //  駆動輪の角速度[rad/s]からRoboClawの指令値[pulses/s]に変換するための定数(2048はエンコーダ分解能) 4逓倍しているが，分母は元は2*piで，通分されている
    return qpps;
}

int ms_qpps(double vel, int ppr, double d, double gia){//m/s ,分解能 ,半径 ,ギア比
    int qpps;
    return qpps = int(2 * vel * gia * ppr / d / M_PI);
}


int flag_field = RED; 
//カメラ RIGHT
#define RIGHT_CAM_MIN_PITCH 2550//3400//上//サーボ
#define RIGHT_CAM_INIT_PITCH 3050
#define RIGHT_CAM_MAX_PITCH 3400//2550//下
#define RIGHT_CAM_MIN_YAW 560//内側
#define RIGHT_CAM_INIT_YAW 720
#define RIGHT_CAM_MAX_YAW 1600//外側

#define RIGHT_CAM_MIN_PITCH_POS 5003//下
#define RIGHT_CAM_INIT_PITCH_POS 6240// 6422
#define RIGHT_CAM_MAX_PITCH_POS 7788//上
#define RIGHT_CAM_MIN_YAW_POS 2068//内側
#define RIGHT_CAM_INIT_YAW_POS 3567//607
#define RIGHT_CAM_MAX_YAW_POS 13179//外側
#define RIGHT_CAM_PITCH_RES 16384
#define RIGHT_CAM_YAW_RES 16384
// 6240 3567 7848 6799

//カメラ LEFT
#define LEFT_CAM_MIN_PITCH 1800//下//サーボ下
#define LEFT_CAM_INIT_PITCH 2330
#define LEFT_CAM_MAX_PITCH 2890//上
#define LEFT_CAM_MIN_YAW 1800//外側
#define LEFT_CAM_INIT_YAW 2670
#define LEFT_CAM_MAX_YAW 2970//内側

#define LEFT_CAM_MIN_PITCH_POS 1628//上
#define LEFT_CAM_INIT_PITCH_POS 7854//7900
#define LEFT_CAM_MAX_PITCH_POS 12862//下
#define LEFT_CAM_MIN_YAW_POS 9715//内側
#define LEFT_CAM_INIT_YAW_POS 6800//6660
#define LEFT_CAM_MAX_YAW_POS 6424//外側
#define LEFT_CAM_PITCH_RES 16384
#define LEFT_CAM_YAW_RES 16384

#define SERVO_RES 4095

double right_cam_pitch = 0.0;
double right_cam_yaw = 0.0;
double left_cam_pitch = 0.0;
double left_cam_yaw = 0.0;
double right_cam_pitch_cmd = 0.0;
double right_cam_yaw_cmd = 0.0;
double left_cam_pitch_cmd = 0.0;
double left_cam_yaw_cmd = 0.0;


void right_rotate_cam(double pitch, double yaw){ //rad
    if(flag_field == BLUE) yaw = -yaw;
    int pitch_dir = 1;
    int pitch_cmd = RIGHT_CAM_INIT_PITCH + pitch_dir * SERVO_DIR0 * (int)(SERVO_RES * pitch / 2 / M_PI);
    int yaw_cmd = RIGHT_CAM_INIT_YAW + SERVO_DIR1 * (int)(SERVO_RES * yaw / 2 / M_PI);

    if(pitch_cmd > RIGHT_CAM_MAX_PITCH) pitch_cmd = RIGHT_CAM_MAX_PITCH;
    else if(pitch_cmd < RIGHT_CAM_MIN_PITCH) pitch_cmd = RIGHT_CAM_MIN_PITCH;
    if(yaw_cmd > RIGHT_CAM_MAX_YAW) yaw_cmd = RIGHT_CAM_MAX_YAW;
    else if(yaw_cmd < RIGHT_CAM_MIN_YAW) yaw_cmd = RIGHT_CAM_MIN_YAW;
    // sprintf(str,"pitch:%d yaw:%d\n",pitch_cmd,yaw_cmd);
    // stolen_cam_pitch_cmd = pitch_cmd;
    // stolen_cam_yaw_cmd = yaw_cmd;

    Servo.cmd(SERVO_ID0, pitch_cmd, 0, 0);
    wait_us(50);
    Servo.cmd(SERVO_ID1, yaw_cmd, 0, 0);
    wait_us(50);
}

void lift_rotate_cam2(double pitch, double yaw){ // rad
    if(flag_field == BLUE) yaw = -yaw;
    int pitch_cmd = LEFT_CAM_INIT_PITCH + SERVO_DIR2 * (int)(SERVO_RES * pitch / 2 / M_PI);
    int yaw_cmd = LEFT_CAM_INIT_YAW + SERVO_DIR3 * (int)(SERVO_RES * yaw / 2 / M_PI);

    if(pitch_cmd > LEFT_CAM_MAX_PITCH) pitch_cmd = LEFT_CAM_MAX_PITCH;
    else if(pitch_cmd < LEFT_CAM_MIN_PITCH) pitch_cmd = LEFT_CAM_MIN_PITCH;
    if(yaw_cmd > LEFT_CAM_MAX_YAW) yaw_cmd = LEFT_CAM_MAX_YAW;
    else if(yaw_cmd < LEFT_CAM_MIN_YAW) yaw_cmd = LEFT_CAM_MIN_YAW;
    // sprintf(str,"pitch:%d yaw:%d\n",pitch_cmd,yaw_cmd);

    Servo.cmd(SERVO_ID2, pitch_cmd, 0, 0);
    wait_us(50);
    Servo.cmd(SERVO_ID3, yaw_cmd, 0, 0);
    wait_us(50);
}

int mms2ms(int mms){
    int ms = mms / 1000;
    return ms;
}

int front_lift(double target, double current, int vel){ //mm,mm,m/s
    int cmd = 0;
    
    // if()
    return cmd;
}

bool sendData(char data[],int n){
    int num = client.sendData(data,n);
    // printf("num:%d\t",num);
    return num;
}

bool receiveData(int mode){
    char buf[50];
    int readCount = 0;
    bool comm_check = false;

    int n = client.upData(buf,50);
    // printf("n:%d\t",n);
    if(n > 0){
        if(n == 8){
            // for(int i=0;i<)
            // receive_data = buf[0];
            // receive_data1 = buf[1];
            // receive_collect_kfs[0] = (int)buf[0];
            // receive_collect_kfs[1] = (int)buf[1];
            // receive_collect_spear[0] = (int)buf[2];
            // receive_collect_spear[1] = (int)buf[3];
            // receive_kfs_posi[0][0] = ((int)buf[4] << 8) | (int)buf[5];
            // receive_kfs_posi[0][1] = ((int)buf[6] << 8) | (int)buf[7];
            comm_check = true;

        }
    }
    // printf("comm check:%d\n",comm_check);
    return comm_check;
}

//************************↓本文↓***********************************************//

int main() {
    //**********************↓setup()↓*********************************************//
    LEDblink(pin_led1, LEDBLINKING_TIME, LEDBLINKING_INTERVAL);
    // setup の進捗具合を表示

    //不具合でリセットされたときに停止するための処理
    sprintf(str,"\n[INFO] set up start\n");
    print(str);

    coords init_pos = {0.0, 0.0, 0.0}; //自己位置の設定
    coords stopVel = {0.0, 0.0, 0.0};  //速度指令0用
    platform.platformInit(init_pos);   //取り敢えずの初期化
    sprintf(str,"[INFO] set platform\n");
    print(str);
    platform.VelocityControl(stopVel); //速度指令0
    sprintf(str,"[INFO] set stopVel\n");
    print(str);

    led1 = led2 = led3 = led4 = LED_G = 1; // ledを一度つける

    int ledloopCount = 0;

    if(flaga_use_controller){
        //コントローラの初期化．タイムアウト時間の設定．
        con.init(CONCOM_AVAILABLETIME, CONCOM_INTERVAL);
        printf("[INFO] wait controller\n");
        print(str);
        // init関数を呼び出さなければタイムアウトの処理は行われない（available関数は常にtrueを返す）
        do { //コントローラが繋がるまで（最低3回は点滅させる）　拡張ボタン基板のled
            if (!dip[0].read())
                led1 = !led1;
            if (!dip[1].read())
                led2 = !led2;
            if (!dip[2].read())
                led3 = !led3;
            if (!dip[3].read())
                led4 = !led4;
            LED_G = !LED_G.read();
            ThisThread::sleep_for(30);
            con.update();
            ledloopCount++;
        } while (ledloopCount <= 5 || !con.available());

        sprintf(str,"[INFO] set controller\n");
        print(str);
    }

    led1 = led2 = led3 = led4 = LED_G = 0; // ledを一度消す

    LEDblink(pin_led2, LEDBLINKING_TIME,
            LEDBLINKING_INTERVAL); // setup の進捗具合を表示

    joyLX_filter.setLowPassPara(MANUAL_LOWPASS_T, 0.0);
    joyLY_filter.setLowPassPara(MANUAL_LOWPASS_T, 0.0);
    joyRX_filter.setLowPassPara(MANUAL_LOWPASS_T, 0.0);
    joyRY_filter.setLowPassPara(MANUAL_LOWPASS_T, 0.0);

    posiPID_x.PIDinit(0.0, 0.0);
    posiPID_y.PIDinit(0.0, 0.0);
    posiPID_z.PIDinit(0.0, 0.0);
    PID_lift_front_up.PIDinit(0.0, 0.0);
    PID_lift_front_down.PIDinit(0.0, 0.0);
    PID_lift_back_up.PIDinit(0.0, 0.0);
    PID_lift_back_down.PIDinit(0.0, 0.0);

    if(!dip[2].read())
        client.init(B_ADDRESS,B_PORT);

    //   lrtb_right_filter.setSecondOrderPara(LRTB_CUTOFF_FREQ, LRTB_FILT_DZETA, 0.0);
    //   lrtb_left_filter.setSecondOrderPara(LRTB_CUTOFF_FREQ, LRTB_FILT_DZETA, 0.0);
    //   lrtb_front_filter.setSecondOrderPara(LRTB_CUTOFF_FREQ, LRTB_FILT_DZETA, 0.0);
    //   cam_pitch_filter.setSecondOrderPara(CAM_CUTOFF_FREQ, CAM_FILT_DZETA, 0.0);
    //   cam_yaw_filter.setSecondOrderPara(CAM_CUTOFF_FREQ, CAM_FILT_DZETA, 0.0);


    //   mySD.init();//SDカードクラスの初期化
    while (mySD.init() != 0) {
        sprintf(str,"[INFO] SD ok!\n");
        print(str);
    }

    
    autonomous.init(&mySD, 0);
    //↑SDカードクラスの初期化を先にやる
    //第二引数はファイルの名前に影響．SDclass.cpp内で確認を．
    sprintf(str,"[INFO] set autoControl\n");
    print(str);

    LEDblink(pin_led3, LEDBLINKING_TIME, LEDBLINKING_INTERVAL); //

    sprintf(str,"[INFO] waitgyro\n");
    print(str);

    if (lpms.init() == 1){
        sprintf(str,"[INFO] set lpms\n");
        print(str);
    }

    encX.init();
    encY.init();
    enc_cam1_pitch.init();
    enc_cam1_yaw.init();
    enc_cam2_pitch.init();
    enc_cam2_yaw.init();
    enc_lift_front_angle.init();
    enc_lift_back_angle.init();
    sprintf(str,"[INFO] set enc\n");
    print(str);

    LEDblink(pin_led4, LEDBLINKING_TIME, LEDBLINKING_INTERVAL); //

    sprintf(str,"[INFO] press right button\n");
    led1 = 1;
    led2 = led3 = led4 = 0;
    do {  //コントローラから右ボタンの入力があるまでループ
        con.update();
        if (led1 == 1) {
            led1 = 0;
            led2 = 1;
        } else if (led2 == 1) {
            led2 = 0;
            led3 = 1;
        } else if (led3 == 1) {
            led3 = 0;
            led4 = 1;
        } else if (led4 == 1) {
            led4 = 0;
            led1 = 1;
        }
        ThisThread::sleep_for(30);
    } while ((con.getButtonState() & MASK_BUTTON_RIGHT) != MASK_BUTTON_RIGHT);
    //}while(con.readButton_bin(RIGHT));でも可

    //割り込み処理の開始
    flipper.attach(&timer_warikomi, INT_TIME);
    sprintf(str,"[INFO] set timer_warikomi\n");
    print(str);

    //このプログラムでは使ってないけど，計測したいときにどうぞ
    timer.start();
    sprintf(str, "[INFO] set timer\n");
    print(str);


    sprintf(str,"[INFO] loop start!!\n\n");
    print(str);

    //**********************↓loop()↓**********************************************//

    while (true) {
        static int buttonState = 0x0000;
        static int addbuttonState = 0x0000;
        static int pre_buttonState = 0x0000;
        static int pre_addbuttonState = 0x0000;
        static double joyLX = 0.0;
        static double joyLY = 0.0;
        static double joyRX = 0.0;
        static double joyRY = 0.0;
        static coords temp_gPosi = {0.0, 0.0, 0.0};
        coords refV = {0.0, 0.0, 0.0};
        static int encXstate, encYstate;//エンコーダの値格納

        bool conUp = true;//コントローラの通信の有無
        static bool joyAndPadState = false; //ジョイコンとパッドへの入力の有無
        static double joyLRad = 0;//joystickの角度
        static double joyLHypo = 0;//joystickLの傾き量
        static int startCount = 0;
        static int joystickAndPad_noInput_count = 0;

        if (flag_inttime) {//INT_TIME毎に処理される．フラグ処理．
        led1 = !led1;
        con.update(); //コントローラボタンの更新
        conUp = con.available();
        if (con.getComCheck()) {
            buttonState = con.getButtonState(); //コントローラボタンの格納
            addbuttonState = con.addButtonState;

            //値を-1~1に加工
            joyLX = joyLX_filter.LowPassFilter(con.readJoyLX());
            joyLY = joyLY_filter.LowPassFilter(con.readJoyLY());
            joyRX = joyRX_filter.LowPassFilter(con.readJoyRX());
            joyRY = joyRY_filter.LowPassFilter(con.readJoyRY());
            
        }

        //controllerからjoystickとPADの入力がしばらくなかったらfalse
        //ホイールへの指令を無くす.
        joyAndPadState = !(joyLX == 0 && joyLY == 0 && joyRX == 0 && joyRY == 0 &&
                            !((buttonState & MASK_BUTTON_PAD) == MASK_BUTTON_PAD));
        if (joyAndPadState)
            joystickAndPad_noInput_count = 0;
        else
            joystickAndPad_noInput_count++;

        if (conUp) { //自己位置の更新や，速度指令はコントローラが繋がっている必要あり
            //自己位置の更新．
            temp_gPosi = platform.getPosi(-encX.getCount(), -encY.getCount(),
                                        (double)lpms.get_z_angle() + corVal.z);

            //もし，センサを使って自己位置を補正するならcorValを使用すればできる

            //最終的な自己位置
            gPosi.x = temp_gPosi.x + corVal.x + INIT_X;
            gPosi.y = temp_gPosi.y + corVal.y + INIT_Y;
            gPosi.z = temp_gPosi.z + corVal.z + INIT_Z;

            //phase == 101 槍先回収のときリミットスイッチ反応したらphase=102にする




















            //速度生成 >>>>>
            //軌道追従．PADボタンを押し続ける必要あり
            if ((buttonState & MASK_BUTTON_PAD) == MASK_BUTTON_PAD) {//if(con.readButton_bin(PAD)){でも可
            refV = autonomous.getRefVel(buttonState);
            led2 = 0;
            } else { //手動制御．joystickで動かす
            led2 = 1;

            // joystickの傾きと角度の計算
            joyLHypo = sqrt(pow(joyLY, 2.0) + pow(joyLX, 2.0));       //con.JoyLTilt()でも可
            joyLRad = atan2(joyLY, joyLX);                           //＜ーーーーーーーーーーーーーーー注意       
            //↑機体の横向きが前になる．普通に使いたいならcon.JoyLTheta()に変更

            // joystickの指令をグローバルに変更
            refV.x = joyLHypo * cos(joyLRad - gPosi.z) * NORMALSPEED_X;
            refV.y = joyLHypo * sin(joyLRad - gPosi.z) * NORMALSPEED_Y;
            refV.z = joyRY * NORMALSPEED_Z; //旋回
            }

            //もし，大きな速度指令になっていた際には，速度を0にする
            if ((fabs(refV.x) >= LIMIT_VEL_X) || (fabs(refV.y) >= LIMIT_VEL_Y) ||
                (fabs(refV.z) >= LIMIT_VEL_Z)) {
            refV.x = 0.0;
            refV.y = 0.0;
            refV.z = 0.0;
            }
            // <<<<<

            //↓速度指令部分．安全のために条件を付けている．
            if (flag_free || joystickAndPad_noInput_count >= NOINPUT_TIME)
            platform.freeWheel(); //速度指令をなしに．pwmで0を出力する．
            else
            platform.VelocityControl(refV); //速度指令　  ('ω')ノ

            startCount++; //通信が再開したときにホイールへの指令を出せるように
            if (startCount >= 100) {
            flag_free = false; //ホイールへの指令が出せるように
            }
        } else {
            startCount = 0;
            flag_free = true; //出力を切る

            platform.freeWheel(); //ホイールへの指令をなくす
            printf("error...error...erro..er...e.......orz\n\n");
            LEDblink(pin_USER_LED, 5, 150);
        }

        flag_inttime = false; // flagを下す．
        }

        if (flag_print) {//PRINT_TIME毎に処理される．フラグ処理．
        led3 = (!flag_free) ? 1 : 0; // flag_freeがtrueだと消灯．falseだと点灯
        led4 = (joyAndPadState) ? 1 : 0;

        printf("%+6f  %+6f  %+6f  \t  %+6f  %+6f  %+6f\n", refV.x, refV.y, refV.z,
                gPosi.x, gPosi.y, gPosi.z);

        flag_print = false; // flagを下す
        }
        ThisThread::sleep_for(1);
    }
}
