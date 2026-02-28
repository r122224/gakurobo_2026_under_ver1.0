#include "AutoControl.h"
// #include "AStar.h"
#include "Controller_gakurobo2023.h"
//#include "Controller.h"
#include "Filter.h"
#include "LpmsMe1Peach.h"
#include "PIDclass.h"
#include "PathTracking.h"
#include "Platform.h"
#include "SDclass.h"
#include "STS_Servo.h"
#include "define.h"
#include "mbed.h"
#include "phaseCounterPeach.h"
//#include "bno085.h"servo
// #include "bno085_UART_RVC.h"
#include "BNO085_SPI.h"
//#include "RoboClaw.h"
#include "RoboClaw_Mbed_Studio.h"
#include "RoboClawRegisters.h"
#include "platform/mbed_thread.h"
#include "SensorComm.h"
#include "CommTalk.h"
#include "Robomas.h"
#include "AMT22.h"
#include "AMT222C.h"
#include "LimitComm.h"
// #include "SR04_peach.h"
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iterator>
#include <math.h>
#include <string>
#include <tuple>

#include "UDPSocketComm.h"

// |---------|---------|---------|
// |---1 5---|---1 6---|---1 7---| 出口
// |---------|---------|---------|
// | 1 2 (9) | 1 3(10) | 1 4(11) |
// |---------|---------|---------|
// |  9 (6)  | 1 0 (7) | 1 1 (8) |
// |---------|---------|---------|
// |  6 (3)  |  7 (4)  |  8 (5)  |
// |---------|---------|---------|
// |  3 (0)  |  4 (1)  |  5 (2)  |
// |---------|---------|---------|
// |----0----|----1----|----2----| 入口
// |---------|---------|---------|

// #define 
//1:回収 2:通過 3:回収通過
forest_route route[15] = {
    {1,2},
    {4,2},
    {5,1},
    {7,2},
    {10,2},
    {9,1},
    {11,1},
    {13,01},
    {16,2},
    {17,2},
    {17,2},
    {17,2},
    {17,2},
    {17,2},
    {17,2}
};
// forest_route route[8] = {
//     {1,2},
//     {4,2},
//     {7,2},
//     {8,1},
//     {6,1},
//     {10,2},
//     {13,3},
//     {16,2}
// };
// forest_route route[9] = {
//     {1,2},
//     {4,2},
//     {5,1},
//     {7,2},
//     {10,2},
//     {9,1},
//     {13,2},
//     {14,1},
//     {16,2}
// };
// forest_route route[15] = {
//     {1,2},
//     {4,2},
//     {5,1},
//     {7,3},
//     {8,1},
//     {10,0},
//     {13,2},
//     {12,1},
//     {16,2},
//     {17,2},
//     {17,2},
//     {17,2},
//     {17,2},
//     {17,2},
//     {17,2}
// };
// forest_route route[15] = {
//     {1,2},
//     {4,2},
//     {5,1},
//     {7,2},
//     {6,1},
//     {10,2},
//     {13,2},
//     {12,1},
//     {16,2},
//     {17,2},
//     {17,2},
//     {17,2},
//     {17,2},
//     {17,2},
//     {17,2}
// };

#define UART 0
#define TCP_SOCKET 1
#define UDP_SOCKET 2
#define COMM_SYS (UDP_SOCKET) // 通信形態の指定
#define BLINKING_RATE_MS   1

// int comm_mode = COMM_SYS;

// ポートの指定
const char* A_ADDRESS = "192.168.10.2"; // 
// const char* A_ADDRESS = "192.168.11.2";
const uint16_t A_PORT = 22222;
const char* B_ADDRESS = "192.168.10.1"; //
// const char* B_ADDRESS = "192.168.11.1"; // 
const uint16_t B_PORT = 44444;
int stolen_receive_len = 0;
// 適当な名前
int receive_collect_kfs[2] = {0,0};
int receive_collect_spear[2] = {0,0};
int receive_kfs_posi[2][2] = {{0,0},{0,0}};

UDP client(A_ADDRESS,A_PORT);

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

//足回り----------------------------------
#define DIR1 1  //ホイールの回転方向-1で逆回転  4 左前
#define DIR2 1   //1　左後ろ
#define DIR3 1  //3　　右後ろ
#define DIR4 1  //2　右前

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

//位置setposiaxis
#define POSIX 0
#define POSIY 1
#define POSIZ 2

//速度-----------------
#define NORMALSPEED_X 1.0 //通常走行速度 m/s
#define NORMALSPEED_Y 1.0
#define NORMALSPEED_Z 1.0
#define LIMIT_VEL_X 3.5   // 速度制限
#define LIMIT_VEL_Y 3.5
#define LIMIT_VEL_Z 3.5
//Mode
#define AUTO_MODE 0 //自動モード
#define MANUAL_MODE 1 //操縦モード
#define STANDBY_MODE 2 //待機，準備モード

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
//今回はLRTBを回さないから回転軸から考えなくていい！

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

//段越えモード
#define STEP_MODE1 1
#define STEP_MODE2 2 


//段の高さ
#define STEP_HEIGHT1 0
#define STEP_HEIGHT2 200
#define STEP_HEIGHT3 400

//段越え　機構距離関連
#define LENGTH_FRONT 0.3//前輪機構長さ[m]
#define LENGTH_CENTER 0.3//中央高さ[m]
#define LENGTH_BACK 0.3//後輪機構高さ[m]
#define WIDTH_FRONT 0.1//前輪横幅[m]
#define WIDTH_CENTER 0.3//中央横幅[m]
#define WIDTH_BACK 0.1//後輪横幅[m]



//段越え エンコーダ基準値
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

//中心
#define CORE_MIN_ANGLE 1450//後ろ側
#define CORE_INIT_ANGLE 2305 //0~4095
#define CORE_MAX_ANGLE 3080//前側
#define CORE_INIT_ANGLE_POSI 2305 //0~16384
#define CORE_ANGLE_RES 16384 //分解能

//コントローラ
#define CONCOM_INTERVAL 10       // ms
#define CONCOM_AVAILABLETIME 850 // ms
#define JOYSTICK_DEAD_BAND 0.03
#define NOINPUT_TIME 1.0 / INT_TIME //回数

//LED   
#define LEDBLINKING_TIME 2      //回数
#define LEDBLINKING_INTERVAL_START 5
#define LEDBLINKING_INTERVAL 30 // ms
#define LED_CYCLE 1

//simu
#define SIMU_BUTTON_UP 0
#define SIMU_BUTTON_RIGHT 1
#define SIMU_BUTTON_DOWN 2
#define SIMU_BUTTON_LEFT 3
#define SIMU_BUTTON_CENTER 4
#define SIMU_BUTTON_SUB1 5
#define SIMU_BUTTON_SUB2 6

//環境-----------------------
//dip
#define USE_DIP ( true )

//simulation
#define SIMULATION_MODE ( false )
#define USE_CONTROLLER ( true )
#define USE_SENSOR ( true )
#define USE_SDCARD ( true )
#define USE_PC ( false )

//フィールドサイズ[m]
#define FIELD_X_MIN 0.000//スタート側の中心を0とする．
#define FIELD_Y_MIN 0.000
#define FIELD_X_MAX 12.000//縦
#define FIELD_Y_MAX 6.000//横

//ロボットサイズ
#define ROBOT_X 400 //mm 縦
#define ROBOT_Y 400 //mm 横

//段越え確認用モード選択
#define OVERSTEP_PUSH_BUTTON 1
#define CONF_MODE_MOTOR 1//モータ単独確認(単に指令値を与えるだけ)
#define CONF_MODE_MOTOR_PID 2//モータのPID確認 機体を上にあげて確認する．
#define CONF_MODE_MOVE 3//シーケンス（モータのPID）
#define CONF_MODE_SEQUENCE 4//smooth動作
int conf_Mode = 2;//制御モードCONF_MODE_MOTOR,CONF_MODE_MOVE,CONF_MODE_SEQUENCE,

//モード選択
#define MODE_ZONE1 1 //ゾーン1
#define MODE_ZONE2 2 //ゾーン2
#define MODE_ZONE3 3 //ゾーン3
#define MODE_FOREST 4//forestゾーン

int mode = MODE_ZONE1;
int field_s = RED;

//SDカード
mySDclass mySD;
int SDcount = 0;
int a[30000], b[30000], c[30000], d[30000], e[30000], f[30000], g[30000], h[30000], i_[30000], j[30000], k[30000], l[30000], m[30000], n[30000], o[30000], p[30000], q[30000], r[30000], s[30000],t[30000],u[30000],v[30000],w[30000],x[30000],y[30000],z[30000],a1[30000];
double A[30000], B[30000], C[30000], D[30000], E[30000], F[30000], G[30000],
    H1[30000], I[30000], J[30000], K[30000], L4[30000], N[30000], M[30000], O[30000],P[30000],Q[30000],R[30000],S[30000],T[30000],U[30000],V[30000],W[30000],X[30000],Y[30000],Z[30000],Y1[30000],Z1[30000],O1[30000],O2[30000],O3[30000],O4[30000];
char str[255];
bool flag_SDwrite = true, LED_SDwrite = false;

//simu
char sbuf[256];
int buttonState_E, pre_buttonState_E;
int obj_inf_B = 0;
int obj_R1 = 0, obj_R11 = 0;
int obj_R2 = 0, obj_R22 = 0;
unsigned int obj_inf_B1 = 0;
//pcから受け取る値
bool obj_true_R1[12] = {
    0, 0, 0,
    0, 0, 0,
    0, 0, 0,
    0, 0, 0
};
bool obj_true_R2[12] = {
    0, 0, 0,
    0, 0, 0,
    0, 0, 0,
    0, 0, 0
};
bool obj_true_Fake[12] = {
    0, 0, 0,
    0, 0, 0,
    0, 0, 0,
    0, 0, 0
};
// int obj_state[12] = {//状態
//     0, 0, 0,
//     0, 0, 0,
//     0, 0, 0,
//     0, 2, 0
// };
int obj_state[12] = {//状態
    2, 2, 2,
    2, 2, 2,
    2, 2, 2,
    2, 2, 2
};

int obj_state_test[12] = {//状態
    0, 0, 0,
    0, 0, 0,
    2, 0, 2,
    0, 0, 0
};
// int obj_state_test[12] = {//状態
//     0, 0, 0,
//     0, 2, 0,
//     0, 2, 0,
//     0, 2, 0
// };
int state_mode = 0;// 1 = R1, 2 = R2, 3 = Fake

// Serial pc(USBTX, USBRX, 230400);
Serial pc(USBTX, USBRX, 115200);
Controller con(P7_4, P7_5, 115200); // xbee
Serial com(P5_3,P5_4,115200);//マイコン間通信
CommTalk talk(&com,115200);//マイコン間通信　
//limitスイッチ
LimitComm lim(P5_6, P5_7, 115200); //tx,rx,BaudRateP5_6, P5_7

//roboclaw-------------------------------（昇降2個，前2個，後ろ1個）　(足回り4個)
RoboClaw roboclaw1(131, 115200*2, P8_14, P8_15);//M1:昇降（前）
RoboClaw roboclaw2(128, 115200*2, P8_14, P8_15);//M1:昇降（後），M2:後輪（1個）,
// RoboClaw roboclaw3(130, 115200*2, P8_14, P8_15);//前輪

//昇降前，昇降後，後輪
int roboclawCmd0 = 0, roboclawCmd1 = 0, roboclawCmd2 = 0;

//spi通信
SPI spi(P10_14, P10_15, P10_12);

//リミットスイッチ（槍先ラック用）
// DigitalIn limit1(P1_13);//ハンド取得判定
// DigitalIn limit2(P1_15);//槍先ラック1
// DigitalIn limit3(P1_14);//槍先ラック2
// DigitalIn limit4(P1_1);//右側スイッチ1
// DigitalIn limit5(P1_8);//右側スイッチ2
// DigitalIn limit6(P1_9);//左側スイッチ1
// DigitalIn limit7(P1_10);//左側スイッチ2

//光電センサー
DigitalIn kouden1(P1_7);
bool kouden1read = 1;//前昇降下向き
bool kouden2read = 1;
bool kouden3read = 1;
bool pre_kouden1read = 1;//段ない時1
bool pre_kouden2read = 1;//段ない時1 外側
bool pre_kouden3read = 1;//内側

bool limit1read, limit2read, limit3read, limit4read, limit5read, limit6read, limit7read, limit8read, limit9read;
bool pre_limit1read = false, pre_limit2read = false, pre_limit3read = false, pre_limit4read = false, pre_limit5read = false, pre_limit6read = false, pre_limit7read = false, pre_limit8read = false, pre_limit9read = false;

//測距HC-RS04------------------------------
// DigitalOut us_trig(D15);
// InterruptIn echo(P2_10);
// SR04 front(P1_1, P1_0);   // ECHOは普通のGPIOでOK



uint32_t echo_t_rise = 0;
uint32_t echo_t_fall = 0;
bool echo_prev = false;
float us_distance = 0;


//ジャイロ----------------------------------
Serial lpmsSerial(P2_5, P2_6);
LpmsMe1 lpms(&lpmsSerial);//lpmsの初期化
double pre_anglez, pre_angley, pre_anglex;
double anglez, angley, anglex;
double pre_angle;
//ジャイロ新規
// Serial bno085Serial(P2_5, P2_6);
// bno085 bno085(&bno085Serial);
// double pre_anglez, pre_angley, pre_anglex;
// double anglez, angley, anglex;
// double pre_angle;

// BNO085 bno085(&spi, P3_13, P3_15, P3_14);
// double pre_anglez, pre_angley, pre_anglex;
// double anglez, angley, anglex;
// double pre_angle;
// int num;
// int pre_num = 1;

//足回り--------------------------------------
Platform platform(DIR1, DIR2, DIR3, DIR4);//129//130/130/129
AutoControl autonomous;
// AStar astar;
int ControlMode = AUTO_MODE;//MANUAL_MODE;//AUTO_MODE
bool autostep_mode = false;
int receive_phase = 0, pre_receive_phase = 0, pre_receive_phase_ = 0, collect_phase = 0;
bool posiPIDmode = false;

PhaseCounter encX(1);//位相係数．接地エンコーダ
PhaseCounter encY(2);


//エンコーダ----------------------------------
//AMT203(アブソリュート)
//カメラ
AMT203V enc_cam1_pitch(&spi, P7_15);//right
AMT203V enc_cam1_yaw(&spi, P2_9);
AMT203V enc_cam2_pitch(&spi, P3_9);//left
AMT203V enc_cam2_yaw(&spi, P8_1);
//昇降
AMT203V enc_lift_front_angle(&spi, P3_8);
AMT203V enc_lift_back_angle(&spi, P2_10);

// AMT222C enc_bleft_wheel(&spi, P8_1);//左後ろ
// AMT222C enc_bright_wheel(&spi, P7_15);//右後ろ

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
//PIDのコマンド値
int first_PID_lift_front_cmd = 0, first_PID_lift_back_cmd = 0;//初期化用
int PID_lift_front_cmd = 0, PID_lift_back_cmd = 0;
bool front_syusoku = false, back_syusoku = false;
bool lift_syusoku = false;

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


//lrtb-----------------------------------------------
SensorComm lrtb(P2_14, P2_15, 230400, 4);//lrtb
int lrtb_0 = 0, lrtb_1 = 0, lrtb_2 = 0, lrtb_3 = 0; //距離データ 前，左，右，後
double filt_lrtb_0, filt_lrtb_1, filt_lrtb_2, filt_lrtb_3;//filterかけるとき
double lrtb_dist_0, lrtb_dist_1,lrtb_dist_2, lrtb_dist_3;//壁からの距離変換
double lrtb_posi_dist_0, lrtb_posi_dist_1, lrtb_posi_dist_2, lrtb_posi_dist_3;//中心からLRTBの出力点までの距離
double lrtb_theta_0, lrtb_theta_1, lrtb_theta_2, lrtb_theta_3;//中心からの角度
double lrtb0_dist_y, lrtb0_dist_x, lrtb1_dist_y, lrtb1_dist_x, lrtb2_dist_y, lrtb2_dist_x, lrtb3_dist_y, lrtb3_dist_x;//中心からlrtb出力点までの垂直距離
double lrtb0_wall_y, lrtb0_wall_x, lrtb1_wall_y, lrtb1_wall_x, lrtb2_wall_y, lrtb2_wall_x, lrtb3_wall_y, lrtb3_wall_x;//lrtb出力点から壁までの垂直距離
double lrtb_front_dist_x, lrtb_left_dist_x, lrtb_right_dist_x, lrtb_back_dist_x;
double lrtb_front_dist_y, lrtb_left_dist_y, lrtb_right_dist_y, lrtb_back_dist_y;
double x_min = FIELD_X_MIN;//スタート側の中心を0とする．
double x_max = FIELD_X_MAX;
double y_min = FIELD_Y_MIN;
double y_max = FIELD_Y_MAX;
double check_distance_out, check_distance_in, check_distance_front, check_distance_back;
double distance_out, distance_in, distance_front, distance_back;
double pre_distance_out, pre_distance_in, pre_distance_front, pre_distance_back;
bool flag_use_front = false, flag_use_in = false, flag_use_out = false, flag_use_back = false;//forestで使えるマスかどうか
bool flag_use_front_next = false, flag_use_in_next = false, flag_use_out_next = false, flag_use_back_next = false;
bool flag_lrtb_front = false, flag_lrtb_in = false, flag_lrtb_out = false, flag_lrtb_back = false;
bool flag_lrtb_diff_x = false, flag_lrtb_diff_y = false;
double left_lrtb_posi, right_lrtb_posi;


//サーボ----------------------------------------------
Serial servoSerial(P8_13, P8_11, 115200);//使用するポートをインスタンス化P8_13, P8_11
STS Servo(&servoSerial, 115200);//クラスのインスタンス化
double cam_pitch = 0.0;
double cam_yaw = 0.0;
double cam_pitch2 = 0.0;
double cam_yaw2 = 0.0;
double cam_pitch_cmd = 0.0;
double cam_yaw_cmd = 0.0;
double cam_pitch_cmd2 = 0.0;
double cam_yaw_cmd2 = 0.0;
// int stolen_cam_pitch_cmd = 0;
// int stolen_cam_yaw_cmd = 0;

int CAM_MIN_PITCH = RIGHT_CAM_MIN_PITCH;
int CAM_INIT_PITCH = RIGHT_CAM_INIT_PITCH; 
int CAM_MAX_PITCH = RIGHT_CAM_MAX_PITCH;
int CAM_MIN_YAW = RIGHT_CAM_MIN_YAW;
int CAM_INIT_YAW = RIGHT_CAM_INIT_YAW; 
int CAM_MAX_YAW = RIGHT_CAM_MAX_YAW;
int CAM_INIT_PITCH_POS = RIGHT_CAM_INIT_PITCH_POS;
int CAM_INIT_YAW_POS = RIGHT_CAM_INIT_YAW_POS;
int CAM_PITCH_RES = RIGHT_CAM_PITCH_RES;
int CAM_YAW_RES = RIGHT_CAM_YAW_RES;

int CAM_MIN_PITCH2 = LEFT_CAM_MIN_PITCH;
int CAM_INIT_PITCH2 = LEFT_CAM_INIT_PITCH; 
int CAM_MAX_PITCH2 = LEFT_CAM_MAX_PITCH;
int CAM_MIN_YAW2 = LEFT_CAM_MIN_YAW;
int CAM_INIT_YAW2 = LEFT_CAM_INIT_YAW; 
int CAM_MAX_YAW2 = LEFT_CAM_MAX_YAW;
int CAM_INIT_PITCH_POS2 = LEFT_CAM_INIT_PITCH_POS;
int CAM_INIT_YAW_POS2 = LEFT_CAM_INIT_YAW_POS;
int CAM_PITCH_RES2 = LEFT_CAM_PITCH_RES;
int CAM_YAW_RES2 = LEFT_CAM_YAW_RES;

//段越え---------------------------------------------
//モータ動作確認用
bool back_wheel_flag = false;//後輪を動かす
bool front_PID_flag = false;//前昇降PID
bool front_PID_flag2 = false;//前昇降下げる
bool back_PID_flag = false;//後昇降PID
bool back_PID_flag2 = false;

bool first_front_up_flag = false;
bool first_front_down_flag = false;
bool first_back_up_flag = false;
bool first_back_down_flag = false;

// double first_front_down_height = ; 
// double first_back_down_height = ;
bool set_height_front = false;
bool set_height_back = false;

bool up_flag = false;
bool down_flag = false;

bool seted_front = false;//昇降初期化終了フラグ
bool seted_back = false;
bool seted_first = false;

int overstep_phase = 0;//シーケンス用phase　下に記載するフラグによって進む
int downstep_phase = 0;
int overstep_R1_phase = 0;
int downstep_R1_phase = 0;

bool overstep_flag = false;//上るフラグ
bool downstep_flag = false;//下りるフラグ
bool notstep_flag = false;//最後の列のエリア3に行く前の準備
bool overstep_R1_flag = false;
bool downstep_R1_flag = false;
bool first_take = false;

//リフト予測制御フラグ
bool is_front_lifting = false;//前のリフトを上げ始める開始フラグ
bool is_front_on_step = false;//前リフトが段に乗ったかを判定するフラグ
bool is_center_lifting = false;//中央リフトを上げ始める開始フラグ
bool is_center_uppped = false;//中央リフトが上がり切ったかどうか判定フラグ
bool is_center_on_step = false;//中央リフトが段に乗ったか判定フラグ
bool is_back_lifting = false;//後のリフトを上げ始める開始フラグ
bool is_back_on_step = false;//後のリフトが段に乗ったか判定フラグ

//ロボット中心から何m離れているか
#define FRONT_LENGTH 0.3
#define CENTER_LENGTH 0.3
#define BACK_LENGTH 0.3
#define FRONT_FRONT_OFFSET 0.3
#define FRONT_BACK_OFFSET 0.3
#define CENTER_FRONT_OFFSET 0.3
#define CENTER_BACK_OFFSET 0.3
#define BACK_FRONT_OFFSET 0.3
#define BACK_BACK_OFFSET 0.3


//段越え用変数---------------------------------------
double distance_to_step_m = 0.0;//ロボット中心から段差壁までの距離
double dist_front_to_step = 0.0, dist_center_to_step = 0.0, dist_back_to_step = 0.0;//段差側面から前輪先頭までの距離[m]
double time_front_lift = 3.0, time_center_lift = 3.1, time_back_lift = 3.2;//リフトを上げ切る時間[s]
double robot_speed_mps = 0.0;//段越え時の通常速度
double dist_front_up = 0.0, dist_center_up = 0.0, dist_back_up = 0.0;//リフト上昇に必要な最低距離
double height_front_pos = 0.0, height_center_pos = 0.0, height_back_pos = 0.0;//床を0とした時の昇降量[m]昇降高さの自己位置
double clearance_margin_front = 0.0, clearance_margin_center = 0.0, clearance_margin_back = 0.0;//安全マージン
double height_target_front_pos = 0.0, height_target_center_pos = 0.0, height_target_back_pos = 0.0;//リフトの目標高さ[m]
double delta_H_front = 0.0, delta_H_center = 0.0, delta_H_back = 0.0;//目標との差[m]
double vel_lift_front = 0.0, vel_lift_center = 0.0, vel_lift_back = 0.0;//リフトの昇降速度[v/m]

//エンコーダカウント
int get_cam1_pitch_count;//right 現在の値 
int get_cam1_yaw_count;
int get_cam2_pitch_count;//left
int get_cam2_yaw_count;
int get_lift_front_count;
int get_lift_back_count;
int pre_get_cam1_pitch_count = get_cam1_pitch_count;
int pre_get_cam1_yaw_count = get_cam1_yaw_count;//前の値
int pre_get_cam2_pitch_count = get_cam2_pitch_count;
int pre_get_cam2_yaw_count = get_cam2_yaw_count;
int pre_get_lift_front_count = get_lift_front_count;
int pre_get_lift_back_count = get_lift_back_count;

//角度,距離
double get_cam1_pitch_angle = 0.0;
double get_cam1_yaw_angle = 0.0;//rad　カメラ角度
double get_cam2_pitch_angle = 0.0;
double get_cam2_yaw_angle = 0.0;
double get_lift_front_posi = 0.0;//mm 昇降位置
double get_lift_back_posi = 0.0;
double pre_get_cam1_pitch_angle = 0.0;
double pre_get_cam1_yaw_angle = 0.0;//前の値　rad　カメラ角度
double pre_get_cam2_pitch_angle = 0.0;
double pre_get_cam2_yaw_angle = 0.0;
double pre_get_lift_front_posi = 0.0;//mm 昇降位置
double pre_get_lift_back_posi = 0.0;

//昇降変動位置
double front_lift_posi = 0.0;
double back_lift_posi = 0.0;

//目標値
double ref_lift_front_posi = 0.0;//mm
double ref_lift_back_posi = 0.0;//mm
double set_floor_diff = 30.0;//50.0;//mm
double set_floor_diff2 = 30.0;//後ろ昇降

//目標角度（機体全体）
static double ref_kakudo = 0.0;
static double ref_angle = 0.0; //[rad]
static double refV_zplus = 0.0;

//ON -> 0, OFF -> 1
//dipスイッチ 通信  [0]lrtb [1] 上半身通信  [2]pc（カメラ）通信 [3]lidar
DigitalIn dip[4] = {P5_2,P5_5,P2_13,P4_0};
DigitalIn userButton(P6_0); // USER BUTTON

//拡張ボタン（1 -> 0で押した判定）--------------------------
DigitalIn btn[5] = {P2_4,P2_7,P4_5,P4_6,P4_7};
bool btn0read = false, btn1read = false, btn2read = false, btn3read = false, btn4read = false;
bool pre_btn0read = false, pre_btn1read = false, pre_btn2read = false, pre_btn3read  = false, pre_btn4read = false;

//LED---------------------
PinName pin_led1(P2_1); // void ledBilnk(PinName,int,int) 用
PinName pin_led2(P2_2);  // PinName で宣言しないと使えなかった
PinName pin_led3(P10_0);
PinName pin_led4(P2_0);
DigitalOut led1(P2_1);
DigitalOut led2(P2_2);
DigitalOut led3(P10_0);
DigitalOut led4(P2_0);
PinName pin_USER_LED(P6_12);
PinName pin_LED_R(P6_13);
PinName pin_LED_G(P6_14);
PinName pin_LED_B(P6_15);
// DigitalOut led[4] = {P2_1,P2_2,P2_0,P10_0};
DigitalOut USER_LED(P6_12); //赤色に光る．
DigitalOut LED_R(P6_13);
DigitalOut LED_G(P6_14);
DigitalOut LED_B(P6_15);

uint16_t dipSwState = 0b00000000;

coords getPosi = {0.0, 0.0, 0.0};//現在地
coords gPosi = {INIT_X, INIT_Y, INIT_Z};//初期の位置を入れる
coords pregPosi = {INIT_X, INIT_Y, INIT_Z};//前の位置
coords lrtbPosi = {0.0, 0.0, 0.0};
coords tar_Posi = {0.0, 0.0, 0.0};//取得目標値
coords cam_posir = {0.0, 0.0, 0.0};
coords cam_posil = {0.0, 0.0, 0.0};
coords refV = {0.0, 0.0, 0.0};
coords setPosi = {0.0, 0.0, 0.0};
//↑ static 使うと autoControl class の extern が使えなくなるので注意
//参考: http://www.picfun.com/pic18/mcc06.html

coords corVal = {0.0, 0.0, 0.0};//自己位置補正医用

//objectの位置指定
//4×3マスの現在のマス目 初期位置
field cubePosi = {-1, 1};//初期位置
field3 cubePosi3 = {-1, 1, 0};//3次元用
field tar_cubePosi = {4, 2};//目標位置
field3 tar_cubePosi3 = {4, 1, 0};//3次元用

int cubeIndex = 0;
int nextIndex = 0;
int re_nextIndex = 0;//12マスにたいおうさせたもの．
int next_box_state = 0;//次に進むマスのbox値（基本的に2か0が返ってくる．R1にふさがれたときは多分1が返ってくる．）

Obstacle objePosi[18] = {//forestの振り分け座標
    {4,0}, {4,1}, {4,2},
    {3,0}, {3,1}, {3,2},
    {2,0}, {2,1}, {2,2},
    {1,0}, {1,1}, {1,2},
    {0,0}, {0,1}, {0,2},
    {-1,0}, {-1,1}, {-1,2}
};

double stepHeight[18] = {//forest段差高さ[mm]
    0.0, 0.0, 0.0,
    0.0, 0.2, 0.0,
    0.2, 0.4, 0.2,
    0.4, 0.2, 0.0,
    0.2, 0.0, 0.2,
    0.0, 0.0, 0.0
};
//forestのlrtb用
side4 forest_Posi[18] = {//前，左，右，後
    {0.0,0.0,0.0,8.0}, {0.0,0.0,0.0,8.0}, {0.0,0.0,0.0,8.0},
    {8.0,1.2,2.4,6.8}, {8.0,2.4,3.6,6.8}, {8.0,3.6,4.8,6.8},
    {6.8,1.2,2.4,5.6}, {6.8,2.4,3.6,5.6}, {6.8,3.6,4.8,5.6},
    {5.6,1.2,2.4,4.4}, {5.6,2.4,3.6,4.4}, {5.6,3.6,4.8,4.4},
    {4.4,1.2,2.4,3.2}, {4.4,2.4,3.6,3.2}, {4.4,3.6,4.8,3.2},
    {3.2,0.0,0.0,0.0}, {3.2,0.0,0.0,0.0}, {3.2,0.0,0.0,0.0},
};

side4 box_Posi[18] = {//前，左(in)，右(out)，後
    {0.0,0.0,0.0,0.0}, {0.0,0.0,0.0,0.0}, {0.0,0.0,0.0,0.0},
    {7.575,1.625,1.975,7.225}, {7.575,2.825,3.175,7.225}, {7.575,4.025,4.375,7.225},
    {6.375,1.625,1.975,6.025}, {6.375,2.825,3.175,6.025}, {6.375,4.025,4.375,6.025},
    {5.175,1.625,1.975,4.825}, {5.175,2.825,3.175,4.825}, {5.175,4.025,4.375,4.825},
    {3.975,1.625,1.975,3.625}, {3.975,2.825,3.175,3.625}, {3.975,4.025,4.375,3.625},
    {0.0,0.0,0.0,0.0}, {3.2,0.0,0.0,0.0}, {0.0,0.0,0.0,0.0},
};

//forest位置
controllPoint forestPosition[18] = {
    {8.6,1.8}, {8.6,3.0}, {8.6,4.2},
    {7.4,1.8}, {7.4,3.0}, {7.4,4.2},
    {6.2,1.8}, {6.2,3.0}, {6.2,4.2},
    {5.0,1.8}, {5.0,3.0}, {5.0,4.2},
    {3.8,1.8}, {3.8,3.0}, {3.8,4.2},
    {2.6,1.8}, {2.6,3.0}, {2.6,4.2}
};

int change_num[18] = {
    15,16,17,
    12,13,14,
    9,10,11,
    6,7,8,
    3,4,5,
    0,1,2
};

// controllPoint forestPosi5[18] = {
//     {2.6,4.2}, {2.6,3.0}, {2.6,1.8},
//     {3.8,4.2}, {3.8,3.0}, {3.8,1.8},
//     {5.0,4.2}, {5.0,3.0}, {5.0,1.8},
//     {6.2,4.2}, {6.2,3.0}, {6.2,1.8},
//     {7.4,4.2}, {7.4,3.0}, {7.4,1.8},
//     {8.6,4.2}, {8.6,3.0}, {8.6,1.8}
// };

//上半身通信用変数--------------------
//受け取り
int up_phase = 0;//上半身phase
int up_box_num = 0;//box取得個数
//送る値
//autonomous.send_num = 0;
//接地エンコーダのエアシリンダ制御用---
bool air_up_flag = false;
bool air_state = 0;
int KFS_height_state = 0;
int also_KFS_hold_num = 0;
bool lrtb_check = false;
//取得時のphase
int hold_phase = 0;

//pc関連--------------------------------------------------
//槍先回収
int spear_rack_true[6] =  {0,0,0,0,0,0};//手前が0


Ticker flipper;            // void timer_warikomi() 用
Timer timer;               //経過時間確認用
Timer t1;
bool flag_int = false; // 15msごとにtrueになる
bool flag_print = false;  
bool flag_100ms = false;
bool flag_tuusin = false;
bool flag_bno085 = false;
bool flag_solenoido = false;
bool flag_pc = false;
bool flag_con_tuusin = false;//20ms コントローラに値を送るとき
bool flag_free = false; //ホイールをpwm出力0にするためのフラグ
bool flag_limitcomm = false;//limitスイッチ
bool limitdata[12];

bool flag_use_controller = false, flag_use_sensor = false, flag_simu = false, flag_use_sdcard = false, pc_comm = false, lrtb_comm = false, up_comm = false, lidar_comm = false, con_comm = false;; //コントローラ
bool flag_auto = false, flag_stop = false;
bool set_print = true;

int count_time[10];

int path = 0;

int tttt = 0;

//関数---------------------------------------------------------------
void timer_warikomi(){
    static int timer_count = 1;
    static uint8_t count4LED = 0;  // LED用
    static uint8_t count4LED_B = 0;  // LED用
    static uint8_t count4Flag = 0; // Flag用
    static uint8_t countflag = 0;
    count4Flag++;
    countflag++;
    count4LED_B++;
    timer_count++;

    
    // if(!flag_int){
    if(1){
        tttt = timer.read_us();
        timer.reset();
        // interval_sum += interval_time;
        // printf("time:%d  \n",interval_time);
        // pc.printf("%d",timer.read_ms());
        flag_int = true;
        flag_con_tuusin = true;
        flag_limitcomm = true;
        //------------
        // bool now = us_echo.read();

        // if (now && !echo_prev) {
        //     echo_t_rise = timer.read_us();   // timerはあなたの既存の Timer
        // }
        // if (!now && echo_prev) {
        //     echo_t_fall = timer.read_us();
        //     uint32_t w = echo_t_fall - echo_t_rise;
        //     us_distance = w * 0.034f / 2.0f;
        // }

        // echo_prev = now;

        // // トリガ発射
        // us_trig = 1;
        // us_trig = 0;



        if (!(count4Flag % int(PRINT_TIME / INT_TIME))) {
            count4Flag = 0;
            count4LED++;
        }
        if(timer_count % 5 == 0){
            flag_100ms = true;
        }
        if(timer_count % 3 == 0){
            flag_tuusin = true;
            flag_pc = true;
        }
        if(timer_count % 1 == 0){
            flag_print = true;   
            // flag_pc = true;
        }
        if(timer_count > 8){
            timer_count = 1;
        }
        // if(timer_count)
        
        if(!dip[0] || !dip[1] || !dip[2] || !dip[3]){
            if(!dip[0] && count4LED_B > 0) {
                led1 = !led1;
                led4 = false;
            }else led1 = false;
            if(!dip[1] && count4LED_B > 3) {
                led1 = false;
                led2 = !led2;
            }else led2 = false;
            if(!dip[2] && count4LED_B > 6) {
                led2 = false;
                led3 = !led3;
            }else led3 = false;
            if(!dip[3] && count4LED_B > 9) {
                led3 = false;
                led4 = !led4; 
            }else led4 = false;
            if(count4LED_B > 11) count4LED_B = 1;
        }else{
            if(count4LED_B > 0) {
                led1 = !led1;
                led4 = false;
            }
            if(count4LED_B > 3) {
                led1 = false;
                led2 = !led2;
            }
            if(count4LED_B > 6) {
                led2 = false;
                led3 = !led3;
            }
            if(count4LED_B > 9) {
                led3 = false;
                led4 = !led4; 
            }
            if(count4LED_B > 11) count4LED_B = 1;
        }

        if(!LED_SDwrite){
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
    }
}

//ソレノイド制御基板用
void timer_bno(){
    if(!flag_solenoido){
        flag_solenoido = true;
    }
}

void invoke_print(char str[]){
    if(set_print) pc.printf("%s",str);
}

//Simulation_send(UART)
bool sendData_maicon(char str[],int n){
    char checksum = 0;
    for(int i = 0;i < n;i++){
        pc.putc(str[i]);
        checksum ^= str[i];
    }
    // pc.putc(checksum);
    // pc.putc('\n');
    return true;
}

//Simulation_recieve(UART)
bool receiveData_maicon(){
    char c;
    char checksum = 0;
    int readCount = 0;
    char rbuf[255];
    char sbuf2[255];
    bool comm_check = false;
    int get_posi[3] = {0, 0, 0};

    while(pc.readable()){
        c = pc.getc();
        if(c == '\n' && (readCount == 15)){//終端コードのチェック
            if(rbuf[14] == checksum){//通信数とチェックサムの確認
                buttonState_E = (int)rbuf[0];
                // obj_R1 = (int)rbuf[0];
                // get_posi[0] = (int)(rbuf[1] << 24 | rbuf[2] << 16 | rbuf[3] << 8 | rbuf[4]);
                // get_posi[1] = (int)(rbuf[5] << 24 | rbuf[6] << 16 | rbuf[7] << 8 | rbuf[8]);
                // get_posi[2] = (int)(rbuf[9] << 24 | rbuf[10] << 16 | rbuf[11] << 8 | rbuf[12]);
                get_posi[0] = (int)(rbuf[1] << 16 | rbuf[2] << 8 | rbuf[3]);
                get_posi[1] = (int)(rbuf[4] << 16 | rbuf[5] << 8 | rbuf[6]);
                // get_posi[2] = (int)(rbuf[7] << 16 | rbuf[8] << 8 | rbuf[9]);
                get_posi[2] = (int)(rbuf[7] << 24 | rbuf[8] << 16 | rbuf[9] << 8 | rbuf[10]);
                // obj_inf_B = (int)rbuf[13];
                // obj_R1 = (int)rbuf[10];
                obj_R11 = (int)rbuf[11];
                obj_R2 = (int)rbuf[12];
                obj_R22 = (int)rbuf[13]; 
                unsigned char stateData[3] = { rbuf[11], rbuf[12], rbuf[13] };
                // obj_inf_B1 = (int)rbuf[13];
                // obj_inf_B = (uint16_t)(rbuf[13] | ((rbuf[14] & 0x0F) << 8));

                for (int i = 0; i < 12; i++) {
                    // obj_inf[i] = (obj_inf_B >> (11 - i)) & 0x01; // 各ビットをbool型に展開
                    // if(i < 8){
                    //     obj_true_R1[i] = (obj_R1 >> i) & 0x01;
                    //     obj_true_R2[i] = (obj_R2 >> i) & 0x01;
                    // }else{
                    //     obj_true_R1[i] = (obj_R11 >> (i - 8)) & 0x01;
                    //     obj_true_R2[i] = (obj_R22 >> (i - 8)) & 0x01;
                    // }
                    int byteIndex = i / 4;       // どのバイトか
                    int bitShift = (i % 4) * 2;  // 2bitの位置
                    obj_state[i] = (stateData[byteIndex] >> bitShift) & 0x03; // 下位2bitだけ取り出す
                    
                    // obj_inf[i] = (obj_inf_B1 >> i) & 0x01;
                    // astar.obj_true[i] = obj_inf[i];//autocontrollの方にしまう
                    if(obj_state[i] == 1){
                        obj_true_R1[i] = true;
                    }else if(obj_state[i] == 2){
                        obj_true_R2[i] = true;
                    }else if(obj_state[i] == 3){
                        obj_true_Fake[i] = true;
                    }else{
                        obj_true_R1[i] = false;
                        obj_true_R2[i] = false;
                        obj_true_Fake[i] = false;
                    }
                    // autonomous.button_true[i] = obj_inf[i];
                    autonomous.button_true_R1[i] = obj_true_R1[i];
                    autonomous.button_true_R2[i] = obj_true_R2[i];
                    autonomous.button_true_Fake[i] = obj_true_Fake[i];
                }

                //  for (int i = 0; i < 12; i++) {
                //     printf("obj_inf[%d] = %d\n", i, obj_inf[i]);
                // }

                gPosi.x = (double)get_posi[0] / 1000;
                gPosi.y = (double)get_posi[1] / 1000;
                gPosi.z = (double)get_posi[2] / 1000;

                comm_check = true;

                while(pc.readable()){
                    pc.getc();
                }
            }
            else{
                comm_check = false;
            }
        } else {//通常の読み取り
            rbuf[readCount] = c;//一時的にデータを保存
            // pc.printf("%d,",c);
            if(readCount < 14) checksum ^= rbuf[readCount];//チェックサムの計算
            sbuf2[readCount] = checksum;
        }               
        readCount++;//読み取る要素番号の加算
        if(readCount > 99) readCount = 99;//読み取り要素数が100を越えそうになったときには加算しない
    }
    return comm_check;
}

//pcへの送信
bool sendData_pc(char data[],int n){
    char checksum = 0;

    for(int i = 0;i < n;i++){
        pc.putc(data[i]);
        checksum ^= data[i];
    }
    pc.putc(checksum);
    pc.putc('\n');

    return true;
}
//pcからの受け取り
bool receiveData_pc(int mode){
    char c;
    char c_;
    char checksum = 0;
    int readCount = 0;
    char buf[100];
    bool comm_check = false;

    while(pc.readable()){
        c = pc.getc();
        if(c == '\n' && (readCount == 3)){//終端コードのチェック
            if(buf[2] == checksum){//通信数とチェックサムの確認
                // receive_collect_ball[0] = (int)buf[0];
                // receive_remove_dir = (int)buf[1];
                // receive_ball_posi[0][0] = ((intbuf[2] << 8) | (int)buf[3];
                // receive_ball_posi[0][1] = ((int)buf[4] << 8) | (int)buf[5];
                // receive_remove_dist[0] = ((int)buf[6] << 8) | (int)buf[7];
                // receive_collect_angle[0] = (int)buf[8];
                // receive_pr_count = (int)buf[9];
                comm_check = true;
                while(pc.readable()){
                    pc.getc();
                }
            }else{
                comm_check = false;
            }
            break;
        } else {//通常の読み取り
            buf[readCount] = c;//一時的にデータを保存
            // pc.printf("%d,",c);
            if(readCount < 2) checksum ^= buf[readCount];//チェックサムの計算
        }               
        readCount++;//読み取る要素番号の加算
        if(readCount > 99){
            readCount = 99;//読み取り要素数が100を越えそうになったときには加算しない
            comm_check = false;
        }
    }

    stolen_receive_len = readCount;
    while(pc.readable()) c_ = pc.getc();
    return comm_check;
}

int readButton(
    unsigned int
        ButtonNum) { //放しているときは０，押しているときは１，押した瞬間は２，放した瞬間は－１
  int result = 0;
  if ((buttonState_E & (0x0001 << ButtonNum)) == (0x0001 << ButtonNum))
    result += 2;
  if ((pre_buttonState_E & (0x0001 << ButtonNum)) == (0x0001 << ButtonNum))
    result -= 1;
  return result;
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

int meas_time(){
    static int pre_time = 0;
    int time = timer.read_ms();
    int interval = time - pre_time;
    pre_time = time;
    return interval;
}

//数値計算関数---------------------------------------------------
double height_count(int enc, int pre_enc, double wheel_r){
        // エンコーダのカウント値から角度の変化量を計算する
        double ang = (double)( enc - pre_enc) * _2PI_RES4;
        double diff = wheel_r * ang; //RADIUS_X はエンコーダの車輪半径
        // グローバル座標系での変化量に変換し，これまでのデータに加算することで自己位置推定完了
        double dist = diff;
        // 1サンプル前のデータとして今回取得したデータを格納
        pre_enc = enc;
    return dist;
}

//段越え昇降　エンコーダ値換算
#define LIFT_ANGLE_RES 16383//エンコーダの分解能（1回転で何カウント増えるか）
#define GEAR_RATIO 1 //1:1なら1
#define FRONT_R 10.0//mm

// double enc_to_lift_height(int enc, int pre_enc, double liner_per_r) {//liner_per_rは半径 mm
//     int diff_count = enc - pre_enc;//カウント差
    
//     if (diff_count >  LIFT_ANGLE_RES / 2) diff_count -= (int)LIFT_ANGLE_RES;
//     if (diff_count < -LIFT_ANGLE_RES / 2) diff_count += (int)LIFT_ANGLE_RES;

//     double theta = (double)diff_count / LIFT_ANGLE_RES * 2*M_PI;
//     double diff = (theta * liner_per_r) / GEAR_RATIO;
//     if(diff_count == 0) diff = 0;
//     // pc.printf("%d",diff_count);
//     // pc.printf("%lf  ", theta);
//     // pc.printf("%lf", diff);

//     return diff;
// }

double enc_to_lift_height(int enc, int pre_enc, double wheel_r){
        // エンコーダのカウント値から角度の変化量を計算する
        double ang = (double)( enc - pre_enc)  * (2*M_PI / LIFT_ANGLE_RES);//エンコーダ100 PPRなら1,200 CPRなら2逓倍，400 CPRなら4逓倍，
        double diff = wheel_r * ang / GEAR_RATIO; //RADIUS_X はエンコーダの車輪半径
        // 1サンプル前のデータとして今回取得したデータを格納
        // pre_enc = enc;
    return diff;//これをこれまでのデータに加算する
}
// double dist = 0;
double dist_count(int enc, int pre_enc, double wheel_r){
        // エンコーダのカウント値から角度の変化量を計算する
        double ang = (double)( enc - pre_enc) * _2PI_RES4;
        double diff = wheel_r * ang; //RADIUS_X はエンコーダの車輪半径
        // グローバル座標系での変化量に変換し，これまでのデータに加算することで自己位置推定完了
        // dist += diff;
        // 1サンプル前のデータとして今回取得したデータを格納
        pre_enc = enc;
    return diff;//これをこれまでのデータに加算する
}

double rpm_to_ms(double rpm, double roller_r){//rpmからm/s取得
    return roller_r * 2 * M_PI * rpm / 60;
}

double ms_to_rpm(double vel_ms, double roller_r){//m/sからrpm取得
    return vel_ms / (roller_r * 2 * M_PI) * 60;//ローラ半径roller_r [m]
}

double ms_to_rps(double vel_ms, double roller_r){//m/sからrps取得
    return vel_ms / (roller_r * 2 * M_PI);
}

double rps_to_rpm(double rps){
    return rps * 60;
}

double rpm_to_rps(double rpm){
    return rpm / 60;
}

double deg2rad(double deg){ //ラジアン変換
    return M_PI * deg / 180;
}

double rad2deg(double rad){//度数変換
    return 180 * rad / M_PI;
}

double pi(double angle){ //-PI ~ PIに変換
    int cor_rotate = int(angle / M_PI);
    return angle - cor_rotate * 2 * M_PI;
}
//-PI ~ PIに変換
double count2rad(int count, int init_count, int res){
    int count_ = count - init_count;
    if(count_ > (res / 2)) count_ -= res;
    else if(count_ < -(res / 2)) count_ += res;
    return 2 * M_PI * count_ / res;
}

int sign(double val){
    if(val >= 0.0) return 1;
    else return -1;
}

int convert_qpps(double rps, int ppr, double d) { //rps,分解能,半径
  // roboclawでの速度指令をQPPSで
  int qpps = 0;
//   qpps = double(vel * ppr) * 4.0 / (d * M_PI); //ローラーの1sごとに進む距離
  qpps = double(rps * ppr) * 4.0 ;
  return qpps;
}

//rps立ち上がり用関数
// double get_rais_smooth(double inc, double T, double A){//時間，立ち上がり時間，目標値
//     t_rise += inc / 2;
//     if (t_rise < 0) return 0;                     // 負の時間は0
//     else if (t_rise < T) return A * (t_rise / T) * (t_rise / T);  // 二次関数で増加
//     else return A;//最終的な値
// }

// //rps下がり関数
// double get_down_smooth(double inc, double T, double A){//時間，立ち上がり時間，目標値
//     t_down += inc / 2;
//     if (t_down < 0) return A;                     // 負の時間はA
//     else if (t_down < T) return A * (1.0 - (t_down / T) * (t_down / T));  // 二次関数で減少
//     else return 0;//最終的な値
// }

//自己位置z正規化
double normalize_pi(double angle){
    if (angle > M_PI) angle -= 2 * M_PI;
    else if (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}



//モータ制御関数-------------------------------------------------
//rightカメラ
void rotate_cam1(double pitch, double yaw){ // rad
    if(field_s == BLUE) yaw = -yaw;
    int pitch_dir = 1;
    int pitch_cmd = CAM_INIT_PITCH + pitch_dir * SERVO_DIR0 * (int)(SERVO_RES * pitch / 2 / M_PI);
    int yaw_cmd = CAM_INIT_YAW + SERVO_DIR1 * (int)(SERVO_RES * yaw / 2 / M_PI);

    if(pitch_cmd > CAM_MAX_PITCH) pitch_cmd = CAM_MAX_PITCH;
    else if(pitch_cmd < CAM_MIN_PITCH) pitch_cmd = CAM_MIN_PITCH;
    if(yaw_cmd > CAM_MAX_YAW) yaw_cmd = CAM_MAX_YAW;
    else if(yaw_cmd < CAM_MIN_YAW) yaw_cmd = CAM_MIN_YAW;
    // sprintf(str,"pitch:%d yaw:%d ",pitch_cmd,yaw_cmd);
    // invoke_print(str);

    Servo.cmd(SERVO_ID0, pitch_cmd, 0, 0);
    wait_us(50);
    Servo.cmd(SERVO_ID1, yaw_cmd, 0, 0);
    wait_us(50);
}
//leftカメラ
void rotate_cam2(double pitch, double yaw){ // rad
    if(field_s == BLUE) yaw = -yaw;
    int pitch_dir = 1;
    int pitch_cmd = CAM_INIT_PITCH2 + pitch_dir * SERVO_DIR0 * (int)(SERVO_RES * pitch / 2 / M_PI);
    int yaw_cmd = CAM_INIT_YAW2 + SERVO_DIR1 * (int)(SERVO_RES * yaw / 2 / M_PI);

    if(pitch_cmd > CAM_MAX_PITCH2) pitch_cmd = CAM_MAX_PITCH2;
    else if(pitch_cmd < CAM_MIN_PITCH2) pitch_cmd = CAM_MIN_PITCH2;
    if(yaw_cmd > CAM_MAX_YAW2) yaw_cmd = CAM_MAX_YAW2;
    else if(yaw_cmd < CAM_MIN_YAW2) yaw_cmd = CAM_MIN_YAW2;
    // sprintf(str,"pitch:%d yaw:%d\n",pitch_cmd,yaw_cmd);
    // invoke_print(str);

    Servo.cmd(SERVO_ID2, pitch_cmd, 0, 0);
    wait_us(50);
    Servo.cmd(SERVO_ID3, yaw_cmd, 0, 0);
    wait_us(50);
}

int ms_qpps(double vel, int ppr, double d, double gia){//m/s ,分解能 ,半径 ,ギア比
    int qpps;
    return qpps = int(2 * vel * gia * ppr / d / M_PI);
}


bool pre_SW1 = false;
bool pre_SW4 = false;

int flag_lift = 0;

//************************↓本文↓***********************************************//
int main() {
    if(USE_CONTROLLER){
        flag_use_controller = true;
    }
    if(USE_SENSOR){
        flag_use_sensor = true;
    }
    if(SIMULATION_MODE){
        set_print = false;
        flag_simu = true;
        autonomous.flag_simu = flag_simu;
    }
    if(USE_SDCARD){
        flag_use_sdcard = true;
    }
    if(USE_PC){
        pc_comm = true;
    }

    LEDblink(pin_led1, LEDBLINKING_TIME, LEDBLINKING_INTERVAL);
    LEDblink(pin_LED_B, LEDBLINKING_TIME, LEDBLINKING_INTERVAL_START);
    LEDblink(pin_led3, LEDBLINKING_TIME, LEDBLINKING_INTERVAL);
    LEDblink(pin_LED_B, LEDBLINKING_TIME, LEDBLINKING_INTERVAL_START);
    sprintf(str,"\n[INFO] set up start\n");
    invoke_print(str);

    USER_LED = false;

    coords init_pos = {0.0, 0.0, 0.0}; //自己位置の設定
    coords stopVel = {0.0, 0.0, 0.0};  //速度指令0用
    platform.platformInit(init_pos);   //取り敢えずの初期化

    
    
    
    // if(odrv_init){
    //     sprintf(str,"[INFO] init_true\n");
    //     invoke_print(str);
    // }
    sprintf(str,"[INFO] platform init done\n");
    invoke_print(str);
    platform.freeWheel();
    sprintf(str,"[INFO] set stopVel\n");
    invoke_print(str);

    led1 = led2 = led3 = led4 = 1;

    int ledloopCount = 0;

    if(flag_use_controller){
        sprintf(str,"[INFO] wait controller\n");
        invoke_print(str);
        do { //コントローラが繋がるまで（最低3回は点滅させる）　拡張ボタン基板のled
            if (!dip[0].read()) led1 = !led1;
            if (!dip[1].read()) led2 = !led2;
            if (!dip[2].read()) led3 = !led3;
            if (!dip[3].read()) led4 = !led4;
            LED_G = !LED_G.read();
            ThisThread::sleep_for(30);
            con.update();
            ledloopCount++;
        } while (ledloopCount <= 5 || !con.available());
    }
  
    led1 = led2 = led3 = led4 = LED_G = 0; // ledを一度消す

    LEDblink(pin_led2, LEDBLINKING_TIME, LEDBLINKING_INTERVAL); // setup の進捗具合を表示
        
    sprintf(str,"[INFO] connected controller13122222222222622\n");
    invoke_print(str);

    joyLX_filter.setLowPassPara(MANUAL_LOWPASS_T, 0.0);
    joyLY_filter.setLowPassPara(MANUAL_LOWPASS_T, 0.0);
    joyRX_filter.setLowPassPara(MANUAL_LOWPASS_T, 0.0);
    joyRY_filter.setLowPassPara(MANUAL_LOWPASS_T, 0.0);
    sprintf(str,"[INFO] waitgyro\n");
    invoke_print(str);

    posiPID_x.PIDinit(0.0, 0.0);//joystickの傾き具合で決めたい。            
    posiPID_y.PIDinit(0.0, 0.0);
    posiPID_z.PIDinit(0.0, 0.0);
    PID_lift_front_up.PIDinit(0.0, 0.0);
    PID_lift_front_down.PIDinit(0.0, 0.0);
    PID_lift_back_up.PIDinit(0.0, 0.0);
    PID_lift_back_down.PIDinit(0.0, 0.0);
    sprintf(str,"[INFO] waitgyro\n");
    invoke_print(str);
    if(!dip[2].read())
        client.init(B_ADDRESS,B_PORT);
    sprintf(str,"[INFO] waitgyro\n");
    invoke_print(str);
    // client.init(B_ADDRESS,B_PORT);
    sprintf(str,"[INFO] waitgyro\n");
    bool sw_state = false;
    sprintf(str,"[INFO] waitgyro\n");
    invoke_print(str);

    wait_us(100);
    if(flag_use_sensor){
        sprintf(str,"[INFO] waitgyro14\n");
        invoke_print(str);
        if(lpms.init() == 1){
            sprintf(str,"[INFO] waitlpms2\n");
            invoke_print(str);
        }else{
            if(USE_DIP){
                while (1) {
                    static int reset_count = 0;
                    if(btn[4].read() == 0) reset_count++;
                    else reset_count = 0;
                    if(reset_count > 66) {
                        NVIC_SystemReset();
                    }
                }
                
            }
        }
        // if(bno085.init()) printf("init\n");
        // spi.format(8,3);
        // spi.frequency(2000000);
        // bno085.init();

        // bno085.input_report1();
        // if(bno.init() == 1){
        //     sprintf(str,"[INFO] lpms init done\n");
        //     invoke_print(str);
        // }else {
        //     sprintf(str,"[INFO] lpms not  init done\n");
        //     invoke_print(str);
        // }
        sprintf(str,"[INFO] gyro init done!\n");
        invoke_print(str);  
    } 

    sprintf(str,"[INFO] HC-RS04 init done!\n");
    invoke_print(str);  


    encX.init();
    encY.init();
    enc_cam1_pitch.init();
    enc_cam1_yaw.init();
    enc_cam2_pitch.init();
    enc_cam2_yaw.init();
    enc_lift_front_angle.init();
    enc_lift_back_angle.init();
    sprintf(str,"[INFO] enc init\n");
    invoke_print(str);

    if(flag_use_sdcard){
        while(mySD.init() != 0){
            USER_LED = !USER_LED;
            // printf("[INFO] please insert the SDcard\n");
            sprintf(str,"[INFO] please insert the SDcard\n");
            invoke_print(str);
        }
    }
    USER_LED = false;

    sprintf(str,"[INFO] mysdmain init done\n");
    invoke_print(str);
    
    // led[0] = led[1] = led[2] = 0;
    //int ledloopCount = 0;
    bool flag_init = false;
    int setup_count = 0;
    
    //dipの確認（ON -> 0, OFF -> 1）
    sprintf(str,"[INFO] dip [0]:%d   [1]:%d   [2]:%d   [3]:%d    \n",dip[0].read(),dip[1].read(),dip[2].read(),dip[3].read());
    invoke_print(str);
    // sprintf(str,"[INFO] SW  [0]:%d   [1]:%d   [2]:%d   [3]:%d   [3]:%d   \n",Board_SW[0].read(),Board_SW[1].read(),Board_SW[2].read(),Board_SW[3].read(),Board_SW[4].read());
    // invoke_print(str);
    sprintf(str,"[INFO] press LEFT button3\n");//kakuninn
    invoke_print(str);
    
    if(flag_simu && !flag_use_controller ){
        do {
            char buf[256];
            // printf("recievedata:%d",receiveData_maicon());
            if (receiveData_maicon()) {
                if (readButton(SIMU_BUTTON_RIGHT) == -1) {
                    pre_buttonState_E = buttonState_E;
                    int num = sprintf(sbuf,"0.0,0.0,0.0,gPosi.y,gPosi.x,gPosi.z,refV.y,refV.x,refV.z,preangle,"
                                "phase,pathnum,t_be,syusoku,onx,ony,angle,Px,Py,lim_refVz,acc_pro\n");
                    // n = client.sendData(sbuf, num);
                    sendData_maicon(sbuf, num);
                    break;
                }
                pre_buttonState_E = buttonState_E;
            }
            int num = sprintf(sbuf, "wait press button\n");
            sendData_maicon(sbuf, num);
            sprintf(str,"[INFO] wait press button\n");
            invoke_print(str);
            ThisThread::sleep_for(20);
        } while (1);
    }else{
        if(flag_use_controller){
            while(true){
                static int buttonState = 0x0000;
                con.update();
                buttonState = con.getButtonState();
                sprintf(str,"[INFO] buttonstate:%d\n",buttonState);//kakuninn
                invoke_print(str);
                //sw_state = !Board_SW[0].read();
                // for (int i = 0; i < 3; i++) {
                //     if (!dip[i]) {
                //         led[i] = 1;
                //         dipSwState |= 1 << i;
                //     } else {
                //         led[i] = 0;
                //         dipSwState &= ~(1 << i);
                //     }
                // }
                if(ledloopCount == 10){
                    LED_R = 1;
                    LED_G = 1;
                    LED_B = !LED_B.read();
                    ledloopCount = 0;
                }

                if(con.readButton(LEFT) == -1 || btn[0].read() == 0) {//leftbutton
                    pc.printf("break\n");
                    break;
                }else if(con.readButton(RIGHT) == -1 || btn[3].read() == 0){
                    flag_init = true;
                    autonomous.flag_retry = true;
                    break;
                }else if(con.readButton(UP) == -1 || btn[2].read() == 0){
                    flag_init = true;
                    autonomous.flag_retry_forest = true;
                    break;
                }

                ledloopCount++;
                //pre_sw_state = sw_state;
                // pre_boardSW_state = boardSW_state;
                ThisThread::sleep_for(50);
            }
        }
    }
    LED_R = 1;
    LED_G = 1;
    LED_B = 0;
    USER_LED = false;
    sprintf(str,"[INFO] wait bno\n");
    invoke_print(str);

    if(flag_use_sensor){
        // bno085.init();
        int lpms_count = 0;
        while(lpms_count < 50){
            // bno085.input_report();
            double dis_data = (double)lpms.get_z_angle();
            // wait_us(100);
            // bno085.up();
            // double dis_data = (double)bno085.anglez;
            if(lpms_count%10 == 0) LED_B = !LED_B.read();
            lpms_count++;
            ThisThread::sleep_for(20);
        }
        
        // pre_anglez = (double)bno085.anglez;
        // pre_angley = (double)bno085.angley;
        // pre_anglex = (double)bno085.anglex;
        pre_anglez = (double)lpms.get_z_angle();
        pre_angley = (double)lpms.get_y_angle();
        pre_anglex = (double)lpms.get_x_angle();
    }

sprintf(str,"[INFO]bno on\n");
            invoke_print(str);
    LED_R = 1;
    LED_G = 0;
    LED_B = 1;

//dipsw選択
    flag_auto = true;
    if(USE_DIP){
        if(dip[0] == 0){//lrtbのon/off
            lrtb_comm = true;
        }
        if(dip[1] == 0){//上半身との通信on/off
            up_comm = true;
        }
        if(dip[2] == 0){//pc（カメラ）との通信
            pc_comm = true;
        } 
        if(dip[3] == 0){//lidarとの通信⇒onでBLUEフィールド
            // lidar_comm = true;
            field_s = BLUE;
            autonomous.field_s = BLUE;
        }
    }else {
        lrtb_comm = false;
        up_comm = false;
        pc_comm = false;
        lidar_comm = false;
    }
    // while(1){
    //     printf("loop");
    //     bool odrive_init = platform.platformInit_Odrive();
    //     printf("init : %d",odrive_init);
    //     if(odrive_init){
    //         printf("break");
    //         break;
    //     }
    //     thread_sleep_for(100);
    // }
    // platform.platformInit_Odrive();
    sprintf(str,"[INFO] odriveinit\n");//kakuninn
    invoke_print(str);

//初期位置設定
    init_pos = {INIT_X, INIT_Y, INIT_Z};
    // init_pos = {RETRY_X, RETRY_Y, RETRY_Z};
    getPosi = platform.setPosi(init_pos);
    platform.enc_init(encX.getCount(), -encY.getCount());
    //platform.setangle(-1);

//SDカード読み込み
    if(flag_auto){
        autonomous.init(&mySD, path);
        sprintf(str,"[INFO] set autoControl parameter\n");
        invoke_print(str);
    }

    LEDblink(pin_LED_B, LEDBLINKING_TIME, LEDBLINKING_INTERVAL); //
    //flag_loop_start = true;
    LEDblink(pin_LED_B, LEDBLINKING_TIME, LEDBLINKING_INTERVAL); //
    //flag_loop_start = true;
    LEDblink(pin_led4, LEDBLINKING_TIME, LEDBLINKING_INTERVAL);

    sprintf(str,"[INFO] press PS button111\n");
    invoke_print(str);
    // led[0] = led[1] = led[2] = 0;
    ledloopCount = 0;
    setup_count = 0;
    //pre_sw_state = !Board_SW[0].read();
    // if(!flag_simu){
        if(flag_use_controller){
            while(true){
                con.update();
                //sw_state = !Board_SW[0].read();

                // for (int i = 0; i < 3; i++) {
                //     if (!dip[i]) {
                //         led[i] = 1;
                //         dipSwState |= 1 << (i + 4);
                //     } else {
                //         led[i] = 0;
                //         dipSwState &= ~(1 << (i + 4));
                //     }
                // }
                if(ledloopCount == 10){
                    LED_R = 1;
                    LED_G = !LED_G.read();
                    LED_B = 1;
                    ledloopCount = 0;
                }

                if(flag_init){
                    //if(!dip[1]) autonomous.flag_retry = true;
                    break;
                }
                // if(readBoardSW(addBoard_SW[2].read(),2,5)) {
                //     if(!addBoard_dip[1]) autonomous.flag_retry = true;
                //     break;
                // }
                if (con.readButton(PS) == -1 || btn[1].read() == 0 ) {
                    //if(!dip[1]) autonomous.flag_retry = true;
                    break;
                }else if (con.readButton(RIGHT) == -1) {
                    break;
                }

                ledloopCount++;
                //pre_sw_state = sw_state;
                ThisThread::sleep_for(50);
            }
        }
    // }
    sprintf(str,"[INFO] raise SD_flag\n");
    invoke_print(str);

    flag_SDwrite = true;
    

    if(autonomous.flag_retry){
        gPosi = {RETRY_X, RETRY_Y, RETRY_Z};
        getPosi = platform.setPosi(coords {RETRY_X, RETRY_Y, RETRY_Z});
        sprintf(str,"[INFO] retry start\n");
    }else if(autonomous.flag_retry_forest){
        gPosi = {RETRY_F_X, RETRY_F_Y, RETRY_F_Z};
        getPosi = platform.setPosi(coords {RETRY_F_X, RETRY_F_Y, RETRY_F_Z});
        sprintf(str,"[INFO] retry start\n");
    }else {
        sprintf(str,"[INFO] program start\n");
    }
    invoke_print(str);

    sprintf(str,"[INFO] program start!\n");
    invoke_print(str);

    // autonomous.route_num = 1;
    // autonomous.phase = 202;
    // mode = MODE_FOREST;

//割り込み処理の開始
  flipper.attach(&timer_warikomi, INT_TIME);
  sprintf(str,"[INFO] set timer_warikomi\n");
  invoke_print(str);
  timer.start();
  t1.start();
//   con.update();

  //**********************↓loop()↓**********************************************//
  while (true) {
    static int buttonState = 0x0000;
    static int addbuttonState = 0x0000;
    static bool addbuttonLeftState = 0x0000;
    static bool addbuttonRightState = 0x0000;
    static bool buttonA = con.M5ButtonA;
    static bool buttonB = con.M5ButtonB;
    static bool buttonC = con.M5ButtonC;
    static int pre_buttonState = 0x0000;
    static int pre_addbuttonState = con.addButtonState;
    static bool pre_addbuttonLeftState = 0x0000;
    static bool pre_addbuttonRightState = 0x0000;
    static bool pre_buttonA = buttonA;
    static bool pre_buttonB = buttonB;
    static bool pre_buttonC = buttonC;


    static bool bin_addbuttonState[12] = {
        0,0,0,0,0,0,0,0,0,0,0,0
    };
    static bool pre_bin_addbuttonState[12] = {
        0,0,0,0,0,0,0,0,0,0,0,0
    };
    static int SWcount[12] = {
        0,0,0,0,0,0,0,0,0,0,0,0
    };
    // static bool addbuttonState = 0x0000;
    // static bool addbuttonState = 0x0000;
    static int obj_count = 0;
    static int up = 0;
    static int con0_count = 0;
    static double joyLX = 0.0;
    static double joyLY = 0.0;
    static double joyRX = 0.0;
    static double joyRY = 0.0;
    static double joyLRad = 0;  // joystickの角度
    static double joyLHypo = 0; // joystickLの傾き量
    static double normalspeed_x = 0.0;
    static double normalspeed_y = 0.0;
    static double normalspeed_z = 0.0;
    static double joyState = 0.0;
    static bool local_mode = false;
    unsigned int nextPhase = 0;
    unsigned int overstep_nextPhase = 0;
    unsigned int downstep_nextPhase = 0;
    static int reset_count; //resetbutton押す時間
    static int time_count = 0;
    static int interval_time = 0;
    static int pre_interval_time = 0;
    static int interval_sum = 0;
    int button_num = 0;

    static double motor1_rps_plus = 10.0;
    static double motor2_rps_plus = 10.0;

    //フラグ
    static bool flag_error = false;
    static bool flag_path = true;
    bool flag_comm = true;
    bool flag_mode_change = false;
    

    static int steer_phase = 0; //ACねじれ

    bool flag_receive = false;
    static bool bno_up = false;


    static bool stepup_flag = false;
    static int stepup_count = 0;
    static bool stepdown_flag = false;
    static int stepdown_count = 0;
    // int flag_lift;
    // if(flag_use_sensor){
    //     // spi.format(8,3);
    //     // spi.frequency(2000000);
    //     num = bno085.read();
    //     // ThisThread::sleep_for(100);
        
    //     // wait_us(100);
    //     if(num == 0){
    //         bno085.up();
    //     }
    // }
    // if(flag_use_sensor){
    //     num = bno085.read();
    //     if(num == 0 && pre_num == 1){
    //         bno085.up();
    //     }
    //     pre_num = num;
    // }

    if (flag_int) {//INT_TIME毎に処理される．フラグ処理
        // interval_time = t1.read_us();
        // t1.reset();
        // interval_sum += interval_time;
        // printf("time:%d  \n",interval_time);
        // printf("time:%d  \n",tttt);
        // invoke_print(str);

        up = con.update();
            if(up == 0 && flag_use_controller){
                con0_count++;
            }else{
                con0_count = 0;
            }
            if(flag_use_controller && up){
            //if(!flag_use_controller){
                buttonState = con.getButtonState();//手動
                addbuttonState = con.addButtonState;
                addbuttonLeftState = con.addButtonLeft;
                addbuttonRightState = con.addButtonRight;
                for(int i = 0; i < 12; i++){
                    bin_addbuttonState[i] = con.readaddButton_bin(i + 1);
                }

                joyLX = joyLX_filter.LowPassFilter(con.readJoyLX());
                joyLY = joyLY_filter.LowPassFilter(con.readJoyLY());
                joyRX = joyRX_filter.LowPassFilter(con.readJoyRX());
                joyRY = joyRY_filter.LowPassFilter(con.readJoyRY());

                if (fabs(joyLX) <= JOYSTICK_DEAD_BAND)
                    joyLX = 0.0;
                if (fabs(joyLY) <= JOYSTICK_DEAD_BAND)
                    joyLY = 0.0;
                if (fabs(joyRX) <= JOYSTICK_DEAD_BAND)
                    joyRX = 0.0;
                if (fabs(joyRY) <= JOYSTICK_DEAD_BAND)
                    joyRY = 0.0;

                joyState = joyLX + joyLY + joyRY; 
                /*-------------自動処理----------------*/


                //ボタン処理------------------------------------------------------------------
                if(!flag_use_controller){
                    ControlMode = AUTO_MODE;
                }
                if(con.readButton(PS) == -1) {
                    flag_stop = false;
                    autonomous.phase = 0;
                    ControlMode = MANUAL_MODE;
                }
                if(con.readButton(L1) == -1) {
                    nextPhase = PUSH_BUTTON;
                    ControlMode = AUTO_MODE;
                }
                if(con.readButton(R1) == -1){//タイヤをフリー状態にする，速度0
                    // ControlMode = MANUAL_MODE;
                    // platform.freeWheel();
                }
                if(con.readButton(LEFT) == -1) {
                    flag_lift = 1;
                    if(conf_Mode == 2){//上げてから下す
                        first_front_up_flag = true;//最初にリミットスイッチまで上げるフラグ
                        first_back_up_flag = true;
                    }else if(conf_Mode == 3 || conf_Mode == 4){//段越えシーケンス
                        overstep_flag = true;
                        if(downstep_flag){
                            downstep_phase = 0;
                            downstep_flag = false;
                        }
                        // overstep_nextPhase = PUSH_BUTTON;
                        // ControlMode = AUTO_MODE;
                    }else if(conf_Mode == 5){
                        overstep_nextPhase = PUSH_BUTTON;
                    }
                }
                if(con.readButton(RIGHT) == -1) {
                    if(conf_Mode == 3 || conf_Mode == 4){//段降りシーケンス
                        downstep_flag = true;
                        if(overstep_flag){
                            overstep_phase = 0;
                            overstep_flag = false;
                        }
                        downstep_nextPhase = PUSH_BUTTON;
                        // ControlMode = AUTO_MODE;
                    }else if(conf_Mode == 5){
                        downstep_nextPhase = PUSH_BUTTON;
                    }
                }
                //押しっぱなし----------------
                if(con.readButton_bin(LEFT)) {
                    if(conf_Mode == 1){//昇降後ろ
                        roboclawCmd1 = 800;
                    }
                }
                if(con.readButton_bin(RIGHT)) {
                    if(conf_Mode == 1){
                        roboclawCmd1 = -800;
                    }
                }
                if(!con.readButton_bin(LEFT) && !con.readButton_bin(RIGHT)){
                    if(conf_Mode == 1){
                        roboclawCmd1 = 0;//昇降止める
                    }
                }
                if(con.readButton(UP) == -1) {//昇降前
                    // if(conf_Mode == 2){
                    //     // front_PID_flag = !front_PID_flag;
                    //     ref_lift_front_posi += 200;
                    // }
                    // if(conf_Mode == 2){//速度一緒か
                    //     up_flag = !up_flag;
                    //     if(up_flag){
                    //         down_flag = false;
                    //     }
                    // }
                    // air_up_flag = !air_up_flag;
                    if(conf_Mode == 5){
                        overstep_R1_flag = true;
                        if(downstep_R1_flag){
                            downstep_R1_phase = 0;
                            downstep_R1_flag = false;
                        }
                        overstep_nextPhase = PUSH_BUTTON;
                    }
                }
                if(con.readButton(DOWN) == -1){
                    // if(conf_Mode == 2){
                    //     // back_PID_flag = !back_PID_flag;
                    //     ref_lift_front_posi -= 200;
                    // }
                    // if(conf_Mode == 2){//速度一緒か
                    //     down_flag = !down_flag;
                    //     if(down_flag){
                    //         up_flag = false;
                    //     }
                    // }
                    // state_mode++;
                    // if(state_mode > 3) state_mode = 0;  // 1→2→3→1…
                    if(conf_Mode == 5){
                        downstep_R1_flag = true;
                        if(overstep_R1_flag){
                            overstep_R1_phase = 0;
                            overstep_R1_flag = false;
                        }
                        downstep_nextPhase = PUSH_BUTTON;
                    }
                }
                //押しっぱなし----------------
                if(con.readButton_bin(UP)) {//昇降前
                    if(conf_Mode == 1){
                        roboclawCmd0 = 800;
                    }
                }
                if(con.readButton_bin(DOWN)){//昇降
                    if(conf_Mode == 1){
                        roboclawCmd0 = -800;
                    }
                }
                if(!con.readButton_bin(UP) && !con.readButton_bin(DOWN)){
                    roboclawCmd0 = 0;//昇降止める
                }
                if(con.readButton_bin(R2)){//R2が押されているときのみ
                    flag_stop = false;
                }
                if(con.readButton_bin(L2)){//L2が押されているときのみ
                    flag_stop = true;
                }
                if(con.readButton(R2) == -1){//R2が離されたとき
                    stepup_count += 1;
                    // platform.platformInit_Odrive();
                }
                if(con.readButton(L2) == -1){//L2が離されたとき
                    stepup_flag = !stepup_flag;
                    // flag_stop = true;//速度0指令
                }
                if(con.readButton(MARU) == -1){
                //    if(conf_Mode == 2){//両方同時に動かす．→　PIDの速度が正しいか
                //        front_PID_flag = !front_PID_flag;
                //        back_PID_flag = !back_PID_flag;
                //    }
                    if(autonomous.phase == 62){
                        ControlMode = AUTO_MODE;//rack中段test
                        autonomous.rack_num = 0;
                        nextPhase = PUSH_BUTTON;
                    } 
                }
                if(con.readButton(BATU) == -1){
                    if(next_box_state == 1){
                        if(nextIndex >= 3 && nextIndex <= 14){
                            obj_state_test[nextIndex-3] = 0;
                        }
                    }
                    // if(conf_Mode == 1){
                    //     back_wheel_flag = !back_wheel_flag;
                    // }
                }
                if(con.readButton(SHIKAKU) == -1){
                    // state_mode++;
                    // if(state_mode > 3) state_mode = 0;  // 1→2→3→1…
                    if(autonomous.phase == 62){
                        ControlMode = AUTO_MODE;
                        autonomous.rack_num = 2;
                        nextPhase = PUSH_BUTTON;
                    }
                }
                if(con.readButton(SANKAKU) == -1){//段越えphaseを進める
                    // overstep_nextPhase = PUSH_BUTTON;
                    if(autonomous.phase == 62){
                        ControlMode = AUTO_MODE;
                        autonomous.rack_num = 1;
                        nextPhase = PUSH_BUTTON;
                    }
                }
                if(con.readButton(OPTION) == -1){
                }
                if(con.readButton(SHARE) == -1){
                }
                if(flag_SDwrite){
                    if(con.readButton(SHARE) == -1 || userButton.read() == 0){
                    //if(Board_SW[3].read() == 0){
                        if(flag_use_sdcard){
                            USER_LED = true;
                            LED_SDwrite = true;
                            sprintf(str,"make logFile\tNO.%d\n", mySD.make_logfile());
                            invoke_print(str);
                            flipper.detach();
                        }
                    }
                }
            /*addbutton--------------------------------------*/
            //障害物あり経路の試し用
            //回収→障害物の順で4つずつ選択する．
            bool sws[12] = {
                con.readaddButton_bin(SW1),
                con.readaddButton_bin(SW4),
                con.readaddButton_bin(SW6),
                con.readaddButton_bin(SW7),
                con.readaddButton_bin(SW10),
                con.readaddButton_bin(SW8),
                con.readaddButton_bin(SW2),
                con.readaddButton_bin(SW5),
                con.readaddButton_bin(SW3),//ここまで左上から順番に
                addbuttonLeftState,
                addbuttonRightState,
                con.readaddButton_bin(SW11)
            };

            // for (int i = 0; i < 12; i++) {
            //     bool current = sws[i];
            //     if (current) {
            //         if ((pre_addbuttonState != addbuttonState) || (addbuttonLeftState == 1 && pre_addbuttonLeftState == 0) || (addbuttonRightState == 1 && pre_addbuttonRightState == 0) && obj_state[i] == false) {
            //             if (obj_count < 8) obj_count++;
            //             SWcount[i]++;
            //             if(state_mode == 1){
            //                 obj_state[i] = 1;
            //             }else if(state_mode == 2){
            //                 obj_state[i] = 2;
            //             }else if(state_mode == 3){
            //                 obj_state[i] = 3;
            //             }else if(state_mode == 0){
            //                 obj_state[i] = 0;
            //             }   
            //         }
            //     } 
            //     if(obj_state[i] == 1){
            //         obj_true_R1[i] = true;
            //     }else if(obj_state[i] == 2){
            //         obj_true_R2[i] = true;
            //     }else if(obj_state[i] == 3){
            //         obj_true_Fake[i] = true;
            //     }else{
            //         obj_true_R1[i] = false;
            //         obj_true_R2[i] = false;
            //         obj_true_Fake[i] = false;
            //     }
            // }

            //設定個数と判別のリセット
            // if(con.readaddButton_bin(SW9)){
            //     if(pre_addbuttonState != addbuttonState){
            //         for (int i = 0; i < 12; i++) {
            //             obj_state[i] = 0;
            //             SWcount[i] = 0;
            //         }
            //         for(int i = 0; i < 4; i++){
            //             // astar.collect_num[i] = 0;
            //         }
            //         obj_count = 0;
            //     }
            // }

            // if(obj_count > 12){
            //     obj_count = 0;
            // }

                pre_buttonState = buttonState;
                pre_addbuttonState = addbuttonState;
                pre_addbuttonLeftState = addbuttonLeftState;
                pre_addbuttonRightState = addbuttonRightState;
                pre_buttonA = buttonA;
                pre_buttonB = buttonB;
                pre_buttonC = buttonC;
                // pre_addbutton_L = addButton_L;
                // pre_addbutton_R = addButton_R;
                // for(int i = 0; i < 12; i++){
                //     pre_bin_addbuttonState[i] = bin_addbuttonState[i];
                // }
            }

    //simuボタン------------------------------------------------------------------------------------
        // if(flag_simu){
        //     if(receiveData_maicon()){
        //         if (readButton(SIMU_BUTTON_RIGHT) == -1) {
        //             nextPhase = PUSH_BUTTON;
        //             ControlMode = AUTO_MODE;
        //         }else if (readButton(SIMU_BUTTON_LEFT) == -1) {
        //             ControlMode = MANUAL_MODE;
        //         }
        //         if (readButton(SIMU_BUTTON_UP) == -1) {
        //             autonomous.collect_count++;
        //         }
        //         if (readButton(SIMU_BUTTON_DOWN) == -1) {
        //             autonomous.collect_count--;
        //         }
        //         flag_comm = true;
        //     }
        // }

        //SDカード--------------------------------------------------------------------------------
        if(flag_SDwrite && LED_SDwrite){//SDカード書き込み
            flipper.detach();
            LED_R = 0;
            USER_LED = false;
            // sprintf(str, "int_time,gPosi.y,gPosi.x,gPosi.z,,refV.y,refV.x,refV.z,,preangle,"
            //      "phase,pathnum,t_be,syusoku,onx,ony,angle,,Px,Py,,lrtb.y,lrtb.x,lrtb.z,enc0,enc1,angle,dist,dete,n_speed,lidat_a,,,,,getx,gety,upenc0,upenc1\n");
            // sprintf(str, "time,time_sum,gPosi.y,gPosi.x,gPosi.z,enc3_0,enc3_1,enc4_0,enc4_1,upenc0,upenc1,get_anglex,get_angley,rps1,rps2,errorstate\n");
            // sprintf(str, "time,phase,syusoku,gPosi.y,gPosi.x,gPosi.z,,refV.y,refV.x,refV.z,,lrtb.y,lrtb.x,lrtb.z,Py,Px,,send_num,air,kouden1,kouden2,kouden3,cubeid,nextid, overstep, downstep, holdstep,,cmd0,cmd1,cmd2,,mode,nbox,,limit,,up,lift,c_M,back,rad,ref,,\n");
            // sprintf(str, "time,phase,syusoku,gPosi.y,gPosi.x,gPosi.z,,refV.y,refV.x,refV.z,,lrtb.y,lrtb.x,lrtb.z,Py,Px,,send_num,air,kouden1,kouden2,kouden3,cubeid,nextid, overstep, downstep, holdstep,,cmd0,cmd1,cmd2,,mode,nbox,,limit,,up,lift,c_M,back,rad,ref,,\n");
            // sprintf(str, "time,phase,syusoku,gPosi.y,gPosi.x,gPosi.z,,refV.y,refV.x,refV.z,,lrtb.y,lrtb.x,lrtb.z,Py,Px,,send_num,air,kouden1,front_syusoku,kouden3,back_syusoku,back_wheel_flag, stepup_count, stepdown_count, holdstep,,cmd0,cmd1,cmd2,,mode,nbox,,limit,,up,lift,c_M,back,rad,ref, ,\n");
            sprintf(str, "time,,gPosi.y,gPosi.x,gPosi.z,refV.y,refV.x,refV.z,,phase,getPathNum,get_t_be,onx,ony,angle,Px(3),Py(3),,lrtbPosi.y,lrtbPosi.x,lrtbPosi.z,,roboclawCmd0,roboclawCmd1,roboclawCmd2,ref_lift_front_posi,ref_lift_back_posi,front_lift_posi,");
            mySD.write_logdata(str);
            sprintf(str, "back_lift_posi,,front_syusoku,back_syusoku,stepup_flag,stepup_count,stepdown_flag,stepdown_count,kouden1read,kouden2read,kouden3read,air_state,limit4read,limit5read,up_num,send_num,route_num,route[autonomous.route_num].num,,cubeIndex,direction_flag,setx,sety,setz,,cubeIndex\n");
            mySD.write_logdata(str);
            // sprintf(str, "gPosi.y,gPosi.x,gPosi.z,refV.y,refV.x,refV.z,pre_angle,autonomous.phase,autonomous.getPathNum(),autonomous.get_t_be(),autonomous.onx(),autonomous.ony(),autonomous.angle()");
            // mySD.write_logdata(str);
            // sprintf(str,",autonomous.Px(3),autonomous.Py(3),autonomous.syusoku,timer.read_ms(),lrtbPosi.y,lrtbPosi.x,lrtbPosi.z,stepup_count,stepup_flag,stepdown_count,stepdown_flag,front_syusoku,back_syusoku,roboclawCmd0,roboclawCmd1,roboclawCmd2,air_up_flag");
            // mySD.write_logdata(str);
            // sprintf(str, "autonomous.send_num,kouden1read,kouden2read,kouden3read,cubeIndex,nextIndex,overstep_phase,downstep_phase,hold_phase,mode,next_box_state,limit4read,limit5read,autonomous.up_num");
            // mySD.write_logdata(str);
            // sprintf(str, ",autonomous.route_num,route[autonomous.route_num].num,back_wheel_flag,autonomous.rotate_radian,ref_lift_front_posi,ref_lift_back_posi,front_lift_posi,back_lift_posi\n");

            // mySD.write_logdata(str);
            int i = 0;
            while (i < SDcount) {
                USER_LED = !USER_LED;
            //     sprintf(str,
            //   "%d,%lf,%lf,%lf,,%lf,%lf,%lf,,%lf,%d,%d,%lf,%d,%lf,%lf,%lf,,%lf,%lf,,%lf,%lf,%lf,%d,%d,%lf,%lf,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d,%d,%d,%d\n",
            //   e[i], A[i], B[i], C[i], D[i], E[i], F[i], K[i], a[i], b[i], G[i], d[i], H1[i], I[i], J[i], L4[i], N[i], M[i], O[i], P[i], f[i] ,g[i],Q[i],R[i],f[i],S[i],T[i],U[i],V[i],W[i],X[i],Y[i],h[i],i_[i],j[i],k[i]);
            // sprintf(str,"%d,%d,%lf,%lf,%lf,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",e[i],n[i],A[i],B[i],C[i],l[i],m[i],f[i],g[i],j[i],k[i],h[i],i_[i],a[i]);
                // sprintf(str,"%d,%d,%d,%lf,%lf,%lf,,%lf,%lf,%lf,,%lf,%lf,%lf,%lf,%lf,,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,,%lf,%lf,%lf,,%d,%d,,%d,%d,%d,%d,%d,%d,%lf,%lf,%lf ,%lf,%lf\n",e[i], a[i], d[i], A[i], B[i], C[i], D[i], E[i], F[i], M[i], O[i], P[i], N[i], L4[i], g[i], f[i], h[i], j[i], p[i], k[i], l[i], m[i], n[i], o[i], V[i], R[i], W[i],q[i], r[i],s[i],t[i],u[i],v[i],r[i],w[i],X[i],Y[i],Z[i],Q[i],G[i]);
                // mySD.write_logdata(str);
                // sprintf(str,"%d,%d,%d,%lf,%lf,%lf,,%lf,%lf,%lf,,%lf,%lf,%lf,%lf,%lf,,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,,%lf,%lf,%lf,,%d,%d,,%d,%d,%d,%d,%d,%d,%lf,%lf,%lf ,%lf,%lf ,%lf ,%d\n",e[i], a[i], d[i], A[i], B[i], C[i], D[i], E[i], F[i], M[i], O[i], P[i], N[i], L4[i], g[i], f[i], h[i], j[i], p[i], k[i], l[i], m[i], n[i], o[i], V[i], R[i], W[i],q[i], r[i],s[i],t[i],u[i],v[i],r[i],w[i],X[i],Y[i],Z[i],Q[i],G[i],T[i],c[i]);
                // sprintf(str,"%d,,%lf,%lf,%lf,%lf,%lf,%lf,,%d,%d,%lf,%lf,%lf,%lf,%lf,%lf,,%lf,%lf,%lf,,%d,%d,%d,%lf,%lf,%lf,%lf,,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,,%d,%d,%lf,%lf,%lf,%d,%d\n",z[i],A[i],B[i],C[i],D[i],E[i],F[i],a[i],b[i],G[i],H1[i],I[i],J[i],K[i],L4[i],M[i],N[i],O[i],c[i],d[i],e[i],P[i],Q[i],R[i],S[i],f[i],g[i],h[i],j[i],k[i],l[i],m[i],n[i],o[i],p[i],q[i],r[i],s[i],t[i],u[i],v[i],w[i],T[i],U[i],V[i],W[i],X[i],x[i],y[i]);
                // mySD.write_logdata(str);
                sprintf(str,"%d,,%lf,%lf,%lf,%lf,%lf,%lf,,%d,%d,%lf,%lf,%lf,%lf,%lf,%lf,,%lf,%lf,%lf,,%d,%d,%d,%lf,%lf,%lf,%lf,,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,,%d,%d,%lf,%lf,%lf,%d,%d\n",z[i],A[i],B[i],C[i],D[i],E[i],F[i],a[i],b[i],G[i],H1[i],I[i],J[i],K[i],L4[i],M[i],N[i],O[i],c[i],d[i],e[i],P[i],Q[i],R[i],S[i],f[i],g[i],h[i],j[i],k[i],l[i],m[i],a1[i],n[i],o[i],p[i],q[i],r[i],s[i],t[i],u[i],v[i],w[i],T[i],U[i],V[i],W[i],X[i],x[i],y[i]);
                mySD.write_logdata(str);
                i++;
            }
            USER_LED = false;
            mySD.close_logdata();

            flag_SDwrite = false;
            LED_SDwrite = false;
            flipper.attach(&timer_warikomi, INT_TIME);
        }

        //拡張ボタン-----------------------------------------------------------------------------
        if(USE_DIP){
            //btnの値保存
            btn0read = btn[0].read();
            btn1read = btn[1].read();
            btn2read = btn[2].read();
            btn3read = btn[3].read();
            btn4read = btn[4].read();
            //reset-----------------------------
            if(btn[4].read() == 0) reset_count++;
            else reset_count = 0;
            if(reset_count > 66) {
                NVIC_SystemReset();
            }
            
            //btn処理--------------------------------
            if(pre_btn2read == 0 && btn1read == 1){
                // air_up_flag = !air_up_flag;
                // conf_Mode = 5;
            }

            pre_btn0read = btn0read;
            pre_btn1read = btn1read;
            pre_btn2read = btn2read;
            pre_btn3read = btn3read;
            pre_btn4read = btn4read;
        }
      //pc.printf("time2:%d\t",timer.read_ms());
      //timer.reset();

        //自己位置--------------------------------------------------------------------------
        if(!flag_simu){
        // if(false){
            //機構自己位置----------------------------------------------------------------
            static bool first_count = false;
            //現在の中心回転軸位置読み取り用
            // printf("自己位置");
            // spi.format(8,0);
            // spi.frequency(1000000);
            enc_cam1_pitch.getRawEncount();
            enc_cam1_yaw.getRawEncount();
            enc_cam2_pitch.getRawEncount();
            enc_cam2_yaw.getRawEncount();
            enc_lift_front_angle.getRawEncount();
            enc_lift_back_angle.getRawEncount();
            //エンコーダカウント値
            get_cam1_pitch_count = enc_cam1_pitch.getAbsCount();
            get_cam1_yaw_count = enc_cam1_yaw.getAbsCount();
            get_cam2_pitch_count = enc_cam2_pitch.getAbsCount();
            get_cam2_yaw_count = enc_cam2_yaw.getAbsCount();
            // get_lift_front_count = enc_lift_front_angle.getAbsCount();
            // get_lift_back_count = enc_lift_back_angle.getAbsCount();
            get_lift_front_count = -enc_lift_front_angle.getEncount();
            get_lift_back_count = -enc_lift_back_angle.getEncount();

            //リミットスイッチ読み取り
            // limit1read = limit1.read(); //ハンド取得判定
            // limit2read = limit2.read(); //槍先ラック1
            // limit3read = limit3.read(); //槍先ラック2
            // limit4read = limit4.read(); //右側スイッチ1
            // limit5read = limit5.read(); //右側スイッチ2
            // limit6read = limit6.read(); //左側スイッチ1
            // limit7read = limit7.read(); //左側スイッチ2

            if(flag_limitcomm){
                bool limitcomm = lim.update();
                
                for(int i = 11; i >= 0; i--){
                    limitdata[i] = lim.sensorData[i];
                    // pc.printf("%d ",lim.sensorData[i]);
                }
                flag_limitcomm = false;
            }

            // limit1read = limitdata[]; //ハンド取得判定
            limit2read = limitdata[2]; //槍先ラック1左(後)//10なんか読めない
            limit3read = limitdata[5]; //槍先ラック2右（後）
            limit4read = limitdata[3]; //前左
            limit5read = limitdata[9]; //前右
            limit6read = limitdata[1]; //左側スイッチ1
            // limit7read = limitdata[3]; //右側スイッチ2
            limit8read = limitdata[11];//前上限
            limit9read = limitdata[4];//後上限

            //光電センサ--------------
            kouden1read = kouden1.read();
            kouden2read = limitdata[8];//外側
            kouden3read = limitdata[6];//内側

            //HC-RS04------------------
            // front.trigger();

            // if(front.update()){
            //     float d = front.get_cm();
            //     pc.printf("dist=%.1f cm\n", d);
            // }

            if(first_count == false){
                pre_get_cam1_pitch_count = get_cam1_pitch_count;
                pre_get_cam1_yaw_count = get_cam1_yaw_count;
                pre_get_cam2_pitch_count = get_cam2_pitch_count;
                pre_get_cam2_yaw_count = get_cam2_yaw_count;
                pre_get_lift_front_count = get_lift_front_count;
                pre_get_lift_back_count = get_lift_back_count;
                first_count = true;
            }
            
            int pitch_dir = 1;
            if(field_s == RED){
                pitch_dir = 1;
            }else if(field_s == BLUE){
                pitch_dir = -1;
            }
            //角度変換
            get_cam1_pitch_angle = pitch_dir * (double)count2rad(get_cam1_pitch_count,RIGHT_CAM_INIT_PITCH_POS,CORE_ANGLE_RES);
            get_cam1_yaw_angle = -(double)count2rad(get_cam1_yaw_count,RIGHT_CAM_INIT_YAW_POS,CORE_ANGLE_RES);
            get_cam2_pitch_angle = pitch_dir * -(double)count2rad(get_cam2_pitch_count,LEFT_CAM_INIT_PITCH_POS,CORE_ANGLE_RES);
            get_cam2_yaw_angle = -(double)count2rad(get_cam2_yaw_count,LEFT_CAM_INIT_YAW_POS,CORE_ANGLE_RES);
            cam_pitch = get_cam1_pitch_angle;
            cam_yaw = get_cam1_yaw_angle;
            cam_pitch2 = get_cam2_pitch_angle;
            cam_yaw2 = get_cam2_yaw_angle;
            //昇降位置変換
            // get_air_angle = -(double)count2rad(enc_air_angle.getAbsCount(),AIR_INIT_ANGLE_POSI,AIR_ANGLE_RES);
            // double front_lift_diff = dist_count(get_lift_front_count,pre_get_lift_front_count,FRONT_R);
            // // double back_lift_diff = dist_count(get_lift_front_count,pre_get_lift_front_count,FRONT_R);
            // pc.printf("%lf, ",front_lift_diff);
            // get_lift_front_posi += front_lift_diff;
            // double front_lift_diff = enc_to_lift_height(get_lift_front_count,pre_get_lift_front_count,FRONT_R);
            // double back_lift_diff = enc_to_lift_height(get_lift_back_count,pre_get_lift_back_count,FRONT_R);
            // get_lift_front_posi += front_lift_diff;
            // get_lift_back_posi += back_lift_diff;
            
            // pc.printf("%lf",get_lift_front_posi);
            // get_lift_front_posi += height_count(get_lift_front_count,pre_get_lift_front_count,FRONT_R);
            

            //昇降の高さ(mm)
            if(set_height_front){
                double front_lift_diff = enc_to_lift_height(get_lift_front_count,pre_get_lift_front_count,FRONT_R);
                get_lift_front_posi += front_lift_diff;
                // front_lift_posi = (-0.0696 * get_lift_front_posi - 0.5299)*10*2;
                front_lift_posi = -(-0.0683 * get_lift_front_posi)*10*2 + front_lift_init;
            }

            if(set_height_back){
                double back_lift_diff = enc_to_lift_height(get_lift_back_count,pre_get_lift_back_count,FRONT_R);
                get_lift_back_posi += back_lift_diff;
                // back_lift_posi = (-0.0677 * get_lift_back_posi - 0.0626)*10*2;
                back_lift_posi = -(-0.067 * get_lift_back_posi)*10*2 + back_lift_init;
            }

            if(!limit8read){
                set_height_front = true;
                front_lift_posi = front_lift_init;
                get_lift_front_posi = 0;
            }
            if(!limit9read){
                set_height_back = true;
                back_lift_posi = back_lift_init;
                get_lift_back_posi = 0;
            }

            //足回り自己位置-------------------------------------------------------------------------
            double get_anglez = 0.000;
            double get_angley = 0.000;
            double get_anglex = 0.000;

            if(flag_use_sensor){
                // num = bno085.read();
                // if(num == 0 && pre_num == 1){
                //     bno085.up();
                // }
                // pre_num = num;
                if(field_s == RED){
                    get_anglez = -(double)lpms.get_z_angle();
                }else if(field_s == BLUE){
                    get_anglez = (double)lpms.get_z_angle();
                }
                // double get_anglez = (double)bno085.get_z_angle();
                // if(bno085.read() == 0){
                    // get_anglez = -(double)bno085.anglez;
                    anglez = get_anglez - pre_anglez;
                    get_angley = (double)lpms.get_y_angle();
                    // get_angley = (double)bno085.angley;
                    angley = get_angley - pre_angley;
                    get_anglex = (double)lpms.get_x_angle();
                    // get_anglex = (double)bno085.anglex;
                    anglex = get_anglex - pre_anglex;
                // }
            }

            int encY_count;
            int encX_count;
            if(!air_up_flag){
                if(field_s == RED){
                    encY_count = -encY.getCount();
                }else if(field_s == BLUE){
                    encY_count = encY.getCount();
                }
                encX_count = -encX.getCount();
            }

            //エンコーダの回転補正
            // encY_count = encY_count + (2000 * ENC_Y_R * anglez * sin(deg2rad(ENC_Y_THETA)) / 2 / M_PI / RADIUS_Y);
            // encX_count = -encX_count + (2000 * ENC_X_R * anglez * sin(deg2rad(ENC_X_THETA)) / 2 / M_PI / RADIUS_X);
            // encXstate += lpmsread / 2 / M_PI * sin(ENC_Y_THETA) * 2 * ENC_X_R /
            //            ENC_X_WHEEL_D * ENC_X_RES4;
            // encYstate += lpmsread / 2 / M_PI * sin(ENC_X_THETA) * 2 * ENC_Y_R /
            //            ENC_Y_WHEEL_D * ENC_Y_RES4;

            getPosi = platform.getPosi(encX_count, encY_count, get_anglez);



            
            if(route[autonomous.route_num].num <= 2){
                cubePosi.x = -1;
            }else if (route[autonomous.route_num].num <= 5 && route[autonomous.route_num].num > 2){
                cubePosi.x = 0;
            }else if (route[autonomous.route_num].num <= 8 && route[autonomous.route_num].num > 5){
                cubePosi.x = 1;
            }else if (route[autonomous.route_num].num <= 11 && route[autonomous.route_num].num > 8){
                cubePosi.x = 2;
            }else if (route[autonomous.route_num].num <= 14 && route[autonomous.route_num].num > 11){
                cubePosi.x = 3;
            }else if (route[autonomous.route_num].num <= 17 && route[autonomous.route_num].num > 14){
                cubePosi.x = 4;
            }

            cubePosi.y =  route[autonomous.route_num].num % 3;

            //forestどのマスにいるか-----------------------------------------
            for(int i = 0; i < 18; i++){
                if(cubePosi.x == objePosi[i].x && cubePosi.y == objePosi[i].y){
                    cubeIndex = i;//その時にいるindex番号
                    autonomous.cubeIndex = cubeIndex;
                }
                if(autonomous.nextX == objePosi[i].x && autonomous.nextY == objePosi[i].y){
                    nextIndex = i;//次に移動するマスのindex番号
                    autonomous.nextIndex = nextIndex;
                }
                
            }
            //次のforestマスに何があるか
            if(nextIndex >= 3 && nextIndex <= 14){
                re_nextIndex = nextIndex;
                // next_box_state = obj_state[nextIndex-3];
                next_box_state = obj_state_test[nextIndex-3];//マスは12マスなので-3で補正
                autonomous.next_box_state = next_box_state;
            }else{
                next_box_state = 0;
                autonomous.next_box_state = next_box_state;
            }
            

            if(!flag_simu){
                for(int i = 0; i < 12; i++){//状態をautonomousに保存
                    if(obj_state_test[11-i] == 1){
                        obj_true_R1[i] = true;
                    }else if(obj_state_test[11-i] == 2){
                        obj_true_R2[i] = true;
                    }else if(obj_state_test[11-i] == 3){
                        obj_true_Fake[i] = true;
                    }else{
                        obj_true_R1[i] = false;
                        obj_true_R2[i] = false;
                        obj_true_Fake[i] = false;
                    }
                    // autonomous.button_true[i] = obj_inf[i];
                    autonomous.button_true_R1[i] = obj_true_R1[i];
                    autonomous.button_true_R2[i] = obj_true_R2[i];
                    autonomous.button_true_Fake[i] = obj_true_Fake[i];
                }
            }

            cubeIndex = change_num[route[autonomous.route_num].num];

            if(mode == MODE_FOREST){
                switch (route[autonomous.route_num].num) {
                    case 0:
                        flag_use_front = true;
                        flag_use_in = false;
                        flag_use_out = false;
                        flag_use_back = false;
                    break;
                    case 1:
                        flag_use_front = true;
                        flag_use_in = false;
                        flag_use_out = false;
                        flag_use_back = false;
                    break;
                    case 2:
                        flag_use_front = true;
                        flag_use_in = false;
                        flag_use_out = false;
                        flag_use_back = false;
                    break;
                    case 3:
                        flag_use_front = true;
                        flag_use_in = false;
                        flag_use_out = false;
                        flag_use_back = false;
                    break;
                    case 4:
                        flag_use_front = true;
                        flag_use_in = true;
                        flag_use_out = false;
                        flag_use_back = false;
                    break;
                    case 5:
                        flag_use_front = false;
                        flag_use_in = false;
                        flag_use_out = false;
                        flag_use_back = false;
                    break;
                    case 6:
                        flag_use_front = false;
                        flag_use_in = false;
                        flag_use_out = false;
                        flag_use_back = false;
                    break;
                    case 7:
                        flag_use_front = true;
                        flag_use_in = true;
                        flag_use_out = false;
                        flag_use_back = false;
                    break;
                    case 8:
                        flag_use_front = true;
                        flag_use_in = true;
                        flag_use_out = false;
                        flag_use_back = true;
                    break;
                    case 9:
                        flag_use_front = false;
                        flag_use_in = false;
                        flag_use_out = true;
                        flag_use_back = true;
                    break;
                    case 10:
                        flag_use_front = false;
                        flag_use_in = false;
                        flag_use_out = false;
                        flag_use_back = false;
                    break;
                    case 11:
                        flag_use_front = false;
                        flag_use_in = true;
                        flag_use_out = false;
                        flag_use_back = false;
                    break;
                    case 12:
                        flag_use_front = false;
                        flag_use_in = false;
                        flag_use_out = true;
                        flag_use_back = true;
                    break;
                    case 13:
                        flag_use_front = false;
                        flag_use_in = false;
                        flag_use_out = false;
                        flag_use_back = true;
                    break;
                    case 14:
                        flag_use_front = false;
                        flag_use_in = true;
                        flag_use_out = false;
                        flag_use_back = false;
                    break;
                    case 15:
                        flag_use_front = false;
                        flag_use_in = false;
                        flag_use_out = false;
                        flag_use_back = false;
                    break;
                    case 16:
                        flag_use_front = false;
                        flag_use_in = false;
                        flag_use_out = false;
                        flag_use_back = true;
                    break;
                    case 17:
                        flag_use_front = false;
                        flag_use_in = false;
                        flag_use_out = false;
                        flag_use_back = false;
                    break;
                }
                //使える壁（旋回の時，前か後）----------------------------------
                // if(cubeIndex == 10 || cubeIndex == 11 || cubeIndex == 12 || cubeIndex == 13 || cubeIndex == 15 || cubeIndex == 17){
                //     flag_use_front = true;
                // }else{
                //     flag_use_front = false;
                // }
                // if(cubeIndex == 5 || cubeIndex == 8 || cubeIndex == 10 || cubeIndex == 11 || cubeIndex == 13){//cubeIndex == 15
                //     flag_use_in = true;
                // }else{
                //     flag_use_in = false;
                // }
                // if(cubeIndex == 3 || cubeIndex == 6 ){//|| cubeIndex == 13 -> 内側だけにする．cubeIndex == 17
                //     flag_use_out = true;
                // }else{
                //     flag_use_out = false;
                // }
                // if(cubeIndex == 3 || cubeIndex == 4 || cubeIndex == 5 || cubeIndex == 6 || cubeIndex == 0 || cubeIndex == 1 || cubeIndex == 2){//|| cubeIndex == 11⇒前だけにする
                //     flag_use_back = true;
                // }else{
                //     flag_use_back = false;
                // }
                // if(cubeIndex == 7 || cubeIndex == 9 || cubeIndex == 14){
                //     flag_use_front = false;
                //     flag_use_in = false;
                //     flag_use_out = false;
                //     flag_use_back = false;
                // }
                // //次のマスのlrtb可能不可-------------------------------
                // if(nextIndex == 10 || nextIndex == 11 || nextIndex == 12 || nextIndex == 13 || nextIndex == 15 || nextIndex == 17){
                //     flag_use_front_next = true;
                // }else{
                //     flag_use_front_next = false;
                // }
                // if(nextIndex == 5 || nextIndex == 8 || nextIndex == 10 || nextIndex == 11 || nextIndex == 13){
                //     flag_use_in_next = true;
                // }else{
                //     flag_use_in_next = false;
                // }
                // if(nextIndex == 3 || nextIndex == 6 ){//|| nextIndex == 13 -> 内側だけにする．
                //     flag_use_out_next = true;
                // }else{
                //     flag_use_out_next = false;
                // }
                // if(nextIndex == 3 || nextIndex == 4 || nextIndex == 5 || nextIndex == 6 || nextIndex == 0 || nextIndex == 1 || nextIndex == 2){//|| nextIndex == 11⇒前だけにする
                //     flag_use_back_next = true;
                // }else{
                //     flag_use_back_next = false;
                // }
                // if(nextIndex == 7 || nextIndex == 9 || nextIndex == 14){
                //     flag_use_front_next = false;
                //     flag_use_in_next = false;
                //     flag_use_out_next = false;
                //     flag_use_back_next = false;
                // }
            }else{
                flag_use_front = false;
                flag_use_in = false;
                flag_use_out = false;
                flag_use_back = false;
                flag_use_front_next = false;
                flag_use_in_next = false;
                flag_use_out_next = false;
                flag_use_back_next = false;
            }
            //forestはマスごとに見る壁の座標（距離）が違うため現在のマス目の位置を基準に変更する．
            // if(cubeIndex == 0){    
            // }
            //LRTB-----------------------------------------------------------
            if(lrtb_comm && lrtb.update()){
            // if(false){
                lrtb_0 = lrtb.sensorData[1];//　前（機体から見て）
                lrtb_1 = lrtb.sensorData[0];//　左
                lrtb_2 = lrtb.sensorData[3];//　右
                lrtb_3 = lrtb.sensorData[2];//　後
                filt_lrtb_0 = lrtb_front_filter.SecondOrderLag((double)lrtb_0);
                filt_lrtb_1 = lrtb_left_filter.SecondOrderLag((double)lrtb_1);
                filt_lrtb_2 = lrtb_right_filter.SecondOrderLag((double)lrtb_2);
                filt_lrtb_3 = lrtb_back_filter.SecondOrderLag((double)lrtb_3);
                double lrtb_diff_3 = 0.000000003 * pow(lrtb_3,2) + 0.00003 * lrtb_3 + 0.001; 
                lrtb_dist_0 = 0.0009 * lrtb_0 + 0.0044;// 前　壁からの距離　//0~2m
                lrtb_dist_1 = 0.0009 * lrtb_1 - 0.0014;//　左
                lrtb_dist_2 = 0.0009 * lrtb_2 - 0.003;//　右
                lrtb_dist_3 = (0.0008 * lrtb_3 - 0.0004) - lrtb_diff_3;//  後

                //LRTB旋回したときの壁からの垂直距離計算---------------
                //lrtbと機体中心との位置している角度
                lrtb_theta_0 = LRTB0_THETA;
                lrtb_theta_1 = LRTB1_THETA;
                lrtb_theta_2 = LRTB2_THETA;
                lrtb_theta_3 = LRTB3_THETA;
                //中心からlrtbまでの距離
                lrtb_posi_dist_0 = LRTB0_DIST;
                lrtb_posi_dist_1 = LRTB1_DIST;
                lrtb_posi_dist_2 = LRTB2_DIST;
                lrtb_posi_dist_3 = LRTB3_DIST;
                //中心からlrtbの出力点までの垂直距離
                lrtb0_dist_x = fabs(lrtb_posi_dist_0 * cos(lrtb_theta_0 + gPosi.z));
                lrtb0_dist_y = fabs(lrtb_posi_dist_0 * sin(lrtb_theta_0 + gPosi.z));
                lrtb1_dist_x = fabs(lrtb_posi_dist_1 * sin(lrtb_theta_1 - gPosi.z));
                lrtb1_dist_y = fabs(lrtb_posi_dist_1 * cos(lrtb_theta_1 - gPosi.z));
                lrtb2_dist_x = fabs(lrtb_posi_dist_2 * sin(lrtb_theta_2 - gPosi.z));
                lrtb2_dist_y = fabs(lrtb_posi_dist_2 * cos(lrtb_theta_2 - gPosi.z));
                lrtb3_dist_x = fabs(lrtb_posi_dist_3 * sin(lrtb_theta_3 - gPosi.z));
                lrtb3_dist_y = fabs(lrtb_posi_dist_3 * cos(lrtb_theta_3 - gPosi.z));
                //lrtb出力点から壁までの垂直距離
                lrtb0_wall_x = fabs(lrtb_dist_0 * cos(-gPosi.z));//lrtbから壁までの距離
                lrtb0_wall_y = fabs(lrtb_dist_0 * sin(-gPosi.z));
                lrtb1_wall_x = fabs(lrtb_dist_1 * sin(-gPosi.z));//左
                lrtb1_wall_y = fabs(lrtb_dist_1 * cos(-gPosi.z));
                lrtb2_wall_x = fabs(lrtb_dist_2 * sin(-gPosi.z));//右
                lrtb2_wall_y = fabs(lrtb_dist_2 * cos(-gPosi.z));
                lrtb3_wall_x = fabs(lrtb_dist_3 * cos(-gPosi.z));//後
                lrtb3_wall_y = fabs(lrtb_dist_3 * sin(-gPosi.z));
                //機体中心からlrtbで読み取った壁までの垂直距離
                //フィールド反転によって左右が逆になる．（基準は青ゾーンとする）
                lrtb_front_dist_x = lrtb0_dist_x + lrtb0_wall_x;
                lrtb_front_dist_y = lrtb0_dist_y + lrtb0_wall_y;
                if(field_s == RED){
                    lrtb_left_dist_x = lrtb1_dist_x + lrtb1_wall_x;
                    lrtb_left_dist_y = lrtb1_dist_y + lrtb1_wall_y;
                    lrtb_right_dist_x = lrtb2_dist_x + lrtb2_wall_x;
                    lrtb_right_dist_y = lrtb2_dist_y + lrtb2_wall_y;

                }
                else if(field_s == BLUE){
                    lrtb_right_dist_x = lrtb1_dist_x + lrtb1_wall_x;
                    lrtb_right_dist_y = lrtb1_dist_y + lrtb1_wall_y;
                    lrtb_left_dist_x = lrtb2_dist_x + lrtb2_wall_x;
                    lrtb_left_dist_y = lrtb2_dist_y + lrtb2_wall_y;
                }
                lrtb_back_dist_x = lrtb3_dist_x + lrtb3_wall_x;
                lrtb_back_dist_y = lrtb3_dist_y + lrtb3_wall_y;

                double lrtb_diff_limit = 0.15;//m

                check_distance_front = lrtb_front_dist_x;
                check_distance_back = lrtb_back_dist_x;
                check_distance_in = lrtb_left_dist_y;
                check_distance_out = lrtb_right_dist_y;

                // double z_norm = normalize_pi(gPosi.z);
                double z_norm = atan2(sin(gPosi.z), cos(gPosi.z));
                // sprintf(str,"z_norm:%lf  ",z_norm);
                // invoke_print(str);

                // 前向き（0度付近）
                if (fabs(z_norm) < M_PI / 4) {
                    distance_front = lrtb_front_dist_x;
                    distance_back  = lrtb_back_dist_x;
                    distance_in    = lrtb_left_dist_y;
                    distance_out   = lrtb_right_dist_y;
                // 右向き（赤の時）（+90度付近）青は左向き
                } else if (fabs(z_norm - M_PI/2) < M_PI / 4) {
                    distance_front = lrtb_left_dist_x;
                    distance_back  = lrtb_right_dist_x;
                    distance_in    = lrtb_back_dist_y;
                    distance_out   = lrtb_front_dist_y;
                // 左向き（赤の時）（-90度付近）青は右向き
                } else if (fabs(z_norm + M_PI/2) < M_PI / 4) {
                    distance_front = lrtb_right_dist_x;
                    distance_back  = lrtb_left_dist_x;
                    distance_in    = lrtb_front_dist_y;
                    distance_out   = lrtb_back_dist_y;
                // 後ろ向き（±180度付近）
                } else { // |z_norm| >= 3π/4
                    distance_front = lrtb_back_dist_x;
                    distance_back  = lrtb_front_dist_x;
                    distance_in    = lrtb_right_dist_y;
                    distance_out   = lrtb_left_dist_y;
                }

                flag_lrtb_front = (fabs(pre_distance_front - distance_front) < lrtb_diff_limit)? true : false;
                flag_lrtb_out = (fabs(pre_distance_out - distance_out) < lrtb_diff_limit)? true : false;
                flag_lrtb_in = (fabs(pre_distance_in- distance_in) < lrtb_diff_limit)? true : false;
                flag_lrtb_back = (fabs(pre_distance_back- distance_back) < lrtb_diff_limit)? true : false;

                // flag_lrtb_diff_x = (fabs(gPosi.x - lrtbPosi.x) < lrtb_diff_limit)? true : false;
                // flag_lrtb_diff_y = (fabs(gPosi.y - lrtbPosi.y) < lrtb_diff_limit)? true : false;

/////////////////////////////////////////////


                //lrtbによる自己位置
                // if(field_s == RED){//青フィールド
                if(mode == MODE_ZONE1){//ゾーン1，phase 事に変更
                    // lrtbPosi.x = x_max - lrtb_front_dist_x;
                    // if(gPosi.y < 2.0){//内側
                    //     lrtbPosi.y = lrtb_left_dist_y;
                    // }else if(gPosi.y >= 4.0){//外側
                        
                    //     lrtbPosi.y = y_max - lrtb_right_dist_y;
                    // }
                    if(flag_lrtb_front){      
                    }
                    if(flag_lrtb_in){
                        // if(autonomous.phase == 200 || autonomous.phase == 3 || autonomous.phase == 302 || autonomous.phase == 304 || autonomous.phase == 315 || autonomous.phase == 306 || autonomous.phase == 325 || autonomous.phase == 4)
                        if(autonomous.phase == 100 || autonomous.phase == 101 || autonomous.phase == 102 || autonomous.phase == 103 || autonomous.phase == 104)
                        {//ラック衝突
                            // lrtbPosi.x = distance_back;
                            lrtbPosi.y = distance_in + 0.15 - 0.025;//+ラックの距離
                            // platform.setAxisPosi(lrtbPosi.x, POSIX);
                            // if(fabs(gPosi.y - lrtbPosi.y) < lrtb_diff_limit){
                                platform.setAxisPosi(lrtbPosi.y, POSIY);
                            // }
                        }
                    }
                    if(flag_lrtb_out){
                    }
                    if(flag_lrtb_back){
                        // if(autonomous.phase == 200 || autonomous.phase == 210 || autonomous.phase == 3 || autonomous.phase == 301 || autonomous.phase == 302 || autonomous.phase == 303 || autonomous.phase == 304 || autonomous.phase == 315 || autonomous.phase == 306 || autonomous.phase == 325  || autonomous.phase == 335 || autonomous.phase == 4)
                        if(autonomous.phase == 100 || autonomous.phase == 101 || autonomous.phase == 102 || autonomous.phase == 103 || autonomous.phase == 104)
                        {//ラック衝突
                            lrtbPosi.x = distance_back;
                            // lrtbPosi.y = distance_in + 0.15;//+ラックの距離
                            // if(fabs(gPosi.x - lrtbPosi.x) < lrtb_diff_limit){
                                platform.setAxisPosi(lrtbPosi.x, POSIX);
                            // }
                            // platform.setAxisPosi(lrtbPosi.y, POSIY);
                        }
                    }
                    // lrtbPosi.x = 
                    // getPosi.x = platform.setAxisPosi(double posi, POSIX)
                }else if(mode == MODE_ZONE2){//ゾーン2
                }else if(mode == MODE_ZONE3){//ゾーン3
                    if(flag_lrtb_front){
                        if(autonomous.phase == 62){
                            lrtbPosi.x = 12.0 - distance_front;
                            if(fabs(gPosi.x - lrtbPosi.x) < lrtb_diff_limit){
                                getPosi.x = platform.setAxisPosi(lrtbPosi.x, POSIX);
                            }
                        }
                    }else if(flag_lrtb_back){

                    }else if(flag_lrtb_in){
                        if(autonomous.phase == 62){
                            lrtbPosi.y = distance_in + 0.25 - 0.025;//ラックを見る
                            if(fabs(gPosi.y - lrtbPosi.y) < lrtb_diff_limit){
                                getPosi.y = platform.setAxisPosi(lrtbPosi.y, POSIY);
                            }
                        }
                    }else if(flag_lrtb_out){

                    }
                }else if(mode == MODE_FOREST){//forest
                    if(autonomous.phase == 214){
                        // if(route[autonomous.route_num].num ==  0 || route[autonomous.route_num].num == 2 || route[autonomous.route_num].num == 3 || route[autonomous.route_num].num == 4 || route[autonomous.route_num].num == 7 || route[autonomous.route_num].num == 8 || route[autonomous.route_num].num == 12){

                        // }
                        // route[autonomous.route_num].num == 
                    }
                    if(autonomous.phase == 226 || autonomous.phase == 237){

                    }

                    if(autonomous.phase == 212 || autonomous.phase == 214 || autonomous.phase == 224 || autonomous.phase == 226  || autonomous.phase == 232 || autonomous.phase == 2351 || autonomous.phase == 237){//旋回前に自己位置の補正
                        // if(next_box_state == 0){
                            if(flag_use_front && flag_lrtb_front){
                                lrtbPosi.x = forest_Posi[cubeIndex].front - distance_front;
                                // if(cubeIndex == 16 && obj_state_test[10] == 0){
                                //     // lrtbPosi.x = forest_Posi[13].front - distance_front;
                                // }
                                getPosi.x = platform.setAxisPosi(lrtbPosi.x, POSIX);
                            }
                            if(flag_use_back && flag_lrtb_back){
                                lrtbPosi.x = forest_Posi[cubeIndex].back + distance_back;
                                getPosi.x = platform.setAxisPosi(lrtbPosi.x, POSIX);
                            }
                            if(flag_use_in && flag_lrtb_in){
                                // lrtbPosi.y = forest_Posi[cubeIndex].left + distance_in;
                                // if(cubeIndex == 15){
                                //     lrtbPosi.y = distance_in;
                                //     if(fabs(gPosi.y - lrtbPosi.y) < lrtb_diff_limit){
                                //         getPosi.y = platform.setAxisPosi(lrtbPosi.y, POSIY);
                                //     }
                                // }else{
                                    lrtbPosi.y = forest_Posi[cubeIndex].left + distance_in;
                                    getPosi.y = platform.setAxisPosi(lrtbPosi.y, POSIY);
                                // }
                            }
                            if(flag_use_out && flag_lrtb_out){
                                // lrtbPosi.y = forest_Posi[cubeIndex].right - distance_out;
                                // if(cubeIndex == 17){
                                //     lrtbPosi.y = 6.0 - distance_out;
                                //     if(fabs(gPosi.y - lrtbPosi.y) < lrtb_diff_limit){
                                //         getPosi.y = platform.setAxisPosi(lrtbPosi.y, POSIY);
                                //     }
                                // }else{
                                    lrtbPosi.y = forest_Posi[cubeIndex].right - distance_out;
                                    getPosi.y = platform.setAxisPosi(lrtbPosi.y, POSIY);
                                // }
                            }
                        // }
                        // platform.setPosi(coords {gPosi.x, 0.15 + 0.365 - 0.025 , INIT_Z});
                    }
                }
                // lrtbPosi.x = forest_Posi[cubeIndex].front - distance_front;
                // getPosi.x = platform.setAxisPosi(lrtbPosi.x, POSIX);
                // }else if(field_s == BLUE){//赤フィールド  

                // }

                // if(flag_lrtb_front) pre_distance_front = distance_front;
                // if(flag_lrtb_out) pre_distance_out = distance_out;
                // if(flag_lrtb_in) pre_distance_in = distance_in;
                // if(flag_lrtb_back) pre_distance_back = distance_back;
                pre_distance_front = distance_front;
                pre_distance_out = distance_out;
                pre_distance_in = distance_in;
                pre_distance_back = distance_back;



            // if(autonomous.phase == 10 && next_box_state == 2){//boxによる自己位置推定
            //     if(flag_use_front && flag_lrtb_front){
            //         if(nextIndex >= 3 && nextIndex <= 11){
            //             lrtbPosi.x = box_Posi[nextIndex].back - distance_front;
            //             platform.setAxisPosi(lrtbPosi.x, POSIX);
            //         }
                    
            //     }
            // }
                if(autonomous.phase == 11){
                    
                }

                if(autonomous.phase == 0){//リトライの始め，動く前自己位置更新
                    if(autonomous.flag_retry){
                        lrtbPosi.x = 12.0 - distance_front;
                        lrtbPosi.y = 6.0 - distance_out;
                        platform.setPosi(coords {lrtbPosi.x, lrtbPosi.y, RETRY_Z});
                    }else if(autonomous.flag_retry_forest){
                        lrtbPosi.x = distance_back;
                        lrtbPosi.y = distance_in + 0.15 - 0.025;
                        platform.setPosi(coords {lrtbPosi.x, lrtbPosi.y, RETRY_F_Z});
                        // platform.setPosi(coords {3.0, 1.8, RETRY_F_Z});
                        // printf("HH");
                    }else {
                        lrtbPosi.x = distance_back;
                        // lrtbPosi.y = distance_in;
                        platform.setAxisPosi(lrtbPosi.x, POSIX);
                    }   
                }

                // if(autonomous.phase == 200){
                //     platform.setPosi(coords {2.0, 1.8, RETRY_F_Z});
                //     printf("HH\n\n\n");
                // }

                //////////////////
            }

            ///////////////////
            
            //limitスイッチ自己位置補正
            if(autonomous.phase == 101 && !limit2read && !limit3read){
                // if(!(autonomous.tar_posi_rack_x = autonomous.spear_posi_x[0]) ||  !(autonomous.tar_posi_rack_x = autonomous.spear_posi_x[5])){
                    platform.setPosi(coords {gPosi.x, 0.15 + 0.365 - 0.025 , INIT_Z});//y：0.49
                    nextPhase = PUSH_BUTTON;//リミットスイッチがぶつかったら次に進めて止まる．
                // }
            }
            if(autonomous.phase == 101 && (!limit2read || !limit3read)){
                // if(!(autonomous.tar_posi_rack_x = autonomous.spear_posi_x[0]) ||  !(autonomous.tar_posi_rack_x = autonomous.spear_posi_x[5])){
                    platform.setPosi(coords {gPosi.x, 0.15 + 0.365 - 0.025 , gPosi.z});//y：0.49
                    nextPhase = PUSH_BUTTON;//リミットスイッチがぶつかったら次に進めて止まる．
                // }
            }

            //limitスイッチによる自己位置補正（誤差の修正なのでgPosi.zを基準に．．．）232,224,212
            //212,232,224
            if((autonomous.phase == 212 || autonomous.phase == 224 || autonomous.phase == 232) && !limit4read && !limit5read){//後
                // platform.setPosi(coords {gPosi.x, gPosi.y, 0.0});
                double z_norm = atan2(sin(gPosi.z), cos(gPosi.z));
                if (fabs(z_norm) < M_PI / 4) {// 前向き（0度付近）上から26.5mm (60 - 26.6 = 33.5mm)-> +33.5mm
                   platform.setAxisPosi(0.000, POSIZ);
                } else if (fabs(z_norm - M_PI/2) < M_PI / 4) {// 右向き（青の時）（+90度付近）
                    platform.setAxisPosi(M_PI/2, POSIZ);
                } else if (fabs(z_norm + M_PI/2) < M_PI / 4) {// 左向き（青の時）（-90度付近）
                    platform.setAxisPosi(-M_PI/2, POSIZ);
                } else { // |z_norm| >= 3π/4// 後ろ向き（±180度付近）
                    platform.setAxisPosi(M_PI, POSIZ);
                }

                switch (autonomous.direction_flag) {
                    case DFRONT:
                        setPosi.x = autonomous.forest[route[autonomous.route_num].num].x + 0.313;
                        platform.setAxisPosi(setPosi.x, POSIX);
                    break;
                    case DRIGHT:
                        setPosi.y = autonomous.forest[route[autonomous.route_num].num].y + 0.313;
                        platform.setAxisPosi(setPosi.y, POSIY);
                    break;
                    case DLEFT:
                        setPosi.y = autonomous.forest[route[autonomous.route_num].num].y - 0.313;
                        platform.setAxisPosi(setPosi.y, POSIY);
                    break;
                    case DBACK:
                        setPosi.x = autonomous.forest[route[autonomous.route_num].num].x - 0.313;
                        platform.setAxisPosi(setPosi.x, POSIX);
                    break;
                }
            }

            if((autonomous.phase == 212 || autonomous.phase == 224 || autonomous.phase == 232) && pre_kouden1read == 1 && kouden1read == 0 && front_syusoku == true){//後
                // platform.setPosi(coords {gPosi.x, gPosi.y, 0.0});
                switch (autonomous.direction_flag) {
                    case DFRONT:
                        setPosi.x = autonomous.forest[route[autonomous.route_num].num].x + 0.308;
                        platform.setAxisPosi(setPosi.x, POSIX);
                    break;
                    case DRIGHT:
                        setPosi.y = autonomous.forest[route[autonomous.route_num].num].y + 0.308;
                        platform.setAxisPosi(setPosi.y, POSIY);
                    break;
                    case DLEFT:
                        setPosi.y = autonomous.forest[route[autonomous.route_num].num].y - 0.308;
                        platform.setAxisPosi(setPosi.y, POSIY);
                    break;
                    case DBACK:
                        setPosi.x = autonomous.forest[route[autonomous.route_num].num].x - 0.308;
                        platform.setAxisPosi(setPosi.x, POSIX);
                    break;
                }
            }
            // else if(!limit4read && !limit5read){//右
            //     platform.setPosi(coords {2.0, 2.0, 1.0});
            // }else if(!limit6read && !limit7read){//左
            //     platform.setPosi(coords {gPosi.x, 3.0, 1.0});
            // }

            //段越え段降り中
            // if((autonomous.phase == 2225 || autonomous.phase == 2236)&& pre_kouden3read == 1 && kouden3read == 0 ){
            //     switch (autonomous.direction_flag) {
            //         case DFRONT:
            //             setPosi.x = autonomous.forest[route[autonomous.route_num].num].x + 0.797;
            //             platform.setAxisPosi(setPosi.x, POSIX);
            //         break;
            //         case DRIGHT:
            //             setPosi.y = autonomous.forest[route[autonomous.route_num].num].y + 0.797;
            //             platform.setAxisPosi(setPosi.y, POSIY);
            //         break;
            //         case DLEFT:
            //             setPosi.y = autonomous.forest[route[autonomous.route_num].num].y - 0.797;
            //             platform.setAxisPosi(setPosi.y, POSIY);
            //         break;
            //         case DBACK:
            //             setPosi.x = autonomous.forest[route[autonomous.route_num].num].x - 0.797;
            //             platform.setAxisPosi(setPosi.x, POSIX);
            //         break;
            //     }
            // }

////////////////////////////////////////
            

            //最終的な自己位置---------------------------------------------------------
            // pc.printf("get: %lf, %lf, %lf\t",getPosi.x,getPosi.y,getPosi.z);
            gPosi = getPosi;
            // pc.printf("g: %lf, %lf, %lf\t",gPosi.x,gPosi.y,gPosi.z);
            //前の値に今の値を格納
            pre_get_cam1_pitch_count = get_cam1_pitch_count;
            pre_get_cam1_yaw_count = get_cam1_yaw_count;
            pre_get_cam2_pitch_count = get_cam2_pitch_count;
            pre_get_cam2_yaw_count = get_cam2_yaw_count;
            pre_get_lift_front_count = get_lift_front_count;
            pre_get_lift_back_count = get_lift_back_count;
            
        }

        // autonomous.route_num = 2;
        // autonomous.phase = 202;

        // int flag_lift;
        //初期化の処理
        switch (flag_lift) {
            case 1://初期化の昇降上げ
                roboclawCmd0 = 2000;
                roboclawCmd1 = 2000;
                if(!limit8read){
                    roboclawCmd0 = 0;
                }
                if(!limit9read){
                    roboclawCmd1 = 0;
                }
                if(!limit8read && !limit9read){//どっちも押されたら次に進む．フラグを立てる．
                    flag_lift = 2;
                    autonomous.set_front_posi = STORAGE_POSI;
                    autonomous.set_back_posi = STEP_UP_BACK_HIGH;
                    // front_lift_posi = STORAGE_POSI;
                    // back_lift_posi = STEP_UP_BACK_HIGH;
                }
            break;
            case 2:
                ref_lift_front_posi = autonomous.set_front_posi;
                ref_lift_back_posi = autonomous.set_back_posi;
                if(autonomous.phase == 2225 || autonomous.phase == 2236){
                    switch (autonomous.hight_flag) {
                        case 2:
                            if(!stepup_flag){
                                stepup_flag = true;
                                stepup_count = 1;
                                back_syusoku = false;
                                front_syusoku = false;
                                ref_lift_front_posi = STEP_UP_FRONT_LOW;
                                ref_lift_back_posi = STEP_UP_BACK_LOW;
                            }
                        break;
                        case -2:
                            if(!stepdown_flag){
                                stepdown_flag = true;
                                stepdown_count = 1;
                                back_syusoku = false;
                                front_syusoku = false;
                                ref_lift_front_posi = STEP_DOWN_FRONT_LOW;
                                ref_lift_back_posi = STEP_DOWN_BACK_LOW;
                            }
                        break;
                    }
                }
                //212,232,224
                if(autonomous.phase == 212){
                    if(pre_kouden1read == 1 && kouden1read == 0 && front_syusoku == true)
                        autonomous.phase = 213;
                    if(!limit4read && !limit5read)
                        autonomous.phase = 213;
                }
                if(autonomous.phase == 232){
                    if(pre_kouden1read == 1 && kouden1read == 0 && front_syusoku == true)
                        autonomous.phase = 233;
                    if(!limit4read && !limit5read)
                        autonomous.phase = 233;
                }
                if(autonomous.phase == 2351){
                    if(pre_kouden1read == 1 && kouden1read == 0 && front_syusoku == true)
                        autonomous.phase = 235;
                    if(!limit4read && !limit5read)
                        autonomous.phase = 235;
                }
                if(autonomous.phase == 224){
                    if(pre_kouden1read == 1 && kouden1read == 0 && front_syusoku == true)
                        autonomous.phase = 2225;
                    if(!limit4read && !limit5read)
                        autonomous.phase = 2225;
                }
                if(autonomous.phase == 212){
                    if(pre_kouden1read == 0 && kouden1read == 1 && front_syusoku == true)
                        autonomous.phase = 213;
                    if(!limit4read && !limit5read)
                        autonomous.phase = 213;
                }
                if(autonomous.phase == 232){
                    if(pre_kouden1read == 0 && kouden1read == 1 && front_syusoku == true )
                        autonomous.phase = 233;
                    if(!limit4read && !limit5read)
                        autonomous.phase = 233;
                }
                if(autonomous.phase == 2351){
                    if(pre_kouden1read == 0 && kouden1read == 1 && front_syusoku == true)
                        autonomous.phase = 235;
                    if(!limit4read && !limit5read)
                        autonomous.phase = 235;
                }
                if(autonomous.phase == 224){
                    if(pre_kouden1read == 0 && kouden1read == 1 && front_syusoku == true)
                        autonomous.phase = 2225;
                    if(!limit4read && !limit5read)
                        autonomous.phase = 2225;
                }

            break;
        }
        //端移動 232,224,212

        // stepup_flag = true;
        // stepup_count = 2;

        if(stepup_flag){
            switch (stepup_count) {
                case 1://上がる
                    ref_lift_front_posi = STEP_UP_FRONT_LOW;
                    ref_lift_back_posi = STEP_UP_BACK_LOW;
                    if(back_syusoku == 1 && front_syusoku == 1){
                        stepup_count = 2;
                        setPosi.z = gPosi.y;
                    }
                break;
                case 2://前移動
                    ref_lift_front_posi = STEP_UP_FRONT_LOW;
                    ref_lift_back_posi = STEP_UP_BACK_LOW;
                    vel_lift_back = 0.8;
                    ControlMode = MANUAL_MODE;
                    autostep_mode = true;
                    air_state = true;
                    air_up_flag = true;
                    refV.x = 0.3;
                    pc.printf("%lf,%d,%d\n",vel_lift_back,air_state,air_up_flag);
                    if(pre_kouden3read == 1 && kouden3read == 0){//止める
                        vel_lift_back = 0.00;
                        ControlMode = AUTO_MODE;
                        autostep_mode = false;
                        refV.x = 0.0;
                        air_state = false;
                        air_up_flag = false;
                        back_syusoku = false;
                        front_syusoku = false;
                        ref_lift_front_posi = STORAGE_POSI;
                        ref_lift_back_posi = STEP_UP_BACK_HIGH;
                        stepup_count = 3;
                    }
                break;
                case 3://昇降を上げる
                    ref_lift_front_posi = STORAGE_POSI;
                    autonomous.set_front_posi = STORAGE_POSI;
                    ref_lift_back_posi = STEP_UP_BACK_HIGH;
                    switch (autonomous.direction_flag) {
                        case DFRONT:
                            setPosi.x = autonomous.forest[route[autonomous.route_num].num].x + 0.889;
                            getPosi.x = platform.setAxisPosi(setPosi.x, POSIX);
                            getPosi.y = platform.setAxisPosi(setPosi.z, POSIY);
                        break;
                        case DRIGHT:
                            setPosi.y = autonomous.forest[route[autonomous.route_num].num].y + 0.889;
                            getPosi.y = platform.setAxisPosi(setPosi.y, POSIY);
                            getPosi.x = platform.setAxisPosi(setPosi.z, POSIX);
                        break;
                        case DLEFT:
                            setPosi.y = autonomous.forest[route[autonomous.route_num].num].y - 0.889;
                            getPosi.y = platform.setAxisPosi(setPosi.y, POSIY);
                            getPosi.x = platform.setAxisPosi(setPosi.z, POSIX);
                        break;
                        case DBACK:
                            setPosi.x = autonomous.forest[route[autonomous.route_num].num].x - 0.889;
                            getPosi.x = platform.setAxisPosi(setPosi.x, POSIX);
                            getPosi.y = platform.setAxisPosi(setPosi.z, POSIY);
                        break;
                    }
                    if(back_syusoku == 1 && front_syusoku == 1){
                        stepup_count = 0;
                        stepup_flag = false;
                        if(autonomous.phase == 2225)
                            autonomous.phase = 225;
                        if(autonomous.phase == 2236)
                            autonomous.phase = 236;
                        if(autonomous.flag_hi2hi)
                            autonomous.send_num = 12;
                    }
                break;
                // case 4://後輪動かす
                //     back_wheel_flag = true;
                //     ControlMode = MANUAL_MODE;
                //     autostep_mode = true;
                //     refV.x = 0.3;
                //     if(pre_kouden3read == 1 && kouden3read == 0){//止める
                //         back_wheel_flag = false;
                //         ControlMode = AUTO_MODE;
                //         autostep_mode = false;
                //         refV.x = 0.0;
                //         stepup_count = 5;
                //     }
                // break;
                // case 5://昇降あげる
                //     ref_lift_front_posi = STORAGE_POSI;
                //     ref_lift_back_posi = STEP_UP_BACK_HIGH;
                //     autonomous.set_front_posi = STORAGE_POSI;
                //     autonomous.set_back_posi = STEP_UP_BACK_HIGH;
                //      if(back_syusoku == 1 && front_syusoku == 1){//止める
                //         stepup_count = 0;
                //         stepup_flag = false;
                //         autonomous.phase = 225;
                //     }
                // break;
            }
        }
        // vel_lift_back = 0.03;
        if(stepdown_flag){
            switch (stepdown_count) {
                case 1://足下げる
                    ref_lift_front_posi = STEP_DOWN_FRONT_LOW;
                    ref_lift_back_posi = STEP_DOWN_BACK_LOW;
                    if(back_syusoku == true && front_syusoku == true)
                        stepdown_count = 2;
                break;
                case 2://前移動
                    ref_lift_front_posi = STEP_DOWN_FRONT_LOW;
                    ref_lift_back_posi = STEP_DOWN_BACK_LOW;
                    vel_lift_back = 0.3;
                    ControlMode = MANUAL_MODE;
                    autostep_mode = true;
                    air_state = true;
                    air_up_flag = true;
                    refV.x = 0.3;
                    if(pre_kouden2read == 0 && kouden2read == 1){//止める
                        vel_lift_back = 0.00;
                        ControlMode = AUTO_MODE;
                        autostep_mode = false;
                        air_state = false;
                        air_up_flag = false;
                        refV.x = 0.0;
                        back_syusoku = false;
                        front_syusoku = false;
                        ref_lift_front_posi = STEP_DOWN_FRONT_HIGH;
                        ref_lift_back_posi = STEP_DOWN_BACK_HIGH;
                        stepdown_count = 3;
                    }
                break;
                case 3://機体下げる
                    ref_lift_back_posi = STEP_DOWN_BACK_HIGH;
                    ref_lift_front_posi = STORAGE_POSI;
                    autonomous.set_front_posi = STORAGE_POSI;
                    autonomous.set_back_posi = STEP_DOWN_BACK_HIGH;
                    if(back_syusoku == 1 && front_syusoku == 1){
                        stepdown_count = 0;
                        stepdown_flag = false;
                        if(autonomous.phase == 2225)
                            autonomous.phase = 225;
                        if(autonomous.phase == 2236)
                            autonomous.phase = 236;
                    }
                break;
                // case 4://後輪動かす
                //     back_wheel_flag = true;
                //     ControlMode = MANUAL_MODE;
                //     autostep_mode = true;
                //     refV.x = 0.3;
                //     if(pre_kouden3read == 1 && kouden3read == 0){//止める
                //         back_wheel_flag = false;
                //         ControlMode = AUTO_MODE;
                //         autostep_mode = false;
                //         refV.x = 0.0;
                //         stepup_count = 5;
                //     }
                // break;
                // case 5://昇降あげる
                //     ref_lift_front_posi = STORAGE_POSI;
                //     ref_lift_back_posi = STEP_UP_BACK_HIGH;
                //     autonomous.set_front_posi = STORAGE_POSI;
                //     autonomous.set_back_posi = STEP_UP_BACK_HIGH;
                //      if(back_syusoku == 1 && front_syusoku == 1){//止める
                //         stepup_count = 0;
                //         stepup_flag = false;
                //         autonomous.phase = 225;
                //     }
                // break;
            }
        }


        //段越え機構関連のモーター指令
        if(flag_lift == 2){
            if(ref_lift_front_posi - front_lift_posi > 80){
                roboclawCmd0 = ms_qpps(0.3,500,0.026,1.0);
                front_syusoku = false;
            }else if(ref_lift_front_posi - front_lift_posi < -80){
                // roboclawCmd0 = ms_qpps(-1.0,500,0.026,1.0);
                roboclawCmd0 = ms_qpps(-1.0,500,0.026,1.0);
                front_syusoku = false;
            }else if(abs(ref_lift_front_posi - front_lift_posi) < 5){
                roboclawCmd0 = 0;
                front_syusoku = true;
            }else{
                front_syusoku = false;
                double vel = PID_lift_front_up.getCmd(ref_lift_front_posi, front_lift_posi, 0.3);
                roboclawCmd0 = ms_qpps(vel,500,0.026,1.0);
            }
            pc.printf(" %d ",roboclawCmd0);
            if(roboclawCmd0 >= 0){
                if(!limit8read){
                    roboclawCmd0 = 0;
                }
            }

            if(ref_lift_back_posi - back_lift_posi > 80){
                roboclawCmd1 = ms_qpps(0.3,500,0.026,1.0);
                back_syusoku = false;
            }else if(ref_lift_back_posi - back_lift_posi < -80){
                // roboclawCmd1 = ms_qpps(-1.0,500,0.026,1.0);
                roboclawCmd1 = ms_qpps(-1.0,500,0.026,1.0);
                back_syusoku = false;
            }else if(abs(ref_lift_back_posi - back_lift_posi) < 5){
                roboclawCmd1 = 0;
                back_syusoku = true;
            }else{
                back_syusoku = false;
                double vel = PID_lift_back_up.getCmd(ref_lift_back_posi, back_lift_posi, 0.3);
                roboclawCmd1 = ms_qpps(vel,500,0.026,1.0);
            }
            if(roboclawCmd1 >= 0){
                if(!limit9read){
                    roboclawCmd1 = 0;
                }
            }
        }

        // if(back_wheel_flag){//後輪（昇降はボタンの上下左右押しっぱなしで確認）
        //     // if(overstep_flag || overstep_R1_flag){
        //     //     vel_lift_back = 0.08;//0.03;
        //     // }else if(downstep_flag){
        //     //     // vel_lift_back = platform.refV_x_cmd;
        //     //     vel_lift_back = 0.02;//こっちは変えない
        //     //     // roboclawCmd2 = platform.refV_x_cmd;
        //     // }else if(downstep_R1_flag){
        //     //     vel_lift_back = -0.02;
        //     // }
        //     if(stepup_flag){
        //         vel_lift_back = 0.08;//0.03;
        //     }else if(downstep_flag){
        //         vel_lift_back = 0.03;
        //     }else {
        //         vel_lift_back = 0.00;
        //     }
        // }else{
        //     vel_lift_back = 0.0;
        //     // roboclawCmd2 = 0.0;
        // }

        roboclawCmd2 = ms_qpps(vel_lift_back,500,0.025,1.0);

        if(flag_stop){
            roboclawCmd0 = 0;
            roboclawCmd1 = 0;
            roboclawCmd2 = 0;
        }

        roboclaw1.SpeedM1(roboclawCmd0);//昇降前
        roboclaw2.SpeedM1(roboclawCmd1);//昇降後
        roboclaw2.SpeedM2(roboclawCmd2);//後輪

        cam_pitch_cmd = 0.0;
        cam_yaw_cmd = 0.0;
        cam_pitch_cmd2 = 0.0;
        cam_yaw_cmd2 = 0.0;

        rotate_cam1(cam_pitch_cmd, cam_yaw_cmd);
        rotate_cam2(cam_pitch_cmd2, cam_yaw_cmd2);
        // Servo.cmd(SERVO_ID1, 0, 0, 0);
        // Servo.cmd(SERVO_ID3, 4095, 0, 0);


        //エアの指令-----------------
        // if(air_up_flag){
        //     air_state = true;
        // }else {
        //     air_state = false;
        // }

        //リミットスイッチ前の値格納
        // pre_limit1read = limit1read;
        pre_limit2read = limit2read;
        pre_limit3read = limit3read;
        pre_limit4read = limit4read;
        pre_limit5read = limit5read;
        pre_limit6read = limit6read;
        pre_limit7read = limit7read;
        pre_limit8read = limit8read;
        pre_limit9read = limit9read;

        pre_kouden1read = kouden1read;
        pre_kouden2read = kouden2read;
        pre_kouden3read = kouden3read;

        

        //autocon phase による指令------------------
        if(autonomous.phase == 0){
            autonomous.send_num = 0;
        }
        if(autonomous.phase == 200){
            mode = MODE_FOREST;
        }else if(autonomous.phase == 60 || autonomous.phase == 62){
            mode = MODE_ZONE3;
        }


        //足回り速度指令部分------------------------------------------------------------------
    if (ControlMode == AUTO_MODE) {
        refV = autonomous.getRefVel(nextPhase);
    }else if(ControlMode == MANUAL_MODE){
        int cor_rotate = 0;
        double cor_angle = 0.0;
        ref_angle = 0.0;
        while (fabs(-gPosi.z + ref_angle + cor_rotate * 2 * M_PI) > M_PI) {
            if (-gPosi.z + ref_angle + cor_rotate * 2 * M_PI > 0)
            cor_rotate--;
            else
            cor_rotate++;
        }
        ref_kakudo = (ref_angle + cor_angle) + cor_rotate * 2 * M_PI;
        refV_zplus = posiPID_z.getCmd(ref_kakudo, gPosi.z, 2.9); //指令
        if ((fabs(ref_angle - gPosi.z) < 0.03) ) {
            refV_zplus = 0.0;
        }
        normalspeed_x = NORMALSPEED_X;
        normalspeed_y = NORMALSPEED_Y;
        normalspeed_z = NORMALSPEED_Z;
        joyLHypo = sqrt(pow(joyLY, 2.0) + pow(joyLX, 2.0));
        joyLRad = atan2(joyLY, joyLX) + INIT_Z;
        // joystickの指令をグローバルに変更
        if(!autostep_mode){
            refV.x = joyLHypo * cos(joyLRad + INIT_Z - gPosi.z) * normalspeed_x;//手動グローバル用
            if(field_s == RED){
                refV.y = -joyLHypo * sin(joyLRad + INIT_Z - gPosi.z) * normalspeed_y;
                // refV.z = -joyRY * normalspeed_z;
                refV.z = -joyRY * normalspeed_z + refV_zplus; //旋回(PID込み)
            }else if(field_s == BLUE){
                refV.y = joyLHypo * sin(joyLRad + INIT_Z - gPosi.z) * normalspeed_y;
                // refV.z = -joyRY * normalspeed_z;
                refV.z = joyRY * normalspeed_z + refV_zplus; //旋回(PID込み)
            } 
        }
    }else if(ControlMode == STANDBY_MODE){
        flag_stop = true;
    }
    //速度制限----------------------------------------------------------------------------------------
    if ((fabs(refV.x) >= LIMIT_VEL_X) || (fabs(refV.y) >= LIMIT_VEL_Y) || (fabs(refV.z) >= LIMIT_VEL_Z)) { //速度制限用
            refV.x = 0.0;
            refV.y = 0.0;
            refV.z = 0.0;
    }
    if(flag_stop || con0_count > 10){//コントローラが切れたときに速度指令を止める
        refV.x = 0.0;
        refV.y = 0.0;
        refV.z = 0.0;
        // platform.freeWheel();
    }
    // if(ControlMode == MANUAL_MODE || ControlMode == AUTO_MODE){
    // }
    //platform.getVel();
    //最終的な速度指令を与える場所
    // refV.z = -refV.z;
    platform.VelocityControl(refV);
    
            //SDカード値格納---------------------------------------------------------------------------
            if(flag_SDwrite && SDcount < 30000){
            // if(false){
                if(flag_simu){
                    A[SDcount] = gPosi.y;
                    B[SDcount] = gPosi.x;
                    C[SDcount] = gPosi.z;
                    D[SDcount] = refV.y;
                    E[SDcount] = refV.x;
                    F[SDcount] = refV.z;
                    K[SDcount] = pre_angle;
                    a[SDcount] = autonomous.phase;
                    // a[SDcount] = receive_collect_ball[0];
                    b[SDcount] = autonomous.getPathNum();
                    G[SDcount] = autonomous.get_t_be();
                    H1[SDcount] = autonomous.onx();
                    I[SDcount] = autonomous.ony();
                    J[SDcount] = autonomous.angle();
                    L4[SDcount] = autonomous.Px(3);
                    N[SDcount] = autonomous.Py(3);
                    //   M[SDcount] = autonomous.lim_refVz();
                    //   c[SDcount] = autonomous.acc_process();
                    O[SDcount] = getPosi.y;
                    P[SDcount] = getPosi.x;
                    Q[SDcount] = getPosi.z;
                    d[SDcount] = autonomous.syusoku;
                    e[SDcount] = interval_time;

                    f[SDcount] = flag_receive;
                    // g[SDcount] = receive_collect_ball[0];
                }else{

                    // %lf,%lf,%lf,%lf,%lf,%lf,%lf,%d,%d,%lf,%lf,%lf,%lf,%lf,%lf,%d,%d,%lf,%lf,%lf,%d,%d,%d,%d,%d,%d,
                    // A[SDcount] = gPosi.y;
                    // B[SDcount] = gPosi.x;
                    // C[SDcount] = gPosi.z;
                    // D[SDcount] = refV.y;
                    // E[SDcount] = refV.x;
                    // F[SDcount] = refV.z;
                    // G[SDcount] = pre_angle;
                    // a[SDcount] = autonomous.phase;
                    // b[SDcount] = autonomous.getPathNum();
                    // H1[SDcount] = autonomous.get_t_be();
                    // I[SDcount] = autonomous.onx();
                    // J[SDcount] = autonomous.ony();
                    // K[SDcount] = autonomous.angle();
                    // L4[SDcount] = autonomous.Px(3);
                    // M[SDcount] = autonomous.Py(3);
                    // //   M[SDcount] = autonomous.lim_refVz();
                    // //   c[SDcount] = autonomous.acc_process();
                    // c[SDcount] = autonomous.syusoku;
                    // // e[SDcount] = interval_time;
                    // d[SDcount] = timer.read_ms();

                    // N[SDcount] = lrtbPosi.y;
                    // O[SDcount] = lrtbPosi.x;
                    // P[SDcount] = lrtbPosi.z;
                    
                    // e[SDcount] = stepup_count;
                    // f[SDcount] = stepup_flag;
                    // g[SDcount] = stepdown_count;
                    // h[SDcount] = stepdown_flag;

                    // j[SDcount] = front_syusoku;
                    // k[SDcount] = back_syusoku;

                    // Q[SDcount] = roboclawCmd0;
                    // R[SDcount] = roboclawCmd1;
                    // S[SDcount] = roboclawCmd2;

                    

                    // l[SDcount] = air_up_flag;
                    // m[SDcount] = autonomous.send_num;
                    // n[SDcount] = kouden1read;
                    // o[SDcount] = kouden2read;
                    // p[SDcount] = kouden3read;
                    // q[SDcount] = cubeIndex;
                    // r[SDcount] = nextIndex;
                    // s[SDcount] = overstep_phase;
                    // t[SDcount] = downstep_phase;
                    // // m[SDcount] = overstep_R1_phase;
                    // // n[SDcount] = downstep_R1_phase;
                    // o[SDcount] = hold_phase;

                    // q[SDcount] = mode;

                    // r[SDcount] = next_box_state;

                    // u[SDcount] = limit4read;
                    // v[SDcount] = limit5read;
                    // w[SDcount] = autonomous.up_num;

                    // // v[SDcount] = autonomous.lift_check;
                    // // r[SDcount] = ControlMode;
                    // // w[SDcount] = back_wheel_flag;
                    // x[SDcount] = autonomous.route_num;
                    // y[SDcount] = route[autonomous.route_num].num;
                    // z[SDcount] = back_wheel_flag;

                    // U[SDcount] = autonomous.rotate_radian;
                    // V[SDcount] = ref_lift_front_posi;
                    // W[SDcount] = ref_lift_back_posi;

                    // X[SDcount] = front_lift_posi;
                    // Y[SDcount] = back_lift_posi;

                    // %d,,%lf,%lf,%lf,%lf,%lf,%lf,,%d,%d,%lf,%lf,%lf,%lf,%lf,%lf,,%lf,%lf,%lf,,%d,%d,%d,%lf,%lf,%lf,%lf,,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n

                    // time,,gPosi.y,gPosi.x,gPosi.z,refV.y,refV.x,refV.z,,phase,getPathNum,get_t_be,onx,ony,angle,Px(3),Py(3),,lrtbPosi.y,lrtbPosi.x,lrtbPosi.z,,roboclawCmd0,roboclawCmd1,roboclawCmd2,ref_lift_front_posi,ref_lift_back_posi,front_lift_posi,back_lift_posi,front_syusoku,back_syusoku,stepup_flag,stepup_count,stepdown_flag,stepdown_count,kouden1read,kouden3read,air_state,limit4read,limit5read,up_num,send_num,route_num,route[autonomous.route_num].num\n

                    // %d,

                    // z[i],,A[i],B[i],C[i],D[i],E[i],F[i],,a[i],b[i],G[i],H1[i],I[i],J[i],K[i],L4[i],,M[i],N[i],O[i],,c[i],d[i],e[i],P[i],Q[i],R[i],S[i],,f[i],g[i],h[i],j[i],k[i],l[i],m[i],n[i],o[i],p[i],q[i],r[i],s[i],t[i],u[i]\n

                    // z[SDcount] = timer.read_ms();
                    z[SDcount] = t1.read_ms();

                    // %lf,%lf,%lf,%lf,%lf,%lf,

                    A[SDcount] = gPosi.y;
                    B[SDcount] = gPosi.x;
                    C[SDcount] = gPosi.z;
                    D[SDcount] = refV.y;
                    E[SDcount] = refV.x;
                    F[SDcount] = refV.z;

                    // %d,%d,

                    a[SDcount] = autonomous.phase;
                    b[SDcount] = autonomous.getPathNum();

                    // %lf,%lf,%lf,%lf,%lf,%lf,

                    // G[SDcount] = autonomous.get_t_be();
                    // H1[SDcount] = autonomous.onx();
                    // I[SDcount] = autonomous.ony();
                    // J[SDcount] = autonomous.angle();
                    G[SDcount] = platform.mdCmdA;
                    H1[SDcount] = platform.mdCmdB;
                    I[SDcount] = platform.mdCmdC;
                    J[SDcount] = platform.mdCmdD;
                    K[SDcount] = autonomous.Px(3);
                    L4[SDcount] = autonomous.Py(3);

                    // %lf,%lf,%lf,

                    M[SDcount] = lrtbPosi.y;
                    N[SDcount] = lrtbPosi.x;
                    O[SDcount] = lrtbPosi.z;

                    // %d,%d,%d,

                    c[SDcount] = roboclawCmd0;
                    d[SDcount] = roboclawCmd1;
                    e[SDcount] = roboclawCmd2;

                    // %lf,%lf,%lf,%lf
                    // %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,

                    

                    P[SDcount] = ref_lift_front_posi;
                    Q[SDcount] = ref_lift_back_posi;

                    R[SDcount] = front_lift_posi;
                    S[SDcount] = back_lift_posi;

                    f[SDcount] = front_syusoku;
                    g[SDcount] = back_syusoku;

                    h[SDcount] = stepup_flag;
                    j[SDcount] = stepup_count;
                    k[SDcount] = stepdown_flag;
                    l[SDcount] = stepdown_count;

                    m[SDcount] = kouden1read;
                    a1[SDcount] = kouden2read;
                    n[SDcount] = kouden3read;
                    o[SDcount] = air_state;

                    p[SDcount] = limit4read;
                    q[SDcount] = limit5read;

                    r[SDcount] = autonomous.up_num;
                    s[SDcount] = autonomous.send_num;
                    t[SDcount] = autonomous.route_num;
                    u[SDcount] = route[autonomous.route_num].num;

                    v[SDcount] = cubeIndex;
                    w[SDcount] = autonomous.direction_flag;
                    T[SDcount] = autonomous.setx;
                    U[SDcount] = autonomous.sety;
                    V[SDcount] = autonomous.setz;

                    W[SDcount] = autonomous.diffx;
                    X[SDcount] = autonomous.diffy;

                    x[SDcount] = autonomous.khs;
                    y[SDcount] = cubeIndex;

                    // O1[SDcount] = platform.mdCmdA;
                    // O2[SDcount] = platform.mdCmdB;
                    // O3[SDcount] = platform.mdCmdC;
                    // O4[SDcount] = platform.mdCmdD;





                    // K[SDcount] = pre_angle;
                    // a[SDcount] = autonomous.phase;
                    // b[SDcount] = autonomous.getPathNum();
                    // G[SDcount] = autonomous.get_t_be();
                    // H1[SDcount] = autonomous.onx();
                    // I[SDcount] = autonomous.ony();
                    // J[SDcount] = autonomous.angle();
                    // L4[SDcount] = autonomous.Px(3);
                    // N[SDcount] = autonomous.Py(3);
                    // //   M[SDcount] = autonomous.lim_refVz();
                    // //   c[SDcount] = autonomous.acc_process();
                    // d[SDcount] = autonomous.syusoku;
                    // // e[SDcount] = interval_time;
                    // e[SDcount] = timer.read_ms();

                    // M[SDcount] = lrtbPosi.y;
                    // O[SDcount] = lrtbPosi.x;
                    // P[SDcount] = lrtbPosi.z;
                    
                    
                    // S[SDcount] = vel_lift_back;
                    // T[SDcount] = normalspeed_y;
                    // U[SDcount] = normalspeed_z;

                    // V[SDcount] = roboclawCmd0;
                    // R[SDcount] = roboclawCmd1;
                    // W[SDcount] = roboclawCmd2;

                    // f[SDcount] = air_up_flag;
                    // g[SDcount] = autonomous.send_num;
                    // h[SDcount] = kouden1read;
                    // j[SDcount] = front_syusoku;
                    // p[SDcount] = kouden3read;
                    // k[SDcount] = back_syusoku;
                    // l[SDcount] = back_wheel_flag;
                    // m[SDcount] = stepup_count;
                    // n[SDcount] = stepdown_count;
                    // // m[SDcount] = overstep_R1_phase;
                    // // n[SDcount] = downstep_R1_phase;
                    // o[SDcount] = hold_phase;

                    // q[SDcount] = mode;

                    // r[SDcount] = next_box_state;

                    // s[SDcount] = limit4read;
                    // t[SDcount] = limit5read;
                    // u[SDcount] = autonomous.up_num;

                    // // v[SDcount] = autonomous.lift_check;
                    // // r[SDcount] = ControlMode;
                    // // w[SDcount] = back_wheel_flag;
                    // v[SDcount] = autonomous.route_num;
                    // r[SDcount] = route[autonomous.route_num].num;
                    // w[SDcount] = back_wheel_flag;

                    // X[SDcount] = autonomous.rotate_radian;
                    // Y[SDcount] = ref_lift_front_posi;
                    // Z[SDcount] = ref_lift_back_posi;

                    // Q[SDcount] = front_lift_posi;
                    // G[SDcount] = back_lift_posi;

                    // c[SDcount] = air_state;


                    // gPosi.y,gPosi.x,gPosi.z,refV.y,refV.x,refV.z,pre_angle,autonomous.phase,autonomous.getPathNum(),autonomous.get_t_be(),autonomous.onx(),autonomous.ony(),autonomous.angle(),autonomous.Px(3),autonomous.Py(3),autonomous.syusoku,timer.read_ms(),lrtbPosi.y,lrtbPosi.x,lrtbPosi.z,stepup_count,stepup_flag,stepdown_count,stepdown_flag,front_syusoku,back_syusoku,roboclawCmd0,roboclawCmd1,roboclawCmd2,air_up_flag,autonomous.send_num,kouden1read,kouden2read,kouden3read,cubeIndex,nextIndex,overstep_phase,downstep_phase,hold_phase,mode,next_box_state,limit4read,limit5read,autonomous.up_num,autonomous.route_num,route[autonomous.route_num].num,back_wheel_flag,autonomous.rotate_radian,ref_lift_front_posi,ref_lift_back_posi,front_lift_posi,back_lift_posi/n


                    // IQ0[SDcount] = platform.Iq_meas0;
                    // IQ1[SDcount] = platform.Iq_meas1;
                    // IQ2[SDcount] = platform.Iq_meas2;
                    // IQ3[SDcount] = platform.Iq_meas3;
                    // IQ4[SDcount] = platform.Iq_meas4;
                    // IQ5[SDcount] = platform.Iq_meas5;
                    // IQ6[SDcount] = platform.Iq_meas6;
                    // IQ7[SDcount] = platform.Iq_meas7;
                }
                SDcount++;
            }

            // if(flag_simu && flag_comm){
            //     int num = sprintf(sbuf,
            //                 "%.4lf,%.4lf,%.4lf,Hello World,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%d;%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%d,"
            //                 "%d,%.4lf,%d,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf\n",
            //                 refV.x, refV.y, refV.z,autonomous.R2_Adjacent_count,autonomous.next_indices,autonomous.syusoku,autonomous.collect_count,cubePosi.x,cubePosi.y,autonomous.nextX,autonomous.nextY,interval_time, autonomous.phase,autonomous.gettheta_b() - autonomous.gettheta_a(),refV.x, refV.y, refV.z,gPosi.x,gPosi.y,gPosi.z,nextPhase,A[SDcount-1], B[SDcount-1], C[SDcount-1], D[SDcount-1],
            //                 E[SDcount-1], F[SDcount-1], K[SDcount-1], a[SDcount-1], b[SDcount-1], G[SDcount-1],d[SDcount-1], H1[SDcount-1], I[SDcount-1],
            //                 J[SDcount-1],L4[SDcount - 1],N[SDcount-1]);
            //     //cubePosi3.x,cubePosi3.y,cubePosi3.z
            //     // int num = sprintf(sbuf,
            //     //           "%.4lf,%.4lf,%.4lf;%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%d,"
            //     //           "%d,%.4lf,%d,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf\n",
            //     //           refV.x, refV.y, refV.z, A[SDcount-1], B[SDcount-1], C[SDcount-1], D[SDcount-1],
            //     //           E[SDcount-1], F[SDcount-1], K[SDcount-1], a[SDcount-1], b[SDcount-1], G[SDcount-1],d[SDcount-1], H1[SDcount-1], I[SDcount-1],
            //     //           J[SDcount-1],L4[SDcount - 1],N[SDcount-1]);
            //     sendData_maicon(sbuf, num);
            // }
            pre_buttonState_E = buttonState_E;
   
        flag_int = false; // flagを下す．
    }
    //print----------------------------------------------------------
    if(flag_print && set_print){
        //制御周期
        // sprintf(str,"time:%d  ",interval_time);
        // invoke_print(str);
        
        //モード
        // sprintf(str,"conMode:%d,", ControlMode);//モード
        // invoke_print(str);
        // sprintf(str,"lpms:%lf", lpms.get_z_angle());//モード
        // invoke_print(str);
        // sprintf(str,"bno:%d,%lf", bno085.read(),bno085.anglez);
        // invoke_print(str);
        //autocon--------------------------------------
        // sprintf(str,"phase:%d  ",autonomous.phase );
        // invoke_print(str);
        // // // //最終的な自己位置
        // sprintf(str,"g,%4.4lf,%4.4lf,%4.4lf ", gPosi.x, gPosi.y, gPosi.z);
        // invoke_print(str);
        // // // // //速度指令
        sprintf(str,"x:%4.4lf,y:%4.4lf,z:%4.4lf vx:%4.4lf,vy;%4.4lf,vz:%4.4lf phase:%d  %d  %d  %d  %d  %d %d %lf %lf %lf %lf %d %d %lf %d %d, %d, %d, %d , %d,get::%4.4lf,%4.4lf,%4.4lf,%4.4lf,%4.4lf\n", gPosi.x, gPosi.y, gPosi.z,refV.x,refV.y,refV.z,autonomous.phase,roboclawCmd0,roboclawCmd1,roboclawCmd2,limit8read,limit9read,flag_lift,front_lift_posi,back_lift_posi,ref_lift_front_posi,ref_lift_back_posi,stepup_count,air_state,vel_lift_back,air_state,stepup_flag,kouden1read,kouden2read,kouden3read,mode,getPosi.x,getPosi.y,getPosi.z,distance_front,lrtbPosi.x);
        // sprintf(str,"%lf,%lf,%lf,%lf,%lf,%lf,%d,%d\n",autonomous.forest[route[autonomous.route_num].num].x,autonomous.forest[route[autonomous.route_num].num].y,autonomous.forest[route[autonomous.route_num + 1].num].x,autonomous.forest[route[autonomous.route_num + 1].num].y,autonomous.diffx,autonomous.diffy,autonomous.direction_flag,autonomous.phase);
        invoke_print(str);

        // sprintf(str, "%d, %lf, %lf, %d",roboclawCmd2,);
        
        // sprintf(str," refkakudo:%4.4lf ",ref_kakudo);
        // invoke_print(str);
        // sprintf(str," y:%4.4lf ",angley);
        // invoke_print(str);
        // sprintf(str," zplus:%4.4lf ",refV_zplus);
        // invoke_print(str);  
        //ジョイスティック
        // sprintf(str,"%lf,%lf,%lf,%lf ", joyLX, joyLY, joyRX, joyRY);
        // invoke_print(str);

        //LRTB（実測値）生データ
        // sprintf(str,"lrtb,0:%d,1:%d,2:%d,3:%d,",lrtb_0,lrtb_1,lrtb_2,lrtb_3);
        // invoke_print(str);
        // sprintf(str,"%d,%d",lrtb_0,lrtb_2);//キャリブレーション用
        // invoke_print(str);
        // sprintf(str,"lrtb 0:%d,1:%d,2:%d,3:%d",lrtb.sensorData[0],lrtb.sensorData[1],lrtb.sensorData[2],lrtb.sensorData[3]);
        // invoke_print(str);
        //LRTB（計算値）距離変換
        // sprintf(str,"dist 0:%lf,1:%lf,2:%lf,3:%lf",lrtb_dist_0,lrtb_dist_1,lrtb_dist_2,lrtb_dist_3);
        // invoke_print(str);
        // sprintf(str,"dist ↑:%lf,←:%lf,→:%lf,↓:%lf",lrtb0_wall_x,lrtb1_wall_x,lrtb2_wall_x,lrtb3_wall_x);
        // invoke_print(str);
        // sprintf(str,"dist ↑:%lf,←:%lf,→:%lf,↓:%lf",lrtb0_wall_y,lrtb1_wall_y,lrtb2_wall_y,lrtb3_wall_y);
        // invoke_print(str);
        // sprintf(str,"dist ↑:%lf,←:%lf,→:%lf,↓:%lf",lrtb_front_dist_x,lrtb_left_dist_y,lrtb_right_dist_y,lrtb_back_dist_y);
        // invoke_print(str);
        // 中心からLRTB出力点までの垂直距離x
        // sprintf(str,"vertical 0:%lf,1%lf,2:%lf,3:%lf",lrtb0_dist_x,lrtb1_dist_x,lrtb2_dist_x,lrtb3_dist_x);
        // invoke_print(str);
        //中心からLRTB出力点までの垂直距離y
        // sprintf(str,"vertical 0:%lf,1%lf,2:%lf,3:%lf",lrtb0_dist_y,lrtb1_dist_y,lrtb2_dist_y,lrtb3_dist_y);
        // invoke_print(str);
        //LRTB出力点から壁までの垂直距離x
        // sprintf(str,"verticalx 0:%lf,1%lf,2:%lf,3:%lf",lrtb0_wall_x,lrtb1_wall_x,lrtb2_wall_x,lrtb3_wall_x);
        // invoke_print(str);
        //LRTB出力点から壁までの垂直距離y
        // sprintf(str,"verticalx 0:%lf,1%lf,2:%lf,3:%lf",lrtb0_wall_y,lrtb1_wall_y,lrtb2_wall_y,lrtb3_wall_y);
        // invoke_print(str);
        //  sprintf(str,"vertical 0:%lf,1%lf,2:%lf,3:%lf",lrtb_back_dist_x,lrtb_back_dist_y,lrtb_right_dist_x,lrtb_right_dist_y);
        // invoke_print(str);
        //  sprintf(str,"distans 0:%lf,1%lf,2:%lf,3:%lf",distance_front,distance_out,distance_in,distance_back);
        // invoke_print(str);


    //回収，障害確認-----------------------------------
        // sprintf(str,"count:%d ", obj_count);
        // invoke_print(str);

        // sprintf(str,"SW_count:");
        // invoke_print(str);
        // for (int i = 0; i < 12; i++) {
        //     sprintf(str,"%d ", SWcount[i]);
        //     invoke_print(str);
        // }
        // sprintf(str,"\t");
        // invoke_print(str);

        //objの状態-----------------------------
        // sprintf(str,"obs_state:");
        // invoke_print(str);
        // for (int i = 0; i < 12; i++) {
        //     sprintf(str,"%d ", obj_state[i]);
        //     invoke_print(str);
        // }
        // sprintf(str,"\t");
        // invoke_print(str);

        // sprintf(str,"binState:");
        // invoke_print(str);

        // for (int i = 0; i < 12; i++) {
        //     sprintf(str,"%d ", bin_addbuttonState[i]);
        //     invoke_print(str);
        // }
        // sprintf(str,"colle_num:");
        // invoke_print(str);

        // for (int i = 0; i < 4; i++) {
        //     sprintf(str,"%d ", autonomous.collect_num[i]);
        //     invoke_print(str);
        // }
        // sprintf(str,"\t");
        // invoke_print(str);
        
        // sprintf(str,"\n");
        // invoke_print(str);

        //ボタンの状態
        // sprintf(str,"button:%d, add_button:%d ", buttonState, addbuttonState);
        // invoke_print(str);
        // sprintf(str,"left:%d, right:%d ", addbuttonLeftState, addbuttonRightState);
        // invoke_print(str);
        // sprintf(str,"A:%d, B:%d , C:%d", buttonA, buttonB, buttonC);
        // invoke_print(str);

    //段越え計算式の変数--------------------------------------------------
        // sprintf(str,"liftvel %lf,%lf,%lf",vel_lift_front,vel_lift_center, vel_lift_back);
        // invoke_print(str);
    //段越え確認----------------------------------------------------------------------
        // sprintf(str,"conf_Mode:%d", conf_Mode);//モード
        // invoke_print(str);
        // sprintf(str,"over_phase:%d", overstep_phase);//モード
        // invoke_print(str);
        // sprintf(str,"down_phase:%d", downstep_phase);//モード
        // invoke_print(str);
        // // //roboclaw
        // sprintf(str,"roboclawCmd1 0:%d 1:%d 2:%d",roboclawCmd0,roboclawCmd1,roboclawCmd2);
        // invoke_print(str);
        // // //PID
        // sprintf(str,"PIDcmd front:%d, back:%d",PID_lift_front_cmd,PID_lift_back_cmd);
        // invoke_print(str);
        //目標値
        // sprintf(str,"昇降　front:%lf, back:%lf",ref_lift_front_posi,ref_lift_back_posi);
        // invoke_print(str);

        //エンコーダカウント（カメラ）
        // sprintf(str,"pitch1:%d, yaw1:%d, pitch2:%d, yaw2:%d ",get_cam1_pitch_count,get_cam1_yaw_count,get_cam2_pitch_count,get_cam2_yaw_count);
        // invoke_print(str);
        // //エンコーダカウント（昇降）
        // sprintf(str,"front:%d, back:%d",get_lift_front_count,get_lift_back_count);
        // invoke_print(str);
        // sprintf(str,"front:%d, back:%d",pre_get_lift_front_count,pre_get_lift_back_count);
        // invoke_print(str);
        //位置変換
        // sprintf(str,"位置 front:%lf, back:%lf",get_lift_front_posi,get_lift_back_posi);
        // invoke_print(str);
        // sprintf(str,"front:%lf, back:%lf",front_lift_posi,back_lift_posi);
        // invoke_print(str);
        // sprintf(str,"frontdiff:%lf, back:%lf",fabs(ref_lift_front_posi - front_lift_posi),fabs(ref_lift_back_posi - back_lift_posi));
        // invoke_print(str);
        // sprintf(str,"set front:%d, back:%d",set_height_front,set_height_back);
        // invoke_print(str);
        // sprintf(str,"limit:%d, back:%d",limit8read,limit9read);
        // invoke_print(str);
        // sprintf(str,"pitch1:%lf, yaw1:%lf, pitch2:%lf, yaw2:%lf ",cam_pitch,cam_yaw,cam_pitch2,cam_yaw2);
        // invoke_print(str);//カメラアングル

        //リミットスイッチ（1:ハンド取得判定，2:槍先ラック1，3:槍先ラック2，4:右側スイッチ1，5:右側スイッチ2，6:左側スイッチ1，7:左側スイッチ2）
        // sprintf(str,"limit 2:%d 3:%d 4:%d 5:%d 6:%d 7:%d", limit2read,limit3read,limit4read,limit5read,limit6read,limit7read);//モード
        // invoke_print(str);
        // sprintf(str,"limit1:%d 2:%d 3:%d 4:%d 5:%d 6:%d 7:%d", pre_limit1read,pre_limit2read,pre_limit3read,pre_limit4read,pre_limit5read,pre_limit6read,pre_limit7read);//モード
        // invoke_print(str);
        // for(int i = 0; i < 12; i++){
        //     sprintf(str,"limit%d:%d,", i,limitdata[i]);//モード
        //     invoke_print(str);
        // }
        //光電----------------------
        // sprintf(str,"kouden 1:%d 2:%d 3:%d",kouden1read,kouden2read,kouden3read);
        // invoke_print(str);

        //HC-RS04------------------
        // sprintf(str,"%4.4lf ", us_distance);
        // invoke_print(str);
        //拡張基盤確認-------------------------------------
        // sprintf(str,"btn 0:%d 1:%d 2:%d 3:%d 4:%d ",btn[0].read(),btn[1].read(),btn[2].read(),btn[3].read(),btn[4].read());
        // invoke_print(str);

        //エアー
        // sprintf(str,"air_flag:%d, ", air_up_flag);
        // invoke_print(str);

    //通信確認----------------------------------------------
        // sprintf(str,"ball_num:%d, ", receive_ball_num[0]);
        // invoke_print(str);
        // sprintf(str,"ball_posi x:%d, y:%d", receive_ball_posi[0][0],receive_ball_posi[0][1]);
        // invoke_print(str);
    //  コントローラ
        // sprintf(str,"statemode:%d, ", state_mode);
        // invoke_print(str);

        // sprintf(str,"senddata:%d, ", autonomous.send_num);
        // invoke_print(str);

        sprintf(str,"\n");
        invoke_print(str);

        flag_print = false; // flagを下す
    }

    if(flag_tuusin && up_comm){
        uint8_t send_data[7];
        uint8_t recievedata[2];//受け取り値
        //0->上半身phase
            //1:槍先取得，2:槍先離した，3:KFS保持した，4:KFSを機体に格納した
        //1->box保持個数
        // sprintf(str,"up");
        // invoke_print(str);

        // static int posi[3] = {0, 0, 0};

        // posi[0] = gPosi.x * 1000;
        // posi[1] = gPosi.y * 1000;
        // posi[2] = gPosi.z * 1000;
        
        
        send_data[0] = autonomous.send_num;
        send_data[1] = air_state;
        send_data[2] = autonomous.khs;//KFS_height_state;
        send_data[3] = 3;//also_KFS_hold_num
        send_data[4] = lrtb_check;
        send_data[5] = (int(abs(front_lift_posi)) & 0b000000111111);
        send_data[6] = (int(abs(front_lift_posi)) & 0b111111000000) >> 6;
         // send_data[2] = front_lift_posi * 10;

        //0個目：0->初期，1->ハンド展開，
        //0個目：10->ハンド倒してほしい時，11->ハンドを上げてほしい時，12-> 昇降が上に格納する高さに行ったら(格納していい，ハンドを離していい時)
        //0個目：20->箱を準備してほしい時（箱を取り出す高さにした時），21->置いてほしい時，
        //1個目：足airのonoff
        //2個目：次に取るKFSの高さ．1->低い，2->高い（最初の真ん中は低いとして送る）
        //3個目：取得予定の個数（テスト時4）
        //4個目：lrtbによる取得範囲（段が下の時だけ）0->範囲外，1->範囲内

        talk.sendData(send_data, 7);
        talk.update(2);

        for(int i = 0; i < 2; i++){//受け取り
            recievedata[i] = talk.data[i];
        }

        autonomous.up_num = (int)recievedata[0];
        autonomous.up_box_num = (int)recievedata[1];

        //0個目：1->槍先取得，2->槍先離した，3->KFSを回収時保持した，4->KFSを機体内に格納済み
        //1個目：box取得個数
    }

    //ソレノイド制御基板通信-------------------------------------------
    // if(flag_solenoido){
    //     //printf("0");
    //     /*str（送信データ）
    //     '0' -> 0b000 ->前ソレノイドOFF
    //     '2' -> 0b010 -> 真ん中のソレノイドON、それ以外OFF
    //     '7' -> 0b111 -> 全ソレノイドON
    //     */
    //     if(flag_sole){//on
    //         sole_b_comma[0] = 0;
    //         sole_b_comma[1] = 1;
    //         sole_b_comma[2] = 0;
    //     }else{
    //         sole_b_comma[0] = 0;
    //         sole_b_comma[1] = 0;
    //         sole_b_comma[2] = 0;
    //     }

    //     bool sole_command[] = {sole_b_comma[0],sole_b_comma[1],sole_b_comma[2]};
    //     sole_send_data = sole_command[0] * pow(2,sole_data2_num-1) + sole_command[1] * pow(2,sole_data2_num-2) + + sole_command[2] * pow(2,sole_data2_num-3); 

    //     char data = sole_send_data;
    //     SoleSerial.printf("%d\r\n", data);
    //     flag_solenoido = false;
    // }

    // pcとの通信--------------------------------------------------------------------
    // if(pc_comm && flag_pc && !flag_error){
    if(pc_comm && flag_pc){
        static int send_posi[6];
        static char send_data[22];
        int data_num = 22;
        int send_pitch;
        int send_yaw;
        char sign1 = 0x00;
        char sign2 = 0x00;
        char sign3 = 0x00;
        char sign4 = 0x00;
        double CAM_THETAR = 47.88;
        double CAM_DISTR = 0.49205;
        double CAM_POINT_DISTR = 0.028;
        double CAM_THETAL = -47.88;
        double CAM_DISTL = 0.49205;
        double CAM_POINT_DISTL = 0.028;
        int send_pitchr = 0;
        int send_pitchl = 0;
        int send_yawr = 0;
        int send_yawl = 0;


        // send_posi[0] = int(cam_posi.x * 1000);
        // send_posi[1] = abs(int(cam_posi.y * 1000));
        // send_posi[2] = abs(int(cam_posi.z * 1000));

        // cam_posir.x = gPosi.x + CAM_DISTR * cos(gPosi.z + deg2rad(CAM_THETAR)) + CAM_POINT_DIST2 * cos(gPosi.z + cam_yaw2 + deg2rad(CAM_POINT_THETA2));
        // cam_posir.y = gPosi.y + CAM_DIST2 * sin(gPosi.z + deg2rad(CAM_THETA2)) + CAM_POINT_DIST2 * sin(gPosi.z + cam_yaw2 + deg2rad(CAM_POINT_THETA2));
        // cam_posir.z = gPosi.z + cam_yaw2 + M_PI;

        cam_posir.x = gPosi.x + CAM_DISTR * cos(gPosi.z + deg2rad(CAM_THETAR)) + CAM_POINT_DISTR * cos(gPosi.z + cam_yaw);
        cam_posir.y = gPosi.y + CAM_DISTR * sin(gPosi.z + deg2rad(CAM_THETAR)) + CAM_POINT_DISTR * sin(gPosi.z + cam_yaw);
        cam_posir.z = gPosi.z + cam_yaw;

        cam_posil.x = gPosi.x + CAM_DISTL * cos(gPosi.z + deg2rad(CAM_THETAL)) + CAM_POINT_DISTL * cos(gPosi.z + cam_yaw2);
        cam_posil.y = gPosi.y + CAM_DISTL * sin(gPosi.z + deg2rad(CAM_THETAL)) + CAM_POINT_DISTL * sin(gPosi.z + cam_yaw2);
        cam_posil.z = gPosi.z + cam_yaw2;

        pc.printf("x:%lf y:%lf z:%lf  RX:%lf , Y:%lf , Z:%lf , yaw:%lf , pitch:%lf , LX:%lf , Y:%lf , Z:%lf , yaw:%lf , pitch:%lf , %d\n",gPosi.x,gPosi.y,gPosi.z,cam_posir.x,cam_posir.y,cam_posir.z,cam_yaw,cam_pitch,cam_posil.x,cam_posil.y,cam_posil.z,cam_yaw2,cam_pitch2,get_cam1_pitch_count);
        // pc.printf("%d %d %d %d\n",get_cam1_pitch_count,get_cam1_yaw_count,get_cam2_pitch_count,get_cam2_yaw_count);
        //6240 3567 7848 6799


        send_posi[0] = int(cam_posir.x * 1000);
        send_posi[1] = abs(int(cam_posir.y * 1000));
        send_posi[2] = abs(int(cam_posir.z * 1000));
        // send_posi[0] = 1500;
        // send_posi[1] = 3000;//abs(int(cam_posir.y * 1000));
        // send_posi[2] = 0;//abs(int(cam_posir.z * 1000));
        send_pitchr = 0;
        send_yawl = 0;
        send_posi[3] = int(cam_posil.x * 1000);
        send_posi[4] = abs(int(cam_posil.y * 1000));
        send_posi[5] = abs(int(cam_posil.z * 1000));
        send_pitchl = 0;
        send_yawl = 0;


        // mode = 2;
        
        //pcに送信するデータ
        // send_data[0] = receive_collect_ball[0];
        send_data[0] = send_posi[0] >> 8 & 0xFF;
        send_data[1] = send_posi[0] & 0xFF;
        send_data[2] = (send_posi[1] >> 8) | sign4;
        send_data[3] = send_posi[1] & 0xFF;
        send_data[4] = (send_posi[2] >> 8 & 0x7F) | sign1;
        send_data[5] = send_posi[2] & 0xFF;
        send_data[6] = (send_pitchr >> 8 & 0x7F) | sign2;
        send_data[7] = send_pitchr & 0xFF;
        send_data[8] = (send_yawr >> 8 & 0x7F) | sign3;
        send_data[9] = send_yawr & 0xFF;
        
        send_data[10] = send_posi[3] >> 8 & 0xFF;
        send_data[11] = send_posi[3] & 0xFF;
        send_data[12] = (send_posi[4] >> 8) | sign4;
        send_data[13] = send_posi[4] & 0xFF;
        send_data[14] = (send_posi[5] >> 8 & 0x7F) | sign1;
        send_data[15] = send_posi[5] & 0xFF;
        send_data[16] = (send_pitchl >> 8 & 0x7F) | sign2;
        send_data[17] = send_pitchl & 0xFF;
        send_data[18] = (send_yawl >> 8 & 0x7F) | sign3;
        send_data[19] = send_yawl & 0xFF;
        send_data[20] = 2;//mode;
        send_data[21] = '\n';

        // send_data[11] = SDcount >> 8 & 0xFF;
        // send_data[12] = SDcount & 0xFF;
        // send_data[13] = send_ball_num & 0xFF;
        // send_data[14] = send_dir & 0xFF;
        // send_data[15] = rice_mode & 0xFF;
        // send_data[16] = autonomous.phase & 0xFF;
        // send_data[17] = collect_phase & 0xFF;
        // send_data[18] = '\n';

        //送信
        // sprintf(str,"senddata:%d, ", send_data[0]);
        // invoke_print(str);

        //受信
        // sprintf(str,"receive_data:%d, ", receive_collect_ball[0]);
        // invoke_print(str);

        //printf("senddata: %d",send_data[0]);
        //sendData_maicon(send_data,18);
        // sendData_pc(send_data,data_num);
        sendData(send_data,data_num);

        // flag_receive = receiveData_pc(mode);
        // flag_receive = receiveData_pc(20);
        flag_receive = receiveData(mode);
        if(flag_receive){
            // pre_collect_ball = receive_collect_ball[0];
            // pre_collect_ball_num = receive_ball_num[0];
            // pre_ball_posi_x = receive_ball_posi[0][0];
            // pre_ball_posi_y = receive_ball_posi[0][1];
            // pre_remove_ball_dist = receive_remove_dist[0] / 1000.0;
            // pre_collect_angle = deg2rad(receive_collect_angle[0] - 90);
        }
        flag_pc = false;
    }

//コントローラへ値送信-----------------------------------------------------------------
    if (flag_con_tuusin && con_comm) {//PRINT_TIME毎に処理される．フラグ処理．
            // led3 = (!flag_free) ? 1 : 0; // flag_freeがtrueだと消灯．falseだと点灯
            // led4 = (joyAndPadState) ? 1 : 0;
            // printf("time:%d  ",interval_time);

            int posi[3] = {0,0,0};
            int vel[3] = {0,0,0};
            uint8_t con_sendData[13] = {0};

            uint8_t sendData_state[] = {}; // 12個の2bitを詰めるので3バイト

            // for (int i = 0; i < 12; i++) {
            //     int byteIndex = i / 4;       // 4個で1バイト
            //     int bitShift = (i % 4) * 2;  // 2bitずつ
            //     sendData_state[byteIndex] |= (obj_state[i] & 0x03) << bitShift;
            // }

            for(int i = 0; i < 12; i++){
                con_sendData[i] =  (uint8_t)obj_state[i];
            }
                con_sendData[12] = state_mode;

            // con_sendData[0] = 53;//bit 左から

            con.sendAnyByte(con_sendData, 13);

            flag_con_tuusin = false; // flagを下す
        }

    ThisThread::sleep_for(1);
  }
}