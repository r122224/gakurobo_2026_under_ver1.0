#ifndef DEFINE_h
#define DEFINE_h

#ifndef M_PI
#define M_PI           3.14159265358979323846
#endif

//学ロボ2026
#define RED 0
#define BLUE 1
#define COLLECT_NUM 4

#define front_lift_init 419.48//397.751118//0//419.48
#define back_lift_init  204.91//196.002715//0//204.91
// -397.751118 -196.002715

#define HIGH_COLLECT_POSI   222.48
#define LOW_COLLECT_POSI     40//22.48
#define STORAGE_POSI         92.48

#define STEP_UP_FRONT_HIGH     240
#define STEP_UP_FRONT_LOW      -20
#define STEP_UP_BACK_HIGH       40
#define STEP_UP_BACK_LOW      -220

#define STEP_DOWN_FRONT_HIGH    30
#define STEP_DOWN_FRONT_LOW   -180
#define STEP_DOWN_BACK_HIGH    230
#define STEP_DOWN_BACK_LOW     -30

#define FRONT_INIT          0//419.48
#define BACK_INIT           0//204.91

#define EDGE_MOVE 0.308

#define DFRONT 0
#define DRIGHT 1
#define DLEFT  2
#define DBACK  3

struct coords {
    double x;
    double y;
    double z;
};

struct forest_posi {
    double x;
    double y;
    int    hight;
};

struct forest_route {
    int num; //経路の順番
    int mode; //1:回収 2:通過 3:回収通過
};

struct dircost {
    int x;
    int y;
    int flag;
};

struct field{
    int x;
    int y;
};

struct field3 {
    int x;
    int y;
    int z;
};

struct side4 {
    double front;
    double left;
    double right;
    double back;
};

enum Direction {//定数の種類，数を明確にしたいときenumを使う．
   UP_dir=0, RIGHT_dir=1, DOWN_dir=2, LEFT_dir=3 
};

struct Node{
    int x;//ノードの位置
    int y;
    int z;
    double f;//f = g + h; これが小さいほど先に探索される．スタートゴール判定
    double g;//スタートから現在ノードまでの実際のコスト
    double h;//ヒューリスティック（推定コスト）
    int px;//親ノード　経路復元のための座標
    int py;
    int pz;
    int dir;//ロボットの方向
    int turn_count;
};

//A*ノード
struct Node1{
    int x;//ノードの位置
    int y;
    double f;//f = g + h; これが小さいほど先に探索される．スタートゴール判定
    double g;//スタートから現在ノードまでの実際のコスト
    double h;//ヒューリスティック（推定コスト）
    int dir;//来た方向
    int px;//親ノード　経路復元のための座標
    int py;
};

struct Node3{
    int x;//ノードの位置
    int y;
    int z;
    double f;//f = g + h; これが小さいほど先に探索される．スタートゴール判定
    double g;//スタートから現在ノードまでの実際のコスト
    double h;//ヒューリスティック（推定コスト）
    int px;//親ノード　経路復元のための座標
    int py;
    int pz;
};

struct State {
    double x;     // m 自己位置gPosi.x
    double y;     // m gPosi.y
    double yaw;   // rad gPOsi.z
    double v;     // m/s refV.x refV.y
    double omega; // rad/s refV.z
};

struct Obstacle {
    float x, y;
};

struct Obstacle3 {
    float x, y, z;
};

struct controllPoint{
    double x;
    double y;
};

struct pidGain {
    float Kp;
    float Ki;
    float Kd;
};


//front 420.18
//back  204.91


// //学ロボ2025
// #define OFFENSE 0
// #define DEFENSE 1
#define GOAL_3CIRCLE_R 3.125
#define GOAL_ENE_Y 4.000 //リング中心
#define GOAL_ENE_X 14.074
#define GOAL_ENE_LEFT_Y 4.225 //4.000//リング左と板
#define GOAL_ENE_LEFT_X 14.450//14.074
#define GOAL_ENE_RIGHT_Y 3.775 //4.000//リング右と板
#define GOAL_ENE_RIGHT_X 14.450//14.074
#define GOAL_ANGLE_ENE_Y 4.000 //ロボットの向く位置
#define GOAL_ANGLE_ENE_X 14.074//14.262//14.450//
#define GOAL_MY_Y 4.000 //味方リング中心
#define GOAL_MY_X 0.926


#define HITTER false
#define SEEKER true


//mbed専用
#define TX_PIN P8_14
#define RX_PIN P8_15
#define ROBO1_ADDRESS 129
#define ROBO2_ADDRESS 130
#define BOUDRATE 115200
//

#define PIN_XBEERESET 66

// スイッチやLEDのピン設定


// 制御周期 ※軌道追従の計算に使用．
#define INT_TIME            ( 0.020 )

#define PRINT_TIME            ( 0.100  )
    //↑軌道追従の計算に使用
    //↑自然数を入れる


// >>> ManualControlで使用 >>>>>>>>>>>>>>>>>>
#define JOY_DEADBAND    ( 2 )
#define JOY_MAXVEL      ( 1.0 )
#define JOY_MAXANGVEL   ( 2.5 )
#define MANUAL_LOWPASS_T  ( 0.01 )
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>> PathTrackingで使用 >>>>>>>>>>>>>>>>>>
#define POSI_X_KP    ( 0.095 )
#define POSI_X_KI    ( 0.15 )
#define POSI_X_KD    ( 0.2 )

#define POSI_Y_KP    ( 0.080 )//0.01
#define POSI_Y_KI    ( 0.002 )//0.05
#define POSI_Y_KD    ( 3.0 )//0.05

#define POSI_Z_KP    ( 4.0 )
#define POSI_Z_KI    ( 0.0 )
#define POSI_Z_KD    ( 0.0 )

#define YOKOZURE_KP    ( 1.90 )//1.9
#define YOKOZURE_KI    ( 0.35 )//0.35
#define YOKOZURE_KD    ( 0.8 )

#define KAKUDO_KP    ( 4.0 )//旋回時のPID?
#define KAKUDO_KI    ( 0.0 )
#define KAKUDO_KD    ( 0.0 )

#define FILT_SOKUDO_OMEGA ( 14.0 )//大きくなるほど動作が早くなる．（振動，収束が早くなる．）
#define FILT_SOKUDO_DZETA ( 1.0 )//1より小さい時，大きくなるほど収束が早い．1より大きい時，大きくなるほど収束が遅い．

#define FILT_KAKUDO_OMEGA ( 5.0 )
#define FILT_KAKUDO_DZETA ( 1.0 )

#define CURVE_DEC_UNIT_LENGTH ( 1.5 )    //m
#define CURVE_DEC_TRIGGER_DEGREE ( 3.0 )   //deg ↓ でradに変換している
#define CURVE_DEC_TRIGGER_RADIAN CURVE_DEC_TRIGGER_DEGREE * M_PI / 180.0    // ↑の単位を deg → rad
#define CURVE_DEC_LIMIT_CENTRIPETAL_ACC (0.5) // m/s^2

#define DEAD_TIME (0.06) //sec 無駄時間
#define DEAD_BAND_ (0.035)  //sec/(m/s^2) 加速度に対する無駄時間

#define LIMIT_ACC (3.0)

// 旋回
#define TURN_ACC (18.00) //rad/s2
#define TURN_MAX_VEL 5.5 //rad/s
#define TURN_DEAD_BAND 0.1 //rad
#define TURN_DEAD_TIME 0.1
#define TURN_DECC_THRESHOLD 0.01 //rad/s

// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>> Platformで使用 >>>>>>>>>>>>>>>>>>>>>
// Platformの種類を設定
#define PLATFORM_MECHANUM      ( 0 )    //4輪メカナム
#define PLATFORM_OMNI4WHEEL    ( 1 )    //4輪オムニ
#define PLATFORM_OMNI3WHEEL    ( 2 )    //3輪オムニ
#define PLATFORM_DUALWHEEL     ( 3 )    //デュアルキャスタ (差動2輪 + 方位角機構)

#define DRIVE_UNIT  ( PLATFORM_MECHANUM )//ここでタイヤの種類を変える

#if DRIVE_UNIT == PLATFORM_DUALWHEEL// 双輪キャスター関連
#define PIN_CSB     ( 10 )    // turntableのPIN(CSB)
#define RADIUS_R    ( 0.04 )    // wheel radius
#define RADIUS_L    ( 0.04 )    // wheel radius
#define W           ( 0.265 )    // tread
#define GEARRATIO   ( 5.5 )
#define TT_RES4     ( 4096 )    // turntableの分解能
#define _2RES_PI    ( 2 * 2048 / M_PI ) // 駆動輪の角速度[rad/s]からRoboClawの指令値[pulses/s]に変換するための定数(2048はエンコーダ分解能) 4逓倍しているが，分母は元は2*piで，通分されている
#define _2RES_PI_T  ( 2 * 500 / M_PI ) //  ターンテーブルの角速度[rad/s]からRoboClawの指令値[pulses/s]に変換するための定数(500はエンコーダ分解能) 4逓倍しているが，分母は元は2*piで，通分されている

#elif DRIVE_UNIT == PLATFORM_MECHANUM// メカナム関連
#define WHEEL_R     ( 0.050875 )    //車輪半径[m]
#define TREAD_2     ( 0.30/2 ) //車両中心から車輪接地点までのY軸方向距離(トレッド/2)
#define WHEELBASE_2 ( 0.20/2 ) //車両中心から車輪接地点までのX軸方向距離(ホイールベース/2)
#define _2RES_PI  ( 2.0 * 500 / M_PI ) //  駆動輪の角速度[rad/s]からRoboClawの指令値[pulses/s]に変換するための定数(2048はエンコーダ分解能) 4逓倍しているが，分母は元は2*piで，通分されている
//270.450.50//101.6
#elif DRIVE_UNIT == PLATFORM_OMNI3WHEEL//3輪オムニ関連
#define WHEEL_R     ( 0.019 )
#define DIST2WHEEL  ( 0.300 )
#define GEARRATIO   ( 10.0 )
#define COS_PI_6    ( 0.86602540378 )
#define SIN_PI_6    ( 0.5 )
#define _2RES_PI    ( 2.0 * 12 / M_PI ) // 駆動輪の角速度[rad/s]からRoboClawの指令値[pulses/s]に変換するための定数(2048はエンコーダ分解能) 4逓倍しているが，分母は元は2*piで，通分されている

        refOmegaT = ( - ( 2 * sinDu / W ) * refV.x + ( 2 * cosDu / W ) * refV.y - refV.z ) * GEARRATIO;// turntable
#elif DRIVE_UNIT == PLATFORM_OMNI4WHEEL
//4輪オムニ関連
#define WHEEL_R (0.0508)
#define DIST2WHEEL  (0.6298/2)           //中心からホイールまでの距離0.244361 /// 230~240
#define GEARRATIO   ( 1.0 )           // 車輪からエンコーダの計測軸までのギヤ比(1:1 なら 1)  480:17
#define COS_PI_4    ( 0.70711 )         // cos(pi/4)
#define SIN_PI_4    ( 0.70711 )         // sin(pi/4)
#define _2RES_PI (2.0 * (500) / M_PI ) //  [rad/s]からRoboClawの指令値[pulses/s]に変換するための定数(1024はエンコーダ分解能) 4逓倍しているが，分母は元は2*piで，通分されている
#endif

// ODrive関連>>>>>>>>>>>>>>>>
//CAN通信関連
#define CANRD P11_12 // P6_4
#define CANTD P11_13 // P6_5
#define CAN_ID_0 0
#define CAN_ID_1 1
#define CAN_ID_2 2
#define CAN_ID_3 3

// 自己位置推定用エンコーダ関連
#define _2PI_RES4   ( 2 * 3.141592 / (500*4))//res*4 = 500*4 = 2000  中心からの距離 0.256m
#define RADIUS_X    ( 0.024 ) // X軸計測輪の半径[m]
#define RADIUS_Y    ( 0.024 ) // Y軸計測輪の半径[m] 
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>> Controllerまわりで使用 >>>>>>>>>>>>>>>>>>>>>
#define CON_ADACHI    ( 0 )
#define CON_ELECOM    ( 1 )
#define CON_DS4       ( 2 )
#define CON_TYPE  ( CON_DS4 )
#if CON_TYPE == CON_ADACHI
    #define MASK_BUTTON_UP    0x01
    #define MASK_BUTTON_RIGHT 0x02
    #define MASK_BUTTON_DOWN  0x04
    #define MASK_BUTTON_LEFT  0x08
    #define MASK_BUTTON_R1    0x10
    #define MASK_BUTTON_R2    0x20
    #define MASK_BUTTON_L1    0x40
    #define MASK_BUTTON_L2    0x80
    #define BUTTON_UP    1
    #define BUTTON_RIGHT 2
    #define BUTTON_DOWN  3
    #define BUTTON_LEFT  4
    #define BUTTON_R1    5
    #define BUTTON_R2    6
    #define BUTTON_L1    7
    #define BUTTON_L2    8
#elif CON_TYPE == CON_ELECOM || CON_TYPE == CON_DS4
    #define MASK_BUTTON_X  0x0001
    #define MASK_BUTTON_Y  0x0002
    #define MASK_BUTTON_A  0x0004
    #define MASK_BUTTON_B  0x0008
    #define MASK_BUTTON_SHIKAKU 0x0001
    #define MASK_BUTTON_SANKAKU 0x0002
    #define MASK_BUTTON_BATU    0x0004
    #define MASK_BUTTON_MARU    0x0008
    #define MASK_BUTTON_L1     0x0010
    #define MASK_BUTTON_R1     0x0020
    #define MASK_BUTTON_L2     0x0040
    #define MASK_BUTTON_R2     0x0080
    #define MASK_BUTTON_PAD     0x0100 // PS4のときはPADボタン
    #define MASK_BUTTON_SLIDO     0x0100//PADボタンに割り込ませてある
    #define MASK_BUTTON_PS      0x0200 // PS4のときはPS
    #define MASK_BUTTON_JOY_L   0x20000 //131072//0x0100
    #define MASK_BUTTON_JOY_R   0x10000//65536//0x0200
    #define MASK_BUTTON_BACK    0x0400
    #define MASK_BUTTON_START   0x0800
    #define MASK_BUTTON_SHARE   0x0400
    #define MASK_BUTTON_OPTION  0x0800
    #define MASK_BUTTON_UP     0x1000
    #define MASK_BUTTON_RIGHT  0x2000
    #define MASK_BUTTON_DOWN   0x4000
    #define MASK_BUTTON_LEFT   0x8000
    #define MASK_BUTTON_0 0x1 //1//拡張基盤
    #define MASK_BUTTON_1 0x2 //8
    #define MASK_BUTTON_2 0x4 //32
    #define MASK_BUTTON_3 0x8 //64
    #define MASK_BUTTON_4 0x10 //512
    #define MASK_BUTTON_5 0x20 //128
    #define MASK_BUTTON_6 0x40 //2
    #define MASK_BUTTON_7 0x80 //16
    #define MASK_BUTTON_8 0x100 //4
    #define MASK_BUTTON_9 0x200 
    #define MASK_BUTTON_10 0x400
    #define MASK_BUTTON_11 0x800 //1024
    #define MASK_BUTTON_R3 0x10000//0x10000000
    #define MASK_BUTTON_L3 0x20000//0x20000000
    #define MASK_BUTTON_M5_A 0x40000000
    #define MASK_BUTTON_M5_B 0x80000000
    #define MASK_BUTTON_M5_C 0x100000000
    #define BUTTON_UP    13
    #define BUTTON_RIGHT 14
    #define BUTTON_DOWN  15
    #define BUTTON_LEFT  16
    #define BUTTON_R1    6
    #define BUTTON_R2    8
    #define BUTTON_L1    5
    #define BUTTON_L2    7

    #define SHIKAKU  1
    #define SANKAKU  2
    #define BATU     3
    #define MARU     4
    #define L1       5
    #define R1       6
    #define L2       7
    #define R2       8
    #define JOY_L    9
    #define JOY_R   10
    #define BACK    11
    #define START   12
    #define PAD      9
    #define PS      10
    #define SHARE   11
    #define OPTION  12
    #define UP      13
    #define RIGHT   14
    #define DOWN    15
    #define LEFT    16

    // #define SW0     11          //左上から0~5,6~12
    // #define SW1     8          //
    // #define SW2     6          //　arduino
    // #define SW3     5          //　 11  8  6  5  2  4 
    // #define SW4     2          //　 10  7  9        1
    // #define SW5     4          //　           3
    // #define SW6     10         //   1  8  32  64  512  128
    // #define SW7     7          //   2  16  4           1024
    // #define SW8     9          //             256
    // #define SW9     1          //  mbed

    // #define SW0     1          //左上から0~5,6~12
    // #define SW1     8          //
    // #define SW2     32          //　arduino
    // #define SW3     64          //　 11  8  6  5  2  4 
    // #define SW4     512          //　 10  7  9        1
    // #define SW5     128          //　           3
    // #define SW6     2           //   1  8  32  64  512  128
    // #define SW7     16          //   2  16  4           1024
    // #define SW8     4          //             256
    // #define SW9     1024          //  mbed

    // #define SW0     1          //左上から0~5,6~12
    // #define SW1     4          //
    // #define SW2     6          //　arduino
    // #define SW3     7          //　 11  8  6  5  2  4 
    // #define SW4     10          //　 10  7  9        1
    // #define SW5     8          //　           3
    // #define SW6     2           //   1  8  32  64  512  128
    // #define SW7     5          //   2  16  4           1024
    // #define SW8     3          //             256
    // #define SW9     11          //  mbed

    //#define SW0     1          //左上から0~5,6~12
    #define SW1     1          //
    #define SW2     2          //　arduino
    #define SW3     3          //　 11  8  6  5  2  4 
    #define SW4     4          //　 10  7  9        1
    #define SW5     5          //　           3
    #define SW6     6           //   1  8  32  64  512  128
    #define SW7     7          //   2  16  4           1024
    #define SW8     8          //             256
    #define SW9     9          //  mbed
    #define SW10    10
    #define SW11    11

#endif
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#endif