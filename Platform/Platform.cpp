// 4輪オムニやメカナム，双輪キャスタなどのプラットフォームごとに
// 各モータの指令速度の計算からRoboClawの制御，自己位置推定を行なうクラス
// Platform.h の DRIVE_UNIT の定義でプラットフォームを選択する
// 作成日：2019年12月30日
// 作成者：上野祐樹

#include "mbed.h"
#include "Platform.h"
//#include "RoboClaw.h"
#include "RoboClaw_Mbed_Studio.h"
#include "PIDclass.h"
#include "ODrive.h"
//#include "AMT203VPeach.h"

//RoboClaw MD(&SERIAL_ROBOCLAW,1);

RoboClaw robo1(ROBO1_ADDRESS, BOUDRATE*2, TX_PIN, RX_PIN);//130 前
RoboClaw robo2(ROBO2_ADDRESS, BOUDRATE*2, TX_PIN, RX_PIN);//129 後

CAN ccan1(CANRD,CANTD, 500000);
ODrive odrvMotor0(&ccan1, CAN_ID_0);
ODrive odrvMotor1(&ccan1, CAN_ID_1);
ODrive odrvMotor2(&ccan1, CAN_ID_2);
ODrive odrvMotor3(&ccan1, CAN_ID_3);

#if DRIVE_UNIT == PLATFORM_DUALWHEEL
//AMT203V amt203(&SPI, PIN_CSB);
#endif

Platform::Platform(int dir1, int dir2, int dir3, int dir4)
{
    rotateDir[0] = (dir1 > 0) - (dir1 < 0); // 1より大きい数値だった場合に，1か-1にするための処理 左前
    rotateDir[1] = (dir2 > 0) - (dir2 < 0);  //左後ろ
    rotateDir[2] = (dir3 > 0) - (dir3 < 0);  // 右後ろ
    rotateDir[3] = (dir4 > 0) - (dir4 < 0);  //右前

    init_done = false;
}

Platform::Platform(int dir1, int dir2, int dir3)
{
    rotateDir[0] = (dir1 > 0) - (dir1 < 0); // 1より大きい数値だった場合に，1か-1にするための処理
    rotateDir[1] = (dir2 > 0) - (dir2 < 0);
    rotateDir[2] = (dir3 > 0) - (dir3 < 0);
    rotateDir[3] = 1;
    init_done = false;
}

Platform::Platform()
{
    rotateDir[0] = 1; // 指示がないときはすべて1
    rotateDir[1] = 1;
    rotateDir[2] = 1;
    rotateDir[3] = 1;

    init_done = false;
}

// 自己位置推定の初期化
void Platform::platformInit(coords initPosi)
{
#if DRIVE_UNIT == PLATFORM_DUALWHEEL
    SPI.begin(); // ここでSPIをbeginしてあげないとちゃんと動かなかった
    SPI.setClockDivider(SPI_CLOCK_DIV16); //SPI通信のクロックを1MHzに設定 beginの後に置かないと，処理が止まる
    stateamt203 = amt203.init();
#endif

    //MD.begin(115200);

    enc_posi = Posi = initPosi;

    preEncX = 0;
    preEncY = 0;
    pre_angle_rad = 0.0;//Posi.z;
    init_done = true;
}

int Platform::platformInit_Odrive()
{
    odrvMotor0.clear();
    odrvMotor0.servoOFF();
    odrvMotor0.setVel(0.0);
    odrvMotor0.servoON();

    odrvMotor1.clear();
    odrvMotor1.servoOFF();
    odrvMotor1.setVel(0.0);
    odrvMotor1.servoON();
    odrvMotor2.clear();
    odrvMotor2.servoOFF();
    odrvMotor2.setVel(0.0);
    odrvMotor2.servoON();
    odrvMotor3.clear();
    odrvMotor3.servoOFF();
    odrvMotor3.setVel(0.0);
    odrvMotor3.servoON();

    // printf("odrive init");
    // odrvMotor1.clear();
    // odrvMotor2.clear();
    // odrvMotor3.clear();
    int init_count = 0;
    int init_state = false;
    // printf("under_ODrive_init");
    // printf("state:%d\n",odrvMotor0.getInitState());
    // if(!odrvMotor0.getInitState()){
    //     printf("ODrive_init0");
    //     odrvMotor0.init(1);
    // }
    // if(!odrvMotor1.getInitState()){
    //     printf("ODrive_init1");
    //     odrvMotor1.init(1);
    // }
    // if(!odrvMotor2.getInitState()){
    //     printf("ODrive_init2");
    //     odrvMotor2.init(1);
    // }
    // if(!odrvMotor3.getInitState()){
    //     printf("ODrive_init33");
    //     odrvMotor3.init(1);
    // }
    // printf("state:%d",odrvMotor0.getInitState());
    // if(odrvMotor0.getInitState()){
    //     printf("getstate");
    //     bool send_check = true;//値が遅れているかの確認
    //     odrvMotor0.clearError();
    //     send_check &= (bool)odrvMotor0.setVel(0.0);
    //     send_check &= (bool)odrvMotor0.servoON();
    //     wait_us(500);
    //     if(send_check){
    //         init_count++;
    //         if(odrvMotor0.getState() == 0x08){
    //             init_state = true;
    //         }else if(init_count > 2){
    //             init_state = true;
    //         }
    //     }
    // }
    // if(odrvMotor0.getInitState() && odrvMotor1.getInitState() && odrvMotor2.getInitState() && odrvMotor3.getInitState()){
    //     printf("getstate");
    //     bool send_check = true;//値が遅れているかの確認
    //     odrvMotor0.clearError();
    //     odrvMotor1.clearError();
    //     odrvMotor2.clearError();
    //     odrvMotor3.clearError();
    //     send_check &= (bool)odrvMotor0.setVel(0.0);
    //     send_check &= (bool)odrvMotor1.setVel(0.0);
    //     send_check &= (bool)odrvMotor2.setVel(0.0);
    //     send_check &= (bool)odrvMotor3.setVel(0.0);
    //     send_check &= (bool)odrvMotor0.servoON();
    //     send_check &= (bool)odrvMotor1.servoON();
    //     send_check &= (bool)odrvMotor2.servoON();
    //     send_check &= (bool)odrvMotor3.servoON();
    //     wait_us(500);
    //     if(send_check){
    //         init_count++;
    //         if(odrvMotor0.getState() == 0x08 && odrvMotor1.getState() == 0x08 && odrvMotor2.getState() == 0x08 && odrvMotor3.getState() == 0x08){
    //             init_state = true;
    //         }else if(init_count > 2){
    //             init_state = true;
    //         }
    //     }
    // }
    return init_state;
}

//エンコーダ初期値
void Platform::enc_init(int enc_x, int enc_y){
    preEncX = enc_x;
    preEncY = enc_y;
}
// 自己位置推定値(Posi)を外部からセット
coords Platform::setPosi(coords tempPosi) { 
    Posi = tempPosi;
    return Posi;
}

double Platform::setAxisPosi(double posi, int axis){
    switch(axis){
    case 0:
        Posi.x = posi;
        break;
    case 1:
        Posi.y = posi;
        break;
    case 2:
        Posi.z = posi;
        break;
    }
    return posi;
}
// エンコーダのカウント値と，ジャイロセンサから取得した角度をもとに自己位置を計算する
coords Platform::getPosi(int encX, int encY, double angle_rad)
{
    if(init_done) {
        // ローカル座標系での変化量を計算(zは角度)
        coords diff;

        // エンコーダのカウント値から角度の変化量を計算する
        double angX, angY;
        angX = (double)( encX - preEncX ) * _2PI_RES4;
        angY = (double)( encY - preEncY ) * _2PI_RES4;/////////////

        double angle_diff;
        angle_diff = angle_rad - pre_angle_rad; // 角度の変化量を計算
        diff.z = angle_diff;
        diff.x = RADIUS_X * angX; //RADIUS_X はX軸エンコーダの車輪半径
        diff.y = RADIUS_Y * angY; //RADIUS_Y はY軸エンコーダの車輪半径

        // グローバル座標系での変化量に変換し，これまでのデータに加算することで自己位置推定完了
        Posi.z += diff.z;
        Posi.x += diff.x * cos( Posi.z ) - diff.y * sin( Posi.z );
        Posi.y += diff.x * sin( Posi.z ) + diff.y * cos( Posi.z );

        //エンコーダの自己位置　参照用
        enc_posi.z += diff.z;
        enc_posi.x += diff.x * cos( enc_posi.z ) - diff.y * sin( enc_posi.z );
        enc_posi.y += diff.x * sin( enc_posi.z ) + diff.y * cos( enc_posi.z );

        // 1サンプル前のデータとして今回取得したデータを格納
        preEncX = encX;
        preEncY = encY;
        pre_angle_rad = angle_rad;
    }
    return Posi;
}

coords Platform::getEncPosi() {
    return enc_posi;
}

void Platform::VelocityControl(coords refV)
{
    if(init_done) {
#if DRIVE_UNIT == PLATFORM_OMNI3WHEEL
        double refOmegaA, refOmegaB, refOmegaC;//mbed用に少し改変
        // 車輪反時計方向が正
        refOmegaA = (-refV.y - refV.z * DIST2WHEEL) / WHEEL_R * GEARRATIO;
        refOmegaB = ( refV.x*COS_PI_6 + refV.y*SIN_PI_6 - refV.z * DIST2WHEEL) / WHEEL_R * GEARRATIO;
        refOmegaC = (-refV.x*COS_PI_6 + refV.y*SIN_PI_6 - refV.z * DIST2WHEEL) / WHEEL_R * GEARRATIO;

        // RoboClawの指令値に変換
        double mdCmdA, mdCmdB, mdCmdC;
        mdCmdA = refOmegaA * _2RES_PI;
        mdCmdB = refOmegaB * _2RES_PI;
        mdCmdC = refOmegaC * _2RES_PI;

        buf[0] = (int)mdCmdA * rotateDir[0];
        buf[1] = (int)mdCmdB * rotateDir[1];
        buf[2] = (int)mdCmdC * rotateDir[2];

        // モータにcmdを送り，回す
        //MD.SpeedM1(ADR_MD1, (int)mdCmdA * rotateDir[0]);// 右前
        //MD.SpeedM2(ADR_MD1, (int)mdCmdB * rotateDir[1]);// 左前
        //MD.SpeedM2(ADR_MD2, (int)mdCmdC * rotateDir[2]);// 右後

        /*robo1.SpeedM1(100);
        robo1.SpeedM2(100);
        robo2.SpeedM2(100);*/

        robo1.SpeedM1((int)mdCmdA * rotateDir[0]);
        robo1.SpeedM2((int)mdCmdB * rotateDir[1]);
        robo2.SpeedM2((int)mdCmdC * rotateDir[2]);
#elif DRIVE_UNIT == PLATFORM_OMNI4WHEEL//オムニ
        double refOmegaA, refOmegaB, refOmegaC, refOmegaD;
        // 車輪反時計方向が正
        //refOmegaA = ( refV.x * COS_PI_4 - refV.y * SIN_PI_4 + refV.z * DIST2WHEEL) / WHEEL_R * GEARRATIO; // [rad/s]
        //refOmegaB = ( refV.x * COS_PI_4 + refV.y * SIN_PI_4 + refV.z * DIST2WHEEL) / WHEEL_R * GEARRATIO;
        //refOmegaC = (-refV.x * COS_PI_4 + refV.y * SIN_PI_4 + refV.z * DIST2WHEEL) / WHEEL_R * GEARRATIO;
        //refOmegaD = (-refV.x * COS_PI_4 - refV.y * SIN_PI_4 + refV.z * DIST2WHEEL) / WHEEL_R * GEARRATIO;
        refOmegaA = (-refV.x * COS_PI_4 + refV.y * SIN_PI_4 + refV.z * DIST2WHEEL) / WHEEL_R * GEARRATIO; // [rad/s]
        refOmegaB = (-refV.x * COS_PI_4 - refV.y * SIN_PI_4 + refV.z * DIST2WHEEL) / WHEEL_R * GEARRATIO;
        refOmegaC = ( refV.x * COS_PI_4 - refV.y * SIN_PI_4 + refV.z * DIST2WHEEL) / WHEEL_R * GEARRATIO;
        refOmegaD = ( refV.x * COS_PI_4 + refV.y * SIN_PI_4 + refV.z * DIST2WHEEL) / WHEEL_R * GEARRATIO;

        // RoboClawの指令値に変換
        double mdCmdA, mdCmdB, mdCmdC, mdCmdD;
        mdCmdA = refOmegaA * _2RES_PI;//左前
        mdCmdB = refOmegaB * _2RES_PI;//左後ろ
        mdCmdC = refOmegaC * _2RES_PI;//右後ろ
        mdCmdD = refOmegaD * _2RES_PI;//右前

        //printf("%lf\t%lf\t%lf\t%lf\n",mdCmdA,mdCmdB,mdCmdC,mdCmdD);

        // robo1.SpeedM1((int)mdCmdA * rotateDir[0]); //左前
        // robo2.SpeedM1((int)mdCmdC * rotateDir[2]); 
        // robo1.SpeedM2((int)mdCmdB * rotateDir[1]); //右前
        // robo2.SpeedM2((int)mdCmdD * rotateDir[3]); 
        // robo2.SpeedM1((int)(mdCmdA) * rotateDir[0]); // 左前 ①
        // robo1.SpeedM1((int)(mdCmdB) * rotateDir[1]); // 左後 ②
        // robo1.SpeedM2((int)(mdCmdC) * rotateDir[2]); // 右後 ③
        // robo2.SpeedM2((int)(mdCmdD) * rotateDir[3]); // 右前 ④

        robo1.SpeedM1((int)(mdCmdA) * rotateDir[0]); // 左前
        robo1.SpeedM2((int)(mdCmdD) * rotateDir[1]); // 右前
        robo2.SpeedM1((int)(mdCmdB) * rotateDir[2]); // 左後
        robo2.SpeedM2((int)(mdCmdC) * rotateDir[3]); // 右後

#elif DRIVE_UNIT == PLATFORM_DUALWHEEL
        // ターンテーブルの角度取得
        double thetaDuEnc, thetaDu;
        static double  preThetaDuEnc = thetaDuEnc;
        thetaDuEnc = amt203.getEncount();
        if( thetaDuEnc == -1 ) {
            thetaDuEnc = preThetaDuEnc; // -1はエラーなので，前の値を格納しておく
        }
        preThetaDuEnc = thetaDuEnc;
        thetaDu = (double)thetaDuEnc*2*PI / TT_RES4;    // 角度に変換

        // 車輪やターンテーブルの指令速度を計算
        double cosDu, sinDu, refOmegaR, refOmegaL, refOmegaT;
        cosDu = cos(thetaDu);
        sinDu = sin(thetaDu);
        refOmegaR = ( ( cosDu - sinDu ) * refV.x + ( sinDu + cosDu ) * refV.y ) / RADIUS_R;// right
        refOmegaL = ( ( cosDu + sinDu ) * refV.x + ( sinDu - cosDu ) * refV.y ) / RADIUS_L;// left
        refOmegaT = ( - ( 2 * sinDu / W ) * refV.x + ( 2 * cosDu / W ) * refV.y - refV.z ) * GEARRATIO;// turntable

        // RoboClawの指令値に変換
        double mdCmdR, mdCmdL, mdCmdT;
        mdCmdR = refOmegaR * _2RES_PI;
        mdCmdL = refOmegaL * _2RES_PI;
        mdCmdT = refOmegaT * _2RES_PI_T;

        // モータにcmdを送り，回す
        robo1.SpeedM1(-(int)mdCmdR);// 右車輪
        robo1.SpeedM2((int)mdCmdL);// 左車輪
        robo2.SpeedM1((int)mdCmdT);// ターンテーブル
#elif DRIVE_UNIT == PLATFORM_MECHANUM//メカナム
        double refOmegaA, refOmegaB, refOmegaC, refOmegaD;
        // 車輪の軸まわりで見て，反時計方向が正
        refOmegaA = (-refV.x + refV.y + refV.z * ( TREAD_2 + WHEELBASE_2 ) ) / WHEEL_R;// 左前
        refOmegaB = (-refV.x - refV.y + refV.z * ( TREAD_2 + WHEELBASE_2 ) ) / WHEEL_R;// 左後
        refOmegaC = (+refV.x - refV.y + refV.z * ( TREAD_2 + WHEELBASE_2 ) ) / WHEEL_R;// 右後
        refOmegaD = (+refV.x + refV.y + refV.z * ( TREAD_2 + WHEELBASE_2 ) ) / WHEEL_R;// 右前 rad/s

        // RoboClawの指令値に変換
        double mdCmdA, mdCmdB, mdCmdC, mdCmdD;
        mdCmdA = refOmegaA * _2RES_PI;
        mdCmdB = refOmegaB * _2RES_PI;
        mdCmdC = refOmegaC * _2RES_PI;
        mdCmdD = refOmegaD * _2RES_PI;

        // robo1.SpeedM1((int)mdCmdA * rotateDir[0]);// 左前
        // robo1.SpeedM2((int)mdCmdB * rotateDir[1]);// 左後
        // robo2.SpeedM1((int)mdCmdC * rotateDir[2]);// 右後
        // robo2.SpeedM2((int)mdCmdD * rotateDir[3]);// 右前
        // robo1.SpeedM1((int)(mdCmdA) * rotateDir[0]); // 左前
        // robo1.SpeedM2((int)(mdCmdD) * rotateDir[1]); // 右前
        // robo2.SpeedM1((int)(mdCmdB) * rotateDir[2]); // 左後
        // robo2.SpeedM2((int)(mdCmdC) * rotateDir[3]); // 右後

        // robo1.SpeedM1((int)(mdCmdB) * rotateDir[0]); // 左後
        // robo1.SpeedM2((int)(mdCmdC) * rotateDir[1]); // 右後
        // robo2.SpeedM1((int)(mdCmdA) * rotateDir[2]); //左前
        // robo2.SpeedM2((int)(mdCmdD) * rotateDir[3]); //右前

        // robo2.SpeedM1((int)(mdCmdA) * -rotateDir[0]); //左前
        // robo1.SpeedM1((int)(mdCmdB) * -rotateDir[1]); // 左後
        // robo1.SpeedM2((int)(mdCmdC) * rotateDir[2]); // 右後
        // robo2.SpeedM2((int)(mdCmdD) * rotateDir[3]); //右前

        robo2.SpeedM1((int)(mdCmdD) * rotateDir[0]); //左前
        robo1.SpeedM1((int)(mdCmdC) * rotateDir[1]); // 左後
        robo1.SpeedM2((int)(mdCmdB) * -rotateDir[2]); // 右後
        robo2.SpeedM2((int)(mdCmdA) * -rotateDir[3]); //右前

        refV_x_cmd = (mdCmdA + mdCmdB + mdCmdC + mdCmdD) / 4;//

        // printf("%d,%d,%d,%d",mdCmdA,mdCmdB,mdCmdC,mdCmdD);

        // float odCmdA, odCmdB, odCmdC, odCmdD;//Odriveの指令値変換（rps）
        // odCmdA = refOmegaA / (2.0 * M_PI);
        // odCmdB = refOmegaB / (2.0 * M_PI);
        // odCmdC = refOmegaC / (2.0 * M_PI);
        // odCmdD = refOmegaD / (2.0 * M_PI);

        // odrvMotor0.setVel(odCmdA * (float)rotateDir[0]);
        // odrvMotor1.setVel(odCmdB * (float)rotateDir[1]);
        // odrvMotor2.setVel(odCmdC * (float)rotateDir[2]);
        // odrvMotor3.setVel(odCmdD * (float)rotateDir[3]);//rps
#endif
    }
}

void Platform::freeWheel(void)
{
    robo1.ForwardM1(0);// 左前 //roboclaw
    robo1.ForwardM2(0);// 左後
    robo2.ForwardM1(0);// 右後
    robo2.ForwardM2(0);// 右前
    // odrvMotor0.servoOFF(); //odrive
    // odrvMotor1.servoOFF();
    // odrvMotor2.servoOFF();
    // odrvMotor3.servoOFF();
}


//以下Odrive段越え用なので注意----------------------------------
//内側に回すやつ
void Platform::insideWheel(double vel){
    float input_rps = vel / (WHEEL_R * 2 * M_PI);
    odrvMotor0.setVel(input_rps * (float)rotateDir[0]);
    odrvMotor1.setVel(-input_rps * (float)rotateDir[1]);
    odrvMotor2.setVel(input_rps * (float)rotateDir[2]);
    odrvMotor3.setVel(-input_rps * (float)rotateDir[3]);//rps
}
//内側に回すやつPID
void Platform::insideWheel_balance(double vel, double PIDcmd){
    double refOmegaA, refOmegaB, refOmegaC, refOmegaD;
        // 車輪の軸まわりで見て，反時計方向が正
        refOmegaA = (-vel + PIDcmd * ( TREAD_2 + WHEELBASE_2 ) ) / WHEEL_R;// 左前
        refOmegaB = (-vel + PIDcmd * ( TREAD_2 + WHEELBASE_2 ) ) / WHEEL_R;// 左後
        refOmegaC = (+vel + PIDcmd * ( TREAD_2 + WHEELBASE_2 ) ) / WHEEL_R;// 右後
        refOmegaD = (+vel + PIDcmd * ( TREAD_2 + WHEELBASE_2 ) ) / WHEEL_R;// 右前 rad/s
}

void Platform::insideWheel_balance2(double vel0, double vel1, double vel2, double vel3, double PIDcmd1){
    double refOmegaA, refOmegaB, refOmegaC, refOmegaD;
        // 車輪の軸まわりで見て，反時計方向が正
        refOmegaA = (-vel0 + PIDcmd1 * ( TREAD_2 + WHEELBASE_2 ) ) / WHEEL_R;// 左前
        refOmegaB = (-vel1 + PIDcmd1 * ( TREAD_2 + WHEELBASE_2 ) ) / WHEEL_R;// 左後
        refOmegaC = (+vel2 + PIDcmd1 * ( TREAD_2 + WHEELBASE_2 ) ) / WHEEL_R;// 右後
        refOmegaD = (+vel3 + PIDcmd1 * ( TREAD_2 + WHEELBASE_2 ) ) / WHEEL_R;// 右前 rad/s
        float odCmdA, odCmdB, odCmdC, odCmdD;//Odriveの指令値変換（rps）
        odCmdA = refOmegaA / (2.0 * M_PI);
        odCmdB = refOmegaB / (2.0 * M_PI);
        odCmdC = refOmegaC / (2.0 * M_PI);
        odCmdD = refOmegaD / (2.0 * M_PI);
        // printf("%lf,%lf,%lf,%lf ",odCmdA,odCmdB,odCmdC,odCmdD);//これ重いだめ
        // printf("%lf",PIDcmd1);
        odrvMotor0.setVel(odCmdA * (float)rotateDir[0]);
        odrvMotor1.setVel(odCmdB * (float)rotateDir[1]);
        odrvMotor2.setVel(odCmdC * (float)rotateDir[2]);
        odrvMotor3.setVel(odCmdD * (float)rotateDir[3]);//rps
}

//前輪だけ前に動かす
void Platform::frontWheel_forward(double vel){
    float input_rps = vel / (WHEEL_R * 2 * M_PI);
    odrvMotor0.setVel(-input_rps * (float)rotateDir[0]);
    odrvMotor3.setVel(input_rps * (float)rotateDir[3]);//rps
}
//後輪だけ前に動かす
void Platform::backWheel_forward(double vel){
    float input_rps = vel / (WHEEL_R * 2 * M_PI);
    odrvMotor1.setVel(-input_rps * (float)rotateDir[1]);
    odrvMotor2.setVel(input_rps * (float)rotateDir[2]);
}
//前輪だけ後ろに動かす
void Platform::frontWheel_backward(double vel){
    float input_rps = vel / (WHEEL_R * 2 * M_PI);
    odrvMotor0.setVel(input_rps * (float)rotateDir[0]);
    odrvMotor3.setVel(-input_rps * (float)rotateDir[3]);//rps
}
//後輪だけ後ろに動かす
void Platform::backWheel_backward(double vel){
    float input_rps = vel / (WHEEL_R * 2 * M_PI);
    odrvMotor1.setVel(input_rps * (float)rotateDir[1]);
    odrvMotor2.setVel(-input_rps * (float)rotateDir[2]);
}

// void Platform::get_Current(){
    
// }

