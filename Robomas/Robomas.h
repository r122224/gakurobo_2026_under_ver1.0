#ifndef ROBOMAS_H
#define ROBOMAS_H

#include "mbed.h"

// ロボマスの定義
#define INTAKE_ROBOMAS 500
#define REJECTION_ROBOMAS 500
#define STORE_ROBOMAS 500
#define SWING_ROBOMAS 500
#define STOP_ROBOMAS 0

// ロボマスモータのモード
#define CURRENT_CONTROL 0 // (mA):(-)10A(10000mA)まで
#define VEL_CONTROL 1 // (rpm):500rpmまで

// モーターの識別子
#define MOTER_FRONT 0
#define MOTER_LOADING 1
#define MOTER_REMOVE_RIGHT 2
#define MOTER_REMOVE_LEFT 3

class Robomas {
public:
    Robomas(PinName tx, PinName rx, int baudrate);
    bool sendCmd();
    bool receiveData();
    void cmd(int id,int rocmd);
    int getRawEnc();
    int getRpm();
    int getTorqueCurrent();
private:
    Serial robomas;
    int number;
    uint16_t rawEnc;       // エンコーダカウント（絶対値）
    int16_t rpm;           // RPM
    int16_t torqueCurrent; // トルク電流
    short robomas_cmd[5]; // short型:-32768~32767(2バイト)
    int robomas_mode[5];
    bool robomas_flag[5];
};

#endif // ROBOMAS_H