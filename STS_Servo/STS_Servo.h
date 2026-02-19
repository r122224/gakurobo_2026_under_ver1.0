#ifndef STS_SERVO_H
#define STS_SERVO_H

//STS系シリアルサーボを動作させるためのクラス

#include "mbed.h"

class STS {

public:
    STS(Serial*, unsigned int baudrate);//uartのアドレス   ボードレート
    void setMode(unsigned int id, unsigned int mode);//モード変更
    void cmd(unsigned int id, unsigned int posData, int timeData, int speedData);//指令コマンド
    unsigned int read(unsigned int id);//現在地読み取り(安定しない)
    //void speed(unsigned int id, unsigned int posData, unsigned int timeData, unsigned int speedData);

private:
    Serial* pSerial;//pointer serial
    //DigitalOut _csPin;//celect pin
    void cmdSend8(unsigned int, unsigned int, unsigned int, unsigned int);
    //void cmdSend4(unsigned int);
    //void cmdSend9(unsigned int, unsigned int, unsigned int, unsigned int);

};

#endif