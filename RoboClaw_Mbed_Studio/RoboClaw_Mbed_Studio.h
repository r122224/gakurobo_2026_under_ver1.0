/*

RoboClaw Mbed用ライブラリ
作成日：2023/02/16
作成者：赤崎恵士

RoboClawのよく使う機能を抜粋してライブラリ化した
→もともとは別の人がオープンにしていたライブラリを利用していたが，Mbed Studioの方で使えない処理が含まれていたので改良した

引用元：https://os.mbed.com/teams/ARES/code/RoboClaw/
RoboClawマニュアル：https://downloads.basicmicro.com/docs/roboclaw_user_manual.pdf

基本的には引用元の関数名と引数をそのまま引き継いでいるので移行も簡単なはず・・・

使える機能
・PWM制御
・速度制御
・速度加速度制御
・位置速度加速度制御
・エンコーダリセット
・エンコーダカウント値セット
・（エンコーダ読み取り）→読み取りに1ms以上かかる
*/



#ifndef RoboClaw_H
#define RoboClaw_H

#include "mbed.h"
#include "RoboClawRegisters.h"
#include <cstdint>

class RoboClaw{

    public:

    //コンストラクタ
    RoboClaw(uint8_t adr, int baudrate, PinName tx, PinName rx);

    void ForwardM1(uint8_t speed);//PWM制御
    void BackwardM1(uint8_t speed);
    void ForwardM2(uint8_t speed);
    void BackwardM2(uint8_t speed);
   
    //void Forward(int speed);
    //void Backward(int speed);
    
    //void ReadFirm();
    
    int32_t ReadEncM1();//エンコーダ読み取り
    int32_t ReadEncM2();

    void SetEncM1(int32_t count);//カウント値変更
    void SetEncM2(int32_t count);

    void ResetEnc();//エンコーダリセット

    void SpeedM1(int32_t speed);//速度制御
    void SpeedM2(int32_t speed);

    void SpeedAccelM1(int32_t accel, int32_t speed);//速度加速度制御
    void SpeedAccelM2(int32_t accel, int32_t speed);

    void SpeedAccelDeccelPositionM1(uint32_t accel, int32_t speed, uint32_t deccel, int32_t position, uint8_t flag);//位置速度加速度制御
    void SpeedAccelDeccelPositionM2(uint32_t accel, int32_t speed, uint32_t deccel, int32_t position, uint8_t flag);//bufferをtrueにしておくと収束していなくとも次の指令値に飛ぶ

private:
    Serial pSerial;
    uint16_t crc;
    uint8_t address;
    void clear_buf();
    void crc_clear();
    void crc_update(uint8_t data);
    uint16_t crc_get();
    void write_enc(uint8_t command);
    void write_0(uint8_t command);
    void write_1(uint8_t command, uint8_t data);
    void write_4(uint8_t command, uint8_t data[]);
    void write_8(uint8_t command, uint8_t data[]);
    void write_16(uint8_t command, uint8_t data[], bool buffer);

};

#endif