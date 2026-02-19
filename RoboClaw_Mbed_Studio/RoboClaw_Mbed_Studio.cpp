#include "RoboClaw_Mbed_Studio.h"
#include <cstdint>

#define MAXTRY 1
//#define SetDWORDval(arg) (uint8_t)(arg>>24),(uint8_t)(arg>>16),(uint8_t)(arg>>8),(uint8_t)arg
//#define SetWORDval(arg) (uint8_t)(arg>>8),(uint8_t)arg

RoboClaw::RoboClaw(uint8_t adr, int baudrate, PinName tx, PinName rx):pSerial(tx,rx,baudrate){
    address = adr;
    crc = 0;
}

void RoboClaw::clear_buf(){
    while(pSerial.readable()){
        pSerial.getc();
    }
}

void RoboClaw::crc_clear(){//crcの値をクリアする（crc=チェックサムの一種）
    crc = 0;
}

void RoboClaw::crc_update (uint8_t data){//チェックサム
    int i;
    crc = crc ^ ((uint16_t)data << 8);
    for (i=0; i<8; i++) {
        if (crc & 0x8000)
            crc = (crc << 1) ^ 0x1021;
        else
            crc <<= 1;
    }
}

uint16_t RoboClaw::crc_get(){//crc参照
    return crc;
}


void RoboClaw::write_16(uint8_t command, uint8_t data[], bool buffer){
    //uint8_t retry = MAXTRY;
    //do {
        crc_clear();//crcの初期化
        
        uint8_t sendData[21] = {};
        
        sendData[0] = address;
        sendData[1] = command;

        for(int i = 2; i < 18; i++){
            sendData[i] = data[i - 2];
        }

        sendData[18] = (uint8_t)buffer;

        for(int i = 0; i < 19; i++){
            crc_update(sendData[i]);
        }

        sendData[19] = (uint8_t)(crc_get() >> 8) & 0xff;//(H)
        sendData[20] = (uint8_t)crc_get() & 0xff;//(L)
        
        
        for(int i = 0; i < 21; i++){//データ送信
            pSerial.putc(sendData[i]);
        }
    //} while(pSerial.getc() != 0xFF);
}

void RoboClaw::write_8(uint8_t command, uint8_t data[]){
    //uint8_t retry = MAXTRY;
    //do {
        crc_clear();//crcの初期化
        
        uint8_t sendData[12] = {};
        
        sendData[0] = address;
        sendData[1] = command;

        for(int i = 2; i < 10; i++){
            sendData[i] = data[i - 2];
        }

        for(int i = 0; i < 10; i++){
            crc_update(sendData[i]);
        }

        sendData[10] = (uint8_t)(crc_get() >> 8) & 0xff;//(H)
        sendData[11] = (uint8_t)crc_get() & 0xff;//(L)
        
        
        for(int i = 0; i < 12; i++){//データ送信
            pSerial.putc(sendData[i]);
        }
    //} while(pSerial.getc() != 0xFF);
}

void RoboClaw::write_4(uint8_t command, uint8_t data[]){
    //uint8_t retry = MAXTRY;
    //do {
        crc_clear();//crcの初期化
        
        uint8_t sendData[8] = {};
        
        sendData[0] = address;
        sendData[1] = command;

        for(int i = 2; i < 6; i++){
            sendData[i] = data[i - 2];
        }

        for(int i = 0; i < 6; i++){
            crc_update(sendData[i]);
        }

        sendData[6] = (uint8_t)(crc_get() >> 8);//(H)
        sendData[7] = (uint8_t)crc_get();//(L)
        
        
        for(int i = 0; i < 8; i++){//データ送信
            pSerial.putc(sendData[i]);
        }
    //} while(pSerial.getc() != 0xFF);
}

void RoboClaw::write_1(uint8_t command, uint8_t data){
    //uint8_t retry = MAXTRY;
    //do {
        crc_clear();//crcの初期化
        
        uint8_t sendData[5] = {};
        
        sendData[0] = address;
        sendData[1] = command;

        sendData[2] = data;

        for(int i = 0; i < 3; i++){
            crc_update(sendData[i]);
        }

        sendData[3] = (uint8_t)(crc_get() >> 8) & 0xff;//(H)
        sendData[4] = (uint8_t)crc_get() & 0xff;//(L)
        
        
        for(int i = 0; i < 5; i++){//データ送信
            pSerial.putc(sendData[i]);
        }
    //} while(pSerial.getc() != 0xFF);
}


void RoboClaw::write_0(uint8_t command){
    crc_clear();//crcの初期化
        
        uint8_t sendData[4] = {};
        
        sendData[0] = address;
        sendData[1] = command;

        for(int i = 0; i < 2; i++){
            crc_update(sendData[i]);
        }

        sendData[2] = (uint8_t)(crc_get() >> 8) & 0xff;//(H)
        sendData[3] = (uint8_t)crc_get() & 0xff;//(L)
        
        
        for(int i = 0; i < 4; i++){//データ送信
            pSerial.putc(sendData[i]);
        }
}


void RoboClaw::write_enc(uint8_t command){//エンコーダ読み取り専用
    //uint8_t retry = MAXTRY;
    //do {
        crc_clear();//crcの初期化
        
        uint8_t sendData[2] = {};
        
        sendData[0] = address;
        sendData[1] = command;
        
        for(int i = 0; i < 2; i++){//データ送信
            pSerial.putc(sendData[i]);
        }
    //} while(pSerial.getc() != 0xFF);
}


void RoboClaw::ForwardM1(uint8_t speed){
    clear_buf();
    write_1(M1FORWARD, speed);
    clear_buf();
}

void RoboClaw::BackwardM1(uint8_t speed){
    clear_buf();
    write_1(M1BACKWARD, speed);
    clear_buf();
}

void RoboClaw::ForwardM2(uint8_t speed){
    clear_buf();
    write_1(M2FORWARD, speed);
    clear_buf();
}

void RoboClaw::BackwardM2(uint8_t speed){
    clear_buf();
    write_1(M2BACKWARD, speed);
    clear_buf();
}

int32_t RoboClaw::ReadEncM1(){
    clear_buf();
    crc_clear();
    static int preEncM1 = 0;
    int32_t encM1;
    unsigned int readNum = 0;
    uint16_t readByte[100] = {};
    bool readCheck = true;
    write_enc(GETM1ENC);
    wait_us(1000);
    //wait_ms(1);
    while(pSerial.readable() & readCheck){
    //printf("pass\n");
        readByte[readNum] = pSerial.getc();
        readNum++;
        if(readNum > 7){
            readCheck = false;
        }
    }

    if(readNum != 7){
        readCheck = false;
    }
    else {
        for(int i = 0; i < 5; i++ ){
            crc_update(readByte[i]);
        }
    }

    if(crc_get() == (((uint16_t)(readByte[5] & 0xff) << 8) | (uint16_t)(readByte[6] & 0xff))){
        for(int i = 0; i < 4; i++){
            encM1 |= (int)(readByte[i] & 0xff) << (8 * (3 - i)); 
        }

        readCheck = true;
    }

    if(readCheck) preEncM1 = encM1;
    else encM1 = preEncM1;

    clear_buf();

    /*for(int i = 0; i < 4; i++){
            encM1 |= (int)(readByte[i] & 0xff) << (8 * (3 - i)); 
    }*/

    encM1 = (int)(readByte[0] & 0xff) << (8 * 3);
    encM1 |= (int)(readByte[1] & 0xff) << (8 * 2);
    encM1 |= (int)(readByte[2] & 0xff) << (8 * 1);
    encM1 |= (int)(readByte[3] & 0xff);

    return encM1;
}
int32_t RoboClaw::ReadEncM2(){
    clear_buf();
    crc_clear();
    static int preEncM2 = 0;
    int32_t encM2;
    unsigned int readNum = 0;
    uint16_t readByte[100] = {};
    bool readCheck = true;
    write_enc(GETM2ENC);
    wait_us(1000);
    //wait_ms(1);
    while(pSerial.readable() & readCheck){
        readByte[readNum] = pSerial.getc();
        readNum++;
        if(readNum > 7){
            readCheck = false;
        }
    }

    if(readNum != 7){
        readCheck = false;
    }
    else {
        for(int i = 0; i < 5; i++ ){
            crc_update(readByte[i]);
        }
    }

    if(crc_get() == (((uint16_t)(readByte[5] & 0xff) << 8) | (uint16_t)(readByte[6] & 0xff))){
        for(int i = 0; i < 4; i++){
            encM2 |= (int)(readByte[i] & 0xff) << (8 * (3 - i)); 
        }

        readCheck = true;
    }

    if(readCheck) preEncM2 = encM2;
    else encM2 = preEncM2;
    clear_buf();

    for(int i = 0; i < 4; i++){
            encM2 |= (int)(readByte[i] & 0xff) << (8 * (3 - i)); 
    }

    return encM2;
}


void RoboClaw::SetEncM1(int32_t count){
    clear_buf();
    uint8_t data[4] = {
        (uint8_t)((count >> 24) & 0xff),
        (uint8_t)((count >> 16) & 0xff),
        (uint8_t)((count >> 8) & 0xff),
        (uint8_t)(count & 0xff)
    };
    write_4(SETM1ENCCOUNT, data);
    clear_buf();
}

void RoboClaw::SetEncM2(int32_t count){
    clear_buf();
    uint8_t data[4] = {
        (uint8_t)((count >> 24) & 0xff),
        (uint8_t)((count >> 16) & 0xff),
        (uint8_t)((count >> 8) & 0xff),
        (uint8_t)(count & 0xff)
    };
    write_4(SETM2ENCCOUNT, data);
    clear_buf();
}

void RoboClaw::ResetEnc(){
    clear_buf();
    write_0(RESETENC);
    clear_buf();
}

void RoboClaw::SpeedM1(int32_t speed){
    clear_buf();
    uint8_t data[4] = {
        (uint8_t)((speed >> 24) & 0xff),
        (uint8_t)((speed >> 16) & 0xff),
        (uint8_t)((speed >> 8) & 0xff),
        (uint8_t)(speed & 0xff)
    };

    write_4(M1SPEED, data);
    clear_buf();
}

void RoboClaw::SpeedM2(int32_t speed){
    clear_buf();
    uint8_t data[4] = {
        (uint8_t)((speed >> 24) & 0xff),
        (uint8_t)((speed >> 16) & 0xff),
        (uint8_t)((speed >> 8) & 0xff),
        (uint8_t)(speed & 0xff)
    };

    write_4(M2SPEED, data);
    clear_buf();
}

void RoboClaw::SpeedAccelM1(int32_t accel, int32_t speed){
    clear_buf();
    uint8_t data[8] = {
        (uint8_t)((accel >> 24) & 0xff),
        (uint8_t)((accel >> 16) & 0xff),
        (uint8_t)((accel >> 8) & 0xff),
        (uint8_t)(accel & 0xff),
        (uint8_t)((speed >> 24) & 0xff),
        (uint8_t)((speed >> 16) & 0xff),
        (uint8_t)((speed >> 8) & 0xff),
        (uint8_t)(speed & 0xff)
    };

    write_8(M1SPEEDACCEL, data);
    clear_buf();
}

void RoboClaw::SpeedAccelM2(int32_t accel, int32_t speed){
    clear_buf();
    uint8_t data[8] = {
        (uint8_t)((accel >> 24) & 0xff),
        (uint8_t)((accel >> 16) & 0xff),
        (uint8_t)((accel >> 8) & 0xff),
        (uint8_t)(accel & 0xff),
        (uint8_t)((speed >> 24) & 0xff),
        (uint8_t)((speed >> 16) & 0xff),
        (uint8_t)((speed >> 8) & 0xff),
        (uint8_t)(speed & 0xff)
    };

    write_8(M2SPEEDACCEL, data);
    clear_buf();
}

void RoboClaw::SpeedAccelDeccelPositionM1(uint32_t accel, int32_t speed, uint32_t deccel, int32_t position, uint8_t flag){
    clear_buf();
    uint8_t data[16] = {
        (uint8_t)((accel >> 24) & 0xff),
        (uint8_t)((accel >> 16) & 0xff),
        (uint8_t)((accel >> 8) & 0xff),
        (uint8_t)(accel & 0xff),
        (uint8_t)((speed >> 24) & 0xff),
        (uint8_t)((speed >> 16) & 0xff),
        (uint8_t)((speed >> 8) & 0xff),
        (uint8_t)(speed & 0xff),
        (uint8_t)((deccel >> 24) & 0xff),
        (uint8_t)((deccel >> 16) & 0xff),
        (uint8_t)((deccel >> 8) & 0xff),
        (uint8_t)(deccel & 0xff),
        (uint8_t)((position >> 24) & 0xff),
        (uint8_t)((position >> 16) & 0xff),
        (uint8_t)((position >> 8) & 0xff),
        (uint8_t)(position & 0xff)
    };

    write_16(M1SPEEDACCELDECCELPOS, data, flag);
    clear_buf();
}

void RoboClaw::SpeedAccelDeccelPositionM2(uint32_t accel, int32_t speed, uint32_t deccel, int32_t position, uint8_t flag){
    clear_buf();
    uint8_t data[16] = {
        (uint8_t)((accel >> 24) & 0xff),
        (uint8_t)((accel >> 16) & 0xff),
        (uint8_t)((accel >> 8) & 0xff),
        (uint8_t)(accel & 0xff),
        (uint8_t)((speed >> 24) & 0xff),
        (uint8_t)((speed >> 16) & 0xff),
        (uint8_t)((speed >> 8) & 0xff),
        (uint8_t)(speed & 0xff),
        (uint8_t)((deccel >> 24) & 0xff),
        (uint8_t)((deccel >> 16) & 0xff),
        (uint8_t)((deccel >> 8) & 0xff),
        (uint8_t)(deccel & 0xff),
        (uint8_t)((position >> 24) & 0xff),
        (uint8_t)((position >> 16) & 0xff),
        (uint8_t)((position >> 8) & 0xff),
        (uint8_t)(position & 0xff)
    };

    write_16(M2SPEEDACCELDECCELPOS, data, flag);
    clear_buf();
}