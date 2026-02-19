#include "STS_Servo.h"

STS::STS(Serial* serial, unsigned int baudrate){
    pSerial = serial;
    pSerial->baud(baudrate);
    //_csPin = false;
}

void STS::cmdSend8(unsigned int id, unsigned int posData, unsigned int timeData, unsigned int speedData){
    char checksum = 0;
    checksum = (char)((id + 0x09 + 0x03 + 0x2A + (posData & 0xff) + (posData >> 8) + (timeData & 0xff) + (timeData >> 8) + (speedData & 0xff) + (speedData >> 8)) & 0xff);
    checksum = ~checksum;

    char buf[13] = {
        0xff, 0xff,
        (char)id,
        0x09,
        0x03,
        0x2A,
        (char)(posData & 0xff),(char)(posData >> 8),
        (char)(timeData & 0xff),(char)(timeData >> 8),
        (char)(speedData & 0xff),(char)(speedData >> 8),
        checksum
    };

    //_csPin = true;

    for(int i = 0; i < 13; i++){
        pSerial->putc(buf[i]);
    }

    //_csPin = false;

}

/*void STS3215::cmdSend9(unsigned int id, unsigned int posData, unsigned int timeData, unsigned int speedData){
    char checksum = 0;
    checksum = (char)((id + 0x0A + 0x03 + 0x29 + 0x32 + (posData & 0xff) + (posData >> 8) + (timeData & 0xff) + (timeData >> 8) + (speedData & 0xff) + (speedData >> 8)) & 0xff);
    checksum = ~checksum;
    char buf[14] = {
        0xff, 0xff,
        (char)id,
        0x0A,
        0x03,
        0x29,
        0x32,
        (char)(posData & 0xff),(char)(posData >> 8),
        (char)(timeData & 0xff),(char)(timeData >> 8),
        (char)(speedData & 0xff),(char)(speedData >> 8),
        (char)((id + 0x0A + 0x03 + 0x29 + 0x32 + (posData & 0xff) + (posData >> 8) + (timeData & 0xff) + (timeData >> 8) + (speedData & 0xff) + (speedData >> 8)) & 0xff)
    };

    _csPin = true;

    for(int i = 0; i < 14; i++){
        pSerial->putc(buf[i]);
    }

    _csPin = false;

}
*/
/*
void STS3215::position(unsigned int id, unsigned int posData, unsigned int timeData, int speedData){
    if(speedData < 0){
        speedData = -speedData;
        speedData |= (1 << 15);
    }
    cmdSend8(id, posData, timeData, speedData);
}

unsigned int STS3215::read(unsigned int id){
    static int preReadData = 0;
    uint16_t readData = 0;
    char recieveBuf[6];
    char c, c1, c2;
    char checksum = 0;
    checksum = ~((char)(((char)id + 0x04 + 0x02 + 0x38 + 0x02) & 0xff));
    //checksum = ~checksum;

    //printf("%x\t", checksum);

    char buf[8] = {
                    0xff, 0xff,
                    (char)id,
                    0x04,
                    0x02,
                    0x38,
                    0x02,
                    checksum
                  };

    for(int i = 0; i < 8; i++){
        pSerial->putc(buf[i]);
    }

    checksum = 0;

    // printf("\n");
    // while(pSerial->readable()){
    //     printf("%x\n", pSerial->getc());
    // }
    // printf("\n");


    

    if(pSerial->readable()){
        for(int i = 0; i < 6; i++){
        pSerial->getc();
        }

        c1 = pSerial->getc();
        c2 = pSerial->getc();

        //printf("%d\t%d\n", (uint8_t)c1, (uint8_t)c2);

        if(c1 == 0xff && c2 == 0xff){
            

            for(int i = 0; i < 6; i++){
                if(pSerial->readable())recieveBuf[i] = pSerial->getc();
                //printf("%x", recieveBuf[i]);
                if(i != 5){
                    //checksum += recieveBuf[i];
                }
            }

            //printf("\n");
            checksum = (recieveBuf[0] + recieveBuf[1] + recieveBuf[2] + recieveBuf[3] + recieveBuf[4]) & 0xff;
            //checksum &= 0xff;
            checksum = ~checksum;

            if(recieveBuf[5] == checksum){
                readData = (recieveBuf[4] << 8) | recieveBuf[3];
            } else {
                readData = preReadData;
                while(pSerial->readable()){
                    pSerial->getc();
                }
            }

        } else {
            readData = preReadData;
        }

    } else {
        readData = preReadData;
    }

    preReadData = readData;

    return readData & 0x0fff;
    
}*/


void STS::setMode(unsigned int id, unsigned int mode){//使えるか分からない
    char checksum = 0;
    checksum = (char)((id + 0x04 + 0x03 + 0x21 + (char)mode) & 0xff);
    checksum = ~checksum;

    char buf[8] = {
        0xff, 0xff,
        (char)id,
        0x04,
        0x03,
        0x21,
        (char)mode,
        checksum
    };

    //_csPin = true;

    for(int i = 0; i < 8; i++){
        pSerial->putc(buf[i]);
    }
}


void STS::cmd(unsigned int id, unsigned int posData, int timeData, int speedData){
    if(speedData < 0){
        speedData = -speedData;
        speedData |= (1 << 15);//回転方向が負の場合
    }

    if(timeData < 0){
        timeData = -timeData;
        timeData |= (1 << 10);
    }

    cmdSend8(id, posData, timeData, speedData);
}

unsigned int STS::read(unsigned int id){
    static int preReadData = 0;
    int loopCount = 0;
    bool stageCheck = true;//各ステージでの通信データが合っているかを示す
    uint16_t readData = 0;
    char recieveBuf[6];
    char c, c1, c2;
    char checksum = 0;
    checksum = ~((char)(((char)id + 0x04 + 0x02 + 0x38 + 0x02) & 0xff));
    //checksum = ~checksum;

    //printf("%x\t", checksum);

    char buf[8] = {
                    0xff, 0xff,
                    (char)id,
                    0x04,
                    0x02,
                    0x38,
                    0x02,
                    checksum
                  };

    for(int i = 0; i < 8; i++){
        pSerial->putc(buf[i]);
    }

    checksum = 0;

/*
    printf("\n");
    while(pSerial->readable()){
        printf("%x\n", pSerial->getc());
    }
    printf("\n");
*/

    

    if(pSerial->readable()){

        while(pSerial->readable() && loopCount < 6) {
            pSerial->getc();
            loopCount++;
        }

        if(loopCount == 6){
            stageCheck = true;
        } else {
            stageCheck = false;
        }

        loopCount = 0;//繰り返し回数の記録をリセット

        if(stageCheck){
            c1 = pSerial->getc();
            c2 = pSerial->getc();
        }

        //printf("%d\t%d\n", (uint8_t)c1, (uint8_t)c2);

        if(c1 == 0xff && c2 == 0xff){
            

            for(int i = 0; i < 6; i++){
                if(pSerial->readable())recieveBuf[i] = pSerial->getc();
                //printf("%x", recieveBuf[i]);
                if(i != 5){
                    //checksum += recieveBuf[i];
                }
            }

            //printf("\n");
            checksum = (recieveBuf[0] + recieveBuf[1] + recieveBuf[2] + recieveBuf[3] + recieveBuf[4]) & 0xff;
            //checksum &= 0xff;
            checksum = ~checksum;

            if(recieveBuf[5] == checksum){
                readData = (recieveBuf[4] << 8) | recieveBuf[3];
            } else {
                readData = preReadData;
                while(pSerial->readable()){
                    pSerial->getc();
                }
            }

        } else {
            readData = preReadData;
        }

    } else {
        readData = preReadData;
    }

    preReadData = readData;

    return readData & 0x0fff;
    
}

/*
void STS3215::speed(unsigned int id, unsigned int posData, unsigned int timeData, unsigned int speedData){
    cmdSend9(id, posData, timeData, speedData);
}*/