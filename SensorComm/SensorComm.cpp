#include "SensorComm.h"

SensorComm::SensorComm(PinName SerialTx, PinName SerialRx, unsigned int BaudRate, int sensorNum) : _comm(SerialTx, SerialRx, BaudRate)
{
    //_comm.baud(BaudRate);
    for (int i = 0; i < 100 ; i++) sensorData[i] = 0;//格納データの初期化
    sendNum = sensorNum;
}


bool SensorComm::update()
{
    uint8_t getNum[100];//受信データ格納変数
    char c;
    int numCount = 0;
    bool numState = true;
    uint8_t checksum = 0;

        while(_comm.readable() & numState) {
            c = (char)_comm.getc();//受信データ格納
            
            if(numCount >= 30){
                numState = false;
            } else {
                numState = true;
            }

            if(c == '\n'){
               //printf("\n");

                //if(numCount == (int)(2 * sendNum + 1)){

                    for(int i = 0; i < 2 * sendNum; i++){
                        checksum ^= getNum[i];
                    }

                    if(checksum == getNum[2 * sendNum]){
                        numState = true;
                        for (int i = 0; i < sendNum; i++){
                            sensorData[i] = (uint16_t)((getNum[2*i] & 0x3f) << 6) | (uint16_t)(getNum[2*i + 1] & 0x3f);
                        }
                        break;
                    } else {
                        numState = false;
                    }
            
                //} else {
                    //numState = false;
                //}

            } else {
                getNum[numCount] = (uint8_t)(c - 0x20);
                //printf("%d\t", (uint8_t)getNum[numCount]);
                numCount++;
            }
        }

        //無駄データの廃棄
        if(!numState){
            while(_comm.readable()) {
                _comm.getc();
            }
        }

    //printf("%d\n", numCount);

    return numState;
}