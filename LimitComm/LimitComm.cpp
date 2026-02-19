#include "LimitComm.h"

LimitComm::LimitComm(PinName SerialTx, PinName SerialRx, unsigned int BaudRate) : _comm(SerialTx, SerialRx, BaudRate)
{
    //_comm.baud(BaudRate);
    for (int i = 0; i < 100 ; i++) sensorData[i] = 0;//格納データの初期化
}

bool LimitComm::update()
{
    char c;
    char checksum = 0;
    unsigned int bufCount = 0;
    char buf[100];
    bool commcheck = false;
    uint32_t dataNum = 2;
    while(_comm.readable()){
        c = _comm.getc();
        if(c == '\n' && dataNum == bufCount - 1){
            for (int i = 0; i < dataNum; i++) {
                checksum ^= buf[i];
                // printf("commtalk_print1\n");
            }
            if (buf[dataNum] == checksum) {
                commcheck = true;
                for (int i = 0; i < dataNum; i++) {
                    data[i] = (uint8_t)buf[i];
                }

                for (int i = 0; i < 6; i++) {
                    sensorData[i] = (data[0] >> i) & 0b1; // L0 ～ L5 に対応
                }

                for (int i = 0; i < 6; i++) {
                    // received_bits のインデックスは 6 から開始 (L6～L11)
                    int index = i + 6; 
                    // buf[1] から i番目のビットを取り出す
                    sensorData[index] = (data[1] >> i) & 0b1; // L6 ～ L11 に対応
                }

                while (_comm.readable()) {
                // printf("commtalk_print2\n");
                _comm.getc();
                }

            } else {
                commcheck = false;
            }

        } else {
            buf[bufCount] = c - 0x20;
            bufCount++;
        }
    }
    return commcheck;
}