#include "mbed.h"

class LimitComm
{
public:
    LimitComm(PinName SerialTx, PinName SerialRx, unsigned int BaudRate);
    bool update();//受信したデータの更新
    uint16_t sensorData[100];
    uint8_t data[100];

private:
    Serial _comm;
    int preData;
    int sendNum;

};