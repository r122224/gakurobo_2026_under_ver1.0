#ifndef SENSOR_BYTE_COMM_H
#define SENSOR_BYTE_COMM_H
#include "mbed.h"
 
 //主にLR-TB5000のデータを送るために作成したマイコン間通信クラス
 //1つのセンサに対して2バイト分の整数データを送ることができる←2バイト以上のデータは正しく送れない
 //送るセンサのデータ数は任意
 //送信側と受信側でインスタンス化で設定を合わせる必要がある(ボーレート，送るデータの個数)
 
class SensorComm
{
public:
    SensorComm(PinName SerialTx, PinName SerialRx, unsigned int BaudRate, int sensorNum);//マイコン側のTx，Rxとボーレート，送るデータの個数
    //bool byteSend(uint16_t value[]);//データを送る
    bool update();//受信したデータの更新
    uint16_t sensorData[100];//受信したデータの格納場所←publicで宣言しているので，この変数をupdate関数を更新してから参照する
    
private:
    Serial _comm;
    int preData;
    int sendNum;
};
 
#endif

