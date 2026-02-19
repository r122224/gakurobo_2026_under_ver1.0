/*

・複数バイトのデータを送受信するためのライブラリ
・送信と受信でそれぞれ任意のバイト数を送受信できる
・マスターとスレーブの関係は存在しない
・1つ前のデータは保持していないので注意
・このライブラリで送受信を行う際には送受信のデータ数を合わせる必要がある

作成日：2022/12/14
作成者：赤崎恵士

*/

#ifndef COMM_TALK_H
#define COMM_TALK_H

#include "mbed.h"
#include <cstdint>

class CommTalk{

public:
CommTalk(Serial*, unsigned int);
bool update(unsigned int);//受信データを更新
bool sendData(uint8_t sData[], unsigned int dataNum);
uint8_t data[100];//格納しておくデータは100個まで
double required_datax, required_datay;//受け取るデータ
unsigned char *send_datax;
unsigned char *send_datay;

private:
Serial *pSerial;


};

#endif