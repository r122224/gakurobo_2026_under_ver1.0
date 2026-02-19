#include "CommTalk.h"
#include <cstdint>

CommTalk::CommTalk(Serial * xSerial, unsigned int baudrate){//インストラクタ
    pSerial = xSerial;
    pSerial->baud(baudrate);
    for(int i = 0; i < 100; i++){
        data[i] = 0;
    }
}


bool CommTalk::sendData(uint8_t sData[], unsigned int dataNum){//1データにつき6ビットまでのデータを格納することができる データ数はチェックサムと終端文字を除く

    char checksum = 0;
    //char checksum_d = 0;
    char buf[dataNum];
    //char senddata[dataNum];

    for(int i = 0; i < dataNum; i++){
        buf[i] = (char)(sData[i] & 0x3f);
       // senddata[i] = (char)(souce_data[i] & 0x3f);
        checksum ^= buf[i];
        //checksum_d ^=  senddata[i];
    }

    for(int i = 0; i < dataNum; i++){
        pSerial->putc(buf[i] + 0x20);
    }

    pSerial->putc(checksum + 0x20);
    pSerial->putc('\n');
    

    return true;
}

bool CommTalk::update(unsigned int dataNum){//受け取るデータの個数が引数(チェックサムと終端文字は除く)
    char c;
    char checksum = 0;
    unsigned int bufCount = 0;
    char buf[100];
    bool commcheck = false;

    // // 受信データ用バッファ
	// unsigned char receive_datax[sizeof(double)];
    // unsigned char receive_datay[sizeof(double)];

        while(pSerial->readable()){
            c = pSerial->getc();
            //printf("%d\n", (uint8_t)(c - 0x20));
           
            if(c == '\n' && dataNum == bufCount - 1){
                //printf("%d\n", bufCount);
                //printf("pass\n");
                
                for(int i = 0; i < dataNum; i++){
                    checksum ^= buf[i];
                }

                if(buf[dataNum] == checksum){
                    
                    commcheck = true;

                    for(int i = 0; i < dataNum; i++){
                        data[i] = (uint8_t)buf[i];
                    }
                     

                    // for (int i = 0; i < sizeof(double); i++){
                    //     receive_data[i] = *((unsigned char*)send_data + i);
                    // } 
	                // required_data = *(double*)receive_data;
	                

                    

                /*while(pSerial->readable()){
                    pSerial->getc();
                }*/

                } else {
                    commcheck = false;
                }

            } else {
                buf[bufCount] = c - 0x20;
                bufCount++;
            }
        }

        while(pSerial->readable()){
            pSerial->getc();
        }

        //printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n", data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9], data[10]);
            return commcheck;

}


