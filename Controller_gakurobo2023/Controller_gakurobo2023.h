/*
 *https://os.mbed.com/users/kikuchi8810/code/controllerForMbed_test//file/ab1c94d6f4fb/main.cpp/
 *↑いつの日かのデータ
 *作成者：いろんな人（上野，田村，菊池，安達，赤崎）
 */

//2023/03/23最終更新→ポールの予約機能追加


#ifndef CONTROLLER_GAKUROBO2023_H
#define CONTROLLER_GAKUROBO2023_H
 
#include "mbed.h"
#include "RawSerial.h"
#include "define.h"
#include <cstdint>

#define BOOK_POLE_NUM 5
 
struct ControllerData{
    unsigned int ButtonState;
    uint8_t RJoyX, RJoyY, LJoyX, LJoyY;
};
 
class Controller{
    public:
        Controller(PinName tx, PinName rx, int baudrate);

        unsigned int addButtonState;//←↓　赤崎担当部分
        unsigned int preAddButtonState;
        bool M5ButtonA;
        bool M5ButtonB;
        bool M5ButtonC;
        bool addButtonLeft;
        bool addButtonRight;
        uint8_t steelData;
        bool sendAnyByte(uint8_t sendData[], unsigned int numByte);//コントローラに値を送る
        bool readButton_bin(unsigned int ButtonNum); //押していない時はfalse(0),押してるときはtrue(1)を返す．　ButtonNumはデータの欲しいボタンの名前を
        bool readaddButton_bin(unsigned int ButtonNum);
        int  readButton(unsigned int ButtonNum);     //上にプラスして 押した瞬間は2，放した瞬間は-1を返す．    define.hを参考に数字を入力しても良い
        int  readaddButton(unsigned int ButtonNum);
        bool getComCheck(void); //値が更新されたときにtrueを返す．
        bool update(); //受信の処理＋ボタンの情報の更新．受信割込みで処理
        bool rate(void);//送信側で入力がある場合にtrue.//多分使わない


        void init(int _time_out_ms, int _int_time_ms); //コントローラの通信速度と通信のタイムアウト時間を設定
        bool available(void);
 
        unsigned int getButtonState();
        //↑分解する前のButtonStateの情報をprint 0~255の値をとる．自分でボタンの処理をしたいとき用
        ControllerData getConData();
    
                                //       X
        double readJoyRX();     //       ^ 
        double readJoyRY();     //       | 
        double readJoyLX();     //  Y<---+----
        double readJoyLY();     //       | 
                                //       | 
                                //  1.0  ~   -1.0
 
                                   //       X
        uint8_t readJoyRXbyte();   //       ^ 
        uint8_t readJoyRYbyte();   //       |
        uint8_t readJoyLXbyte();   //  Y<---+----
        uint8_t readJoyLYbyte();   //       | 
                                   //       |
                                   //  255  ~    0
        double JoyRtilt();      //ジョイティックの倒れ具合 0 ~ 1 
        double JoyLtilt();      //倒した方向に関係なく，倒し切ったときが1となるように計算される

        double JoyRTheta();     //Xの正に傾いている時を0としたジョイスティックの傾いている方向
        double JoyLTheta();     //半時計方向が正，-π ~ π (rad)

        int nextPole();
        int next2Pole();
        bool poleShift(int);//2023用　先行予約されているポールの配列を１つずつずらす
        int countPoleNum();//先行予約されているポールの数を返す
        bool clearBook();//先行予約されているポール情報をすべてクリアする
 

    private:
        
        RawSerial serial;
 
        bool comCheck;
        ControllerData conData;
        ControllerData pre_conData;
        unsigned int lastButtonState;

        int bookPole[BOOK_POLE_NUM];//2023用　各ポールの先行入力用の変数
        int bookPoleNum;
        int shootingPole;
        int shooting2Pole;
        
        int time_out_ms;
        double int_time_ms;
        bool conAvailable;
 
        uint8_t serial_recieve(){//CON_ADACHIでのみ使用．
            char temp;
                temp = serial.getc();
            //CONTROL.write(temp);    //受け取ったデータをTXピンからそのまま送っている．他のマイコンにも流したいとき用．
            return temp;
        }

        
};
 
#endif
