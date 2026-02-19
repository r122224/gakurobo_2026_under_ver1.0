#include "Controller_gakurobo2023.h"
#include <cstdint>


 
Controller::Controller(PinName tx, PinName rx, int baudrate) : serial(tx, rx, baudrate)
{
    conData.ButtonState = 0;
    conData.RJoyX = 127, conData.RJoyY = 127, conData.LJoyX = 127, conData.LJoyY = 127;
 
    lastButtonState = 0;
    addButtonState = 0;
    steelData = 0;
    M5ButtonA = M5ButtonB = M5ButtonC = addButtonLeft = addButtonRight = false;

    for(int i = 0; i < BOOK_POLE_NUM; i++){//先行予約用の配列の中身を初期化
        bookPole[i] = -1;
    }

    bookPoleNum = 0;//初期はポールの予約数は0
    shootingPole = shooting2Pole = -1;
 
    comCheck = false;
    conAvailable = false;
    time_out_ms = -1;
}
 
void Controller::init(int _time_out_ms, int _int_time_ms)
{
    time_out_ms = _time_out_ms;
    int_time_ms = _int_time_ms;
}

bool Controller::sendAnyByte(uint8_t sendData[], unsigned int numByte){
    uint8_t smgs[numByte + 2];
    smgs[numByte] = 0;
    for(int i = 0; i < numByte; i++){
        smgs[i] = sendData[i] & 0b00111111;
        smgs[numByte] ^= smgs[i];
    }
    
    smgs[numByte + 1] = '\n';

    for(int i = 0; i < (numByte + 1); i++){
        smgs[i] += 0x20;
    } 

    for(int i = 0; i < numByte + 2; i++){
    serial.putc(smgs[i]);  
    }

     //serial.putc('\n');

    //serial.printf("%d\n", smgs);

    //serial.printf("%s\n", smgs);
    //serial.printf("\n");
    //serial.putc(smgs[numByte + 1]);

    return true;
}
 
bool Controller::update()
{
    static int count_ms = time_out_ms, pre_count_ms = 0; //受信時刻と前回の受信時刻
    count_ms += int_time_ms;
 
    char receive_data[1000];
    unsigned int loop_count = 0, checksum = 0x00;
    comCheck = false;
 
#if CON_TYPE == CON_ADACHI // 安達君開発のコントローラを使う場合の処理(どのコントローラを使うかはdefine.hで設定)
    while (loop_count < 10 && serial.readable())
    {
        if (serial_recieve() == '\n')
        {
            for (int i = 0; i < 8; i++)
                receive_data[i] = serial_recieve();
            for (int i = 0; i < 8; i++)
                receive_data[i] -= 0x20;
            for (int i = 0; i < 7; i++)
                checksum ^= receive_data[i];
 
            if (receive_data[7] == checksum & 0xFF)
            {
                comCheck = true;
 
                //pre_conData.ButtonState = conData.ButtonState;
                lastButtonState = ((receive_data[0] & 0x3F) << 2) | ((receive_data[1] & 0x30) >> 4);
                conData.ButtonState |= lastButtonState;
 
                conData.RJoyX = ((receive_data[1] & 0x0F) << 4) | ((receive_data[2] & 0x3C) >> 2);
                conData.RJoyY = ((receive_data[2] & 0x03) << 6) | (receive_data[3] & 0x3F);
                conData.LJoyX = ((receive_data[4] & 0x3F) << 2) | ((receive_data[5] & 0x30) >> 4);
                conData.LJoyY = ((receive_data[5] & 0x0F) << 4) | (receive_data[6] & 0x0F);
 
                break;
            }
            
            pre_count_ms = count_ms; //受信時間の更新
        }
        loop_count++;
    }
#elif CON_TYPE == CON_ELECOM // ELECOMのコントローラを使う場合の処理(どのコントローラを使うかはdefine.hで設定)
    // コントローラデータを取得する部分
    static int recv_num = 0;
    char c;
    while (serial.readable())
    {
        c = serial.getc();
        if (c == '\n')
        {
            if (recv_num == 10)
            { // チェックサムは無く，9個受信したら値を格納
                for (int i = 0; i < 9; i++)
                    checksum += (unsigned int)(receive_data[i] - 0x20); // チェックサムの計算
                if ((checksum & 0x3F) == (receive_data[9] - 0x20))
                { // チェックサムの計算が合っていた場合のみ値を格納
                    comCheck = true;
 
                    //conData.ButtonState = 0;
                    conData.LJoyX = 0, conData.LJoyY = 0, conData.RJoyX = 0, conData.RJoyY = 0;
                    lastButtonState = (unsigned int)(receive_data[0] - 0x20);
                    lastButtonState |= (unsigned int)(receive_data[1] - 0x20) << 6;
                    lastButtonState |= (unsigned int)(receive_data[2] - 0x20) << 12;
 
                    conData.LJoyX |= (unsigned int)(receive_data[3] - 0x20);
                    conData.LJoyX |= (unsigned int)((receive_data[4] - 0x20) & 0x03) << 6;
                    conData.LJoyX = abs(conData.LJoyX - 0xFF);
 
                    conData.LJoyY |= (unsigned int)((receive_data[4] - 0x20) & 0x3C) >> 2;
                    conData.LJoyY |= (unsigned int)((receive_data[5] - 0x20) & 0x0F) << 4;
                    conData.LJoyY = abs(conData.LJoyY - 0xFF);
 
                    conData.RJoyX |= (unsigned int)((receive_data[5] - 0x20) & 0x30) >> 4;
                    conData.RJoyX |= (unsigned int)((receive_data[6] - 0x20) & 0x3F) << 2;
                    conData.RJoyX = abs(conData.RJoyX - 0xFF);
 
                    conData.RJoyY |= (unsigned int)(receive_data[7] - 0x20);
                    conData.RJoyY |= (unsigned int)((receive_data[8] - 0x20) & 0x03) << 6;
                    conData.RJoyY = abs(conData.RJoyY - 0xFF);


 
                    int buttonPushNum = 0;
                    for (int i = 0; i < 16; i++)
                    {
                        buttonPushNum += (conData.ButtonState >> i) & 0x0001;
                    }
                    if (buttonPushNum > 5)
                    {
                        //conData.ButtonState = pre_conData.ButtonState;
                        comCheck = false;
                    }
                    else
                    {
                        conData.ButtonState |= lastButtonState;
                    }
                    
                    pre_count_ms = count_ms; //受信時間の更新
                }
            }
            recv_num = 0;
        }
        else
        {
            receive_data[recv_num] = c;
            recv_num++;
        }
    }
#elif CON_TYPE == CON_DS4    // DualShock4を使う場合の処理(どのコントローラを使うかはdefine.hで設定)
    // コントローラデータを取得する部分
    static int recv_num = 0;
    //static unsigned int preAddButtonState = 0;
    preAddButtonState = 0;
    char c;
    int log2Buf;
    while (serial.readable())
    {
        c = serial.getc();
        //Serial.print(c);
        if (c == '\n')
        {
            if (recv_num == 12)
            { // データ数はチェックサム含めて12個(0~11)
                checksum = 0;
                for (int i = 0; i < 11; i++)
                    checksum ^= (unsigned int)(receive_data[i] - 0x20); // チェックサムの計算
                if ((checksum & 0x3F) == (receive_data[11] - 0x20))
                { // チェックサムの計算が合っていた場合のみ値を格納
                    comCheck = true;
                    steelData = (uint8_t)receive_data[2];
                    pre_conData.ButtonState = conData.ButtonState;
                    conData.LJoyX = 0, conData.LJoyY = 0, conData.RJoyX = 0, conData.RJoyY = 0;
                    lastButtonState = (unsigned int)(receive_data[0] - 0x20) & 0x3F;
                    lastButtonState |= (unsigned int)((receive_data[1] - 0x20) & 0x3F) << 6;
                    lastButtonState |= (unsigned int)((receive_data[2] - 0x20) & 0x0F) << 12;
                    lastButtonState |= (unsigned int)(((receive_data[2] - 0x20) >> 4) & 0x03) << 16;//ジョイスティック押し込み
                    addButtonState = (unsigned int)((receive_data[3] - 0x20) & 0x3F);
                    addButtonState |= (unsigned int)((receive_data[4] - 0x20) & 0x1F) << 6; 

                    conData.LJoyX |= (unsigned int)(receive_data[5] - 0x20);
                    conData.LJoyX |= (unsigned int)((receive_data[6] - 0x20) & 0x03) << 6;
                    conData.LJoyX = abs(conData.LJoyX - 0xFF);
 
                    conData.LJoyY |= (unsigned int)((receive_data[6] - 0x20) & 0x3C) >> 2;
                    conData.LJoyY |= (unsigned int)((receive_data[7] - 0x20) & 0x0F) << 4;
                    conData.LJoyY = abs(conData.LJoyY - 0xFF);
 
                    conData.RJoyX |= (unsigned int)((receive_data[7] - 0x20) & 0x30) >> 4;
                    conData.RJoyX |= (unsigned int)((receive_data[8] - 0x20) & 0x3F) << 2;
                    conData.RJoyX = abs(conData.RJoyX - 0xFF);
 
                    conData.RJoyY |= (unsigned int)(receive_data[9] - 0x20);
                    conData.RJoyY |= (unsigned int)((receive_data[10] - 0x20) & 0x03) << 6;
                    conData.RJoyY = abs(conData.RJoyY - 0xFF);

                    M5ButtonA = (bool)(((receive_data[10] - 0x20) >> 2) & 0x01);
                    M5ButtonB = (bool)(((receive_data[10] - 0x20) >> 3) & 0x01);
                    M5ButtonC = (bool)(((receive_data[10] - 0x20) >> 4) & 0x01);

                    addButtonRight = (bool)(((receive_data[4] - 0x20) >> 5) & 0x01);
                    addButtonLeft = (bool)(((receive_data[10] - 0x20) >> 5) & 0x01);

                    bookPoleNum = 0;//毎回初期化しておく

                    for(int i = 0; i < BOOK_POLE_NUM; i++){//
                             if(bookPole[i] != -1) bookPoleNum++;
                        }

                    if(addButtonState != preAddButtonState){
                        //printf("pass\n");
                        log2Buf = (int)(log2((double)addButtonState));
                        if(abs(log2Buf) > 10){
                            log2Buf = -1;
                        }

                        poleShift(log2Buf);
                    
                    }

                    shootingPole = bookPole[0];
                    shooting2Pole = bookPole[1];
 
                    // 通信ミスであり得ない数のボタン数押されていた場合に無視する処理
                    int buttonPushNum = 0;
                    for (int i = 0; i < 16; i++)
                    {
                        buttonPushNum += (lastButtonState >> i) & 0x0001;
                    }
                    if (buttonPushNum > 5)
                    {
                        comCheck = false;
                    }
                    else
                    {
                        conData.ButtonState = lastButtonState & 0xFFFFFFFF;
                    }
 
                    pre_count_ms = count_ms; //受信時間の更新
                }
            }
        }
        else
        {
            receive_data[recv_num] = c;
            recv_num++;
        }
    }
                recv_num = 0;


    preAddButtonState = addButtonState;

 
#endif
    // 新井さんのクラス
    if(!(time_out_ms == -1)) conAvailable = (time_out_ms > (count_ms - pre_count_ms)); //タイムアウトとインターバルの比較
    else conAvailable = true;
    if(count_ms > time_out_ms * 1000) count_ms = time_out_ms; //オーバーフロー対策
 
    return comCheck;

    // ふじさん
    // if (!(time_out_ms == -1))
    //     conAvailable =
    //         (time_out_ms >
    //         (count_ms - pre_count_ms)); //タイムアウトとインターバルの比較
    // else
    //     conAvailable = true;
    // if (count_ms > time_out_ms * 1000)
    //     count_ms = time_out_ms; //オーバーフロー対策

    // return comCheck;
}
 
bool Controller::getComCheck(void){return comCheck;}
 
bool Controller::available(void){return conAvailable;}
 
bool Controller::rate(void) {
  bool _ButtunState = false, _RX = false, _RY = false, _LX = false, _LY = false;
  if (comCheck) {
    _ButtunState = (conData.ButtonState != 0) ? true : false;
    _RX = (conData.RJoyX != 128) ? true : false;
    _RY = (conData.RJoyY != 128) ? true : false;
    _LX = (conData.LJoyX != 128) ? true : false;
    _LY = (conData.LJoyY != 128) ? true : false;
  }
  // printf("%d,%d,%d,%d,%d\n",_ButtunState,_RX,_RY,_LX,_LY);
  return _ButtunState | _RX | _RY | _LX | _LY;
}

bool Controller::readButton_bin(unsigned int ButtonNum)
{ //放しているときは０，押しているときは１
    return ((conData.ButtonState & (0x0001 << (ButtonNum - 1))) == (0x0001 << (ButtonNum - 1))) ? true : false;
}

bool Controller::readaddButton_bin(unsigned int ButtonNum){
    return ((addButtonState & (0x0001 << (ButtonNum - 1))) == (0x0001 << (ButtonNum - 1))) ? true : false;
}

int Controller::readButton(unsigned int ButtonNum)
{ //放しているときは０，押しているときは１，押した瞬間は２，放した瞬間は－１
    int result = 0;
    if ((conData.ButtonState & (0x0001 << (ButtonNum - 1))) == (0x0001 << (ButtonNum - 1)))
        result += 2;
    if ((pre_conData.ButtonState & (0x0001 << (ButtonNum - 1))) == (0x0001 << (ButtonNum - 1)))
        result -= 1;
    return result;
}

int Controller::readaddButton(unsigned int ButtonNum){
    int result = 0;
    if ((addButtonState & (0x0001 << (ButtonNum - 1))) == (0x0001 << (ButtonNum - 1)))
        result += 2;
    if ((preAddButtonState & (0x0001 << (ButtonNum - 1))) == (0x0001 << (ButtonNum - 1)))
        result -= 1;
    return result;
}
 
unsigned int Controller::getButtonState(){return conData.ButtonState;}
 
ControllerData Controller::getConData(){return conData;}
 
double Controller::readJoyRX() {
  double tempJoy = ((double)conData.RJoyX - 127.5) / 127.5;
  if (fabs(tempJoy) > 0.1)
    return tempJoy;
  return 0;
}

double Controller::readJoyRY() {
  double tempJoy = ((double)conData.RJoyY - 127.5) / 127.5;
  if (fabs(tempJoy) > 0.1)
    return tempJoy;
  return 0;
}

double Controller::readJoyLX() {
  double tempJoy = ((double)conData.LJoyX - 127.5) / 127.5;
  if (fabs(tempJoy) > 0.1)
    return tempJoy;
  return 0;
}

double Controller::readJoyLY() {
  double tempJoy = ((double)conData.LJoyY - 127.5) / 127.5;
  if (fabs(tempJoy) > 0.1)
    return tempJoy;
  return 0;
}
 
uint8_t Controller::readJoyRXbyte() { return conData.RJoyX; }

uint8_t Controller::readJoyRYbyte() { return conData.RJoyY; }

uint8_t Controller::readJoyLXbyte() { return conData.LJoyX; }

uint8_t Controller::readJoyLYbyte() { return conData.LJoyY; }

double Controller::JoyRtilt(){
    double tempTilt = sqrt(pow(readJoyRX(), 2.0) + pow(readJoyRY(), 2.0));
    if(tempTilt > 1)
        return 1.0;
    else
        return tempTilt;
}

double Controller::JoyLtilt(){
    double tempTilt = sqrt(pow(readJoyLX(), 2.0) + pow(readJoyLY(), 2.0));
    if(tempTilt > 1)
        return 1.0;
    else
        return tempTilt;
}

double Controller::JoyRTheta(){return atan2(readJoyRY(), readJoyRX());}

double Controller::JoyLTheta(){return atan2(readJoyLY(), readJoyLX());}

int Controller::nextPole(){
    return shootingPole;
}

int Controller::next2Pole(){
    return shooting2Pole;
}

bool Controller::poleShift(int shiftPoleNum){//ポール番号は-1~10で管理する．-1はどのポールも狙われていないとき
    static int buf[4];
    //bookPoleNum++;

    if(shiftPoleNum < -2 && 10 < shiftPoleNum){//範囲外の数が来たら-1
        shiftPoleNum = -1;
    }
    
    for(int i = 0; i < BOOK_POLE_NUM - 1; i++){//一時的にデータを避難
        buf[i] = bookPole[i + 1];
    }

    if(bookPoleNum == BOOK_POLE_NUM || shiftPoleNum == -2){
        if(shiftPoleNum == -2) shiftPoleNum = -1;
        for(int i = 0; i < BOOK_POLE_NUM; i++){//最新データの格納とデータずらし
            if(i == (BOOK_POLE_NUM - 1)) {
                bookPole[BOOK_POLE_NUM - 1] = shiftPoleNum;
            } else {
                bookPole[i] = buf[i];
            }
        }
    } else {
        bookPole[bookPoleNum] = shiftPoleNum;
    }

    for(int i = 0; i < BOOK_POLE_NUM; i++){//並び替え
        if(bookPole[i] == -1){
            for(int k = i + 1; k < BOOK_POLE_NUM; k++){
                if(bookPole[i] == -1 && bookPole[k] > -1){
                    int temp = bookPole[i];
                    bookPole[i] = bookPole[k];
                    bookPole[k] = temp;
                }
            }
        }
    }

    return true;
}

int Controller::countPoleNum(){
    return bookPoleNum;
}

bool Controller::clearBook(){//配列の中身を初期化
    for(int i = 0; i < BOOK_POLE_NUM; i++){
        bookPole[i] = -1;
    }

    bookPoleNum = 0;

    return true;
}

