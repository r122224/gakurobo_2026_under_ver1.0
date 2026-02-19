#include "bno085_UART_RVC.h"
#include <cstdint>
#include <math.h>
//#include <mbed.h>
//# include "BufferedSerial.h"

bno085::bno085(Serial* xserial){
  serial = xserial;
  serial->baud(115200);
  anglex = 0.0;
  angley = 0.0;
  anglez = 0.0;
  rawanglez = 0.0;
  pre_rawanglex = 0.0;
  pre_rawangley = 0.0;
  pre_rawanglez = 0.0;
  diff_rawanglez = 0.0;
}
// センサデータの送信要求
void bno085::get_sensor_data(){
  serial->putc(0xAA);
  serial->putc(0xDE);
  serial->putc(0x01);
  serial->putc(0x00);
  serial->putc(0x92);
  serial->putc(0xFF);
  serial->putc(0x25);
  serial->putc(0x08);//checksum
  serial->putc(0x8D);
  serial->putc(0xFE);
  serial->putc(0xEC);
  serial->putc(0xFF);
  serial->putc(0xD1);
  serial->putc(0x03);
  serial->putc(0x00);
  serial->putc(0x00);
  serial->putc(0x00);
  serial->putc(0xE7);
}

float bno085::get_z_angle(){

checksum = (buffer[0] + buffer[1] + buffer[2] + buffer[3] + buffer[4] + buffer[5] + buffer[6] + buffer[7] + buffer[8] + buffer[9] + buffer[10] + buffer[11] + buffer[12] + buffer[13] + buffer[14] + buffer[15]) & 0xFF;
  s = 0;
  if(checksum == buffer[16] && buffer[13] == 0x00 && buffer[14] == 0x00 && buffer[15] == 0x00){
    buffer_16[0] = buffer[1] |  buffer[2] << 8;
    rawanglez = (float)buffer_16[0] * DEGREE_SCALE * M_PI / 180 - init_anglez;
    s = 1;
    checksum = 0;
  }

  diff_rawanglez = pre_rawanglez - rawanglez;
  if(fabs(diff_rawanglez) >= M_PI){
    if(rawanglez < 0){
      anglez += -M_PI*2 + diff_rawanglez;
    }else{
      anglez += M_PI*2 + diff_rawanglez;
    }
  }else{
    anglez += diff_rawanglez;
  }
  pre_rawanglez = rawanglez;
  
  return anglez;
}

bool bno085::update(){
  bool flag = true;
  uint8_t get;
  uint8_t pre_get;
  static int length = 0;
  bool reada = serial->readable();

if(reada){
    //printf("reada:%d",reada);
    while(flag){
        get = serial->getc();
        if(get == pre_get && get ==  0xAA){
        flag = false;
        }
        pre_get = get;
    }

    //if(reada){
    for(int i = 0;i < 17;i++){
        buffer[i] = serial->getc();
    }
}
//}
//    while(serial->readable()){
//     get = serial->getc();
    

  //if(serial->readable()){
      //int length = serial->read((uint16_t)*buffer, sizeof(buffer));
      //int length = serial->read(buffer, 1);
      //buffer[length] = '\0';
  //}

  

//   pre_get = get;
//   }
return true;
}

bool bno085::init(){
  do{
  bool flag = true;
  while(flag){
    if(serial->getc() == 0xAA){
      if(serial->getc() == 0xAA){
        flag = false;
      }
    }
  }

  for(int i = 0; i < 17;i++){
    buffer[i] = serial->getc();
  }
  }while(!(buffer[13] == 0x00 && buffer[14] == 0x00 && buffer[15] ==0x00));

  buffer_16[0] = (uint16_t)buffer[1] | ((uint16_t)buffer[2] << 8);
  init_anglez =  (float)buffer_16[0] * DEGREE_SCALE * M_PI / 180;
//   init_anglez = rawanglez * M_PI / 180;
  return true;
}
