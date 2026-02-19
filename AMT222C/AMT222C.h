#ifndef _AMT222C_H
#define _AMT222C_H
 
#include "mbed.h"
#include "define.h"
//#include <SPI.h>
 
class AMT222C{
  public:
  AMT222C(SPI*, PinName);
 
  int spi_write(int msg);
  int init();
  int getEncount();
  int getEncount2();
  int setZeroPos();
  int getRawEncount();
  int getAbsCount();
  int getRotation();
  int getCoreRotation();
  void updateCount();
  int setLimDeltaCount(int);
 
  private:
  bool init_done;
  bool angle_error;
  //unsigned char CSBpin;
  const int res = 0x0FFF;//4096;
  int ABSposition;  // rawdata
  int preABSposition;
  int delta_position;
  int rotation = 0;
  int rotation_ = 0;
  int prerotation = 0;
  int prerotation2 = 0;
  int next_rotation = 0;
  int core_rotation = 0;
  int core_rotation_ = 0;
  int countR = 0;
  int encount;        // 渡す値
  int pre_encount;
  int temp_encount;
  int delta_count;
  int delta_count_lim = 3000;
  int delta_count_lim_;
  int temp_rotate_sign;
  int temp[2];
  bool flag_count_core;
  bool flag_first;
 
  SPI *pSPI;
  DigitalOut *CSBpin;
 
};
 
#endif
 
            