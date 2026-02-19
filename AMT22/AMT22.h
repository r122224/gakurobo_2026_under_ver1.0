#ifndef _AMT22_H
#define _AMT22_H

#include "mbed.h"
#include "define.h"
//#include <SPI.h>

class AMT203V{
  public:
  AMT203V(SPI*, PinName);

  int spi_write(int msg);
  int init();
  int getEncount();
  void updateCount();
  int setZeroPos();
  int getRawEncount();
  int getAbsCount();

  unsigned int ABSposition;  // rawdata
  unsigned int preABSposition;

  private:
  bool init_done;
  //unsigned char CSBpin;
  const int res = 0x3FFF;//4096;
  
  int rotation = 0;
  int encount;        // 渡す値
  int temp[2];

  SPI *pSPI;
  DigitalOut *CSBpin;

};

#endif