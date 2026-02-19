#include "AMT22.h"

AMT203V::AMT203V(SPI* xSPI, PinName xCSBpin){
  CSBpin = new DigitalOut(xCSBpin);
  pSPI = xSPI;
  CSBpin->write(1);
  init_done = false;
}

// SPI送信部分
int AMT203V::spi_write(int msg){
  int msg_temp = 0;
  CSBpin->write(0);
  wait_us(3);
  msg_temp = pSPI->write(msg);
  CSBpin->write(1);
  wait_us(3);
  return(msg_temp);
}

int AMT203V::init(){

  CSBpin->write(1);
//   pSPI->format(8, 0);//変更
  pSPI->frequency(1000000);

//   int ret = getRawEncount();
  int ret = 1;
  preABSposition = ABSposition;
  
  if(ret == -1) return -1;
  else init_done = true;
  
  return 1;
}

int AMT203V::getRawEncount(){
  int recieved;
  int recieve_count = 0;
  int error_count = 0;
  bool recieve_done = false;

  unsigned int  rawValue = 0;
  bool correct_answer = false;

//   pSPI->format(8, 0);      // 変更
//   pSPI->frequency(1000000);//変更
  
  CSBpin->write(0);
  wait_ns(2500);
  rawValue |= pSPI->write(0x00) << 8;
  wait_ns(2500);
  rawValue |= pSPI->write(0x00);
  wait_ns(3000);
  CSBpin->write(1);
  wait_us(40);
    
  bool odd, even;
  for(int i = 0; i < 14; i++){
      if(i % 2){
          odd ^= (rawValue >> i) & 0x01;
      }else{
          even ^= (rawValue >> i) & 0x01;
      }
  }
  odd = !odd;
  even = !even;

  correct_answer = (((rawValue >> 15) & 0x01) == odd) && (((rawValue >> 14) & 0x01) == even);

  if(correct_answer){
      ABSposition = rawValue & 0x3FFF;
      //printf("ret : %d %x ", ABSposition, rawValue);
  }/*else{
      printf("xxx : %d %x\n", rawValue, rawValue);
  }*/
  
  return 1;
}

int AMT203V::getEncount(){
  if(init_done){
    getRawEncount();
    updateCount();
    
    encount = rotation * res + ABSposition;
    preABSposition = ABSposition;

    //printf("  encount %d abs position %d       ", encount, ABSposition);
  }
  else{
    return -1;
  }
  return encount;
}

// 2,1,0の次に4095ではなくマイナスの値になるように，4093,4094,4095の次に0にならず大きな値になるようにしてる関数
void AMT203V::updateCount(){
  if(abs((int)(preABSposition - ABSposition)) >= (int)(res * 0.75)){//3000){
    if(preABSposition > ABSposition){
      rotation++;
    }else if(preABSposition < ABSposition){
      rotation--;
    }
  }
}

int AMT203V::getAbsCount(){
    return ABSposition;
}

int AMT203V::setZeroPos()
{
  int response;
  int count = 0;

  CSBpin->write(0);
  wait_us(3);
  response |= pSPI->write(0x00) << 8;
  wait_us(3);
  response |= pSPI->write(0x70);
  wait_us(3);
  CSBpin->write(1);

  printf("response %x\n", response);

  getRawEncount();
  preABSposition = ABSposition;

  return 1;
}