#include "AMT222C.h"
#include <cstdio>

AMT222C::AMT222C(SPI *xSPI, PinName xCSBpin) {
  CSBpin = new DigitalOut(xCSBpin);
  pSPI = xSPI;
  CSBpin->write(1);
  init_done = false;
}

// SPI送信部分
int AMT222C::spi_write(int msg) {
  int msg_temp = 0;
  CSBpin->write(0);
  wait_us(3);
  msg_temp = pSPI->write(msg);
  CSBpin->write(1);
  wait_us(3);
  return (msg_temp);
}

int AMT222C::init() {

  CSBpin->write(1);
  pSPI->frequency(1000000);

  int ret = getEncount();
//   preABSposition = ABSposition;
//   printf("%d\n",preABSposition);

  init_done = true;

  return 1;
}

int AMT222C::getRawEncount() {
  int recieved;
  int recieve_count = 0;
  int error_count = 0;
  bool recieve_done = false;

  int rawValue = 0;
  bool correct_answer = false;

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
      ABSposition = rawValue >> 2 & 0x0FFF;
      //printf("ret : %d %x ", ABSposition, rawValue);
  }

  return 1;
}

int AMT222C::getEncount() {
  int recieved;
  int recieve_count = 0;
  int error_count = 0;
  bool recieve_done = false;

  int rawValue1 = 0;
  int rawValue2 = 0;
  bool correct_answer1 = false, correct_answer2 = false;

  CSBpin->write(0);
  wait_ns(2500);
  rawValue1 |= pSPI->write(0x00) << 6;
  wait_ns(2500);
  rawValue1 |= pSPI->write(0xA0) >> 2;
  wait_ns(2500);
  rawValue2 |= pSPI->write(0x00) << 8;
  wait_ns(2500);
  rawValue2 |= pSPI->write(0x00);
  wait_ns(3000);
  CSBpin->write(1);
  wait_us(40);

  // printf("P:%x R:%x\n", rawValue1, rawValue2);

  bool odd1 = false, even1 = false, odd2 = false, even2 = false;
  for (int i = 0; i < 12; i++) {
    if (i % 2) {
      odd1 ^= (bool)((rawValue1 >> i) & 0x01);
    } else {
      even1 ^= (bool)((rawValue1 >> i) & 0x01);
    }
  }
  for (int i = 0; i < 14; i++) {
    if (i % 2) {
      odd2 ^= (bool)((rawValue2 >> i) & 0x01);
    } else {
      even2 ^= (bool)((rawValue2 >> i) & 0x01);
    }
  }
  odd1 = !odd1;
  even1 = !even1;
  odd2 = !odd2;
  even2 = !even2;

  correct_answer1 = (((rawValue1 >> 13) & 0x01) == odd1) &&
                    (((rawValue1 >> 12) & 0x01) == even1);
  correct_answer2 = (((rawValue2 >> 15) & 0x01) == odd2) &&
                    (((rawValue2 >> 14) & 0x01) == even2);
//   printf("raw:%d\n",rawValue1);
//   printf("V:%d O:%d V:%d E:%d\n", (rawValue1 >> 13) & 0x01, odd1,(rawValue1 >> 12) & 0x01, even1);

  if (correct_answer1 && correct_answer2) {
    ABSposition = rawValue1 & 0x0FFF;
    rotation = rawValue2 & 0x3FFF;

    // printf("pos:%d rot:%d\n",ABSposition,rotation);

    if (rotation > 8000) {
        rotation = rotation - 16384;
    }
    rotation++;

    // if(abs(rotation - prerotation) < 5) {
    //     if(abs(int((core_rotation + rotation - prerotation) * 4095 + ABSposition - pre_encount)) < 4095) 
    //         core_rotation += (rotation - prerotation);
    // }
    // if(abs(rotation - prerotation) < 3) {
    //     core_rotation += (rotation - prerotation);
    // }
    // if(abs(rotation - prerotation) < 3) {
    //     // int temp_rotate = core_rotation + (rotation - prerotation);
    //     // if(abs(temp_rotate * 4095 + ABSposition - pre_encount) < 6000) core_rotation += (rotation - prerotation);
    //     if(abs(rotation - prerotation) < 2){
    //         core_rotation += (rotation - prerotation);
    //         prerotation2 = rotation;
    //     }
    // }
    // else if(abs(rotation - prerotation2) < 2){
    //     core_rotation += (rotation - prerotation2);
    //     prerotation2 = rotation;
    // }
    rotation_ = rotation;
    if(flag_first){
        /*
        temp_encount = (core_rotation + rotation_ - prerotation) * 4095 + ABSposition;
        delta_count = abs(temp_encount - encount);
        delta_position = abs(ABSposition - preABSposition);

        if(delta_count_lim < 1000) delta_count_lim = 1000;
        else if(delta_count_lim > 4500) delta_count_lim = 4500;

        if(delta_count > delta_count_lim){
            if(ABSposition == 0 && preABSposition < 2048 && abs(rotation_ - prerotation) == 1){
                rotation_++;
            }
            else if(ABSposition == 4095 && preABSposition >= 2048 && abs(rotation_ - prerotation) == 1){
                rotation_--;
            }
            else if(ABSposition == 0 && preABSposition >= 2048 && abs(rotation_ - prerotation) == 0){
                rotation_++;
            }
            else if(ABSposition == 4095 && preABSposition < 2048 && abs(rotation_ - prerotation) == 0){
                rotation_--;
            }
            temp_encount = (core_rotation + rotation_ - prerotation) * 4095 + ABSposition;
            delta_count = abs(temp_encount - encount);
        }
        else if(delta_count > 1000){
            if(ABSposition == 0 && preABSposition > 3000 && abs(rotation_ - prerotation) == 0){
                rotation_++;
            }
            else if(ABSposition == 4095 && preABSposition < 1000 && abs(rotation_ - prerotation) == 0){
                rotation_--;
            }
            temp_encount = (core_rotation + rotation_ - prerotation) * 4095 + ABSposition;
            delta_count = abs(temp_encount - encount);
        }

        if(flag_count_core){
            temp_encount = (core_rotation + rotation_ - prerotation2) * 4095 + ABSposition;
            delta_count = abs(temp_encount - encount);
            delta_position = abs(ABSposition - preABSposition);
            if(delta_count < delta_count_lim && abs(rotation_ - prerotation2) < 2){
                core_rotation += (rotation_ - prerotation2);
                prerotation2 = rotation_;
            }
            else if(temp_rotate_sign == 1 && preABSposition > 2048 && ABSposition <= 2048 && delta_position > 100) core_rotation++;
            else if(temp_rotate_sign == -1 && preABSposition <= 2048 && ABSposition > 2048 && delta_position > 100) core_rotation--;
            else if(abs(delta_count_lim_) < 500 && preABSposition > 3000 && ABSposition <= 1000) core_rotation++;
            else if(abs(delta_count_lim_) < 500 && preABSposition <= 1000 && ABSposition > 3000) core_rotation--;
            flag_count_core = false;
        }
        else if(delta_count < delta_count_lim){
            if(abs(rotation_ - prerotation) < 2){
                core_rotation += (rotation_ - prerotation);
                prerotation2 = rotation_;
            }
            else if(abs(rotation_ - prerotation2) < 2){
                core_rotation += (rotation_ - prerotation2);
                prerotation2 = rotation_;
            }
        }
        else{
            if(temp_rotate_sign == 1 && preABSposition > 2048 && ABSposition <= 2048 && delta_position > 100) core_rotation++;
            else if(temp_rotate_sign == -1 && preABSposition <= 2048 && ABSposition > 2048 && delta_position > 100) core_rotation--;
            else if(abs(delta_count_lim_) < 500 && preABSposition > 3000 && ABSposition <= 1000) core_rotation++;
            else if(abs(delta_count_lim_) < 500 && preABSposition <= 1000 && ABSposition > 3000) core_rotation--;
            else flag_count_core = true;
        }*/
        if(abs(rotation_ - prerotation) < 3) core_rotation += (rotation_ - prerotation);
    }
    else{
        prerotation2 = rotation;
        flag_first = true;
    }
        
    encount = core_rotation * 4095 + ABSposition;
    prerotation = rotation_;
    pre_encount = encount;
    preABSposition = ABSposition;
        // count = rawValue2 >> 12;
    // printf("R:%d P:%d\n", rotation, rawValue2);
    // printf("P:%d V1:%x\n", ABSposition, rawValue1);
    // printf("pos:%d rot:%d\n",ABSposition,rotation);
  }


  return encount;
}

int AMT222C::getEncount2(){
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

void AMT222C::updateCount(){
  if(abs((int)(preABSposition - ABSposition)) >= (int)(res * 0.75)){//3000){
    if(preABSposition > ABSposition){
      rotation++;
    }else if(preABSposition < ABSposition){
      rotation--;
    }
  }
}

int AMT222C::getAbsCount(){
    return ABSposition;
}

int AMT222C::getRotation(){
    return rotation;
}

int AMT222C::getCoreRotation(){
    return core_rotation;
}

int AMT222C::setLimDeltaCount(int count_){
    delta_count_lim_ = count_;
    delta_count_lim = abs(count_);
    if(count_ > 0) temp_rotate_sign = -1;
    else temp_rotate_sign = 1;
    return delta_count_lim;
}

/*int AMT222C::setZeroPos() {
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
}*/
