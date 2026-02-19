#include "BNO085_SPI.h"
#include <cstdio>

BNO085::BNO085(SPI* xSPI, PinName xCSpin, PinName xINTpin, PinName xRSTpin){ // CS , H_INTN , RST
    CSpin = new DigitalOut(xCSpin);     // CS
    INTpin = new DigitalIn(xINTpin);     // H_INTN
    RSTpin = new DigitalOut(xRSTpin);    // RST
    pSPI = xSPI;

    init_ignore = true;
}

int BNO085::init(){
    anglex = 0.0;
    angley = 0.0;
    anglez = 0.0;
    pre_rawanglex = 0.0;
    pre_rawangley = 0.0;
    pre_rawanglez = 0.0;

    CSpin->write(1);
    RSTpin->write(0);
    wait_us(10);
    RSTpin->write(1);
    wait_us(300);
    pSPI->format(8, 3);
    pSPI->frequency(1000000);

    input_report();

    flag_init = true;

    while (flag_init) {
        read_int = INTpin->read();
        if(read_int == 0 && pre_read_int == 1)
            flag_init = false;
        pre_read_int = read_int;
    }

    // Set_Feature_Command(SH2_GAME_ROTATION_VECTOR);
    Set_Feature_Command(SH2_ARVR_STABILIZED_GAME_ROTATION_VECTOR);

    flag_init = true;

    while (flag_init) {
        read_int = INTpin->read();
        if(read_int == 0 && pre_read_int == 1){
            input_report();
            flag_init = false;
        }
        pre_read_int = read_int;
        // if(INTpin->read() == 0)
        //     flag_init = false;
    }
    wait_ms(1000);
    input_report();
    input_report();

    // init_anglez = qz;
    init_ignore = false;

    return 1;
}

void BNO085::Set_Feature_Command(uint8_t cmd){
    
    CSpin->write(0);

    //Header
    pSPI->write(0x15);  //Length LSB
    pSPI->write(0x00);  //Length MSB
    pSPI->write(0x02);  //Channel
    pSPI->write(0x00);  //SeqNum

    pSPI->write(0xFD);  //Report ID
    pSPI->write(cmd);   //Feature Report ID
    pSPI->write(0x00);  //Feature flags

    //Change sensitivity
    pSPI->write(0x00);
    pSPI->write(0x00);

    //Report Interval
    //Report Interval=1,000,000/出力周波数(Hz)
    // pSPI->write(0x10); //??4.5ms?
    // pSPI->write(0x17);
    pSPI->write(0x98); //??10ms?
    pSPI->write(0x3A);
    // pSPI->write(0x20); //??20ms?
    // pSPI->write(0x4E);
    pSPI->write(0x00);
    pSPI->write(0x00);

    //Batch Interval
    pSPI->write(0x00);
    pSPI->write(0x00);
    pSPI->write(0x00);
    pSPI->write(0x00);

    //Sensor-specific
    pSPI->write(0x00);
    pSPI->write(0x00);
    pSPI->write(0x00);
    pSPI->write(0x00);

    CSpin->write(1);
}

void BNO085::input_report(){
    flag_init = true;

    while (flag_init) {
        read_int = INTpin->read();
        // printf("%d",read_int);
        if(read_int == 0 && pre_read_int == 1)
            flag_init = false;
        pre_read_int = read_int;
        // if(INTpin->read() == 0)
        //     flag_init = false;
    }

    CSpin->write(0);

    Header[0] = pSPI->write(0x00);
    Header[1] = pSPI->write(0x00);
    Header[2] = pSPI->write(0x00);
    Header[3] = pSPI->write(0x00);

    len = (Header[0] | ((Header[1] & 0x7F) << 8)) - 4;

    for (int i = 0; i < len; i++) {
        payload[i] = pSPI->write(0x00);
    }

    CSpin->write(1);

    if(payload[5] == SH2_ACCELEROMETER){
        buffer_16[0] = payload[9]  |  payload[10] << 8;
        buffer_16[1] = payload[11]  |  payload[12] << 8;
        buffer_16[2] = payload[13]  |  payload[14] << 8;
        // anglex = buffer_16[0] * SCALE_Q(8);
        // angley = buffer_16[1] * SCALE_Q(8);
        // anglez = buffer_16[2] * SCALE_Q(8);
        printf("x:%lf , y;%lf , z:%lf\n",anglex,angley,anglez);
    }else if(payload[5] == SH2_GAME_ROTATION_VECTOR){
        buffer_16[0] = payload[9]  |  payload[10] << 8;
        buffer_16[1] = payload[11]  |  payload[12] << 8;
        buffer_16[2] = payload[13]  |  payload[14] << 8;
        buffer_16[3] = payload[15]  |  payload[16] << 8;
        real = buffer_16[0] * SCALE_Q(14);
        I = buffer_16[1] * SCALE_Q(14);
        J = buffer_16[2] * SCALE_Q(14);
        K = buffer_16[3] * SCALE_Q(14);
        printf("real:%lf , I:%lf , J:%lf , K:%lf\n",real,I,J,K);
    }else if(payload[5] == SH2_ARVR_STABILIZED_GAME_ROTATION_VECTOR){
        buffer_16[0] = payload[9]  |  payload[10] << 8;
        buffer_16[1] = payload[11]  |  payload[12] << 8;
        buffer_16[2] = payload[13]  |  payload[14] << 8;
        buffer_16[3] = payload[15]  |  payload[16] << 8;
        qx = buffer_16[0] * SCALE_Q(14);
        qy = buffer_16[1] * SCALE_Q(14);
        qz = buffer_16[2] * SCALE_Q(14);
        qw = buffer_16[3] * SCALE_Q(14);
        a = atan2f(2.0f * (qw * qz + qx * qy),1.0f - 2.0f * (qy * qy + qz * qz));
        rawanglez = atan2f(2.0f * (qw * qz + qx * qy),1.0f - 2.0f * (qy * qy + qz * qz));
        // printf("x:%lf , y:%lf , z:%lf , w:%lf\n",qx,qy,qz,qw);
        // rawanglez = qz - init_anglez;
        // diff_rawanglez = pre_rawanglez - rawanglez;
        // if(fabs(diff_rawanglez) >= 0.5){
        //     if(rawanglez < 0){
        //     anglez += -1.0 + diff_rawanglez;
        //     }else{
        //     anglez += 1.0 + diff_rawanglez;
        //     }
        // }else{
        //     anglez += diff_rawanglez;
        // }
        // pre_rawanglez = rawanglez;

    }

    float diff_rawanglez;
    if(init_ignore) {
      diff_rawanglez = 0.0;
    }else{
      diff_rawanglez = rawanglez - pre_rawanglez;
    }

    if(fabs(diff_rawanglez) >= 3.0){ 
      if(rawanglez < 0){ //+から-へ回ったとき
        anglez += M_PI*2 + diff_rawanglez;
      }else{ // -から+へ回ったとき
        anglez += -M_PI*2 + diff_rawanglez;
      }
    }else{
      anglez += diff_rawanglez;
    }
    pre_rawanglez = rawanglez;

    // rawanglez = qz - init_anglez;
    // diff_rawanglez = rawanglez - pre_rawanglez;

    // if(fabs(diff_rawanglez) >= 2.0){
    //     if(rawanglez < 0){
    //         anglez += 2.0 + diff_rawanglez;
    //     }else{
    //         anglez += -2.0 + diff_rawanglez;
    //     }
    // }else{
    //     anglez += diff_rawanglez;
    // }
    // pre_rawanglez = rawanglez;

}

bool BNO085::read(){
    return INTpin->read();
}

void BNO085::up(){
    CSpin->write(0);

    Header[0] = pSPI->write(0x00);
    Header[1] = pSPI->write(0x00);
    Header[2] = pSPI->write(0x00);
    Header[3] = pSPI->write(0x00);

    len = (Header[0] | ((Header[1] & 0x7F) << 8)) - 4;

    for (int i = 0; i < len; i++) {
        payload[i] = pSPI->write(0x00);
    }

    CSpin->write(1);

    if(payload[5] == SH2_ACCELEROMETER){
        buffer_16[0] = payload[9]  |  payload[10] << 8;
        buffer_16[1] = payload[11]  |  payload[12] << 8;
        buffer_16[2] = payload[13]  |  payload[14] << 8;
        // anglex = buffer_16[0] * SCALE_Q(8);
        // angley = buffer_16[1] * SCALE_Q(8);
        // anglez = buffer_16[2] * SCALE_Q(8);
        printf("x:%lf , y;%lf , z:%lf\n",anglex,angley,anglez);
    }else if(payload[5] == SH2_GAME_ROTATION_VECTOR){
        buffer_16[0] = payload[9]  |  payload[10] << 8;
        buffer_16[1] = payload[11]  |  payload[12] << 8;
        buffer_16[2] = payload[13]  |  payload[14] << 8;
        buffer_16[3] = payload[15]  |  payload[16] << 8;
        real = buffer_16[0] * SCALE_Q(14);
        I = buffer_16[1] * SCALE_Q(14);
        J = buffer_16[2] * SCALE_Q(14);
        K = buffer_16[3] * SCALE_Q(14);
        printf("real:%lf , I:%lf , J:%lf , K:%lf\n",real,I,J,K);
    }else if(payload[5] == SH2_ARVR_STABILIZED_GAME_ROTATION_VECTOR){
        buffer_16[0] = payload[9]  |  payload[10] << 8;
        buffer_16[1] = payload[11]  |  payload[12] << 8;
        buffer_16[2] = payload[13]  |  payload[14] << 8;
        buffer_16[3] = payload[15]  |  payload[16] << 8;
        qx = buffer_16[0] * SCALE_Q(14);
        qy = buffer_16[1] * SCALE_Q(14);
        qz = buffer_16[2] * SCALE_Q(14);
        qw = buffer_16[3] * SCALE_Q(14);

        rawanglez = atan2f(2.0f * (qw * qz + qx * qy),1.0f - 2.0f * (qy * qy + qz * qz));
    }

    float diff_rawanglez;
    if(init_ignore) {
      diff_rawanglez = 0.0;
    }else{
      diff_rawanglez = rawanglez - pre_rawanglez;
    }

    if(fabs(diff_rawanglez) >= 3.0){ 
      if(rawanglez < 0){ //+から-へ回ったとき
        anglez += M_PI*2 + diff_rawanglez;
      }else{ // -から+へ回ったとき
        anglez += -M_PI*2 + diff_rawanglez;
      }
    }else{
      anglez += diff_rawanglez;
    }
    pre_rawanglez = rawanglez;
}

void BNO085::update(){
    read_int = read();

    if(read_int == 0 && pre_read_int == 1)
        up();
    
    pre_read_int = read_int;
}