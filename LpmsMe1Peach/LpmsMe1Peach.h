#ifndef LPMS_ME1_h
#define LPMS_ME1_h

#include "mbed.h"
//#include "RawSerial.h"
//#include <Arduino.h>
#include "RZ_A1H.h"

class LpmsMe1{
public:
    //LpmsMe1(Serial*);
    LpmsMe1(Serial*);
    void goto_command_mode();
    void set_transmit_data();
    void set_filter_mode();
    void set_offset();
    void reset_orientation_offset();
    void get_sensor_data();
    float get_z_angle();
    float get_y_angle();
    float get_x_angle();
    int recv_proc(int);
    int init();

    int tm = 0;
    int lpms_state[1000];
    int lpms_timer[1000];

private:
    Serial* serial;
    unsigned char buffer[25];
    float anglex, angley, anglez;
    float pre_rawanglex, pre_rawangley, pre_rawanglez;
    bool init_ignore;
    bool init_done; // 初期化が終わったかどうか
};

#endif

