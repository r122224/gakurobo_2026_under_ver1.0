#ifndef BNO085_UART_RVC_H
#define BNO085_UART_RVC_H

#include "mbed.h"

#define MILLI_G_TO_MS2 0.0098067 ///< Scalar to convert milli-gs to m/s^2
#define DEGREE_SCALE 0.01        ///< To convert the degree values

class bno085{

public:
  bno085(Serial*);
  void get_sensor_data();
  float get_z_angle();
  bool update();
  bool init();
  

  uint8_t buffer[17];
  uint8_t pre_buffer[17];
  uint8_t buffer_n[19];
  int16_t buffer_16[6];
  float num[6];
  float pre_num[6];
  float anglex, angley, anglez;
  float rawanglex,rawangley,rawanglez;
  float init_anglex, init_angley, init_anglez;
  float diff_rawanglex, diff_rawangley, diff_rawanglez;
  float pre_rawanglex, pre_rawangley, pre_rawanglez;
  float datax,datay,dataz;
  uint16_t checksum;
  bool s;

private:
  Serial* serial;
  float accelx,accely,accelz;


};

#endif