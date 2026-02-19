#include "mbed.h"

#define SH2_ACCELEROMETER                           0x01
#define SH2_GYROSCOPE                               0x02
#define SH2_MAGNETIC_FIELD                          0x03
#define SH2_LINEAR_ACCELERATION                     0x04
#define SH2_GRAVITY                                 0x05
#define SH2_ROTATION_VECTOR                         0x06
#define SH2_GEOMAGNETIC_ROTATION_VECTOR             0x07
#define SH2_GAME_ROTATION_VECTOR                    0x08
#define SH2_STEP_COUNTER                            0x09
#define SH2_STABILITY_CLASSIFIER                    0x0A
#define SH2_ACTIVITY_CLASSIFIER                     0x0B
#define SH2_RAW_ACCELEROMETER                       0x0E
#define SH2_RAW_GYROSCOPE                           0x0F
#define SH2_RAW_MAGNETOMETER                        0x10
#define SH2_UNCALIBRATED_GYROSCOPE                  0x13
#define SH2_UNCALIBRATED_MAGNETOMETER               0x14
#define SH2_TAP_DETECTOR                            0x19
#define SH2_SHAKE_DETECTOR                          0x1A
#define SH2_FLIP_DETECTOR                           0x1B
#define SH2_PICKUP_DETECTOR                         0x1C
#define SH2_STABILITY_DETECTOR                      0x1D
#define SH2_PERSONAL_ACTIVITY_CLASSIFIER            0x1E
#define SH2_SLEEP_DETECTOR                          0x1F
#define SH2_CIRCLE_DETECTOR                         0x20
#define SH2_HEART_RATE_MONITOR                      0x21
#define SH2_ARVR_STABILIZED_ROTATION_VECTOR         0x27
#define SH2_ARVR_STABILIZED_GAME_ROTATION_VECTOR    0x28
#define SH2_GYRO_INTEGRATED_ROTATION_VECTOR         0x29
#define SH2_GYRO_INTEGRATED_RV_PREDICTED            0x2A
#define SH2_CUSTOM_INPUT_REPORT                     0x2B

#define SCALE_Q(n) (1.0f / (1 << n))

#define M_PI 3.14259265358979323846264338327

class BNO085{
    public:
    BNO085(SPI*, PinName, PinName, PinName);

    int init();
    void Set_Feature_Command(uint8_t cmd);
    void input_report();
    bool read();
    void up();

    void update();

    float qx,qy,qz,qw;
    float anglex, angley, anglez;
    float init_anglex, init_angley, init_anglez;
    float rawanglex,rawangley,rawanglez;
    float diff_rawanglex, diff_rawangley, diff_rawanglez;
    float pre_rawanglex, pre_rawangley, pre_rawanglez;
    float a;
    

    private:
    uint8_t Header[4];
    uint8_t payload[512];
    int len;
    int read_int;
    int pre_read_int;
    bool flag_init;

    int16_t buffer_16[6];
    float float_16[6];
    // float anglex, angley, anglez;
    float real,I,J,K;
    // float qx,qy,qz,qw;

    bool init_ignore;

    SPI *pSPI;
    DigitalOut *CSpin;
    DigitalOut *RSTpin;
    DigitalIn  *INTpin;

};