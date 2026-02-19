
#ifndef ODRIVE_H
#define ODRIVE_H

#include "mbed.h"
#include "define.h"

#define CMD_HEARTBEAT                   0x001
#define CMD_GET_MOTOR_ERROR             0x003
#define CMD_GET_ENCODER_ERROR           0x004
#define CMD_SET_AXIS_REQUESTED_STATE    0x007
#define CMD_GET_ENCODER_ESTIMATES       0x009
#define CMD_GET_ENCODER_COUNT           0x00A
#define CMD_SET_CONTROL_MODE            0x00B
#define CMD_SET_INPUT_POS               0x00C
#define CMD_SET_INPUT_VEL               0x00D
#define CMD_SET_INPUT_TORQUE            0x00E
#define CMD_SET_LIMITS                  0x00F
#define CMD_GET_IQ                      0x014
#define CMD_CLEAR_ERROR                 0x018
#define CMD_SET_VEL_GAINS               0x01B

#define STATE_NONE                          0x00
#define STATE_IDLE                          0x01
#define STATE_STARTUP_SEQUENCE              0x02
#define STATE_FULL_CALIBRATION              0x03
#define STATE_MOTOR_CALIBRATION             0x04
#define STATE_ENCODER_INDEX_SEARCH          0x06
#define STATE_ENCODER_OFFSET_CALIBRATION    0x07
#define STATE_CLOSED_LOOP_CONTROL           0x08

class ODrive
{
public:
    ODrive(CAN* xCANIF, int xCANID); // コンストラクタ
    virtual ~ODrive();

    int init(int);
    int encoderIndexSearch();
    int fullCalibration();
    int clearError();

    int servoON();
    int servoOFF();

    int getMotorError();
    int getEncoderError();

    bool getIQ();
    float iq_set,iq_meas;

    int getState();
    bool getInitState();

    int setVel(float vel);
    float getVel();
    int getEncCount();
    void clear();

private:
    int CANID;
    CAN *pCANIF;
    int handle;
    bool initDone;
    
    CANMessage heartbeatMsg;    
    int currentState;
    int initStep = 0, timeoutCounter = 0;
};

#endif