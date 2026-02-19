#include "ODrive.h"

ODrive::ODrive(CAN* xCANIF, int xCANID)
{
    pCANIF = xCANIF;
    CANID = xCANID;

    initDone = false;
    currentState = STATE_NONE;
}

ODrive::~ODrive()
{
}

int ODrive::init(int mode){
     printf("odrive init\n");
    if(initStep == 0){
        if(getEncoderError() != 0 || getMotorError() != 0){
            clearError();
             printf("ID%d clear err\n", CANID);
        }
        wait_us(500);
        if(mode == 1){
            printf("init_done");
            initDone = true;
            return 1;
        }
        encoderIndexSearch();
        printf("encIndex");
        wait_us(500);
        initStep++; 
    }
    else{
        printf("else");
        handle = pCANIF->filter(CANID << 5, 0x7E0, CANStandard, CANID);
        while(!pCANIF->read(heartbeatMsg, handle)){
            printf("A\n");
            wait_us(100);
        }
         printf("AXIS:%x, ID:%x, data4:%x\n", CANID, heartbeatMsg.id, heartbeatMsg.data[4]);
        if(heartbeatMsg.id == (CANID << 5 | CMD_HEARTBEAT)){
            if(initStep == 1 && (heartbeatMsg.data[4] == STATE_ENCODER_INDEX_SEARCH)){
                initStep++;
            }else if(initStep == 2){
                if(heartbeatMsg.data[4] == STATE_ENCODER_INDEX_SEARCH){
                    timeoutCounter = 0;
                }else if(heartbeatMsg.data[4] == STATE_IDLE){
                    initStep = 0;
                    timeoutCounter = 0;
                    initDone = true;
                    return 1;
                }
            }
        }
        timeoutCounter++;
        if(timeoutCounter >= 5){
            initStep = 0;
            timeoutCounter = 0;
            return -1;
        }
    }
    return 0;
}

int ODrive::encoderIndexSearch(){
    CANMessage recvMsgs;
    uint8_t buf[8] = {0};
    buf[0] = STATE_ENCODER_INDEX_SEARCH;
    int ret = pCANIF->write(CANMessage((CANID << 5 | CMD_SET_AXIS_REQUESTED_STATE), buf));

    if(ret){
        return 1;
    }
    return 0;
}

int ODrive::fullCalibration(){
    uint8_t buf[8] = {0};
    buf[0] = STATE_FULL_CALIBRATION;
    int ret = pCANIF->write(CANMessage((CANID << 5 | CMD_SET_AXIS_REQUESTED_STATE), buf));
    wait_us(250);
    return ret;
}

int ODrive::clearError(){
    uint8_t buf[8] = {0};
    int ret = pCANIF->write(CANMessage((CANID << 5 | CMD_CLEAR_ERROR), buf));
    wait_us(250);
    return ret;
}

int ODrive::servoON(){
    uint8_t buf[8] = {0};
    buf[0] = STATE_CLOSED_LOOP_CONTROL;
    int ret = pCANIF->write(CANMessage((CANID << 5 | CMD_SET_AXIS_REQUESTED_STATE), buf));
    wait_us(250);
    
    return ret;
}
int ODrive::servoOFF(){
    uint8_t buf[8] = {0};
    buf[0] = STATE_IDLE;
    int ret = pCANIF->write(CANMessage((CANID << 5 | CMD_SET_AXIS_REQUESTED_STATE), buf));
    wait_us(250);
    return ret;
}

int ODrive::getMotorError(){
    CANMessage recvMsgs;
    uint8_t buf[8] = {0};
    handle = pCANIF->filter(CANID << 5, 0x7E0, CANStandard, CANID);
    unsigned int sendID = CANID << 5 | CMD_GET_MOTOR_ERROR;
    int ret = pCANIF->write(CANMessage(sendID, buf, 8, CANRemote, CANStandard));

    if(ret){
        int count = 0;
        do{
            int time_out = 0;
            while(!pCANIF->read(recvMsgs, handle)){
                wait_us(100);
                time_out++;
                if(time_out > 100){
                    return -1;
                }
            }
            //printf("ID:%x, length:%x\n", recvMsgs.id, recvMsgs.len);
            if((recvMsgs.id & 0x7E0) == CANID << 5){
                if(recvMsgs.id == (CANID << 5 | CMD_GET_MOTOR_ERROR)){
                    int err1 = recvMsgs.data[0] | recvMsgs.data[1]<<8 | recvMsgs.data[2]<<16 | recvMsgs.data[3]<<24;
                    int err2 = recvMsgs.data[4] | recvMsgs.data[5]<<8 | recvMsgs.data[6]<<16 | recvMsgs.data[7]<<24;
                    // printf("ID%x Motor: %x, %x\n", CANID, err1, err2);
                    return err1 ;
                // }else{
                //     count++;
                }
            }
            count++;
        }while(count <= 10);
        // printf("Motor recv err\n");
    }
    return 0;
}

int ODrive::getEncoderError(){
    CANMessage recvMsgs;
    uint8_t buf[8] = {0};
    handle = pCANIF->filter(CANID << 5, 0x7E0, CANStandard, CANID);
    unsigned int sendID = CANID << 5 | CMD_GET_ENCODER_ERROR;
    int ret = pCANIF->write(CANMessage(sendID, buf, 8, CANRemote, CANStandard));

    if(ret){
        int count = 0;
        do{
            int time_out = 0;
            while(!pCANIF->read(recvMsgs, handle)){
                //printf("A\n");
                wait_us(100);
                time_out++;
                if(time_out > 100){
                    return -1;
                }
            }
            if((recvMsgs.id & 0x7E0) == CANID << 5){
                if(recvMsgs.id == sendID){
                    int err = recvMsgs.data[0] | recvMsgs.data[1]<<8 | recvMsgs.data[2]<<16 | recvMsgs.data[3]<<24;
                    //printf("ID%x Enc: %x\n", CANID, err);
                    return err;
                // }else{
                //     count++;
                }
            }
            count++;
        }while(count <= 10);
        //printf("Enc recv err\n");
    }
    return -1;
}

bool ODrive::getIQ(){
    CANMessage recvMsgs;
    uint8_t buf[8] = {0};
    handle = pCANIF->filter(CANID << 5, 0x7E0, CANStandard, CANID);
    unsigned int sendID = CANID << 5 | CMD_GET_IQ;
    int ret = pCANIF->write(CANMessage(sendID, buf, 8, CANRemote, CANStandard));

    if(ret){
        int count = 0;
        do{
            int time_out = 0;
            while(!pCANIF->read(recvMsgs, handle)){
                //printf("A\n");
                wait_us(100);
                time_out++;
                if(time_out > 100){
                    return -1;
                }
            }
            if((recvMsgs.id & 0x7E0) == CANID << 5){
                if(recvMsgs.id == sendID){
                    // iq_set = recvMsgs.data[0] | recvMsgs.data[1]<<8 | recvMsgs.data[2]<<16 | recvMsgs.data[3]<<24;
                    // iq_meas = recvMsgs.data[4] | recvMsgs.data[5]<<8 | recvMsgs.data[6]<<16 | recvMsgs.data[7]<<24;
                    //std::memcpy(&iq_set, &recvMsgs.data[0], 4);
                    std::memcpy(&iq_meas, &recvMsgs.data[4], 4);
                    //printf("ID%x iq_set:%f iq_meas:%f\n", CANID, iq_set,iq_meas);
                    return true;
                // }else{
                //     count++;
                }
            }
            count++;
        }while(count <= 10);
        //printf("Enc recv err\n");
    }
    return false;
}

int ODrive::getState(){
    wait_us(250);
    handle = pCANIF->filter(CANID << 5, 0x7E0, CANStandard, CANID);
    while(!pCANIF->read(heartbeatMsg, handle)){
        wait_us(100);
    }
    // printf("AXIS:%x, ID:%x, data4:%x\n", CANID, heartbeatMsg.id, heartbeatMsg.data[4]);
    if(heartbeatMsg.id == (CANID << 5 | CMD_HEARTBEAT)){
        return heartbeatMsg.data[4];
    }
    else{
        return -1;
    }
}

int ODrive::setVel(float vel){
    uint8_t buf[8] = {0};
    std::memcpy(&buf[0], &vel, sizeof(vel));
    int ret = pCANIF->write(CANMessage((CANID << 5 | CMD_SET_INPUT_VEL), buf));
    wait_us(250);
    return ret;
}

float ODrive::getVel(){
    CANMessage recvMsgs;
    uint8_t buf[8] = {0};
    handle = pCANIF->filter(CANID << 5, 0x7E0, CANStandard, CANID);
    unsigned int sendID = CANID << 5 | CMD_GET_ENCODER_ESTIMATES;
    int ret = pCANIF->write(CANMessage(sendID, buf, 8, CANRemote, CANStandard));

    if(ret){
        int count = 0;
        do{
            int time_out = 0;
            while(!pCANIF->read(recvMsgs, handle)){
                wait_us(100);
                time_out++;
                if(time_out > 100){
                    return -1;
                }
            }
            if((recvMsgs.id & 0x7E0) == CANID << 5){
                if(recvMsgs.id == sendID){
                    float cpr;
                    std::memcpy(&cpr, &recvMsgs.data[4], 4);
                    //std::memcpy(&enc, &recvMsgs.data[0], 4);
                    return cpr;
                // }else{
                //     count++;
                }
            }
            count++;
        }while(count <= 10);
        // printf("Vel recv err\n");
    }
    return -1;
}

int ODrive::getEncCount(){
    CANMessage recvMsgs;
    uint8_t buf[8] = {0};
    handle = pCANIF->filter(CANID << 5, 0x7E0, CANStandard, CANID);
    unsigned int sendID = CANID << 5 | CMD_GET_ENCODER_COUNT;
    int ret = pCANIF->write(CANMessage(sendID, buf, 8, CANRemote, CANStandard));

    if(ret){
        int count = 0;
        do{
            int time_out = 0;
            while(!pCANIF->read(recvMsgs, handle)){
                wait_us(100);
                time_out++;
                if(time_out > 100){
                    return -1;
                }
            }
            if((recvMsgs.id & 0x7E0) == CANID << 5){
                if(recvMsgs.id == sendID){
                    int enc;
                    std::memcpy(&enc, &recvMsgs.data[0], 4);
                    return enc;
                // }else{
                //     count++;
                }
            }
            count++;
        }while(count <= 10);
        // printf("Vel recv err\n");
    }
    return -1;
}

bool ODrive::getInitState(){
    return initDone;
}

void ODrive::clear(){ initDone = false; }