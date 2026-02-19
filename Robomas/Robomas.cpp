/*

*/
#include "Robomas.h"

int mc = 4;//モータ数

Robomas::Robomas(PinName tx, PinName rx, int baudrate)
    : robomas(tx, rx, baudrate), rawEnc(0), rpm(0), torqueCurrent(0) {
    for (int i = 0; i < mc; i++) {
        robomas_cmd[i] = 0;
        robomas_mode[i] = CURRENT_CONTROL;//最大10000 //20000
        robomas_flag[i] = false;
    }
}

void Robomas::cmd(int id,int rocmd){
    robomas_cmd[id] = rocmd; 
    // printf("%d\r\n",robomas_cmd[id]);  
}

bool Robomas::sendCmd() {
    uint8_t checksum = 0;
    uint8_t buf[15];
    uint8_t receive[4] = {0x00, 0x00, 0x00, 0x00};

    for (int i = 0; i < mc; i++) {
        if (robomas_flag[i])
            receive[i] = 0x80;
    }

    buf[0] = (robomas_cmd[0] >> 8 & 0xFF);
    buf[1] = (robomas_cmd[0] & 0xFF);
    buf[2] = (robomas_cmd[1] >> 8 & 0xFF);
    buf[3] = (robomas_cmd[1] & 0xFF);
    buf[4] = (robomas_cmd[2] >> 8 & 0xFF);
    buf[5] = (robomas_cmd[2] & 0xFF);
    buf[6] = (robomas_cmd[3] >> 8 & 0xFF);
    buf[7] = (robomas_cmd[3] & 0xFF);
    // buf[8] = (robomas_cmd[4] >> 8 & 0xFF);
    // buf[9] = (robomas_cmd[4] & 0xFF);
    buf[8] = robomas_mode[0] | receive[0];
    buf[9] = robomas_mode[1] | receive[1];
    buf[10] = robomas_mode[2] | receive[2];
    buf[11] = robomas_mode[3] | receive[3];
    // buf[14] = robomas_mode[4] | receive[4];

    // for (int i = 0; i < 6; i++) {
    //     robomas.putc(buf[i]);
    //     checksum ^= buf[i];
    // }
    for (int i = 0; i < 12; i++) {
        robomas.putc(buf[i]);
        checksum ^= buf[i];
    }
    robomas.putc(checksum);
    robomas.putc('\n');

    return true;
}

bool Robomas::receiveData() {
    char c;
    char checksum = 0;
    int readCount = 0;
    char buf[100];
    bool comm_check = false;

    while (robomas.readable()) {
        c = robomas.getc();

        if (c == '\n' && readCount == 9) { // Check for end character
            if (buf[6] == checksum) {      // Verify checksum
                rawEnc = (uint16_t)((uint16_t)buf[0] << 8) | (uint16_t)buf[1]; // Absolute position
                rpm = (int16_t)((uint16_t)buf[2] << 8) | (uint16_t)buf[3];    // Rotation speed
                torqueCurrent = (int16_t)((uint16_t)buf[4] << 8) | (uint16_t)buf[5]; // Torque

                comm_check = true;
            } else {
                comm_check = false;
            }
            break;
        } else {
            buf[readCount] = c; // Temporarily store data
            if (readCount < 9)
                checksum ^= buf[readCount]; // Calculate checksum
        }

        readCount++;
        if (readCount > 99)
            readCount = 99; // Prevent overflow
    }

    while (robomas.readable())
        robomas.getc();

    return comm_check;
}

int Robomas::getRawEnc(){
    return rawEnc;
}
int Robomas::getRpm(){
    return rpm;
}
int Robomas::getTorqueCurrent(){
    return torqueCurrent;
}