#ifndef UDPSOCKET_COMM_H
#define UDPSOCKET_COMM_H

#include "mbed.h"
#include "EthernetInterface.h"

class UDP{

public:
UDP(const char*,uint16_t);
bool init(const char*,uint16_t);
int upData(char *rbuf, unsigned int);
int sendData(char *buf, unsigned int);
void close();

private:
EthernetInterface net;
UDPSocket socket;
SocketAddress socketadd;

const char* MY_ADDRESS;// 自分
uint16_t MY_PORT;
const char* ADDRESS;// 相手
uint16_t PORT;
};

#endif