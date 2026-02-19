#include "UDPSocketComm.h"

UDP::UDP(const char* my_addres,uint16_t my_port){
    MY_ADDRESS = my_addres;
    MY_PORT = my_port;
}

bool UDP::init(const char* addres,uint16_t port){
    ADDRESS = addres;
    PORT = port;

    net.set_dhcp(false);
    net.set_network(MY_ADDRESS, "255.255.255.0", "192.168.1.10");
    if(0 != net.connect()) {
        printf("Error connecting\n");
        return false;
    }
    else{
        printf("Succeed connecting\n");
    }
 
    // Show the network address
    const char *ip = net.get_ip_address();
    printf("IP address is: %s\n", ip ? ip : "No IP");
    
    socketadd.set_ip_address(ADDRESS);
    socketadd.set_port(PORT);
    socket.open(&net);
    socket.set_blocking(false);
    // socket.set_timeout(5);

    // while(socket.bind(PORT) != 0) thread_sleep_for(1);
    printf("Connected to port at %d\n",PORT);

    return true;
}

int UDP::upData(char *rbuf, unsigned int size){
    int n = socket.recvfrom(&socketadd,rbuf, size);
    if(n>0){
        return n;
    }
    else{
        return -1;
    }
}

int UDP::sendData(char *buf, unsigned int size){
    int n = socket.sendto(socketadd, buf, size);
    if(n>0){
        return n;
    }
    else{
        return -1;
    }
}

void UDP::close(){
    socket.close();
    net.disconnect();
}