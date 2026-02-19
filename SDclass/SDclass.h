#ifndef SDCLASS_h
#define SDCLASS_h

#include "mbed.h"
//#include <SD.h>
#include "define.h"
#include "FATFileSystem.h"
#include "SDBlockDevice.h"
#include <string>

class mySDclass{
public:
    mySDclass();
    
    bool SD_enable = false;
    string logFileName;
    FILE *dataFile;
    
    int init();
    int make_logfile();
    int write_logdata(string);
    void close_logdata();

    int path_read(int, double*, double*, double*, double*, int*, double*, double*);

    double str2double(char*, int);
    int str2uint(char* str, int num);
};

#endif