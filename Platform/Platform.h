#ifndef PLATFORM_h
#define PLATFORM_h

#include "define.h"


class Platform{
public:
    /*********** 変数宣言 ***********/

    /*********** 関数宣言 ***********/
    Platform(int dir1 = 1, int dir2 = 1, int dir3 = 1, int dir4 = 1);
    Platform(int dir1 = 1, int dir2 = 1, int dir3 = 1);
    Platform();
    void platformInit(coords);
    int platformInit_Odrive();
    void enc_init(int, int);
    coords setPosi(coords);
    double setAxisPosi(double posi, int axis);
    coords getPosi(int, int, double);
    coords getEncPosi();
    void VelocityControl(coords);
    int buf[3];
    //double refOmegaA, refOmegaB, refOmegaC;//後から追加(元々ローカル変数としてあったもの)
    void freeWheel(void);//
    void insideWheel(double vel);
    void insideWheel_balance(double vel, double PIDcmd);
    void insideWheel_balance2(double vel0, double vel1, double vel2, double vel3, double PIDcmd);
    void frontWheel_forward(double vel);
    void backWheel_forward(double vel);
    void frontWheel_backward(double vel);
    void backWheel_backward(double vel);
    void get_Current();

    double refV_x_cmd;//モータの速度を一定にして渡したいとき，同期したい

private:
    coords Posi;
    coords enc_posi;
    int preEncX, preEncY;
    double pre_angle_rad;
    bool init_done;
    int rotateDir[4];
};

#endif