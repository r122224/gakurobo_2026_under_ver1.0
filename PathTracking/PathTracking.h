#ifndef PATHTRACKING_h
#define PATHTRACKING_h

#include "mbed.h"
#include "PIDclass.h"
#include "Filter.h"
#include "define.h"
#include <vector>  // std::vector
#include <utility> // std::pair, std::make_pair

#define PATHNUM 50
#define POINTNUM 200

#define FOLLOW_TANGENT  ( 0 )
#define FOLLOW_COMMAND  ( 1 )
#define POSITION_PID    ( 2 )
#define POSI_PID    ( 3 )

#define MODE_START  ( 0 )
#define MODE_NORMAL ( 1 )
#define MODE_STOP   ( 2 )
#define MODE_START_STOP   ( 3 )
#define MODE_TRACK  ( 4 )

#define TYPE_START  (0)
#define TYPE_NORMAL (1)
#define TYPE_ACC    (2)
#define TYPE_DEC    (3)
#define TYPE_STOP   (4)

#define CALC_L_RES 100

class PathTracking{
public:
    /*********** 変数宣言 ***********/
    double Px[POINTNUM], Py[POINTNUM];
    double refangle[PATHNUM], refvel[PATHNUM], acc_param[PATHNUM];
    int acc_mode[PATHNUM],acc_count[PATHNUM];
    double dec_param[PATHNUM];
    
    // 距離の配列
    double accL[PATHNUM],decL[PATHNUM],allL[PATHNUM];
    
    //距離を計算する際の分割数
    int divideNum[PATHNUM];   
    
    // ベジエ曲線関連
    double Ax[ PATHNUM ];
    double Bx[ PATHNUM ];
    double Cx[ PATHNUM ];
    double Dx[ PATHNUM ];

    double Ay[ PATHNUM ];
    double By[ PATHNUM ];
    double Cy[ PATHNUM ];
    double Dy[ PATHNUM ];

    // 内積関連
    double a_be[ PATHNUM ];
    double b_be[ PATHNUM ];
    double c_be[ PATHNUM ];
    double d_be[ PATHNUM ];
    double e_be[ PATHNUM ];
    double f_be[ PATHNUM ];
    double d_be_[ PATHNUM ];
    double e_be_[ PATHNUM ];
    double f_be_[ PATHNUM ];

    double curve_r;
    double curve_ox, curve_oy;
    double curve_x_,curve_y_;

    double onx, ony;
    double angle, dist;
    double t_be, pre_t_be;
    double phi, phi_con, pre_phi, phi_;
    double phi_res;
    double t_deadBand;
    double dist2goal;
    double epsilon;

    coords tar_posi;

    double refVx, refVy, refVz, refVec;
    double refKakudo;
    double pre_refVz, lim_refVz;
    double tmpPx, tmpPy;
    double pre_Px,pre_Py,Px0,Py0;

    double tan, per, rot;
    
    double stolen_t_be = 0, stolen_phi = 0;
    double stolen_turn_proc,stolen_turn_theta,stolen_turn_e;
    double stolen_limit_vel;

    int acc_process;

    double rotate_r = 0;
    double rotate_theta = 0;
    double rotate_k = 0;
    double theta_a = 0;
    double theta_b = 0;
    double add_theta = 0;

    bool angle_mode = false;
    bool turn_mode;

    /*********** 関数宣言 ***********/
    PathTracking(int xmode);
    //無駄時間後の世界を見るための処理（無駄時間はdefine.hで設定）
    void posiDataUpdate(double _realVel = -9999);//引数に何も入れなければ勝手にgPosiから計算する
    // tを求めるための方程式
    double func(int p, double t);
    double dfunc(int p, double t);
    // tにおけるベジエ曲線の座標を求める関数
    double bezier_x(int p, double t);
    double bezier_y(int p, double t);
    // ベジエ曲線式の1階微分
    double dbezier_x(int p, double t);
    double dbezier_y(int p, double t);

    double func2(double phi);
    double dfunc2(double phi);
    double curve_x(double phi);
    double curve_y(double phi);
    double dcurve_x(double phi);
    double dcurve_y(double phi);

    void initSettings();
    void calcPathParam();

    void calcRefpoint();
    void calcRefpoint_curve();
    void calcAccAndDecParam(); 
    void calcAccDecParam_re(int start,int end,bool velacc,double vel,double acc,double dec); 
    void setAccDecParam(int num,double vel,double acc,double dec);
    void setModecalcParam(int num,int mode,int dir);
    int calcRefvel();
    int calcRefvel_curve();
    double calcRefvel_posi(double,double,double,double,double);
    double calcRefvelZ(double);
    
    //void incrPathnum(double conv_length, double conv_tnum);
    void makePath(int num, double Pointx[4], double Pointy[4], double angle);
    void makePath_circle(int num, double Point2x[2], double Point2y[2], double angle);
    int changePath(int num, double acc, double vec, double angle, coords tar_posi, coords tar_posi2, bool flag_);
    void changeDestPath(int num,int mode,double pre_vel,double Pointx[4],double Pointy[4],double angle,bool flag_param,double vel,double acc,double dec);
    void incrPathnum();
    void decrPathnum();
    //収束判定の変更
    void setConvPara(double conv_length, double conv_tnum);
    int getPathNum();
    void setPathNum(int,bool);
    void setPathEnd(int,double,double);
    void setPathStart(int,double,double);
    void setParam();
    void setParam_re();
    void setParam(int);
    void setParam_re(int);
    void setAngle(double);
    void setConst(bool);
    void closeFlagAngle();
    void addCondAngle();

    void setMode(int);
    void setAccMode(int,int);
    void setPathAccMode(int,int);
    int getMode();
    int getAccMode();
    void setConstTurn(bool);

    void setMaxPathnum(int);

    void setPosiPIDxPara(float xKp, float xKi, float xKd);
    void setPosiPIDyPara(float xKp, float xKi, float xKd);
    void setPosiPIDzPara(float xKp, float xKi, float xKd);
    void setYokozurePIDPara(float xKp, float xKi, float xKd);
    void setKakudoPIDPara(float xKp, float xKi, float xKd);
    void kakudoPIDinit();
    void setRefKakudo();
    void setOffsetAngle();

    double getRefVper();
    double getRefVrot();
    double getResL();
    double getDist2goal();
    void getAutoPhase(int);
    void setInseidPer(int);
    void setCurveDec(bool);

    //DWA用関数
    State motion(State s, float v, float omega);
    std::vector<Obstacle> obstacles;
    float headingEval(State s, float gx, float gy);
    float distEval(State s, const std::vector<Obstacle>& obs, double R);
    std::pair<float,float> dwaControl(State s, float gx, float gy,
                                  const std::vector<Obstacle>& obs);
    int calcRefvel_DWA();
    State s = {0,0};
    float goalX, goalY;
    double obstacleR;
    std::vector<Obstacle> obs = {};//カメラなどから場所を受け取るときは更新する．

private:
    int path_num;
    int pathNum_deadBand;
    int mode;
    int max_pathnum;
    int count_acc;

    int pre_refType;

    double realPosi_x, pre_realPosi_x, realPosi_y, pre_realPosi_y;
    double realVel;
    double dist_deadBand;
    double get_resL;

    double conv_length;
    double conv_tnum;
    double conv_theta;

    double max_turnSpeed;
    double pre_refvel;
    double refKakudo_;
    double keep_vel;
    // #if DRIVE_UNIT == PLATFORM_OMNI4WHEEL
    // double limit_rot_acc = 3.0 / DIST2WHEEL * SIN_PI_4;
    // #elif DRIVE_UNIT == ACTIVECASTER_4WHEEL
    // double limit_rot_acc = 3.0 / DIST2WHEEL;
    // #endif
    double limit_rot_acc = 6.0 / 0.33;
    double pre_rot;
    coords pre_posi;

    bool mode_changed,path_mode,set_angle,flag_pid = true;
    bool path_change, flag_const;
    bool const_turn,flag_angle;
    bool init_done;
    int flag_inside_per;
    bool flag_curve_dec = true;

    int auto_phase = 0;
    int sign = 1;
};

#endif