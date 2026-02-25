#ifndef AUTOCONTROL_h
#define AUTOCONTROL_h

#include "PathTracking.h"
// #include "AStar.h"
#include "define.h"
#include "SDclass.h"

// ボタン
#define PUSH_BUTTON 1
#define PUSH_LIMSW 2
#define RECO_PHOTO 4
#define MAKE_PATH 8
#define MAKE_PATH2 16
#define PUSH_LIMSW_R 32
#define PUSH_LIMSW_L 64

// 経路
#define DEFAULT 0
#define REVERSE 1
#define CON_PATH 2
#define RE_CON_PATH 3

#define MODE_STOP_VEL 0
#define MODE_KEEP_VEL 1
#define MODE_STAY 2

// define 追加
#define FLAME_THETA 0.829

class AutoControl{
    public:
    // AStar astar;
    AutoControl();
    void init(mySDclass*, int);
    void gPosiInit();
    coords pathTrackingMode(int mode, int state, int nextPhase, bool d_mode);
    void calibrationGposi(double tempX, double tempY, double tempZ);
    coords commandMode_vel(double tempX, double tempY, double tempZ);
    void commandMode(int nextPhase, bool next = true);
    int getMode();
    int getAccMode();
    void setMakePath(double Px[4], double Py[4], double angle);
    void setTarPosi(coords);
    coords getRefVel(unsigned int nextPhase);

    int field_s = RED;

    //中間地点
    double goal_c1_posi_x = 0.0, goal_c1_posi_y = 0.0;
    double goal_c2_posi_x = 0.0, goal_c2_posi_y = 0.0;
    double goal_posi_x = 0.0, goal_posi_y = 0.0;

    int Box_Number[5];
    int box_number = 0;
    int box_colle_number = 0;
    int box_count = 0;
    bool box_count_reset = false;
    int R2_Adjacent_count = 0;

    int phase = 0,pre_phase = 0,pre_under_phase = 0,syusoku = 0;
    int goal_point = 0;
    int pre_posi,pre_tar_posi,pre_tar_posi_,tar_posi,tar_posi2;
    double tar_angle;
    double dist_goal;
    int stop_mode = MODE_STOP_VEL;
    int remove_dir = 0, remove_dir2 = 0;
    bool selectPath=false,flag_start = false,flag_retry = false, flag_retry_forest = false, send_state=false,flag_MA = false,flag_stop = false;
    bool limsw_y = false;
    bool setPath = true;
    int collect_count;
    int scat_area;
    bool change_path_end;
    bool tof_sensor;
    bool flag_simu;
    bool flag_change, flag_out_collect, flag_end_collect, flag_store_stuck, flag_receive_, flag_first_receive;
    bool first_center_flag2 = false;


    bool lrtb_flag_x = false, lrtb_flag_y = false; //lrtbを使う場所をこっちから指定
    int mode;
    bool manu_mode = false;
    bool platturn_mode;
    bool flag_goalangle_mode = false;
    bool ball_have = false;

    int Colle_Mode = 0;
    bool button_0;
    int button_00;
    bool button_true[12];
    int button_true1[12];
    bool button_true_R1[12];//mainからの受け取り
    bool button_true_R2[12];
    bool button_true_Fake[12];
    bool pre_button_true_R1[12];
    bool pre_button_true_R2[12];
    bool pre_button_true_Fake[12];
    int true_set = 0;
    int total_path = 0;
    double slopefront_vel = 0;
    double rotate_radian = 0.0;
    controllPoint forestfrontPosi = {0.0,0.0};
    int nextX = 0, nextY = 0, nextZ = 0;
    int current_x = 0, current_y = 0, current_z = 0;
    int Fake_x = 0, Fake_y = 0;
    int uncole1 = 0;
    int uncole2 = 0;
    int turn_cost = 0;
    int posifront_count = 0;
    int posifront_count_first = 0;
    int posifront_count_second = 0;
    int next_indices;
    bool step_flag = false;//段越えを挟むかどうか最初の真ん中マスのように，そのまま行く場合はfalseとする.LRTB用
    bool not_step = false;

    //ラック位置とflag
    bool spear_true[6] = {0,1,0,0,0,0};//手前が0
    int spear_count = 0;
    //槍先-------------
    double spear_posi_x[6] = {0.45+0.02, 0.65+0.02, 0.85+0.02, 1.05+0.02, 1.25+0.02, 1.45+0.02};//槍先のx方向中心位置
    double rack_wall_posi = 0.15 - 0.025;//ラックの表面壁の中心からの距離
    double tar_posi_rack_x;
    double tar_posi_rack_y;
    bool flag_spear_catch = false;//槍先を取得できたか同課の判定
    bool flag_spear_ret = false;

    //KFSラック-------------
    double kfs_rack_posi_x[3] = {10.21, 10.75, 11.29};//下側から
    double kfs_rack_posi_y = 0.15 - 0.025 + 1.0 - 0.350 -0.075;
    double tar_posi_kfs_rack_x;
    double tar_posi_kfs_rack_y;
    int rack_num = 0;
    bool set_KFS_height = false;

    //box forest
    bool next_hold = false;
    bool lift_check = false;
    int cubeIndex = 0;
    int nextIndex = 0;
    double tarposi_diff = 0.0;
    int next_box_state = 0;
    int KFS_height_state = 0;
    bool lrtb_check = false;

    int send_num = 0;//上半身に送る値．
    int up_num = 0;//上半身から受け取った値．
    int up_box_num = 0;//上半身から受け取ったboxの値

    bool return_mode = false;

    // mainプログラムとPathTrackingの媒介的な
    double Px(int);
    double Py(int);
    double onx();
    double ony();
    double angle();
    double dist();
    double tan();
    double per();
    double refKakudo();
    double lim_refVz();
    double limit_vel();
    double turn_proc();
    double turn_theta();
    double turn_e();
    double resL();
    double dist2goal();

    void initSettings();
    void setConvPara(double conv_length, double conv_tnum);
    void setMaxPathnum(int);
    int getPathNum();
    int acc_process();
    double get_t_be();
    void calcDecParam();//new
    int getCount();
    bool getTerminat(double);
    bool getConvAngle(double,double);
    double getrotate_r();
    double gettheta_a();
    double gettheta_b();
    double getrotate_k();
    double getadd_theta();
    void set_para(double goalx, double goal_y, double rad, double vel, double acc, double dec);
    void set_para2(double path_x[4], double path_y[4], double rad, double vel, double acc, double dec);

    /////
    forest_posi forest[18] = {
        {2.6,1.8,0}, {2.6,3.0,0}, {2.6,4.2,0},
        {3.8,1.8,2}, {3.8,3.0,0}, {3.8,4.2,2},
        {5.0,1.8,4}, {5.0,3.0,2}, {5.0,4.2,0},
        {6.2,1.8,2}, {6.2,3.0,4}, {6.2,4.2,2},
        {7.4,1.8,0}, {7.4,3.0,2}, {7.4,4.2,0},
        {8.6,1.8,0}, {8.6,3.0,0}, {8.6,4.2,0}
    };
    //経路をフォレストの数字で送る
    //{経路,0|1|2} 0:通過　1:回収通過 2:回収のみ
    //0:止まれ 1:回収 2:通過 3:回収通過
    /////

    int spear_num = 4; 
    double gpath_x[4];
    double gpath_y[4];
    double setx;
    double sety;
    double setz;
    int route_num = 0;
    double set_front_posi;
    double set_back_posi;
    
    int hight_flag = 0;
    int hight_flag2 = 0;
    int direction_flag = 0;

    double diffx = 0;
    double diffy = 0;

    int khs = 0;

    int flag_hi2hi = 0;


    private:
    coords tar_posi_ = {0.0,0.0,0.0};
    coords tar_posi2_ = {0.0,0.0,0.0};
    coords tar_posi3_ = {0.0,0.0,0.0};
    double goal_3point_x[5] = {12.8, 11.44, 10.35, 11.44, 12.8};//味方側から見て左から順に
    double goal_3point_y[5] = {7.500, 6.634, 4, 1.366, 0.499};//ロボットの移動位置（ゴール前）
    double pass_DR_posi[2] = {1.330,9.840};//x,y パスするときのDR位置
    double pass_PR_posi[2] = {6.450,6.775};//パスの時のPR位置
    //double goal_3point_theta[5] = {1.221, 0.785, 0, -0.785, -1.221};
    double mygoal_3point_x[5] = {2.20, 3.56, 4.651, 3.56, 2.20};
    double make_path_x[4], make_path_y[4];
    double path_angle;
    double core_max_vel;
    double conv_range_;
    bool flag_curve, flag_inc = true, get_d_mode;
    bool flag_silo_change;
    bool flag_out_collect_, flag_front_collect;
    int stop_count;
};

#endif