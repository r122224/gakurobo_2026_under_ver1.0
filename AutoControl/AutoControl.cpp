#include "AutoControl.h"
#include "PathTracking.h"
// #include "AStar.h"
#include <cmath>
#include <cstdio>
#include <iterator>

extern coords gPosi;//自己位置　スタート
extern coords tar_Posi;//目標場所
extern field cubePosi;
extern field3 cubePosi3;
extern field tar_cubePosi;
extern field3 tar_cubePosi3;
extern field obsPosi[4];
extern field forestXYMAX;
extern field spearPosi;
extern Node Open;
extern Node Close;
extern Obstacle objePosi[12];
extern Obstacle objePosi2[12];
//extern dircost forest_dir[4];//向き
//extern dim2 silo[5];
extern coords remove_posi[4];
extern int field;
extern int receive_phase;
extern int path;
extern int rice_mode;
extern int collect_phase;
extern bool set_print;

extern forest_route route[7];
extern bool front_syusoku;
extern bool back_syusoku;

PathTracking motion(FOLLOW_COMMAND); // 経路追従(接線方向向く)モードでとりあえず初期化

AutoControl::AutoControl() {}

// SDのデータを読み込んで，PathTrackingの変数に格納
void AutoControl::init(mySDclass *mySD, int round) {
  int actpathnum =  mySD->path_read(round, motion.Px, motion.Py, motion.refvel, motion.refangle, motion.acc_mode, motion.acc_param, motion.dec_param);
  motion.initSettings(); //  これをやっていないと足回りの指令速度生成しない
  if(set_print) printf("actpathnum :  %d\n", actpathnum);
  motion.setMaxPathnum(actpathnum); // パス数の最大値
  motion.calcAccAndDecParam();
//   setConvPara(0.02,0.96);
}

void AutoControl::gPosiInit() {
  gPosi.x = motion.Px[0];
  gPosi.y = motion.Py[0];
  gPosi.z = motion.refangle[0];
}

coords AutoControl::pathTrackingMode(int mode, int state, int nextPhase, bool d_mode) // 軌道追従モード
// state→実行するpathnumの値,nextpahseつぎのpahseの値
{
  coords refV = {0.0,0.0,0.0};

  int pathNum = getPathNum();
  // printf("pathnum %d state %d", pathNum, state);
  // motion.setConvPara(0.2, 0.99);
  if (motion.getMode() != mode)
    motion.setMode(mode);

  get_d_mode = d_mode;

  if(!d_mode && (pathNum > state)) return refV;
  else if(d_mode && (pathNum < state)) return refV;

  syusoku = motion.calcRefvel();

//   if(!flag_curve) syusoku = motion.calcRefvel(); // 収束していれば　1　が返ってくる
  //if(!flag_curve) syusoku = motion.calcRefvel_DWA();
//   else syusoku = motion.calcRefvel_curve(); // 収束していれば　1　が返ってくる

//   if(phase == 7 && (pathNum == 7)){
//     if(syusoku == 2) syusoku = 1;
//   }
//   else if(phase == 8 && (pathNum == 10)){
//     if(syusoku == 2) syusoku = 1;
//   }
  if(phase == 302){
      if(syusoku == 0){//判断phase に入ったら止める．
        syusoku = 1;
      }
  }else if(phase == 111 || phase == 121 || phase == 401){
      if(syusoku == 0){
          syusoku = 1;
      }
  }

//   motion.angle_mode = flag_goalangle_mode;

  if (syusoku == 1) { // 収束して次の曲線へ
    if(flag_curve) flag_curve = false;
    if (!d_mode && (pathNum <= state)) {
      if (pathNum == state){
        phase = nextPhase;
        if(stop_mode == MODE_STOP_VEL){
          motion.incrPathnum(); // 次の曲線へ．括弧の中身は収束に使う数値
        }
        else if(stop_mode == MODE_KEEP_VEL){
          refV.x = motion.refVx;
          refV.y = motion.refVy;
          refV.z = motion.refVz;
          motion.incrPathnum(); // 次の曲線へ．括弧の中身は収束に使う数値
        }
        else if(stop_mode == MODE_STAY){
          refV.x = motion.refVx;
          refV.y = motion.refVy;
          refV.z = motion.refVz;
        }
      }
      else if(getMode()<2){
        if(flag_inc) {
            refV.x = motion.refVx;
            refV.y = motion.refVy;
            refV.z = motion.refVz;
            motion.incrPathnum(); // 次の曲線へ．括弧の中身は収束に使う数値
        }
      }
      else{
        if(flag_inc) motion.incrPathnum(); // 次の曲線へ．括弧の中身は収束に使う数値
      }
    }
    else if(d_mode && (pathNum >= state)){
      if (pathNum == state){
        phase = nextPhase;
        if(stop_mode == MODE_STOP_VEL){
          motion.decrPathnum(); // 次の曲線へ．括弧の中身は収束に使う数値
        }
        else if(stop_mode == MODE_KEEP_VEL){
          refV.x = motion.refVx;
          refV.y = motion.refVy;
          refV.z = motion.refVz;
          motion.decrPathnum(); // 次の曲線へ．括弧の中身は収束に使う数値
        }
        else if(stop_mode == MODE_STAY){
          refV.x = motion.refVx;
          refV.y = motion.refVy;
          refV.z = motion.refVz;
        }
      }
      else if(getMode()<2){
        if(flag_inc) {
            refV.x = motion.refVx;
            refV.y = motion.refVy;
            refV.z = motion.refVz;
            motion.decrPathnum(); // 次の曲線へ．括弧の中身は収束に使う数値
        }
      }
      else{
        if(flag_inc) motion.decrPathnum(); // 次の曲線へ．括弧の中身は収束に使う数値
      }
    }
  } else if (syusoku == 2) {
    refV.x = motion.refVx;
    refV.y = motion.refVy;
    refV.z = motion.refVz;
  } else if (syusoku == 0) { // まだ収束していない，軌道追従中
    refV.x = motion.refVx;
    refV.y = motion.refVy;
    refV.z = motion.refVz;
  } else { // それ以外は問題ありなので止める
    refV.x = 0.0;
    refV.y = 0.0;
    refV.z = 0.0;
  }

  // printf("syusoku : %d  ||
  // %+.3f,%+.3f,%+.3f\t",syusoku,refV.x,refV.y,refV.z);
//フィールドの反転
  if(field_s == RED){
    refV.y = refV.y;
    refV.z = refV.z;
  }else if(field_s == BLUE){
      refV.y = -refV.y;
      refV.z = -refV.z;
  }
  return refV;
}

void AutoControl::calibrationGposi(double tempX, double tempY, double tempZ) {
  gPosi.x = tempX;
  gPosi.y = tempY;
  gPosi.z = tempZ;
}

coords AutoControl::commandMode_vel(double tempX, double tempY, double tempZ) {
  coords refV;
  refV.x = tempX;
  refV.y = tempY;
  refV.z = tempZ;
  return refV;
}

void AutoControl::commandMode(int nextPhase, bool next /*=true*/) // 指定した速度で動かすとき
{
  int pathNum = getPathNum();

  if (next) { // この動きを一つの曲線とみなす場合
    motion.Px[3 * pathNum + 3] = gPosi.x;
    motion.Py[3 * pathNum + 3] = gPosi.y;
    motion.incrPathnum(); // 次の曲線へ．括弧の中身は収束に使う数値
  } else {
    motion.Px[3 * pathNum] = gPosi.x;
    motion.Py[3 * pathNum] = gPosi.y;
  }

  phase = nextPhase;
}

void AutoControl::setMakePath(double Px[4], double Py[4], double angle){
  for(int i = 0; i < 4; i++){
    make_path_x[i] = Px[i];
    make_path_y[i] = Py[i];
  }
  path_angle = angle;
}

void AutoControl::setTarPosi(coords posi) { tar_posi_ = posi; }

int AutoControl::getMode() { return motion.getMode(); }

int AutoControl::getAccMode() { return motion.getAccMode(); }

int AutoControl::getPathNum() { return motion.getPathNum(); }

int AutoControl::acc_process() { return motion.acc_process; }

double AutoControl::Px(int point) { return motion.Px[point]; }

double AutoControl::Py(int point) { return motion.Py[point]; }

double AutoControl::onx() { return motion.onx; }

double AutoControl::ony() { return motion.ony; }

double AutoControl::angle() { return motion.angle; }

double AutoControl::dist() { return motion.dist; }

double AutoControl::tan() { return motion.tan; }

double AutoControl::per() { return motion.per; }

double AutoControl::refKakudo() { return motion.refKakudo; }

double AutoControl::lim_refVz() { return motion.lim_refVz; }

double AutoControl::limit_vel() { return motion.stolen_limit_vel; }

double AutoControl::turn_proc() { return motion.stolen_turn_proc; }

double AutoControl::turn_theta() { return motion.stolen_turn_theta; }

double AutoControl::turn_e() { return motion.stolen_turn_e; }

double AutoControl::dist2goal() { return motion.getDist2goal(); }

void AutoControl::setConvPara(double conv_length, double conv_tnum) {
  motion.setConvPara(conv_length, conv_tnum);
}

double AutoControl::get_t_be(void) { return motion.stolen_t_be; }

int AutoControl::getCount() {return collect_count;}

bool AutoControl::getTerminat(double t_be_threshold) { 
    double t_be_ = motion.t_be;
    return t_be_ >= t_be_threshold; 
}

bool AutoControl::getConvAngle(double angle, double angle_per_e){
    return fabs(angle - refKakudo()) < angle_per_e;
}

double AutoControl::getrotate_r(){ return motion.rotate_r; }

double AutoControl::gettheta_a(){ return motion.theta_a; }

double AutoControl::gettheta_b(){ return motion.theta_b; }

double AutoControl::getrotate_k(){ return motion.rotate_k; }

double AutoControl::getadd_theta(){ return motion.add_theta; }

//直線移動
void AutoControl::set_para(double goalx, double goal_y, double rad, double vel, double acc, double dec){ 
    double path_x[4] = {gPosi.x, goalx, goalx, goalx}; 
    double path_y[4] = {gPosi.y, goal_y, goal_y, goal_y};
    motion.makePath(0, path_x, path_y, rad);
    motion.setParam();
    motion.calcAccDecParam_re(0, 0, true, vel, acc, dec);
}

void AutoControl::set_para2(double path_x[4], double path_y[4], double rad, double vel, double acc, double dec){ 
    motion.makePath(0, path_x, path_y, rad);
    motion.setParam();
    motion.calcAccDecParam_re(0, 0, true, vel, acc, dec);
}

// このメソッドの中身はユーザーが書き換える必要あり
coords AutoControl::getRefVel(unsigned int nextPhase) {
  coords refV = {0.0, 0.0, 0.0};
  static coords pre_refV = {0.0,0.0,0.0};
  static int change_mode = 0;
//   static int collect_count;
  static int change_phase;
  static int pre_path_num;
  static bool waiting_for_collect = false;
//   int current_target = astar.wait_id; // 今狙ってる collect_posi_true の番号

//   total_path = astar.total;
  for (int i = 0; i < 12; i++) {
    // astar.obs_posi_true[i] = button_true_R1[i];
    // astar.collect_posi_true[i] = button_true_R2[i];
    // astar.R1KFS_posi[i] = button_true_R1[i];//マスのどこにあるか
    // astar.R2KFS_posi[i] = button_true_R2[i];
    // astar.Fake_posi[i] = button_true_Fake[i];
    // astar.pre_R1KFS_posi[i] = pre_button_true_R1[i];
    // astar.pre_R2KFS_posi[i] = pre_button_true_R2[i];
    // astar.pre_Fake_posi[i] = pre_button_true_Fake[i];
  }
//   astar.collect_count = collect_count;

  //astarから引用（表示用）
//   nextX = astar.nextX;
//   nextY = astar.nextY;
//   nextZ = astar.nextZ;
//   current_x = astar.current_x;
//   current_y = astar.current_y;
//   current_z = astar.current_z;
//   Fake_x = astar.Fake_posix;
//   Fake_y = astar.Fake_posiy;
//   uncole1 = astar.uncole1;
//   uncole2 = astar.uncole2;
//   R2_Adjacent_count = astar.R2_Adjacent_count;
//   turn_cost = astar.turn_cost;
//   next_indices = astar.next_indices;
//   if(true_set == 0){
//     astar.collect_posi_true[1] = true;
//     astar.collect_posi_true[2] = true;
//   }else if(true_set == 1){
//     astar.collect_posi_true[1] = false;
//     astar.collect_posi_true[2] = true;
//   }else if(true_set == 2){
//     astar.collect_posi_true[1] = false;
//     astar.collect_posi_true[2] = false;
//   }
  
    // printf("1:%d,2:%d",astar.collect_posi_true[1],astar.collect_posi_true[2]);

  Colle_Mode = 0;
  motion.turn_mode = platturn_mode;

  flag_change = true;//経路変更用
  motion.getAutoPhase(phase);

 switch(phase){
    //100:エリア1
    //200:エリア2
    //300:エリア3
    //
    case 0:
        motion.setPathNum(0, 0);
        setConvPara(0.05, 0.992);
        send_num = 0;
        // phase = 100;
        // phase = 200;
        // phase = 300;
        if ((nextPhase & PUSH_BUTTON) == PUSH_BUTTON) {
            // phase = 1;//phase = 107;
            if(flag_simu || flag_retry_forest){
                // phase = 107;
                phase = 200;
            }else{
                if(flag_retry){
                    phase = 300;//ラック前
                }else{
                    phase = 100;
                } 
            }
        }
    break;
    case 100:
        send_num = 1;//ハンド展開
        motion.setPathNum(0, 0);
        setConvPara(0.01, 0.998);
        // double path_x[4] = {gPosi.x, spear_posi_x[spear_num],spear_posi_x[spear_num],spear_posi_x[spear_num]};
        // double path_y[4] = {gPosi.y, 1,0.8,0.52};
        // set_para2(path_x, path_y, M_PI/2, 0.5, 3.00, 3.00);
        gpath_x[0] = gPosi.x;
        gpath_x[1] = spear_posi_x[spear_num];
        gpath_x[2] = spear_posi_x[spear_num];
        gpath_x[3] = spear_posi_x[spear_num];
        gpath_y[0] = gPosi.y;
        gpath_y[1] = 1;
        gpath_y[2] = 0.8;
        gpath_y[3] = 0.49;
        // gpath_y[3] = 0.45;
        set_para2(gpath_x, gpath_y, M_PI/2, 0.5, 3.00, 3.00);
        phase = 101;
    break;
    case 101: //ベッドラックに移動＆回収
        refV = pathTrackingMode(FOLLOW_COMMAND, 0, 102, DEFAULT);
        if ((nextPhase & PUSH_BUTTON) == PUSH_BUTTON) {
            phase = 102;
        }
    break;
    case 102://リミットスイッチ反応したら止める
        //
        if(up_num == 1)//槍先回収した
            phase = 103;
    break;
    case 103: //後ろに下がる
        motion.setPathNum(0, 0);
        setConvPara(0.01, 0.998);
        setx = 0.95;
        sety = 1.00;
        set_para(setx, sety, M_PI/2, 0.5, 3.00, 3.00);
        phase = 104;
    break;
    case 104: //下がる
        refV = pathTrackingMode(FOLLOW_COMMAND, 0, 105, DEFAULT);
    break;
    case 105://旋回する．
        //ここで回収できていなかったらphase = 3に戻り再回収をする．
        motion.setPathNum(0, 0);
        setConvPara(0.01, 0.998);
        set_para(setx, sety, -M_PI/2, 3.0, 3.00, 3.00);
        phase = 106;
    break;
    case 106://旋回する．
        refV = pathTrackingMode(POSITION_PID, 0, 107, DEFAULT);
    break;
    case 107:
        // send_num = 2;

        // if(up_num == 2)//槍先を離した
        //     phase = 108;
        if ((nextPhase & PUSH_BUTTON) == PUSH_BUTTON) {
            send_num = 2;
            // if(up_num == 2){//槍先を離した
            //     phase = 108;
            // }
        }
        if(up_num == 2){//槍先を離した
            phase = 108;
        }
    break;
    case 108: //少し離れる
        motion.setPathNum(0, 0);
        setConvPara(0.01, 0.998);
        set_para(gPosi.x, gPosi.y - 0.2, -M_PI/2, 3.0, 3.00, 3.00);
        phase = 109;
    break;
    case 109:
        refV = pathTrackingMode(FOLLOW_COMMAND, 0, 110, DEFAULT);
    break;
    case 110: //旋回
        motion.setPathNum(0, 0);
        setConvPara(0.01, 0.998);
        set_para(gPosi.x, gPosi.y, 0.000, 0.5, 3.00, 3.00);
        phase = 111;
    break;
    case 111: //旋回
        refV = pathTrackingMode(FOLLOW_COMMAND, 0, 200, DEFAULT);
    break;

    case 200://forest前に移動
        motion.setPathNum(0, 0);
        setConvPara(0.01, 0.998);
        // route[0].num;
        route_num = 0;
        set_para(forest[route[route_num].num].x, forest[route[route_num].num].y, 0.000, 1.5, 3.00, 3.00);
        phase = 201;
    break;
    case 201: //forest前に移動
        refV = pathTrackingMode(FOLLOW_COMMAND, 0, 202, DEFAULT);
    break;
    case 202://旋回の確認
        if(route[route_num].num >= 15){
            phase = 240;
        }

        diffx = forest[route[route_num + 1].num].x - forest[route[route_num].num].x;
        diffy = forest[route[route_num + 1].num].y - forest[route[route_num].num].y;
        //ひとつ前のマスに移動

        if(diffx > 1.0 && abs(diffy) < 0.2){
            setz = 0.00;
            direction_flag = DFRONT;
        }else if(diffx < -1.0 && abs(diffy) < 0.2){
            setz = M_PI;
            direction_flag = DBACK;
        }else if(abs(diffx) < 0.2 && diffy > 1.0){
            setz = M_PI/2;
            direction_flag = DRIGHT;
        }else if(abs(diffx) < 0.2 && diffy < -1.0){
            setz = -M_PI/2;
            direction_flag = DLEFT;
        }else{
            direction_flag = 10;
        }
        // if(diffx == 1.2){
        //     direction_flag = 13;
        // }
        // if(forest[route[route_num].num].y == forest[route[route_num + 1].num].y && forest[route[route_num + 1].num].x - forest[route[route_num].num].x == 1.2){
        //     setz = 0.00;
        //     direction_flag = DFRONT;
        // }
        // //右隣
        // else if(forest[route[route_num + 1].num].y - forest[route[route_num].num].y == 1.2 && forest[route[route_num].num].x == forest[route[route_num + 1].num].x){
        //     setz = -M_PI/2;
        //     direction_flag = DRIGHT;
        // }
            
        // //左隣
        // else if(forest[route[route_num + 1].num].y - forest[route[route_num].num].y == -1.2 && forest[route[route_num].num].x == forest[route[route_num + 1].num].x){
        //     setz = M_PI/2;
        //     direction_flag = DLEFT;
        // }  
        // else if(forest[route[route_num].num].y == forest[route[route_num + 1].num].y && forest[route[route_num + 1].num].x - forest[route[route_num].num].x == -1.2){
        //     setz = M_PI;
        //     direction_flag = DBACK;
        // }
        // else
        //     ;
        motion.setPathNum(0, 0);
        setConvPara(0.01, 0.998);
        set_para(forest[route[route_num].num].x, forest[route[route_num].num].y, setz, 1.0, 3.00, 3.00);
        phase = 203;
    break;
    case 203: //旋回
        refV = pathTrackingMode(POSITION_PID, 0, 204, DEFAULT);
    break;
    case 204: //1
        switch (route[route_num + 1].mode) {
            case 1://回収
                phase = 210;
            break;
            case 2://通過
                phase = 220;
            break;
            case 3://回収通過
                phase = 230;
            break;
        }
    break;

    ///////回収///////
    case 210:
        switch (forest[route[route_num + 1].num].hight - forest[route[route_num].num].hight) {
            case +2:
                //上段回収
                hight_flag = 2;
                // KFS_height_state = 2;
                khs = 2;
                set_front_posi = HIGH_COLLECT_POSI;
            break;
            case  0:
                //中段回収
                hight_flag = 0;
                set_front_posi = HIGH_COLLECT_POSI;
            break;
            case -2:
                //下段回収
                // KFS_height_state = 1;
                khs = 1;
                hight_flag = -2;
                set_front_posi = LOW_COLLECT_POSI;
            break;
        }
        switch (forest[route[route_num + 2].num].hight - forest[route[route_num].num].hight) {
            case +2:
                //上段回収
                hight_flag2 = 2;
            break;
            case  0:
                //中段回収
                hight_flag2 = 0;
            break;
            case -2:
                //下段回収
                hight_flag2 = -2;
            break;
        }
        phase = 211;
    break;
    case 211:
        //前に移動．ただし，上回収のときはきをつける
        motion.setPathNum(0, 0);
        setConvPara(0.01, 0.998);
        switch (direction_flag) {
            case DFRONT:
                setx = forest[route[route_num].num].x + EDGE_MOVE;
                sety = forest[route[route_num].num].y;
            break;
            case DRIGHT:
                setx = forest[route[route_num].num].x;
                sety = forest[route[route_num].num].y + EDGE_MOVE;
            break;
            case DLEFT:
                setx = forest[route[route_num].num].x;
                sety = forest[route[route_num].num].y - EDGE_MOVE;
            break;
            case DBACK:
                setx = forest[route[route_num].num].x - EDGE_MOVE;
                sety = forest[route[route_num].num].y;
            break;
        }
        set_para(setx , sety, setz, 0.3, 3.00, 1.00);
        //////
        if(front_syusoku)
            phase = 212;
    break;
    case 212: //回収地点に行く
        refV = pathTrackingMode(FOLLOW_COMMAND, 0, 2213, DEFAULT);
    break;
    case 2213:
        // if(hight_flag == -2)
        //     set_front_posi = LOW_COLLECT_POSI;
        // else

            phase = 213;
    break;
    case 213: //
        send_num = 10;//回収していいよ！！
        motion.setPathNum(0, 0);
        setConvPara(0.01, 0.998);
        set_para(forest[route[route_num].num].x, forest[route[route_num].num].y, setz, 1.0, 3.00, 3.00);
        if(up_num == 3)//回収したとき
            phase = 2214;
    break;
    case 2214:
        send_num = 11;
        if(hight_flag == -2){//格納する//格納完了したら214にする
            set_front_posi = STORAGE_POSI;
            // if(front_syusoku){
            //     send_num = 12;
            //     phase = 2214;
            // }
        }
        // }else
            // phase = 214;
        phase = 22214;
    break;
    case 22214:
        if(up_num == 5){//上半身の仰角上げた
            if(hight_flag == -2){
                if(front_syusoku){
                    send_num = 12;
                    phase = 214;
                }
            }else{
                phase = 214;
            }
        }
        // if(front_syusoku){
        //     send_num = 12;
        //     phase = 214;
        // }
    break;
    case 214://MFの中心に戻る
        // send_num = 11;
        refV = pathTrackingMode(FOLLOW_COMMAND, 0, 215, DEFAULT);
    break;
    case 215:
        //次が回収しないで段越えのときは昇降そのまま
        //次が通過でhight_flag = 2で次も2のとき昇降そのまま段越えの
        // route[route_num + 1] = route[route_num];
        // route_num += 1;

        if(hight_flag == 2 && hight_flag2 == 2){
            set_front_posi = HIGH_COLLECT_POSI;
            flag_hi2hi = true;
            phase = 216;
        }else{
            set_front_posi = STORAGE_POSI;
            if(front_syusoku){
                send_num = 12;
                // if(up_num == 4)
                // phase = 202;
            }
        }
        if(up_num == 4){
            phase = 216;
        }
            //収束したら格納していいよ，格納
            //格納終わったらphase = 202に返す

        // route[route_num + 1] = route[route_num];
        // route_num += 1;

    break;
    case 216:
        route[route_num + 1] = route[route_num];
        route_num += 1;
        phase = 202;
    break;


    //通過する
    case 220:
        switch (forest[route[route_num + 2].num].hight - forest[route[route_num].num].hight) {
            case +2:
                //上段回収
                hight_flag2 = 2;
            break;
            case  0:
                //中段回収
                hight_flag2 = 0;
            break;
            case -2:
                //下段回収
                hight_flag2 = -2;
            break;
        }
        switch (forest[route[route_num + 1].num].hight - forest[route[route_num].num].hight) {
            case +2:
                //段越え
                hight_flag = 2;
                set_front_posi = STEP_UP_FRONT_HIGH;
                set_back_posi = STEP_UP_BACK_HIGH;
                if(back_syusoku == 1 && front_syusoku == 1)
                    phase = 223;
                //段越えする
            break;
            case  0:
                //中段回収
                hight_flag = 0;
                // set_front_posi = HIGH_COLLECT_POSI;
                motion.setPathNum(0, 0);
                setConvPara(0.01, 0.998);
                set_para(forest[route[route_num].num].x, forest[route[route_num].num].y, setz, 1.0, 3.00, 3.00);
                phase = 221;
            break;
            case -2:
                //段降り
                hight_flag = -2;
                phase = 223;
                // set_front_posi = LOW_COLLECT_POSI;
                //だんおりする
            break;
        }
    break;
    case 221://同段の通過
        refV = pathTrackingMode(FOLLOW_COMMAND, 0, 222, DEFAULT);
    break;
    case 222:
        route_num += 1;
        phase = 202;
    break;

    case 223: //段越え段降りmainで昇降が収束したらこのフェーズに変える
        //はじまで移動する
        motion.setPathNum(0, 0);
        setConvPara(0.01, 0.998);
        switch (direction_flag) {
            case DFRONT:
                setx = forest[route[route_num].num].x + EDGE_MOVE;
                sety = forest[route[route_num].num].y;
            break;
            case DRIGHT:
                setx = forest[route[route_num].num].x;
                sety = forest[route[route_num].num].y + EDGE_MOVE;
            break;
            case DLEFT:
                setx = forest[route[route_num].num].x;
                sety = forest[route[route_num].num].y - EDGE_MOVE;
            break;
            case DBACK:
                setx = forest[route[route_num].num].x - EDGE_MOVE;
                sety = forest[route[route_num].num].y;
            break;
        }
        set_para(setx , sety, setz, 0.3, 3.00, 1.00);
        phase = 224;
    break;

    case 224: //はじにいどうする
        refV = pathTrackingMode(FOLLOW_COMMAND, 0, 2225, DEFAULT);
    break;

    case 2225:
        //段越え段降りの処理
    break;


    case 225://mainで登った，または降りたらこのフェーズに移行するに
        motion.setPathNum(0, 0);
        setConvPara(0.01, 0.998);
        set_para(forest[route[route_num + 1].num].x, forest[route[route_num + 1].num].y, setz, 0.5, 3.00, 3.00);
        if(flag_hi2hi){
            if(up_num == 4)
                phase = 226;
        }else{
            phase = 226;
        }
    break;

    case 226://次のマスの中心に移動する
        refV = pathTrackingMode(FOLLOW_COMMAND, 0, 227, DEFAULT);
        // route_num += 1;
    break;

    case 227:
        route_num += 1;
        phase = 202;
    break;

    //回収＋段越え段降り
    case 230:
        switch (forest[route[route_num + 1].num].hight - forest[route[route_num].num].hight) {
            case +2:
                //上段回収
                hight_flag = 2;
                // KFS_height_state = 2;
                khs = 2;
                set_front_posi = HIGH_COLLECT_POSI;
            break;
            case  0:
                //中段回収
                hight_flag = 0;
                set_front_posi = HIGH_COLLECT_POSI;
            break;
            case -2:
                //下段回収
                hight_flag = -2;
                // KFS_height_state = 1;/
                khs = 1;
                set_front_posi = LOW_COLLECT_POSI;
            break;
        }
        phase = 231;
    break;

    case 231://はじに移動する//mainで上と中段のときは収束したら変更する
        motion.setPathNum(0, 0);
        setConvPara(0.01, 0.998);
        switch (direction_flag) {
            case DFRONT:
                setx = forest[route[route_num].num].x + EDGE_MOVE;
                sety = forest[route[route_num].num].y;
            break;
            case DRIGHT:
                setx = forest[route[route_num].num].x;
                sety = forest[route[route_num].num].y + EDGE_MOVE;
            break;
            case DLEFT:
                setx = forest[route[route_num].num].x;
                sety = forest[route[route_num].num].y - EDGE_MOVE;
            break;
            case DBACK:
                setx = forest[route[route_num].num].x - EDGE_MOVE;
                sety = forest[route[route_num].num].y;
            break;
        }
        set_para(setx, sety, setz, 0.5, 3.00, 3.00);
        phase = 232;
    break;

    case 232: //はじに移動する
        refV = pathTrackingMode(FOLLOW_COMMAND, 0, 233, DEFAULT);
    break;

    case 233: //mainで前の昇降が収束していたら234にする
        if(front_syusoku)
            phase = 234;
    break;

    case 234:
        send_num = 10;//回収していいよ
        if(up_num == 3)
            phase = 235;
    break;

    case 235://前後昇降の設定
        send_num = 11;
        switch (hight_flag) {
            case 2://段越え
                set_front_posi = STEP_UP_FRONT_HIGH;
                set_back_posi = STEP_UP_BACK_HIGH;
                phase = 2236;
            break;
            case 0://中段
                phase = 236;
                // set_front_posi
                // set_back_posi
            break;
            case -2://段降り
                set_front_posi = STEP_DOWN_FRONT_LOW;
                set_back_posi = STEP_DOWN_BACK_LOW;
                phase = 2236;
            break;
        }
    break;

    case 2236:
    break;

    case 236://段移動完了後
        set_front_posi = STORAGE_POSI;
        motion.setPathNum(0, 0);
        setConvPara(0.01, 0.998);
        set_para(forest[route[route_num + 1].num].x, forest[route[route_num + 1].num].y, setz, 1.0, 3.00, 3.00);
        phase = 237;
    break;

    case 237://段の中央に移動
        // route_num += 1;
        if(front_syusoku && up_num == 5)
            send_num = 12;
        refV = pathTrackingMode(FOLLOW_COMMAND, 0, 238, DEFAULT);
    break;

    case 238:
        route_num += 1;
        phase = 202;
    break;

    case 240://旋回して横移動
        motion.setPathNum(0, 0);
        setConvPara(0.01, 0.998);

    break;
    case 241:
        
    break;


    case 300:
        
    break;


    // case 0://初期動作　作成
    //     send_num = 0;
    //     motion.setPathNum(0, 0);
    //     setConvPara(0.05, 0.992);
    //     send_num = 0;
    //     // astar.make_plusdist_stepwise();
    //     if ((nextPhase & PUSH_BUTTON) == PUSH_BUTTON) {
    //         // phase = 1;//phase = 107;
    //         if(flag_simu || flag_retry_forest){
    //             phase = 107;
    //         }else{
    //             if(flag_retry){
    //                 phase = 60;//ラック前
    //             }else{
    //                 phase = 1;
    //             } 
    //         }
    //     }
    // break;
    // //---------------------------------------------------------
    // //startから槍先回収まで
    // case 1://ラック前移動(リミットスイッチが当たるところまで横に直線移動)移動する位置はカメラによって決定
    //     send_num = 0;
    //     if(spear_true[0]){//0から優先的に移動する．
    //        tar_posi_rack_x = spear_posi_x[0];
    //     }else if(spear_true[1]){
    //         tar_posi_rack_x = spear_posi_x[1];
    //     }else if(spear_true[2]){
    //         tar_posi_rack_x = spear_posi_x[2];
    //     }else if(spear_true[3]){
    //         tar_posi_rack_x = spear_posi_x[3];
    //     }else if(spear_true[4]){
    //         tar_posi_rack_x = spear_posi_x[4];
    //     }else if(spear_true[5]){
    //         tar_posi_rack_x = spear_posi_x[5];
    //     }
    //     tar_posi_rack_y = gPosi.y;
    //     // if ((nextPhase & PUSH_BUTTON) == PUSH_BUTTON) {
    //         send_num = 1;//ハンド展開
    //         motion.setPathNum(0, 0);
    //         setConvPara(0.01, 0.998);
    //         set_para(tar_posi_rack_x, tar_posi_rack_y, M_PI/2, 0.5, 3.00, 3.00);//M_PI/2
    //         // set_para(gPosi.x + 0.0, gPosi.y + 0.0, gPosi.z - M_PI/2, 0.5, 3.00, 3.00);//test用 マイナスで右に曲がる
    //         // if(flag_change){
    //         //     double path_x[4] = {gPosi.x, tar_posi_rack_x, tar_posi_rack_x, tar_posi_rack_x};//下から2番目
    //         //     double path_y[4] = {gPosi.y, 1.3525, 1.0, 0.15+0.365};//0.365
    //         //     set_para2(path_x, path_y, -M_PI/2, 0.5, 2.00, 2.00);
    //         // }
    //         phase = 2;
    //     // }
    // break;
    // case 2://ラック前に直線移動+ラック回収
    //     refV = pathTrackingMode(FOLLOW_COMMAND, 0, 200, DEFAULT);
    //     // refV = pathTrackingMode(POSITION_PID, 0, 3, DEFAULT);//回転方向同じM_PI/2で右
    // break;
    // case 200://ラック前移動前に位置補正する．
    //     motion.setPathNum(0, 0);
    //     setConvPara(0.01, 0.998);
    //     set_para(tar_posi_rack_x, tar_posi_rack_y, M_PI/2, 0.5, 3.00, 3.00);
    //     phase = 210;
    // break;
    // case 210://移動phase
    //     refV = pathTrackingMode(FOLLOW_COMMAND, 0, 3, DEFAULT);
    // break;
    // case 3://ラックに衝突
    //     tar_posi_rack_x = gPosi.x;
    //     tar_posi_rack_y = rack_wall_posi + 0.365 - 0.025 - 0.05;//0.4は機体中心からリミットスイッチの当たる場所までの距離 
    //     if(spear_true[0]){//0から優先的に移動する．
    //        tar_posi_rack_x = spear_posi_x[0];
    //     }else if(spear_true[1]){
    //         tar_posi_rack_x = spear_posi_x[1];
    //     }else if(spear_true[2]){
    //         tar_posi_rack_x = spear_posi_x[2];
    //     }else if(spear_true[3]){
    //         tar_posi_rack_x = spear_posi_x[3];
    //     }else if(spear_true[4]){
    //         tar_posi_rack_x = spear_posi_x[4];
    //     }else if(spear_true[5]){
    //         tar_posi_rack_x = spear_posi_x[5];
    //     }
    //     // if ((nextPhase & PUSH_BUTTON) == PUSH_BUTTON) {
    //         motion.setPathNum(0, 0);
    //         setConvPara(0.01, 0.998);
    //         set_para(tar_posi_rack_x, tar_posi_rack_y, M_PI/2, 0.3, 1.00, 1.00);
    //         phase = 301;
    //     // }
    // break;
    // case 301://ラックに衝突
    //     refV = pathTrackingMode(FOLLOW_COMMAND, 0, 302, DEFAULT);
    //     if ((nextPhase & PUSH_BUTTON) == PUSH_BUTTON) {
    //         phase = 302;
    //     }
    // break;
    // case 302://取得したら少し後ろに下がる
    //     tar_posi_rack_x = gPosi.x;
    //     tar_posi_rack_y = rack_wall_posi + 0.365 + 0.5;//50cm下がる
    //     // if ((nextPhase & PUSH_BUTTON) == PUSH_BUTTON) {
    //         if(up_num == 1 || flag_simu){
    //             motion.setPathNum(0, 0);
    //             setConvPara(0.01, 0.998);
    //             set_para(tar_posi_rack_x, tar_posi_rack_y, M_PI/2, 0.5, 3.00, 3.00);
    //             flag_spear_catch = true;
    //             phase = 303;
    //         // }else {//取得できていなかったとき
    //         //     motion.setPathNum(0, 0);
    //         //     setConvPara(0.01, 0.998);
    //         //     set_para(tar_posi_rack_x, tar_posi_rack_y, M_PI/2, 0.5, 3.00, 3.00);
    //         //     flag_spear_catch = false;
    //         //     phase = 303;
    //         // }
    //         // if(flag_simu){
    //         //     phase = 303;
    //         }
    //     // }
    // break;
    // case 303://取得したら少し後ろに下がる.していなくても後ろに下がる→phase1に戻す
    //     // if(up_num == 1){
    //     //     flag_spear_catch = true;
    //     // }else{
    //     //     flag_spear_catch = false;
    //     // }
    //     if(flag_spear_catch){
    //         refV = pathTrackingMode(FOLLOW_COMMAND, 0, 304, DEFAULT);
    //     }else {
    //         refV = pathTrackingMode(FOLLOW_COMMAND, 0, 1, DEFAULT);//phaseを1に戻す
    //     }
    // break;
    // case 304://後ろに下がったら旋回する．
    //     // if ((nextPhase & PUSH_BUTTON) == PUSH_BUTTON) {
    //         motion.setPathNum(0, 0);
    //         setConvPara(0.01, 0.998);
    //         set_para(gPosi.x, gPosi.y, -M_PI/2, 3.0, 3.00, 3.00);
    //         phase = 305;
    //     // }
    // break;
    // case 305://後ろに下がったら旋回する．
    //     refV = pathTrackingMode(POSITION_PID, 0, 315, DEFAULT);
    // break;
    // case 315://槍先を離す
    //     if ((nextPhase & PUSH_BUTTON) == PUSH_BUTTON) {
    //         send_num = 2;
    //         if(up_num == 2){//槍先を離した
    //             phase = 325;
    //         }
    //     }
    //     if(flag_simu){
    //         phase = 325;
    //     }
    // break;
    // case 325://槍先を離したら少し下げてから
    //     motion.setPathNum(0, 0);
    //     setConvPara(0.01, 0.998);
    //     set_para(gPosi.x, gPosi.y - 0.2, -M_PI/2, 0.5, 3.00, 3.00);
    //     phase = 335;
    // break;
    // case 335:
    //     refV = pathTrackingMode(FOLLOW_COMMAND, 0, 306, DEFAULT);
    // break;
    // case 306://槍先を離したら旋回させる→この時，連続でとるかで分ける． 
    //     // send_num = 2;       
    //     // if ((nextPhase & PUSH_BUTTON) == PUSH_BUTTON) {
    //         motion.setPathNum(0, 0);
    //         setConvPara(0.01, 0.998);
    //         if(next_hold){
    //             set_para(gPosi.x, gPosi.y, M_PI/2, 0.5, 3.00, 3.00);
    //             send_num = 1;
    //         }else{
    //             send_num = 0;
    //             set_para(gPosi.x, gPosi.y, 0.000, 0.5, 3.00, 3.00);
    //         }
    //         phase = 307;
    //     // }
    // break;//次の連続取得かforest前移動のため旋回
    // case 307:
    //     refV = pathTrackingMode(POSITION_PID, 0, 4, DEFAULT);
    // break;
    // case 4:////
    //     if(next_hold){//連続
    //         if(spear_count == 0){
    //             spear_true[1] = false;
    //             // spear_true[1] = true;
    //             next_hold = false;
    //         }
    //         // else if(spear_count == 1){
    //         //     spear_true[1] = false;
    //         //     spear_true[5] = true;
    //         // }else if(spear_count == 2){
    //         //     spear_true[5] = false;
    //         //     spear_true[2] = true;
    //         //     // next_hold = false;
    //         // }else if(spear_count == 3){
    //         //     spear_true[2] = false;
    //         //     spear_true[3] = true;
    //         // }else if(spear_count == 4){
    //         //     spear_true[3] = false;
    //         //     spear_true[0] = true;
    //         //     next_hold = false;
    //         // }
    //         spear_count++;
    //         phase = 1;
    //     }else {//forest行き
    //         phase = 7;
    //     }
    // break;
    // //---------------------------------------------------------
    // //test用前言ってからforest前に移動
    // case 107:
    //     motion.setPathNum(0, 0);
    //     setConvPara(0.01, 0.998);
    //     set_para(2.6, gPosi.y, 0.000, 1.0, 3.00, 3.00);
    //     phase = 117;
    // break;
    // case 117:
    //     refV = pathTrackingMode(FOLLOW_COMMAND, 0, 7, DEFAULT);
    // break;
    // //----------------------------------------------------------
    // //forest前まで移動する動作
    // case 7://forest前まで移動
    //     motion.setPathNum(0, 0);
    //     setConvPara(0.01, 0.998);
    //     posifront_count = 0;
    //     posifront_count_first = 0;
    //     posifront_count_second = 0;
    //     for(int i = 0; i < 6; i++){//前2行を見る
    //         if(astar.R2KFS_posi[i]){
    //             if(i < 3){
    //                 posifront_count_first++;
    //             }else{
    //                 posifront_count_second++;
    //             }
    //             posifront_count++;
    //         }
    //     }
    //     if(posifront_count < 2){//前2行に1つしかないとき
    //         if(astar.R2KFS_posi[2]){//内側
    //             forestfrontPosi = astar.forestPosi2[2];
    //             cubePosi = {-1, 0};
    //         }else if(astar.R2KFS_posi[0]){//外側
    //             forestfrontPosi = astar.forestPosi2[0];
    //             cubePosi = {-1, 2};
    //         }else if(astar.R2KFS_posi[1]){//真ん中　最優先にする
    //             forestfrontPosi = astar.forestPosi2[1];
    //             cubePosi = {-1, 1};
    //         }else{//最初の3マスに無かった時→2行目で判断
    //             if(astar.R2KFS_posi[5]){//内側
    //                 forestfrontPosi = astar.forestPosi2[2];
    //                 cubePosi = {-1, 0};
    //             }else if(astar.R2KFS_posi[3]){//外側
    //                 forestfrontPosi = astar.forestPosi2[0];
    //                 cubePosi = {-1, 2};
    //             }else if(astar.R2KFS_posi[4]){//真ん中　最優先にする
    //                 forestfrontPosi = astar.forestPosi2[1];
    //                 cubePosi = {-1, 1};
    //             }else{//とりあえず真ん中
    //                 forestfrontPosi = astar.forestPosi2[1];
    //                 cubePosi = {-1, 1};
    //             }
    //         }
    //     }else {//前2行に2つ以上あるとき
    //         if(astar.R2KFS_posi[0] && astar.R2KFS_posi[1] && astar.R2KFS_posi[2]){//1行目すべてにあるとき
    //             //2行目の状況で判断する．→3つ全てあるときは後1個だけだから，2行目に無いところから回収
    //             if(astar.R2KFS_posi[5]){//内側にあるとき
    //                 forestfrontPosi = astar.forestPosi2[0];
    //                 cubePosi = {-1, 2};
    //             }else if(astar.R2KFS_posi[3]){//外側にあった時
    //                 forestfrontPosi = astar.forestPosi2[2];
    //                 cubePosi = {-1, 0};
    //             }else if(astar.R2KFS_posi[4]){//真ん中にあるとき　内側を最優先にする→そのあと外側
    //                 first_center_flag2 = true;
    //                 forestfrontPosi = astar.forestPosi2[2];
    //                 cubePosi = {-1, 0};
    //             }else{//2行目にはなかったとき→とりあえず近いフィールドの内側から
    //                 forestfrontPosi = astar.forestPosi2[2];
    //                 cubePosi = {-1, 0};
    //             }
    //         }else if(astar.R2KFS_posi[0] || astar.R2KFS_posi[1] || astar.R2KFS_posi[2]){//1行目のどこかにあるとき
    //             if(posifront_count_first < 2){//1行目に1個だけの時
    //                 if(astar.R2KFS_posi[2]){//内側
    //                     forestfrontPosi = astar.forestPosi2[2];
    //                     cubePosi = {-1, 0};
    //                 }else if(astar.R2KFS_posi[0]){//外側
    //                     forestfrontPosi = astar.forestPosi2[0];
    //                     cubePosi = {-1, 2};
    //                 }else if(astar.R2KFS_posi[1]){//真ん中
    //                     forestfrontPosi = astar.forestPosi2[1];
    //                     cubePosi = {-1, 1};
    //                 }
    //             }else{//1行目に2個あるとき(3個は↑にあるのでない)→2行目にあったらそこを最後にする．
    //                 if(posifront_count_second > 0){//2行目にあれば（1行目と2行目どちらにもある．）
    //                     if(astar.R2KFS_posi[2] && astar.R2KFS_posi[5]){//内側
    //                         if(astar.R2KFS_posi[0]){
    //                             forestfrontPosi = astar.forestPosi2[0];
    //                             cubePosi = {-1, 2};
    //                         }else if(astar.R2KFS_posi[1]){
    //                             forestfrontPosi = astar.forestPosi2[1];
    //                             cubePosi = {-1, 1};
    //                         }
    //                     }else if(astar.R2KFS_posi[0] && astar.R2KFS_posi[3]){//外側
    //                         if(astar.R2KFS_posi[1]){
    //                             forestfrontPosi = astar.forestPosi2[1];
    //                             cubePosi = {-1, 1};
    //                         }else if(astar.R2KFS_posi[2]){
    //                             forestfrontPosi = astar.forestPosi2[2];
    //                             cubePosi = {-1, 0};
    //                         }
    //                     }else if(astar.R2KFS_posi[1] && astar.R2KFS_posi[4]){//真ん中
    //                         if(astar.R2KFS_posi[0]){
    //                             forestfrontPosi = astar.forestPosi2[0];
    //                             cubePosi = {-1, 2};
    //                         }else if(astar.R2KFS_posi[2]){
    //                             forestfrontPosi = astar.forestPosi2[2];
    //                             cubePosi = {-1, 0};
    //                         }
                            
    //                     }
    //                 }else{//2行目に無い
    //                     if(astar.R2KFS_posi[2]){//内側
    //                         forestfrontPosi = astar.forestPosi2[2];
    //                         cubePosi = {-1, 0};
    //                     }else if(astar.R2KFS_posi[1]){//真ん中
    //                         forestfrontPosi = astar.forestPosi2[1];
    //                         cubePosi = {-1, 1};
    //                     }else if(astar.R2KFS_posi[0]){//外側
    //                         forestfrontPosi = astar.forestPosi2[0];
    //                         cubePosi = {-1, 2};
    //                     }
    //                 }
    //             }
    //         }else{//1行目のどこにもない時→2行目にある状態
    //             if(astar.R2KFS_posi[5]){//内側
    //                 forestfrontPosi = astar.forestPosi2[2];
    //                 cubePosi = {-1, 0};
    //             }else if(astar.R2KFS_posi[3]){//外側
    //                 forestfrontPosi = astar.forestPosi2[0];
    //                 cubePosi = {-1, 2};
    //             }else if(astar.R2KFS_posi[4]){//真ん中　最優先にする
    //                 forestfrontPosi = astar.forestPosi2[1];
    //                 cubePosi = {-1, 1};
    //             }else{//とりあえず真ん中
    //                 forestfrontPosi = astar.forestPosi2[1];
    //                 cubePosi = {-1, 1};
    //             }
    //         }
    //     }
        
    //     set_para(forestfrontPosi.x, forestfrontPosi.y, 0.000, 1.0, 3.00, 3.00);
    //     // if ((nextPhase & PUSH_BUTTON) == PUSH_BUTTON) {
    //         phase = 8;
    //     // }
    // break;
    // case 8://forest前まで移動
    //     refV = pathTrackingMode(FOLLOW_COMMAND, 0, 9, DEFAULT);
    // break;
    // case 9://ここからforest移動
    //     posifront_count = 0;
    //     for(int i = 0; i < 6; i++){
    //         if(astar.R2KFS_posi[i]){
    //             posifront_count++; 
    //         }
    //     }
    //     if ((nextPhase & PUSH_BUTTON) == PUSH_BUTTON) {
    //         if(posifront_count >= 2){
    //             astar.make_plusdist_stepwise();
    //             if(astar.dist_plus_x > 0 && posifront_count_first < 2){
    //                 phase = 20;
    //             }else{
    //                 return_mode = true;
    //                 phase = 20;//移動しきらないで戻る（forestに入らない）
    //                 // phase = 20;
    //             }
    //         }else{
    //             rotate_radian = 0.000;
    //             phase = 20;//旋回動作へ
    //         }
    //     }
    // break;
    // case 20://旋回
    //     astar.make_plusdist_stepwise();
    //     if(cubePosi.x != -1){
    //         // if(!astar.waitobj && !astar.empty_flag){

    //         // }
    //         if(astar.dist_plus_x > 0){//rad -> +:右回り　-:左回り
    //             rotate_radian = 0.000;
    //         }else if(astar.dist_plus_x < 0){
    //             rotate_radian = M_PI;
    //         }else if(astar.dist_plus_y > 0){//右
    //             rotate_radian = +M_PI/2;
    //         }else if(astar.dist_plus_y < 0){//左
    //             rotate_radian = -M_PI/2;
    //         }
    //     }else{
    //         rotate_radian = 0.000;
    //     }
    //     if(cubePosi.x == -1){
    //         if ((nextPhase & PUSH_BUTTON) == PUSH_BUTTON) {
    //             motion.setPathNum(0, 0);
    //             setConvPara(0.02, 0.997);
    //             set_para(gPosi.x, gPosi.y, rotate_radian, 2.0, 3.00, 3.00);
    //             if(next_box_state == 2 || next_box_state == 0){
    //                 phase = 21;
    //             }
    //         }
    //     }else{
    //         if ((nextPhase & PUSH_BUTTON) == PUSH_BUTTON) {//旋回をする前までに状態を見る．box取ってから旋回
    //             // if(!astar.waitobj && !astar.empty_flag){
    //                 motion.setPathNum(0, 0);
    //                 setConvPara(0.02, 0.997);
    //                 set_para(gPosi.x, gPosi.y, rotate_radian, 2.0, 3.00, 3.00);
    //                 if(next_box_state == 2 || next_box_state == 0){
    //                     phase = 21;
    //                 }
    //             // }
    //         }
    //     }
    // break;
    // case 21://旋回→マス移動へ
    //     if(astar.samePosi_flag || astar.samePosi_flag_Adjacent){//移動しきらない
    //         not_step = true;
    //         refV = pathTrackingMode(POSITION_PID, 0, 10, DEFAULT);
    //     }else{
    //         not_step = false;
    //         refV = pathTrackingMode(POSITION_PID, 0, 10, DEFAULT);//マス移動
    //     }
    // break;
    // case 201://旋回した後に位置を合わせる．（値設定）

    // break;
    // case 211://旋回した後に位置を合わせる．

    // break;
    // case 10: //マス目移動（通常）次のマス手前まで移動
    //     motion.setPathNum(0, 0);
    //     setConvPara(0.03, 0.995);
    //     if(flag_change){
    //         astar.make_plusdist_stepwise();//一回ここ消す
    //     }
    //     // if ((nextPhase & PUSH_BUTTON) == PUSH_BUTTON) {
    //         if(!astar.waitobj && !astar.empty_flag){
    //             if(next_box_state == 2 && KFS_height_state == 1 && !(nextIndex == 13 && cubePosi.x == -1 && cubePosi.y == 1)){//段降り時のbox取得時は移動距離変更．
    //                 motion.setPathNum(0, 0);
    //                 setConvPara(0.03, 0.995);
    //                 //ここで位置補正する．
    //                 if(astar.dist_plus_x > 0){//rad -> +:右回り　-:左回り
    //                     // set_para(astar.tar_posi_x - 0.175 - 0.535 - 0.146, astar.tar_posi_y, rotate_radian, 0.3, 3.00, 1.00);//前向きの時
    //                     set_para(astar.tar_posi_x, astar.tar_posi_y, rotate_radian, 0.3, 3.00, 1.00);
    //                 }else if(astar.dist_plus_x < 0){//後ろ
    //                     // set_para(astar.tar_posi_x + 0.175 + 0.535 + 0.146, astar.tar_posi_y, rotate_radian, 0.3, 3.00, 1.00);
    //                     set_para(astar.tar_posi_x, astar.tar_posi_y, rotate_radian, 0.3, 3.00, 1.00);
    //                 }else if(astar.dist_plus_y > 0){//右
    //                     // set_para(astar.tar_posi_x, astar.tar_posi_y - 0.175 - 0.535 - 0.146, rotate_radian, 0.3, 3.00, 1.00);
    //                     set_para(astar.tar_posi_x, astar.tar_posi_y, rotate_radian, 0.3, 3.00, 1.00);
    //                 }else if(astar.dist_plus_y < 0){//左
    //                     // set_para(astar.tar_posi_x, astar.tar_posi_y + 0.175 + 0.535 + 0.146, rotate_radian, 0.3, 3.00, 1.00);
    //                     set_para(astar.tar_posi_x, astar.tar_posi_y, rotate_radian, 0.3, 3.00, 1.00);
    //                 }
    //                 // set_para(astar.tar_posi_x , astar.tar_posi_y, rotate_radian, 0.3, 3.00, 1.00);//前向きの時
    //                 // if(lift_check){
    //                     phase = 11; 
    //                 // }
    //             }else if(nextIndex == 13 && cubePosi.x == -1 && cubePosi.y == 1){//ここ段越えない
    //                 motion.setPathNum(0, 0);
    //                 setConvPara(0.03, 0.995);
    //                 if(next_box_state == 0){//13の時でbox何もない時
    //                     set_para(astar.tar_posi_x, astar.tar_posi_y, rotate_radian, 0.3, 3.00, 1.00);//前向きの時
    //                 }else if(next_box_state == 2){//13の時でbox回収回収時
    //                     set_para(astar.tar_posi_x - 0.6 - 0.296, astar.tar_posi_y, rotate_radian, 0.3, 3.00, 1.00);
    //                     // set_para(astar.tar_posi_x , astar.tar_posi_y, rotate_radian, 0.3, 3.00, 1.00);
    //                 }
    //                 phase = 11;
    //             }else{
    //                 motion.setPathNum(0, 0);
    //                 setConvPara(0.03, 0.995);
    //                 // if(astar.dist_plus_x > 0){//rad -> +:右回り　-:左回り
    //                 //     set_para(astar.tar_posi_x - 0.6 - 0.296, astar.tar_posi_y, rotate_radian, 0.3, 3.00, 1.00);//前向きの時
    //                 // }else if(astar.dist_plus_x < 0){//後ろ
    //                 //     set_para(astar.tar_posi_x + 0.6 + 0.296, astar.tar_posi_y, rotate_radian, 0.3, 3.00, 1.00);
    //                 // }else if(astar.dist_plus_y > 0){//右
    //                 //     set_para(astar.tar_posi_x, astar.tar_posi_y - 0.6 - 0.296, rotate_radian, 0.3, 3.00, 1.00);
    //                 // }else if(astar.dist_plus_y < 0){//左
    //                 //     set_para(astar.tar_posi_x, astar.tar_posi_y + 0.6 + 0.296, rotate_radian, 0.3, 3.00, 1.00);
    //                 // }
    //                 // set_para(astar.tar_posi_x - 0.6 - 0.296 + 0.1, astar.tar_posi_y, rotate_radian, 0.3, 3.00, 1.00);//前向きの時
                    
    //                 // if(not_step){
    //                 //     set_para(astar.now_posi_x , astar.now_posi_y, rotate_radian, 0.3, 3.00, 1.00);
    //                 // }else{
    //                     set_para(astar.tar_posi_x , astar.tar_posi_y, rotate_radian, 0.3, 3.00, 1.00);
    //                 // }
                    
    //                 // if(flag_change){
    //                 //     // astar.make_plusdist_stepwise();//一回ここ消す
    //                 //     double path_x[4] = {gPosi.x, astar.tar_posi_x, astar.tar_posi_x, astar.tar_posi_x}; //ハーフライン前まで移動
    //                 //     double path_y[4] = {gPosi.y, astar.tar_posi_y, astar.tar_posi_y, astar.tar_posi_y};
    //                 //     motion.makePath(0, path_x, path_y, -rotate_radian);
    //                 //     motion.setParam();
    //                 //     motion.calcAccDecParam_re(0, 0, true, 1.0, 3.0, 3.0);
    //                 // }   
    //                 // phase = 11;
    //                 if(lift_check){
    //                     phase = 11; 
    //                 }
    //             }
    //             if(flag_simu){
    //                 phase = 11; 
    //             }
    //         }
    //         // if(astar.empty_steps < astar.empty_total){//障害物にふさがれたとき
    //         //     // astar.empty_steps = astar.empty_total - 1;
    //         //     phase = 11;
    //         // }
    //     // }
    // break;
    // case 11://動作　マス移動→段の端に行くまで（上るときは昇降が上がったら移動）
    //     step_flag = false;
    //     if(nextIndex == 13){
    //         if(next_box_state != 2){
    //             refV = pathTrackingMode(FOLLOW_COMMAND, 0, 121, DEFAULT);
    //         }else{
    //             refV = pathTrackingMode(FOLLOW_COMMAND, 0, 111, DEFAULT);
    //         }
    //     }else{
    //         if(next_box_state != 2){
    //             refV = pathTrackingMode(FOLLOW_COMMAND, 0, 121, DEFAULT);
    //         }else{
    //             refV = pathTrackingMode(FOLLOW_COMMAND, 0, 111, DEFAULT);
    //         }
            
    //     }
    //     //今はただ次のマスの中心に移動しているだけ
    //     //実際にはここに回収から段越えの動作を入れる．
    //     //旋回して角度を整えた後に段越えを行う．
    //     //回収して段を超えて枠の中心に来たら次のマスに進む
    //     // if ((nextPhase & PUSH_BUTTON) == PUSH_BUTTON) {//段差読んだら強制で次のphase
    //     //         phase = 111;
    //     // }
    // break;
    // case 111://11のphaseで段差に近づいたら111に来る．（LRTBか光電）
    //     //マニュアル操作-> ここで段越え
    //     step_flag = true;
    //     if(flag_simu){//|| nextIndex == 15 || nextIndex == 16 || nextIndex == 17
    //         phase = 121;
    //     }
    // break;
    // case 121://登ったらこのphase に移行してマスの中心までもっていく（経路値設定）
    //     motion.setPathNum(0, 0);
    //     setConvPara(0.01, 0.999);
    //     // if(not_step){//上らないとき
    //     //     set_para(astar.now_posi_x, astar.now_posi_y, gPosi.z, 0.2, 1.00, 1.00);
    //     // }else{
    //         set_para(astar.tar_posi_x, astar.tar_posi_y, rotate_radian, 0.2, 1.00, 1.00);
    //     // }
    //     phase = 131;
    // break;
    // case 131://マスの中心に移動
    //     refV = pathTrackingMode(FOLLOW_COMMAND, 0, 141, DEFAULT);
    // break;
    // case 141://中心で位置合わせ
    //     motion.setPathNum(0, 0);
    //     setConvPara(0.01, 0.999);
    //     set_para(astar.tar_posi_x, astar.tar_posi_y, rotate_radian, 0.2, 1.00, 1.00);
    //     phase = 151;
    // break;
    // case 151:
    //     refV = pathTrackingMode(FOLLOW_COMMAND, 0, 12, DEFAULT);
    // break;
    // case 12://マス位置更新 → phase20で旋回
    //     if(!astar.goal_flag){
    //         posifront_count = 0;
    //         for(int i = 0; i < 3; i++){
    //             if(astar.R2KFS_posi[i]){
    //                 posifront_count++;
    //             }
    //         }
    //         if(cubePosi.x == -1 && (astar.R2KFS_posi[0] || astar.R2KFS_posi[1] || astar.R2KFS_posi[2]) && posifront_count >= 2){
    //             if(posifront_count >= 2 ){
    //                 // astar.make_plusdist_stepwise();
    //                 return_mode = true;
    //                 phase = 20;//移動しきらないで戻る（forestに入らない）
    //                 // phase = 20;
    //             }else{
    //                 rotate_radian = 0.000;
    //                 phase = 20;//旋回動作へ
    //             } 
    //             if(!astar.empty_flag){//障害物にふさがれていないときだけ
    //                 // if(!not_step){//段越えするときだけしないときは更新しない
    //                     cubePosi.x = astar.nextX;//収束したら位置を更新する．
    //                     cubePosi.y = astar.nextY;
    //                 // }
    //             }
    //         }else{
    //             if(!astar.empty_flag){//障害物にふさがれていないときだけ
    //                 // if(!not_step){//段越えしないときだけ
    //                     cubePosi.x = astar.nextX;//収束したら位置を更新する．
    //                     cubePosi.y = astar.nextY;
    //                 // }
    //             }
    //             if(cubePosi.x == 4){//出口に到達
    //                 if(cubePosi.y == 0){
    //                     slopefront_vel = 0.5;
    //                 }else if(cubePosi.y == 1){
    //                     slopefront_vel = 0.5;
    //                 }else if(cubePosi.y == 2){
    //                     slopefront_vel = 0.5;
    //                 }
    //                 phase = 13;//坂前
    //             }else{
    //                 if(astar.empty_steps < astar.empty_total - 1){
    //                     astar.empty_steps++;
    //                 }
    //                 // if ((nextPhase & PUSH_BUTTON) == PUSH_BUTTON) {
    //                     phase = 20;//旋回
    //                 // }
    //             }
    //         }
    //     }else{
    //         slopefront_vel = 0.5;
    //         phase = 13;//坂前
    //     }
    // break;
    // case 13:
    //     motion.setPathNum(0, 0);
    //     setConvPara(0.03, 0.995);
    //     set_para(8.65, 5.25, 0.000, slopefront_vel, 3.00, 3.00);
    //     if ((nextPhase & PUSH_BUTTON) == PUSH_BUTTON) {
    //         phase = 14;
    //     }
    // break;
    // case 14://坂の前まで移動
    //     refV = pathTrackingMode(FOLLOW_COMMAND, 0, 50, DEFAULT);//ゾーン3移動
    // break;
    // //-----------------------------------------------------------------
    // //マス移動のイレギュラー移動しないで現在のマスからとる場合
    // case 30://マスの端に移動（値設定）　phase が12に行かないのでマス自己位置の更新がないから気を付ける．
    //     motion.setPathNum(0, 0);
    //     setConvPara(0.05, 0.992);
    //     set_para(astar.tar_posi_x, astar.tar_posi_y, rotate_radian, 0.3, 3.00, 3.00);
    //     phase = 31;
    // break;
    // case 31://マスの端に移動
    //     refV = pathTrackingMode(FOLLOW_COMMAND, 0, 310, DEFAULT);
    // break;
    // case 310://phase 111
    //     //マニュアル操作-> ここで段越え
    //     step_flag = true;
    //     if(flag_simu){
    //         phase = 32;
    //     }
    // break;
    // case 32://取得したらマスの中心に戻る（値設定）
    //     motion.setPathNum(0, 0);
    //     setConvPara(0.05, 0.992);
    //     set_para(astar.now_posi_x, astar.now_posi_y, rotate_radian, 0.3, 3.00, 3.00);
    //     // astar.samePosi_flag = false;
    //     if ((nextPhase & PUSH_BUTTON) == PUSH_BUTTON) {
    //         phase = 33;
    //     }
    // break;
    // case 33://取得したらマスの中心に戻る
    //     refV = pathTrackingMode(FOLLOW_COMMAND, 0, 20, DEFAULT);
    // break;
    // //--------------------------------------------------------
    // //初期動作，forestに入らないでKFSの取得
    // case 40://マスの端に移動（値設定）　phase が12に行かないのでマス自己位置の更新がないから気を付ける．
    //     motion.setPathNum(0, 0);
    //     setConvPara(0.05, 0.992);
    //     set_para(gPosi.x+0.4, gPosi.y, rotate_radian, 0.3, 3.00, 3.00);
    //     phase = 41;
    // break;
    // case 41://マスの端に移動(実質phase11)
    //     refV = pathTrackingMode(FOLLOW_COMMAND, 0, 401, DEFAULT);
    // break;
    // case 401://phase 111と一緒
    //     //マニュアル操作-> ここで段越え
    //     step_flag = true;
    //     if(flag_simu){
    //         phase = 42;
    //     }
    // break;
    // case 42://取得したらマスの中心に戻る（値設定）
    //     motion.setPathNum(0, 0);
    //     setConvPara(0.05, 0.992);
    //     set_para(gPosi.x-0.4, gPosi.y, rotate_radian, 0.3, 3.00, 3.00);
    //     // astar.samePosi_flag = false;
    //     if ((nextPhase & PUSH_BUTTON) == PUSH_BUTTON) {//上半身からの取得判定に置き換える．
    //         phase = 43;
    //     }
    // break;
    // case 43://取得したらマスの中心に戻る
    //     if(!first_center_flag2){
    //         refV = pathTrackingMode(FOLLOW_COMMAND, 0, 20, DEFAULT);
    //     }else {//通路側からとるとき
    //         refV = pathTrackingMode(FOLLOW_COMMAND, 0, 44, DEFAULT);
    //     }
    // break;
    // case 44://右側のマスに移動する．
    //     first_center_flag2 = false;
    //     motion.setPathNum(0, 0);
    //     setConvPara(0.05, 0.992);
    //     if(cubePosi.y == 0){
    //         forestfrontPosi = astar.forestPosi2[0];
    //         cubePosi = {-1, 2};
    //     }else if(cubePosi.y == 2){
    //         forestfrontPosi = astar.forestPosi2[0];
    //         // cubePosi = {-1, 0};
    //     }
    //     set_para(forestfrontPosi.x, forestfrontPosi.y, rotate_radian, 3.0, 3.00, 3.00);
    //     // astar.samePosi_flag = false;
    //     // if ((nextPhase & PUSH_BUTTON) == PUSH_BUTTON) {
    //         phase = 45;
    //     // }
    // break;
    // case 45:
    //     refV = pathTrackingMode(FOLLOW_COMMAND, 0, 9, DEFAULT);
    // break;

    // //---------------------------------------------------------------
    // //ゾーン3前からR1に乗るまで
    // case 50://リトライ用
    //     motion.setPathNum(0, 0);
    //     setConvPara(0.01, 0.998);
    //     if(flag_change){
    //         double path_x[4] = {gPosi.x, 11.0, 11.0, 11.0};
    //         double path_y[4] = {gPosi.y, 5.25, 5.25, 2.0};
    //         set_para2(path_x, path_y, -M_PI/2, 1.0, 4.00, 3.00);
    //     }
    //     phase = 51;
    // break;
    // case 51://ラックから少し離れたところに移動する
    //     // refV = pathTrackingMode(FOLLOW_COMMAND, 0, 611, DEFAULT);
    //     phase = 611;
    // break;
    // //----------------------------------------------------
    // case 60://リトライ用
    //     motion.setPathNum(0, 0);
    //     setConvPara(0.01, 0.998);
    //     // if(flag_change){
    //     //     double path_x[4] = {gPosi.x, 11.0, 11.4, 11.4};
    //     //     double path_y[4] = {gPosi.y, 5.25, 5.25, 1.0};
    //     //     set_para2(path_x, path_y, -M_PI/2, 1.0, 4.00, 3.00);
    //     // }
    //     set_para(10.75, 2.0, -M_PI/2, 0.5, 3.00, 3.00);
    //     phase = 61;
    // break;
    // case 61://ラックから少し離れたところに移動する
    //     // refV = pathTrackingMode(FOLLOW_COMMAND, 0, 611, DEFAULT);
    //     phase = 611;
    // break;
    // case 611:
    //     send_num = 20;
    //     if ((nextPhase & PUSH_BUTTON) == PUSH_BUTTON) {
    //         phase = 62;
    //     }
    // break;
    // case 62://ここでどこに行くかを選択する．
    //     motion.setPathNum(0, 0);
    //     setConvPara(0.01, 0.998);
    //     if ((nextPhase & PUSH_BUTTON) == PUSH_BUTTON) {
    //         tar_posi_kfs_rack_y = kfs_rack_posi_y;
    //         // if(rack_num == 0){
    //         //     tar_posi_kfs_rack_x = kfs_rack_posi_x[0];
    //         // }else if(rack_num == 1){
    //         //     tar_posi_kfs_rack_x = kfs_rack_posi_x[1];
    //         // }else if(rack_num == 2){
    //         //     tar_posi_kfs_rack_x = kfs_rack_posi_x[2];
    //         // }
    //         tar_posi_kfs_rack_x = kfs_rack_posi_x[1];
    //         set_para(tar_posi_kfs_rack_x, tar_posi_kfs_rack_y, -M_PI/2, 0.5, 3.00, 3.00);
    //         phase = 63;
    //     }
    // break;
    // case 63:
    //     // refV = pathTrackingMode(FOLLOW_COMMAND, 0, 64, DEFAULT);
    //     phase = 64;
    // break;
    // case 64://ここで昇降動かす．接地動作
    //     if ((nextPhase & PUSH_BUTTON) == PUSH_BUTTON) {
    //         if(set_KFS_height){
    //             send_num = 21;//おいてもらう．
    //             phase = 640;
    //         }  
    //     }
    // break;
    // case 640://後ろに下がる．
    //     if ((nextPhase & PUSH_BUTTON) == PUSH_BUTTON) {
    //         phase = 60;
    //     }
    // break;      
 }

  pre_refV = refV;
  for (int i = 0; i < 12; i++) {
    // astar.pre_obj_true[i] = button_true[i];
    pre_button_true_R1[i] = button_true_R1[i];
    pre_button_true_R2[i] = button_true_R2[i];
    pre_button_true_Fake[i] = button_true_Fake[i];
  }

  pre_path_num = motion.getPathNum();
  if(phase != pre_phase) {
    pre_under_phase = pre_phase;
    pre_phase = phase;
    change_phase = true;
  }
  else{
    change_phase = false;
  }
  return refV;
}