//-----------------------------------------
// 軌道追従や位置のPID制御を行うためのクラス
// 作成：2019/05/15 by Yuki Ueno
// 編集：Miki Nakaone
//-----------------------------------------
#include "PathTracking.h"
#include <cmath>
#include <vector>  // std::vector
#include <utility> // std::pair, std::make_pair
#include <algorithm>

extern coords gPosi;
extern int field;

PID posiPIDx(POSI_X_KP, POSI_X_KI, POSI_X_KD, INT_TIME);
PID posiPIDy(POSI_Y_KP, POSI_Y_KI, POSI_Y_KD, INT_TIME);
PID posiPIDz(POSI_Z_KP, POSI_Z_KI, POSI_Z_KD, INT_TIME);

PID yokozurePID(YOKOZURE_KP, YOKOZURE_KI, YOKOZURE_KD,
                INT_TIME); //(3.0, 0.0, 0.0, INT_TIME);
PID kakudoPID(KAKUDO_KP, KAKUDO_KI, KAKUDO_KD, INT_TIME);

// 二次遅れ使えるようになる
Filter sokduo_filter(INT_TIME);
Filter kakudo_filter(INT_TIME);

// コンストラクタ
PathTracking::PathTracking(int xmode) {
  path_num = 0;
  max_pathnum = 0;
  t_be = 0.0;
  pre_t_be = 0.1;
  t_deadBand = 0.0;
  epsilon = 1.0;

  pre_refType = TYPE_START;

  refVx = 0.0;
  refVy = 0.0;
  refVz = 0.0;

  conv_length = 0.02;
  conv_tnum = 0.997;
  conv_theta = 0.0174;

  count_acc = 0;

  realPosi_x = gPosi.x;
  pre_realPosi_x = realPosi_x;
  realPosi_y = gPosi.y;
  pre_realPosi_y = realPosi_y;

  realVel = 0;

  dist_deadBand = 0;

  mode = xmode;

  mode_changed = true;
  init_done = false;

//DWA
  s = {2.8,3.0,0,0,0};
  goalX = 7.4, goalY = 3.6;
  obstacleR = 0.6;
  obs = {
      {3.8, 1.8}, {5.0, 3.6}
    //  {3.8, 1.8}, {5.0, 3.6}, {6.8, 3.6}, {6.8, 3.0}
    //   {3.2, 1.2}, {4.4, 1.2}, {5.6, 1.2}, {6.8, 1.2},
    //   {8.0, 1.2}, {8.0, 2.4}, {8.0, 3.6}, {8.0, 4.8},
    //   {6.8, 4.8}, {5.6, 4.8}, {4.4, 4.8}, {3.2, 4.8},
    //   {3.2, 3.6}, {3.2, 2.4},
    //   {4.4, 2.4}, {4.4, 3.6}, {5.6, 2.4}, {5.6, 3.6}, {6.8, 2.4}, {6.8, 3.6}
  };//カメラなどから場所を受け取るときは更新する．
}

void PathTracking::posiDataUpdate(
    double
        _realVel) { //毎周期更新してほしい（INT_TIME毎）
                    ////引数に何も入れなければ勝手に計算されるので，入れなくてもいい
  if (_realVel == -9999) {
    pre_realPosi_x = realPosi_x;
    pre_realPosi_y = realPosi_y;
    realPosi_x = gPosi.x;
    realPosi_y = gPosi.y;

    realVel = sqrt(pow(realPosi_x - pre_realPosi_x, 2.0) +
                   pow(realPosi_y - pre_realPosi_y, 2.0)) /
              INT_TIME;
  } else {
    realVel = _realVel;
  }

  double acc = 0;
  double time = 0;

  switch (pre_refType) {
  case TYPE_START:
    acc = acc_param[pathNum_deadBand];
    time = acc * DEAD_BAND_;
    break;
  case TYPE_NORMAL:
    acc = 0;
    time = dec_param[pathNum_deadBand] * DEAD_BAND_;
    break;
  case TYPE_ACC:
    acc = LIMIT_ACC;
    time = acc * DEAD_BAND_;
    break;
  case TYPE_DEC:
    acc = -LIMIT_ACC;
    time = -acc * DEAD_BAND_;
    break;
  case TYPE_STOP:
    acc = dec_param[pathNum_deadBand];
    time = acc * DEAD_BAND_;
    break;
  }

  //無駄時間中に進む距離の計算．
  dist_deadBand = realVel * time + 0.5 * acc * pow(time, 2.0);
  if (dist_deadBand < 0)
    dist_deadBand = 0;
}

// tを求めるための方程式
double PathTracking::func(int p, double t) {
  return ((a_be[p] * pow(t, 5.0)) + (b_be[p] * pow(t, 4.0)) +
          (c_be[p] * pow(t, 3.0)) + (d_be[p] * pow(t, 2.0)) + (e_be[p] * t) +
          f_be[p]);
}
// tを求めるための方程式の1階微分
double PathTracking::dfunc(int p, double t) {
  return 5.0 * a_be[p] * pow(t, 4.0) + 4.0 * b_be[p] * pow(t, 3.0) +
         3.0 * c_be[p] * pow(t, 2.0) + 2.0 * d_be[p] * t + e_be[p];
}

// tにおけるベジエ曲線の座標を求める関数
double PathTracking::bezier_x(int p, double t) {
  return Ax[p] * pow(t, 3.0) + 3.0 * Bx[p] * pow(t, 2.0) + 3.0 * Cx[p] * t +
         Dx[p];
}
double PathTracking::bezier_y(int p, double t) {
  return Ay[p] * pow(t, 3.0) + 3.0 * By[p] * pow(t, 2.0) + 3.0 * Cy[p] * t +
         Dy[p];
}

// ベジエ曲線式の1階微分
double PathTracking::dbezier_x(int p, double t) {
  return 3.0 * Ax[p] * pow(t, 2.0) + 6.0 * Bx[p] * t + 3.0 * Cx[p];
}
double PathTracking::dbezier_y(int p, double t) {
  return 3.0 * Ay[p] * pow(t, 2.0) + 6.0 * By[p] * t + 3.0 * Cy[p];
}

// カーブの方程式
double PathTracking::func2(double phi) {
  return (tar_posi.x - curve_ox) * cos(phi) +
         (tar_posi.y - curve_oy) * sin(phi) - curve_r;
}

double PathTracking::dfunc2(double phi) {
  return -(tar_posi.x - curve_ox) * sin(phi) +
         (tar_posi.y - curve_oy) * cos(phi);
}

// カーブで交点を求めるための方程式
double PathTracking::curve_x(double phi) {
  return curve_ox + curve_r * cos(phi);
}

double PathTracking::curve_y(double phi) {
  return curve_oy + curve_r * sin(phi);
}

// カーブの1階微分
double PathTracking::dcurve_x(double phi) { return -curve_r * sin(phi); }

double PathTracking::dcurve_y(double phi) { return curve_r * cos(phi); }

// ニュートン法のための係数の初期化
void PathTracking::initSettings() {
  // PID関連初期化
  posiPIDx.PIDinit(0.0, 0.0);
  posiPIDy.PIDinit(0.0, 0.0);
  posiPIDz.PIDinit(0.0, 0.0);

  yokozurePID.PIDinit(0.0, 0.0);
  kakudoPID.PIDinit(0.0, 0.0);

  sokduo_filter.setSecondOrderPara(FILT_SOKUDO_OMEGA, FILT_SOKUDO_DZETA, 0.0);
  kakudo_filter.setSecondOrderPara(FILT_KAKUDO_OMEGA, FILT_KAKUDO_DZETA, 0.0);

  calcPathParam();

  init_done = true;
}

void PathTracking::calcPathParam()
//ベジエの内積等の計算を行う．動き始めの位置の更新をしたらこの関数も動かしてあげる．
{
  for (int i = 0; i < PATHNUM; i++) {
    Ax[i] =
        Px[4 * i + 3] - 3 * Px[4 * i + 2] + 3 * Px[4 * i + 1] - Px[4 * i + 0];
    Ay[i] =
        Py[4 * i + 3] - 3 * Py[4 * i + 2] + 3 * Py[4 * i + 1] - Py[4 * i + 0];
    Bx[i] = Px[4 * i + 2] - 2 * Px[4 * i + 1] + Px[4 * i + 0];
    By[i] = Py[4 * i + 2] - 2 * Py[4 * i + 1] + Py[4 * i + 0];
    Cx[i] = Px[4 * i + 1] - Px[4 * i + 0];
    Cy[i] = Py[4 * i + 1] - Py[4 * i + 0];
    Dx[i] = Px[4 * i + 0];
    Dy[i] = Py[4 * i + 0];
    // printf("path:%d Ax:%lf Bx:%lf Cx:%lf
    // Dx:%lf\n",i,Ax[i],Bx[i],Cx[i],Dx[i]);
  }

  for (int i = 0; i < PATHNUM; i++) {
    a_be[i] = pow(Ax[i], 2.0) + pow(Ay[i], 2.0);
    b_be[i] = 5 * (Ax[i] * Bx[i] + Ay[i] * By[i]);
    c_be[i] = 2 * ((3 * pow(Bx[i], 2.0) + 2 * Ax[i] * Cx[i]) +
                   (3 * pow(By[i], 2.0) + 2 * Ay[i] * Cy[i]));
    d_be_[i] = 9 * Bx[i] * Cx[i] + 9 * By[i] * Cy[i];
    e_be_[i] = 3 * pow(Cx[i], 2.0) + 3 * pow(Cy[i], 2.0);
    f_be_[i] = 0;
  }
  Px0 = pre_Px = Px[path_num];
  Py0 = pre_Py = Py[path_num];
}

//各経路の距離を計算
void PathTracking::calcAccAndDecParam() // AutoControlクラス内で使用
{
  for (int i = 0; i <= max_pathnum; i++) {
    // allL（経路のすべて）の計算
    for (double j = 0; j < 1000; j++)
      allL[i] +=
          pow(pow(bezier_x(i, j / 1000) - bezier_x(i, (j + 1) / 1000), 2) +
                  pow(bezier_y(i, j / 1000) - bezier_y(i, (j + 1) / 1000), 2),
              0.5);

    if (acc_mode[i] == MODE_STOP || acc_mode[i] == MODE_START_STOP) {
      //加速度から加速に必要なカウント数を計算
      //   acc_count[i] = refvel[i] / acc_param[i] / INT_TIME;
      //   printf("%d:%d\n", i, acc_count[i]);

      // decL，accL（減速，加速にかかる距離）の計算
      decL[i] = pow(refvel[i], 2) / fabs(dec_param[i]) / 2;

      if (acc_mode[i] == MODE_START_STOP) {
        //加速度から加速に必要なカウント数を計算
        acc_count[i] = refvel[i] / acc_param[i] / INT_TIME;
        // printf("%d:%d\n", i, acc_count[i]);
        accL[i] = refvel[i] * INT_TIME * acc_count[i] / 2;
        if (allL[i] < (accL[i] + decL[i])) {
          accL[i] = accL[i] * allL[i] / (accL[i] + decL[i]);
          decL[i] = allL[i] - accL[i];
          refvel[i] = sqrt(2 * acc_param[i] * accL[i]);
          acc_count[i] = accL[i] * 2 / refvel[i] / INT_TIME;
          // printf("2 %d:%d:%lf\n", i, acc_count[i],refvel[i]);
        }

      } else if (refvel[i] > refvel[i - 1]) {
        //加速度から加速に必要なカウント数を計算
        acc_count[i] = (refvel[i] - refvel[i - 1]) / acc_param[i] / INT_TIME;
        // printf("%d:%d\n", i, acc_count[i]);
        accL[i] = (refvel[i] + refvel[i - 1]) * INT_TIME * acc_count[i] / 2;
        if (allL[i] < (accL[i] + decL[i])) {
          decL[i] =
              decL[i] *
              (allL[i] + (pow(refvel[i - 1], 2) / 2 / fabs(acc_param[i]))) /
              (accL[i] + decL[i] +
               (pow(refvel[i - 1], 2) / 2 / fabs(acc_param[i])));
          accL[i] = allL[i] - decL[i];
          refvel[i] = sqrt(2 * dec_param[i] * decL[i]);
          acc_count[i] = accL[i] * 2 / (refvel[i] + refvel[i - 1]) / INT_TIME;
          // printf("2 %d:%d:%lf\n", i, acc_count[i],refvel[i]);
        }
      } else {
        acc_count[i] = 0;
        // printf("%d:%d\n", i, acc_count[i]);
      }

      dec_param[i] = pow(refvel[i], 2) / 2 / decL[i];
      // printf("pathnum:%d,acc_count:%d,vel:%+.3lf\n", i,
      // acc_count[i],refvel[i]);
      // printf("mode:%d,accL:%+.3lf,decL:%+.3lf,allL:%+.3lf,decParam:%+.3lf\n",
      // acc_mode[i], accL[i], decL[i], allL[i], dec_param[i]);
    } else if (acc_mode[i] == MODE_START) {
      //加速度から加速に必要なカウント数を計算
      acc_count[i] = refvel[i] / acc_param[i] / INT_TIME;
      //   printf("%d:%d\n", i, acc_count[i]);

      if (refvel[i] > refvel[i + 1]) {
        decL[i] = (refvel[i] + refvel[i + 1]) * (refvel[i] - refvel[i + 1]) /
                  fabs(dec_param[i]) / 2;
        accL[i] = refvel[i] * INT_TIME * acc_count[i] / 2;

        if (allL[i] < (accL[i] + decL[i])) {
          accL[i] =
              accL[i] *
              (allL[i] + (pow(refvel[i + 1], 2) / 2 / fabs(dec_param[i]))) /
              (accL[i] + decL[i] +
               (pow(refvel[i + 1], 2) / 2 / fabs(dec_param[i])));
          decL[i] = allL[i] - accL[i];
          refvel[i] = sqrt(2 * acc_param[i] * accL[i]);
          acc_count[i] = accL[i] * 2 / refvel[i] / INT_TIME;
          // printf("2 %d:%d:%lf\n", i, acc_count[i],refvel[i]);
          dec_param[i] = (refvel[i] + refvel[i + 1]) *
                         (refvel[i] - refvel[i + 1]) / 2 / decL[i];
        }
      }
      // printf("pathnum:%d,acc_count:%d,vel:%+.3lf\n", i,
      // acc_count[i],refvel[i]);
      // printf("mode:%d,accL:%+.3lf,decL:%+.3lf,allL:%+.3lf,decParam:%+.3lf\n",
      // acc_mode[i], accL[i], decL[i], allL[i], dec_param[i]);
    } else if (acc_mode[i] == MODE_NORMAL) {
      if (refvel[i] > refvel[i - 1]) {
        //加速度から加速に必要なカウント数を計算
        acc_count[i] = (refvel[i] - refvel[i - 1]) / acc_param[i] / INT_TIME;
        // printf("%d:%d\n", i, acc_count[i]);
        accL[i] = (refvel[i] + refvel[i - 1]) * INT_TIME * acc_count[i] / 2;
      }

      if (acc_mode[i + 1] == MODE_NORMAL || acc_mode[i + 1] == MODE_STOP) {
        if (refvel[i] > refvel[i + 1]) {
          decL[i] = (refvel[i] + refvel[i + 1]) * (refvel[i] - refvel[i + 1]) /
                    fabs(dec_param[i]) / 2;
          if (allL[i] < (accL[i] + decL[i])) {
            decL[i] = (decL[i] - (refvel[i + 1] * (refvel[i] - refvel[i + 1]) /
                                  fabs(dec_param[i]))) *
                          allL[i] / (accL[i] + decL[i]) +
                      (refvel[i + 1] * (refvel[i] - refvel[i + 1]) /
                       fabs(dec_param[i])) *
                          pow(allL[i] / (accL[i] + decL[i]), 2);
            accL[i] = allL[i] - decL[i];
            refvel[i] =
                sqrt(2 * dec_param[i] * decL[i] - pow(refvel[i + 1], 2));
            acc_count[i] = accL[i] * 2 / refvel[i] / INT_TIME;
            // printf("2 %d:%d:%lf\n", i, acc_count[i],refvel[i]);
            dec_param[i] = (refvel[i] + refvel[i + 1]) *
                           (refvel[i] - refvel[i + 1]) / 2 / decL[i];
          }
        } else {
          decL[i] = -1.0;
        }
      }

      //   if(refvel[i] > refvel[i-1] || (refvel[i] > refvel[i+1]))
      // printf("pathnum:%d,acc_count:%d,vel:%+.3lf\n", i,
      // acc_count[i],refvel[i]);
      // printf("mode:%d,accL:%+.3lf,decL:%+.3lf,allL:%+.3lf,decParam:%+.3lf\n",
      // acc_mode[i], accL[i], decL[i], allL[i], dec_param[i]);
    }

    // dividNumの計算
    divideNum[i] =
        int(allL[i] / CURVE_DEC_UNIT_LENGTH); //全体の距離を0.1mで割っている
  }
}

void PathTracking::calcAccDecParam_re(int start, int end, bool velacc,
                                      double vel, double acc,
                                      double dec) // AutoControlクラス内で使用
{
  int min_num, max_num;
  int sign = 1;
  if (start == end) {
    acc_mode[start] = MODE_START_STOP;
    min_num = start;
    max_num = end;
  } else {
    acc_mode[start] = MODE_START;
    acc_mode[end] = MODE_STOP;
    if (start < end) {
      min_num = start;
      max_num = end;
    } else if (start > end) {
      sign = -1;
      min_num = end;
      max_num = start;
    }
    for (int i = min_num + 1; i < max_num; i++)
      acc_mode[i] = MODE_NORMAL;
  }
  if (flag_const)
    acc_mode[end] = MODE_NORMAL;
  //加速度から加速に必要なカウント数を計算
  for (int i = min_num; i <= max_num; i++) {
    // allL（経路のすべて）の計算
    if (allL[i] == 0.0) {
      for (double j = 0; j < 1000; j++)
        allL[i] +=
            pow(pow(bezier_x(i, j / 1000) - bezier_x(i, (j + 1) / 1000), 2) +
                    pow(bezier_y(i, j / 1000) - bezier_y(i, (j + 1) / 1000), 2),
                0.5);
    }
    decL[i] = 0;
    accL[i] = 0;
    acc_count [i] = 0;

    if (velacc) {
      refvel[i] = vel;
      acc_param[i] = acc;
      dec_param[i] = dec;
    }
  }

  for (int i = min_num; i <= max_num; i++) {
    if (acc_mode[i] == MODE_STOP || acc_mode[i] == MODE_START_STOP) {
      //加速度から加速に必要なカウント数を計算
      //   acc_count[i] = refvel[i] / acc_param[i] / INT_TIME;
      //   printf("%d:%d\n", i, acc_count[i]);

      // decL，accL（減速，加速にかかる距離）の計算
      decL[i] = pow(refvel[i], 2) / fabs(dec_param[i]) / 2;

      if (acc_mode[i] == MODE_START_STOP) {
        //加速度から加速に必要なカウント数を計算
        acc_count[i] = refvel[i] / acc_param[i] / INT_TIME;
        // printf("%d:%d\n", i, acc_count[i]);
        accL[i] = refvel[i] * INT_TIME * acc_count[i] / 2;
        // printf("pathnum:%d,mode:%d,accL:%+.3lf,decL:%+.3lf,allL:%+.3lf,decParam:%+.3lf\n",
        // i, acc_mode[i], accL[i], decL[i], allL[i], dec_param[i]);
        if (allL[i] < accL[i] + decL[i]) {
          accL[i] = accL[i] * allL[i] / (accL[i] + decL[i]);
          decL[i] = allL[i] - accL[i];
          refvel[i] = sqrt(2 * acc_param[i] * accL[i]);
          acc_count[i] = accL[i] * 2 / refvel[i] / INT_TIME;
          // printf("2 %d:%d:%lf\n", i, acc_count[i],refvel[i]);
        }
      } else if (refvel[i] > refvel[i - sign]) {
        //加速度から加速に必要なカウント数を計算
        acc_count[i] = (refvel[i] - refvel[i - sign]) / acc_param[i] / INT_TIME;
        // printf("%d:%d\n", i, acc_count[i]);
        accL[i] = (refvel[i] + refvel[i - sign]) * INT_TIME * acc_count[i] / 2;
        if (allL[i] < accL[i] + decL[i]) {
          decL[i] =
              decL[i] *
              (allL[i] + (pow(refvel[i - sign], 2) / 2 / fabs(acc_param[i]))) /
              (accL[i] + decL[i] +
               (pow(refvel[i - sign], 2) / 2 / fabs(acc_param[i])));
          accL[i] = allL[i] - decL[i];
          refvel[i] = sqrt(2 * dec_param[i] * decL[i]);
          acc_count[i] =
              accL[i] * 2 / (refvel[i] + refvel[i - sign]) / INT_TIME;
          // printf("2 %d:%d:%lf\n", i, acc_count[i],refvel[i]);
        }
      } else {
        acc_count[i] = 0;
        // printf("%d:%d\n", i, acc_count[i]);
      }

      //   if (allL[i] < accL[i] + decL[i])
      //     accL[i] = allL[i] - decL[i];
      dec_param[i] = pow(refvel[i], 2) / 2 / decL[i];
      //   printf("pathnum:%d,mode:%d,accL:%+.3lf,decL:%+.3lf,allL:%+.3lf,decParam:%+.3lf\n",
      //   i, acc_mode[i], accL[i], decL[i], allL[i], dec_param[i]);
    } else if (acc_mode[i] == MODE_START) {
      //加速度から加速に必要なカウント数を計算
      acc_count[i] = refvel[i] / acc_param[i] / INT_TIME;
      //   printf("%d:%d\n", i, acc_count[i]);

      if (refvel[i] > refvel[i + sign]) {
        decL[i] = (refvel[i] + refvel[i + sign]) *
                  (refvel[i] - refvel[i + sign]) / fabs(dec_param[i]) / 2;
        accL[i] = refvel[i] * INT_TIME * acc_count[i] / 2;

        if (allL[i] < (accL[i] + decL[i])) {
          accL[i] =
              accL[i] *
              (allL[i] + (pow(refvel[i + sign], 2) / 2 / fabs(dec_param[i]))) /
              (accL[i] + decL[i] +
               (pow(refvel[i + sign], 2) / 2 / fabs(dec_param[i])));
          decL[i] = allL[i] - accL[i];
          refvel[i] = sqrt(2 * acc_param[i] * accL[i]);
          acc_count[i] = accL[i] * 2 / refvel[i] / INT_TIME;
          // printf("2 %d:%d:%lf\n", i, acc_count[i],refvel[i]);
          dec_param[i] = (refvel[i] + refvel[i + sign]) *
                         (refvel[i] - refvel[i + sign]) / 2 / decL[i];
        }
      }
      //   printf("pathnum:%d,mode:%d,accL:%+.3lf,decL:%+.3lf,allL:%+.3lf,decParam:%+.3lf\n",
      //   i, acc_mode[i], accL[i], decL[i], allL[i], dec_param[i]);
    } else if (acc_mode[i] == MODE_NORMAL) {
      if (refvel[i] > refvel[i - sign]) {
        //加速度から加速に必要なカウント数を計算
        acc_count[i] = (refvel[i] - refvel[i - sign]) / acc_param[i] / INT_TIME;
        // printf("%d:%d\n", i, acc_count[i]);
        accL[i] = (refvel[i] + refvel[i - sign]) * INT_TIME * acc_count[i] / 2;
      }

      if (acc_mode[i + sign] == MODE_NORMAL ||
          acc_mode[i + sign] == MODE_STOP) {
        if (refvel[i] > refvel[i + sign]) {
          decL[i] = (refvel[i] + refvel[i + sign]) *
                    (refvel[i] - refvel[i + sign]) / fabs(dec_param[i]) / 2;
          if (allL[i] < accL[i] + decL[i]) {
            decL[i] =
                (decL[i] - (refvel[i + sign] * (refvel[i] - refvel[i + sign]) /
                            fabs(dec_param[i]))) *
                    allL[i] / (accL[i] + decL[i]) +
                (refvel[i + sign] * (refvel[i] - refvel[i + sign]) /
                 fabs(dec_param[i])) *
                    pow(allL[i] / (accL[i] + decL[i]), 2);
            accL[i] = allL[i] - decL[i];
            refvel[i] =
                sqrt(2 * dec_param[i] * decL[i] - pow(refvel[i + 1], 2));
            acc_count[i] = accL[i] * 2 / refvel[i] / INT_TIME;
            // printf("2 %d:%d:%lf\n", i, acc_count[i],refvel[i]);
            dec_param[i] = (refvel[i] + refvel[i + sign]) *
                           (refvel[i] - refvel[i + sign]) / 2 / decL[i];
          }
        } else {
          decL[i] = -1.0;
        }
      }

      //   if(refvel[i] > refvel[i-1] || (refvel[i] > refvel[i+1]))
      //   printf("pathnum:%d,mode:%d,accL:%+.3lf,decL:%+.3lf,allL:%+.3lf,decParam:%+.3lf\n",
      //   i, acc_mode[i], accL[i], decL[i], allL[i], dec_param[i]);
    }

    // dividNumの計算
    divideNum[i] =
        int(allL[i] / CURVE_DEC_UNIT_LENGTH); //全体の距離を0.1mで割っている
  }
  flag_const = false;
}

void PathTracking::setAccDecParam(int num, double vel, double acc, double dec) 
{
  refvel[num] = vel;
  acc_param[num] = acc;
  dec_param[num] = dec;
}

void PathTracking::setModecalcParam(int num, int mode, int dir) // AutoControlクラス内で使用
{
  acc_mode[num] = mode;
  int sign = 1;
  if (dir == 1) sign = -1;
  //加速度から加速に必要なカウント数を計算
  // allL（経路のすべて）の計算
  if (allL[num] == 0.0) {
    for (double j = 0; j < 1000; j++)
      allL[num] += pow(
          pow(bezier_x(num, j / 1000) - bezier_x(num, (j + 1) / 1000), 2) +
              pow(bezier_y(num, j / 1000) - bezier_y(num, (j + 1) / 1000), 2),
          0.5);
  }
  decL[num] = 0;
  accL[num] = 0;
  acc_count[num] = 0;

  if (acc_mode[num] == MODE_STOP || acc_mode[num] == MODE_START_STOP) {
    //加速度から加速に必要なカウント数を計算
    //   acc_count[num] = refvel[num] / acc_param[num] / INT_TIME;
    //   printf("%d:%d\n", i, acc_count[num]);

    // decL，accL（減速，加速にかかる距離）の計算
    decL[num] = pow(refvel[num], 2) / fabs(dec_param[num]) / 2;

    if (acc_mode[num] == MODE_START_STOP) {
      //加速度から加速に必要なカウント数を計算
      acc_count[num] = refvel[num] / acc_param[num] / INT_TIME;
      // printf("%d:%d\n", i, acc_count[num]);
      accL[num] = refvel[num] * INT_TIME * acc_count[num] / 2;
      // printf("pathnum:%d,mode:%d,accL:%+.3lf,decL:%+.3lf,allL:%+.3lf,decParam:%+.3lf\n",
      // i, acc_mode[num], accL[num], decL[num], allL[num], dec_param[num]);
      if (allL[num] < accL[num] + decL[num]) {
        accL[num] = accL[num] * allL[num] / (accL[num] + decL[num]);
        decL[num] = allL[num] - accL[num];
        refvel[num] = sqrt(2 * acc_param[num] * accL[num]);
        acc_count[num] = accL[num] * 2 / refvel[num] / INT_TIME;
        // printf("2 %d:%d:%lf\n", i, acc_count[num],refvel[num]);
      }
    } else if (refvel[num] > refvel[num - sign]) {
      //加速度から加速に必要なカウント数を計算
      acc_count[num] =
          (refvel[num] - refvel[num - sign]) / acc_param[num] / INT_TIME;
      // printf("%d:%d\n", i, acc_count[num]);
      accL[num] =
          (refvel[num] + refvel[num - sign]) * INT_TIME * acc_count[num] / 2;
      if (allL[num] < accL[num] + decL[num]) {
        decL[num] = decL[num] *
                    (allL[num] +
                     (pow(refvel[num - sign], 2) / 2 / fabs(acc_param[num]))) /
                    (accL[num] + decL[num] +
                     (pow(refvel[num - sign], 2) / 2 / fabs(acc_param[num])));
        accL[num] = allL[num] - decL[num];
        refvel[num] = sqrt(2 * dec_param[num] * decL[num]);
        acc_count[num] =
            accL[num] * 2 / (refvel[num] + refvel[num - sign]) / INT_TIME;
        // printf("2 %d:%d:%lf\n", i, acc_count[num],refvel[num]);
      }
    } else {
      acc_count[num] = 0;
      // printf("%d:%d\n", i, acc_count[num]);
    }

    //   if (allL[num] < accL[num] + decL[num])
    //     accL[num] = allL[num] - decL[num];
    dec_param[num] = pow(refvel[num], 2) / 2 / decL[num];
    //   printf("pathnum:%d,mode:%d,accL:%+.3lf,decL:%+.3lf,allL:%+.3lf,decParam:%+.3lf\n",
    //   i, acc_mode[num], accL[num], decL[num], allL[num], dec_param[num]);
  } else if (acc_mode[num] == MODE_START) {
    //加速度から加速に必要なカウント数を計算
    acc_count[num] = refvel[num] / acc_param[num] / INT_TIME;
    //   printf("%d:%d\n", i, acc_count[num]);

    if (refvel[num] > refvel[num + sign]) {
      decL[num] = (refvel[num] + refvel[num + sign]) *
                  (refvel[num] - refvel[num + sign]) / fabs(dec_param[num]) / 2;
      accL[num] = refvel[num] * INT_TIME * acc_count[num] / 2;

      if (allL[num] < (accL[num] + decL[num])) {
        accL[num] = accL[num] *
                    (allL[num] +
                     (pow(refvel[num + sign], 2) / 2 / fabs(dec_param[num]))) /
                    (accL[num] + decL[num] +
                     (pow(refvel[num + sign], 2) / 2 / fabs(dec_param[num])));
        decL[num] = allL[num] - accL[num];
        refvel[num] = sqrt(2 * acc_param[num] * accL[num]);
        acc_count[num] = accL[num] * 2 / refvel[num] / INT_TIME;
        // printf("2 %d:%d:%lf\n", i, acc_count[num],refvel[num]);
        dec_param[num] = (refvel[num] + refvel[num + sign]) *
                         (refvel[num] - refvel[num + sign]) / 2 / decL[num];
      }
    }
    //   printf("pathnum:%d,mode:%d,accL:%+.3lf,decL:%+.3lf,allL:%+.3lf,decParam:%+.3lf\n",
    //   i, acc_mode[num], accL[num], decL[num], allL[num], dec_param[num]);
  } else if (acc_mode[num] == MODE_NORMAL) {
    if (refvel[num] > refvel[num - sign]) {
      //加速度から加速に必要なカウント数を計算
      acc_count[num] =
          (refvel[num] - refvel[num - sign]) / acc_param[num] / INT_TIME;
      // printf("%d:%d\n", i, acc_count[num]);
      accL[num] =
          (refvel[num] + refvel[num - sign]) * INT_TIME * acc_count[num] / 2;
    }

    if (acc_mode[num + sign] == MODE_NORMAL ||
        acc_mode[num + sign] == MODE_STOP) {
      if (refvel[num] > refvel[num + sign]) {
        decL[num] = (refvel[num] + refvel[num + sign]) *
                    (refvel[num] - refvel[num + sign]) / fabs(dec_param[num]) /
                    2;
        if (allL[num] < accL[num] + decL[num]) {
          decL[num] = (decL[num] - (refvel[num + sign] *
                                    (refvel[num] - refvel[num + sign]) /
                                    fabs(dec_param[num]))) *
                          allL[num] / (accL[num] + decL[num]) +
                      (refvel[num + sign] * (refvel[num] - refvel[num + sign]) /
                       fabs(dec_param[num])) *
                          pow(allL[num] / (accL[num] + decL[num]), 2);
          accL[num] = allL[num] - decL[num];
          refvel[num] =
              sqrt(2 * dec_param[num] * decL[num] - pow(refvel[num + 1], 2));
          acc_count[num] = accL[num] * 2 / refvel[num] / INT_TIME;
          // printf("2 %d:%d:%lf\n", i, acc_count[num],refvel[num]);
          dec_param[num] = (refvel[num] + refvel[num + sign]) *
                           (refvel[num] - refvel[num + sign]) / 2 / decL[num];
        }
      } else {
        decL[num] = -1.0;
      }
    }

    //   if(refvel[num] > refvel[i-1] || (refvel[num] > refvel[i+1]))
    //   printf("pathnum:%d,mode:%d,accL:%+.3lf,decL:%+.3lf,allL:%+.3lf,decParam:%+.3lf\n",
    //   i, acc_mode[num], accL[num], decL[num], allL[num], dec_param[num]);
  }

  // dividNumの計算
  divideNum[num] =
      int(allL[num] / CURVE_DEC_UNIT_LENGTH); //全体の距離を0.1mで割っている
  flag_const = false;
}

// ベジエ曲線までの垂線距離をニュートン法で求めて，そこまでの距離と接線角度を計算する
void PathTracking::calcRefpoint() {
  if (init_done) {
    double tmpx = Px0 - gPosi.x;
    double tmpy = Py0 - gPosi.y;
    // double tmpx = Px[path_num * 3] - gPosi.x;
    // double tmpy = Py[path_num * 3] - gPosi.y;

    d_be[path_num] =
        d_be_[path_num] + Ax[path_num] * tmpx + Ay[path_num] * tmpy;
    e_be[path_num] =
        e_be_[path_num] + 2 * Bx[path_num] * tmpx + 2 * By[path_num] * tmpy;
    f_be[path_num] =
        f_be_[path_num] + Cx[path_num] * tmpx + Cy[path_num] * tmpy;

    int count_newton = 0;
    if (pre_t_be == 0)
      pre_t_be = 0.1;

    do {
      double temp_dfunc = dfunc(path_num, pre_t_be);
      if (temp_dfunc == 0)
        t_be = pre_t_be;
      else
        t_be = pre_t_be - func(path_num, pre_t_be) / temp_dfunc;

      if (t_be < 0)
        t_be = 0.01;
      epsilon = fabs((t_be - pre_t_be) / pre_t_be);
      //   printf("t:%lf\n",t_be);

      pre_t_be = t_be;
      count_newton++;
    } while (epsilon >= 1e-4 && count_newton <= 50);

    //ベジエ曲線上の点
    onx = bezier_x(path_num, t_be);
    ony = bezier_y(path_num, t_be);
    // printf("\nbezier %.3f,%.3f",onx,ony);

    // 外積による距離導出
    if (dbezier_y(path_num, t_be) + dbezier_x(path_num, t_be) == 0)
      angle = atan2(bezier_y(path_num, t_be + 0.01) - bezier_y(path_num, t_be),
                    bezier_x(path_num, t_be + 0.01) - bezier_x(path_num, t_be));
    else
      angle = atan2(dbezier_y(path_num, t_be), dbezier_x(path_num, t_be));
    // ↑ベジエ曲線の接線方向

    dist = (ony - gPosi.y) * cos(angle) - (onx - gPosi.x) * sin(angle);
    // if (gPosi.y < 0.4 && (auto_phase == 5 || auto_phase == 12 || auto_phase == 51 || auto_phase == 53)) {
    //   dist = bezier_x(path_num, 1.0) - gPosi.x;
    //   angle = -1.5707;
    // }

    epsilon = 1.0;

    bool flag_deadBand = true;
    // t_deadBand = t_be + 1.0 / divideNum[pathNum_deadBand] / 20;
    t_deadBand = t_be;
    double pre_t_deadBand = t_be;
    pathNum_deadBand = path_num;
    int pre_pathNum_deadBand = pathNum_deadBand;
    double L_deadBand = 0;

    while (flag_deadBand == true && dist_deadBand != 0.0) {
      if (t_deadBand < 1.0) {
        //距離の積算
        L_deadBand +=
            sqrt(pow(bezier_x(pathNum_deadBand, t_deadBand) -
                         bezier_x(pre_pathNum_deadBand, pre_t_deadBand),
                     2.0) +
                 pow(bezier_y(pathNum_deadBand, t_deadBand) -
                         bezier_y(pre_pathNum_deadBand, pre_t_deadBand),
                     2.0));

        pre_pathNum_deadBand = pathNum_deadBand;
        pre_t_deadBand = t_deadBand;

        if (L_deadBand > dist_deadBand) {
          t_deadBand -= 1.0 / divideNum[pathNum_deadBand] / 20;
          flag_deadBand = false;
        } else {
          // tを次の値に
          t_deadBand += 1.0 / divideNum[pathNum_deadBand] / 20;
        }
      } else if (acc_mode[pathNum_deadBand] == MODE_START ||
                 acc_mode[pathNum_deadBand] == MODE_NORMAL) {
        pre_pathNum_deadBand = pathNum_deadBand;
        pathNum_deadBand++;
        t_deadBand = 0;
      } else { //減速するモードでtが1を越えた時
        // t_deadBand = 1.0 - 1.0 / divideNum[pathNum_deadBand] / 20;
        t_deadBand = 1.0;
        flag_deadBand = false;
      }
    }
  }
}

// モードによって，それぞれ指令速度を計算する
int PathTracking::calcRefvel() {
  double refVxg, refVyg, refVzg; // グローバル座標系の指定速度
  double tmpPx, tmpPy;

  if (init_done) {
    // printf("init done mode = %d",mode);

    if (path_num <= max_pathnum) { // パスが存在する場合は以下の処理を行う
      if (mode == FOLLOW_TANGENT ||
          mode == FOLLOW_COMMAND) { // ベジエ曲線追従モード
        calcRefpoint();             // t_beが決定されている

        stolen_t_be = t_be; //変更箇所．

        //残りの経路距離を計算
        double resL = 0;
        // if(t_deadBand != 1.0){
        //     for (double i = t_deadBand; i < 1.0; i += (1.0 - t_deadBand) /
        //     20)
        //         resL += pow(pow(bezier_x(pathNum_deadBand, i) -
        //         bezier_x(pathNum_deadBand, i + (1.0 - t_deadBand) / 20), 2)
        //                   + pow(bezier_y(pathNum_deadBand, i) -
        //                   bezier_y(pathNum_deadBand, i + (1.0 - t_deadBand) /
        //                   20), 2), 0.5);
        // }
        for (double i = t_be; i < 1.0; i += (1.0 - t_be) / 20)
          resL += pow(pow(bezier_x(path_num, i) - bezier_x(path_num, i + (1.0 - t_be) / 20), 2) + pow(bezier_y(path_num, i) - bezier_y(path_num, i + (1.0 - t_be) / 20), 2), 0.5);

        resL -=
            tan *
            DEAD_TIME; //前回指令値で無駄時間分動いたとしたときの残りの距離．
        if (resL < 0)
          resL = 0.001; //負になっていたら残り距離を1mmにする

        get_resL = resL;

        // printf("res:%lf dec%lf\n",resL,decL[path_num]);
        //曲線の速度制限関係>>
        int curveDec_pathNum = path_num, curveDec_count = 0;

        //計算範囲の計算
        int curveDec_limit_count =
            int(pow(refvel[path_num], 2) / CURVE_DEC_LIMIT_CENTRIPETAL_ACC *
                CURVE_DEC_TRIGGER_RADIAN / CURVE_DEC_UNIT_LENGTH) +
            5;

        //計算に使用するフラグ．
        bool curveDec_tSerch_flag = true, curveDec_calcVel_flag = false;

        double curveDec_t = t_be, curveDec_angle, curveDec_formed_angle = 0;
        double curveDec_distance, curveDec_r, curveDec_centripetal_acc = 0;

        //最終的な制限速度を格納
        double limit_vel;
        if(!flag_curve_dec) curveDec_tSerch_flag = false;
        if(path_num == 5 && acc_process == 4) curveDec_tSerch_flag = false;

        //計算範囲外になるまで計算をする．
        while (curveDec_count < curveDec_limit_count && curveDec_tSerch_flag) {
          if (curveDec_t < 1.0) {
            //対象の点の接線角度を求める
            curveDec_angle = atan2(dbezier_y(curveDec_pathNum, curveDec_t),
                                   dbezier_x(curveDec_pathNum, curveDec_t));
            //現在の接線角度と対象の点の接線角度のなす角を計算
            curveDec_formed_angle = fabs(curveDec_angle - angle);
            if (curveDec_formed_angle > M_PI)
              curveDec_formed_angle = fabs(curveDec_formed_angle - 2 * M_PI);
            if (curveDec_formed_angle > CURVE_DEC_TRIGGER_RADIAN) {
              //↑設定角度以上のなす角になったら
              curveDec_tSerch_flag = false; //計算終了
              curveDec_calcVel_flag = true; //制限速度計算のフラグ
            }
            curveDec_count++;
            curveDec_t += 1.0 / divideNum[curveDec_pathNum]; //計算点を動かす
          } else if (acc_mode[curveDec_pathNum] == MODE_START ||
                     acc_mode[curveDec_pathNum] == MODE_NORMAL) {
            //パスの狭間の場合．今のフェーズが加速or通常なら次の計算の範囲を次のパスに移す
            curveDec_pathNum++;
            curveDec_t = 0;
          } else {
            curveDec_tSerch_flag = false; //計算終了
          }
        }

        if (curveDec_calcVel_flag) { //ひとつ前のwhile文でフラグが立っていたら制限速度の計算
          // 2点間の距離を計算
          curveDec_distance =
              pow(pow(bezier_x(curveDec_pathNum, curveDec_t) - onx, 2) +
                      pow(bezier_y(curveDec_pathNum, curveDec_t) - ony, 2),
                  0.5);
          //回転半径の計算
          curveDec_r = curveDec_distance / 2 / sin(curveDec_formed_angle / 2);
          //向心加速度（遠心加速度）の計算
          curveDec_centripetal_acc = pow(refvel[path_num], 2) / curveDec_r;
          //制限速度の計算
          limit_vel = pow(CURVE_DEC_LIMIT_CENTRIPETAL_ACC * curveDec_r, 0.5);
        } else { //フラグがなければ制限は元の指令速度
          limit_vel = refvel[path_num];
        }

        //<< 曲線の速度制限関係終わり

        //取り敢えず前回の速度指令を代入．
        double refVtan = tan, refVper = per, refVrot = rot;
        int acc_sign = 1;
        if (path_mode)
          acc_sign = -1;
        acc_process = 0;

        if ((acc_mode[path_num] == MODE_START ||
             acc_mode[path_num] == MODE_START_STOP) &&
            count_acc <= acc_count[path_num]) {
          //加速フェーズ・加速減速フェーズで加速の処理
          count_acc++;
          refVtan = refvel[path_num] * count_acc / (double)acc_count[path_num];
          if (count_acc < acc_count[path_num]) {    
            pre_refType = TYPE_START;
          } else {
            pre_refType = TYPE_NORMAL;
          }

          if ((refVtan > limit_vel) &&
              (CURVE_DEC_LIMIT_CENTRIPETAL_ACC < curveDec_centripetal_acc))
            refVtan = limit_vel; //向心加速度が制限を超えていた時の処理
          else if (refVtan > limit_vel) {
            refVtan = limit_vel;
          }
          acc_process = 1;
        } else if ((acc_mode[path_num] == MODE_NORMAL ||
                    acc_mode[path_num] == MODE_STOP) &&
                   count_acc < acc_count[path_num]) {
          //ノーマルフェーズ・減速フェーズで加速の処理
          count_acc++;
          refVtan = pre_refvel + (refvel[path_num] - pre_refvel) * count_acc /
                                     (double)acc_count[path_num];
          //   if(path_num == 0){
          //     printf("%d,%lf\n",acc_sign,refVtan);
          //   }
          if (count_acc < acc_count[path_num]) {
            pre_refType = TYPE_ACC;
          } else {
            pre_refType = TYPE_NORMAL;
          }

          if ((refVtan > limit_vel) &&
              (CURVE_DEC_LIMIT_CENTRIPETAL_ACC < curveDec_centripetal_acc))
            refVtan = limit_vel; //向心加速度が制限を超えていた時の処理
          acc_process = 2;
        } else if (resL > decL[pathNum_deadBand]) {
          //加速・減速フェーズのどちらでもない際の処理
          if (CURVE_DEC_LIMIT_CENTRIPETAL_ACC < curveDec_centripetal_acc)
            //向心加速度が制限を超えていた時の処理
            refVtan = limit_vel;
          else
            //通常の速度
            refVtan = refvel[pathNum_deadBand];

          pre_refType = TYPE_NORMAL;

          //速度制限がいきなりなくなったとき用の再加速処理
          if ((refVtan - tan) / INT_TIME > LIMIT_ACC) {
            refVtan = tan + LIMIT_ACC * INT_TIME;
            pre_refType = TYPE_ACC;
          } else if ((refVtan - tan) / INT_TIME < -LIMIT_ACC) {
            refVtan = tan - LIMIT_ACC * INT_TIME;
            pre_refType = TYPE_DEC;
          }
          acc_process = 3;
        } else if ((acc_mode[pathNum_deadBand] == MODE_STOP ||
                    (acc_mode[pathNum_deadBand] == MODE_START_STOP)) &&
                   (resL <= decL[pathNum_deadBand])) {
          //減速フェーズで減速の処理
          refVtan = sqrt(pow(refvel[pathNum_deadBand], 2) -
                         2 * dec_param[pathNum_deadBand] *
                             (decL[pathNum_deadBand] - resL));
          pre_refType = TYPE_STOP;

          if ((refVtan > limit_vel) &&
              (CURVE_DEC_LIMIT_CENTRIPETAL_ACC < curveDec_centripetal_acc))
            refVtan = limit_vel; //曲線減衰の速度制限に引っかかっていた際の処理
          if ((refVtan - tan) / INT_TIME > LIMIT_ACC)
            refVtan = tan + LIMIT_ACC * INT_TIME; //再加速時の加速度制限
          acc_process = 4;
        } else if ((acc_mode[path_num] == MODE_START ||
                    (acc_mode[path_num] == MODE_NORMAL)) &&
                   (resL <= decL[path_num])) {
          //ノーマル・加速フェーズで減速の処理
          refVtan = pow(pow(refvel[path_num], 2) -
                            2 * dec_param[path_num] * (decL[path_num] - resL),
                        0.5);
          pre_refType = TYPE_DEC;
          // refVtan = pow(2*dec_param[path_num]*resL,0.5);

          // if(refVtan < refvel[path_num + acc_sign])
          //   refVtan = refvel[path_num + acc_sign];
          if ((refVtan > limit_vel) &&
              (CURVE_DEC_LIMIT_CENTRIPETAL_ACC < curveDec_centripetal_acc))
            refVtan = limit_vel; //曲線減衰の速度制限に引っかかっていた際の処理
          if ((refVtan - tan) / INT_TIME > LIMIT_ACC){
            refVtan = tan + LIMIT_ACC * INT_TIME; //再加速時の加速度制限
          } else if ((refVtan - tan) / INT_TIME < -LIMIT_ACC) {
            refVtan = tan - LIMIT_ACC * INT_TIME;
            pre_refType = TYPE_DEC;
          }
          acc_process = 5;
        }

        if (path_num == 8 || (path_num == 11))
          refVper = yokozurePID.getCmd(dist, 0.0,
                                       0.3); // 横(法線)
        else
          refVper = yokozurePID.getCmd(
              dist, 0.0, refvel[path_num] / 2); // 横(法線)方向速度

        if(flag_inside_per == 1 && refVper < 0 && t_be > 0.6) refVper = refVper * 2.0;
        else if(flag_inside_per == 2 && refVper > 0 && t_be > 0.6) refVper = refVper * 2.0;

        // 旋回は以下の2種類を mode によって変える
        if (mode == FOLLOW_TANGENT) {
          if (mode_changed) {
            kakudoPID.PIDinit(gPosi.z, gPosi.z);
            mode_changed = false;
          }

          //機体の回転量が±180°以上になった際の旋回指令の修正値計算
          int cor_rotate = 0;
          while (fabs(gPosi.z - angle + refangle[path_num] +
                      cor_rotate * 2 * M_PI) > M_PI) {
            if (gPosi.z - angle + refangle[path_num] + cor_rotate * 2 * M_PI >
                0)
              cor_rotate--;
            else
              cor_rotate++;
          }

          refVrot = kakudoPID.getCmd(angle - refangle[path_num] -
                                         cor_rotate * 2 * M_PI,
                                     gPosi.z, 2.90);

        } else {
          if (mode_changed) {
            kakudoPID.PIDinit(gPosi.z, gPosi.z);
            kakudo_filter.initPrevData(gPosi.z);
            mode_changed = false;
          }

          //機体の回転量が±180°以上になった際の旋回指令の修正値計算
          int cor_rotate = 0;
        while (fabs(gPosi.z + refangle[path_num] + cor_rotate * 2 * M_PI) > M_PI) {
            if (gPosi.z + refangle[path_num] + cor_rotate * 2 * M_PI > 0)
            cor_rotate--;
            else
            cor_rotate++;
        }
          //}

        
            refKakudo = -refangle[path_num] - cor_rotate * 2 * M_PI;

          if (const_turn) {
            // if (set_angle)
            //   refKakudo = refKakudo_ - cor_rotate * 2 * M_PI;
            // else
            //   refKakudo = -refangle[path_num] - cor_rotate * 2 * M_PI;

            // refVrot = kakudoPID.getCmd(refKakudo, gPosi.z, 2.9);
            refVrot = calcRefvelZ(refKakudo);
          } else {
            refVrot = kakudoPID.getCmd(refKakudo, gPosi.z, 2.9);
          }

        
          pre_refVz = refVrot;
        }

        // if ((refVrot - rot) / INT_TIME > limit_rot_acc){
        //     refVrot = rot + limit_rot_acc * INT_TIME;
        // }else if ((refVrot - rot) / INT_TIME < -limit_rot_acc){
        //     refVrot = rot - limit_rot_acc * INT_TIME;
        // }

        if(acc_process == 4 && (refVtan < limit_vel)) limit_vel = refVtan;
        refVec = sqrt(pow(refVtan, 2) + pow(refVper, 2));
        if (refVec > limit_vel) {
          refVtan = refVtan * limit_vel / refVec;
          refVper = refVper * limit_vel / refVec;
        }

        tan = refVtan; //前回値の格納
        per = refVper;
        rot = refVrot;
        stolen_limit_vel = limit_vel;
        // ローカル座標系の指令速度(グローバル座標系のも込み込み)
        // refVxとrefVyをコメントアウトするとkakudoPIDのパラメータ調整が出来る(手で押してみて…)
        // refVperだけにするとyokozurePIDのパラメータ調整できる
        // refVtan = 0;
        // refVper = 0;
        // refVrot = 0;

        refVx = refVtan * cos(gPosi.z - angle) + refVper * sin(gPosi.z - angle);
        refVy = -refVtan * sin(gPosi.z - angle) + refVper * cos(gPosi.z - angle);
        refVz = refVrot;

        // yokozure 確認用
        // refVx = refVper * sin( gPosi.z - angle );
        // refVy = refVper * cos( gPosi.z - angle );

      } else { // PID位置制御モード
        if (mode == POSITION_PID) {
          if (mode_changed) {
          Px[3 * path_num] = gPosi.x;
          Py[3 * path_num] = gPosi.y;
          posiPIDx.PIDinit(Px[3 * path_num], gPosi.x); 
          posiPIDy.PIDinit(Py[3 * path_num], gPosi.y);
          posiPIDz.PIDinit(gPosi.z, gPosi.z);         //<----------------------
          kakudo_filter.initPrevData(gPosi.z);
        //   printf("gPosi.z = %+.3lf\n", gPosi.z);
        //   printf("refangle[path_num] = %+.3lf\n", refangle[path_num]);
          setRefKakudo();
          mode_changed = false;
        }
        

        if (count_acc <= acc_count[path_num]) {
          count_acc++;
          tmpPx = Px[3 * path_num] + (Px[3 * path_num + 3] - Px[3 * path_num]) *
                                         count_acc /
                                         (double)acc_count[path_num];
          tmpPy = Py[3 * path_num] + (Py[3 * path_num + 3] - Py[3 * path_num]) *
                                         count_acc /
                                         (double)acc_count[path_num];
        } else {
          tmpPx = (Px[3 * path_num + 3]);
          tmpPy = (Py[3 * path_num + 3]);
        }

        

        // //機体の回転量が±180°以上になった際の旋回指令の修正値計算
        // int cor_rotate = 0;
        // while (fabs(-gPosi.z + refangle[path_num] + cor_rotate * 2 * M_PI) >
        //        M_PI) {
        //   if (-gPosi.z + refangle[path_num] + cor_rotate * 2 * M_PI > 0)
        //     cor_rotate--;
        //   else
        //     cor_rotate++;
        // }
         int cor_rotate = 0;
        while (fabs(gPosi.z + refangle[path_num] + cor_rotate * 2 * M_PI) > M_PI) {
            if (gPosi.z + refangle[path_num] + cor_rotate * 2 * M_PI > 0)
            cor_rotate--;
            else
            cor_rotate++;
        }
        // refKakudo = refangle[path_num] + cor_rotate * 2 * M_PI;
        refKakudo = -refangle[path_num] - cor_rotate * 2 * M_PI;

        // PIDクラスを使って位置制御を行う(速度の指令値を得る)
        refVxg = posiPIDx.getCmd(tmpPx, gPosi.x, refvel[path_num]); 
        refVyg = posiPIDy.getCmd(tmpPy, gPosi.y,refvel[path_num]); 
        refVzg = posiPIDz.getCmd(refKakudo, gPosi.z, 2.9); 
    
        // 上記はグローバル座標系における速度のため，ローカルに変換
        refVx = refVxg * cos(gPosi.z) + refVyg * sin(gPosi.z);
        refVy = -refVxg * sin(gPosi.z) + refVyg * cos(gPosi.z);
        refVz = refVzg;
    
        } else if (mode == POSI_PID) {
          if (mode_changed) {
            posiPIDx.PIDinit(pre_posi.x, gPosi.x);
            posiPIDy.PIDinit(pre_posi.y, gPosi.y);
            posiPIDz.PIDinit(pre_posi.z, gPosi.z); //<----------------------
            // kakudo_filter.initPrevData(gPosi.z);
            //   printf("gPosi.z = %+.3lf\n", gPosi.z);
            //   printf("refangle[path_num] = %+.3lf\n", refangle[path_num]);
            // setRefKakudo();
            mode_changed = false;
          }
          tmpPx = (pre_posi.x);
          tmpPy = (pre_posi.y);

          // PIDクラスを使って位置制御を行う(速度の指令値を得る)
          refVxg = posiPIDx.getCmd(tmpPx, gPosi.x, 1.0);
          refVyg = posiPIDy.getCmd(tmpPy, gPosi.y, 1.0);

          refKakudo = pre_posi.z;

          refVzg = posiPIDz.getCmd(refKakudo, gPosi.z, 2.9);

          pre_refVz = refVzg;
        }

        // 上記はグローバル座標系における速度のため，ローカルに変換
        refVx = refVxg * cos(gPosi.z) + refVyg * sin(gPosi.z);
        refVy = -refVxg * sin(gPosi.z) + refVyg * cos(gPosi.z);
        refVz = refVzg;
      }

      // 収束判定して，収束していたら　1　を返す
      dist2goal = sqrt(pow(gPosi.x - Px[4 * path_num + 3], 2.0) +
                       pow(gPosi.y - Py[4 * path_num + 3], 2.0));
      if (path_mode) {
        dist2goal = sqrt(pow(gPosi.x - Px[4 * path_num], 2.0) +
                         pow(gPosi.y - Py[4 * path_num], 2.0));
      }

      if (mode == POSI_PID) {
        dist2goal = sqrt(pow(gPosi.x - tmpPx, 2.0) + pow(gPosi.y - tmpPy, 2.0));
      }

      if (mode == FOLLOW_TANGENT || mode == FOLLOW_COMMAND) {
        // 軌道追従制御なら，到達位置からの距離とベジエ曲線の t のどちらかの条件
        if (dist2goal <= conv_length || t_be >= conv_tnum) {
          // printf("len:%lf t:%lf\n", dist2goal, t_be);
          flag_curve_dec = true;
         
            if (dist2goal <= conv_length && fabs(refKakudo - gPosi.z) < 0.02) {
              
              // if(path_num == (7 + path) || (path_num == (10 + path))) return
              // 2;
              if (path_num == 8 || (path_num == 11))
                return 2;
              else
                return 1;
            
          } else {
            set_angle = false;
            const_turn = false;
            // if(path_num == (7 + path) || (path_num == (10 + path))) return 2;
            if (path_num == 8 || (path_num == 11))
              return 2;
            else
              return 1;
          }
        }
      } else if (mode == POSITION_PID) {
        // 位置制御なら，目標位置と角度両方を見る
        // if ((dist2goal <= conv_length || t_be >= conv_tnum) && (fabs(refangle[path_num] - gPosi.z) <
        //                                  0.05)) { //<---------------------
        if ( (fabs(refKakudo - gPosi.z) <
                                         0.01)) {
          return 1;
        }
      } else if (mode == POSI_PID) {
        if (dist2goal <= conv_length && (fabs(refKakudo - gPosi.z) < 0.05)) {
          return 2;
        }
      }
      // 収束していなかったら　0　を返す
      return 0;

    } else {
      // path_num が設定された max_pathnum を超えたら　-2　を返す
      return -2;
    }

  } else {
    // 初期化されていなかったら　-1　を返す
    return -1;
  }
}

void PathTracking::calcRefpoint_curve() {
  if (init_done) {
    phi_ = atan2((gPosi.y - curve_oy), (gPosi.x - curve_ox));
    angle = atan2(dcurve_y(phi_), dcurve_x(phi_));
    if (cos(phi_) > 0)
      angle -= M_PI;

    //カーブ上の点
    curve_x_ = curve_x(phi_);
    curve_y_ = curve_y(phi_);
    onx = curve_x_;
    ony = curve_y_;
    // printf("\ncurve %.3f,%.3f",curve_x_,curve_y_);
    if (cos(pre_phi) > 0) {
      if (phi_ < phi_con)
        phi_ = phi_con;
    } else {
      if ((pre_phi < phi_con) && (phi_ > phi_con))
        phi_ = phi_con;
      else if (pre_phi > phi_con) {
        if (phi_ < 0 && (phi_ > phi_con))
          phi_ = phi_con;
      }
    }

    dist = (curve_y_ - gPosi.y) * cos(angle) - (curve_x_ - gPosi.x) * sin(angle);
  }
}

// モードによって，それぞれ指令速度を計算する
int PathTracking::calcRefvel_curve() {
  double refVxg, refVyg, refVzg; // グローバル座標系の指定速度
  double tmpPx, tmpPy;

  if (init_done) {
    // printf("init done mode = %d",mode);

    if (path_num <= max_pathnum) { // パスが存在する場合は以下の処理を行う
      calcRefpoint_curve(); // t_beが決定されている

      stolen_t_be = phi_; //変更箇所．

      //残りの経路距離を計算
      double phi_error = fabs(phi_con - phi_);
      if (phi_ > M_PI / 2 && (phi_con < -M_PI / 2))
        phi_error = fabs(2 * M_PI + phi_con - phi_);
      else if (phi_ < -M_PI / 2 && (phi_con > M_PI / 2))
        phi_error = fabs(2 * M_PI - phi_con + phi_);
      double resL = phi_error * curve_r;

      if (resL < 0)
        resL = 0.001; //負になっていたら残り距離を1mmにする

      // printf("res:%lf dec%lf\n",resL,decL[path_num]);
      /*
      //曲線の速度制限関係>>
      int curveDec_pathNum = path_num, curveDec_count = 0;

      //計算範囲の計算
      int curveDec_limit_count =
          int(pow(refvel[path_num], 2) / CURVE_DEC_LIMIT_CENTRIPETAL_ACC *
              CURVE_DEC_TRIGGER_RADIAN / CURVE_DEC_UNIT_LENGTH) + 5;

      //計算に使用するフラグ．
      bool curveDec_tSerch_flag = true, curveDec_calcVel_flag = false;

      double curveDec_t = t_be, curveDec_angle, curveDec_formed_angle = 0;
      double curveDec_distance, curveDec_r, curveDec_centripetal_acc = 0;

      //最終的な制限速度を格納
      double limit_vel;

      //計算範囲外になるまで計算をする．
      while (curveDec_count < curveDec_limit_count && curveDec_tSerch_flag) {
          if (curveDec_t < 1.0) {
              //対象の点の接線角度を求める
          curveDec_angle = atan2(dbezier_y(curveDec_pathNum, curveDec_t),
                                  dbezier_x(curveDec_pathNum, curveDec_t));
          //現在の接線角度と対象の点の接線角度のなす角を計算
          curveDec_formed_angle = fabs(curveDec_angle - angle);
          if (curveDec_formed_angle > M_PI)
              curveDec_formed_angle = fabs(curveDec_formed_angle - 2 * M_PI);
          if (curveDec_formed_angle > CURVE_DEC_TRIGGER_RADIAN) {
              //↑設定角度以上のなす角になったら
              curveDec_tSerch_flag = false;//計算終了
              curveDec_calcVel_flag = true;//制限速度計算のフラグ
          }
          curveDec_count++;
          curveDec_t += 1.0 / divideNum[curveDec_pathNum];//計算点を動かす
          } else if (acc_mode[curveDec_pathNum] == MODE_START ||
                      acc_mode[curveDec_pathNum] == MODE_NORMAL) {
          //パスの狭間の場合．今のフェーズが加速or通常なら次の計算の範囲を次のパスに移す
          curveDec_pathNum++;
          curveDec_t = 0;
          } else {
          curveDec_tSerch_flag = false;//計算終了
          }
      }

      if (curveDec_calcVel_flag)
      {//ひとつ前のwhile文でフラグが立っていたら制限速度の計算
          //2点間の距離を計算
          curveDec_distance =
              pow(pow(bezier_x(curveDec_pathNum, curveDec_t) - onx, 2) +
                      pow(bezier_y(curveDec_pathNum, curveDec_t) - ony, 2),
                  0.5);
          //回転半径の計算
          curveDec_r = curveDec_distance / 2 / sin(curveDec_formed_angle / 2);
          //向心加速度（遠心加速度）の計算
          curveDec_centripetal_acc = pow(refvel[path_num], 2) / curveDec_r;
          //制限速度の計算
          limit_vel = pow(CURVE_DEC_LIMIT_CENTRIPETAL_ACC * curveDec_r, 0.5);
      } else {//フラグがなければ制限は元の指令速度
          limit_vel = refvel[path_num];
      }

      //<< 曲線の速度制限関係終わり
      */

      //取り敢えず前回の速度指令を代入．
      double refVtan = tan, refVper = per, refVrot = rot;
      int acc_sign = 1;
      if (path_mode)
        acc_sign = -1;
      acc_process = 0;

      double limit_vel = refvel[path_num];

      if (resL > decL[path_num]) {
        //加速・減速フェーズのどちらでもない際の処理
        //通常の速度
        refVtan = refvel[path_num];

        pre_refType = TYPE_NORMAL;

        //速度制限がいきなりなくなったとき用の再加速処理
        if ((refVtan - tan) / INT_TIME > LIMIT_ACC) {
          refVtan = tan + LIMIT_ACC * INT_TIME;
          pre_refType = TYPE_ACC;
        } else if ((refVtan - tan) / INT_TIME < -LIMIT_ACC) {
          refVtan = tan - LIMIT_ACC * INT_TIME;
          pre_refType = TYPE_DEC;
        }
        acc_process = 3;
      } else if (resL <= decL[path_num]) {
        //ノーマル・加速フェーズで減速の処理
        refVtan = pow(pow(refvel[path_num], 2) -
                          2 * dec_param[path_num] * (decL[path_num] - resL),
                      0.5);
        pre_refType = TYPE_DEC;
        // refVtan = pow(2*dec_param[path_num]*resL,0.5);

        if (refVtan < refvel[path_num + acc_sign])
          refVtan = refvel[path_num + acc_sign];
        if (refVtan > limit_vel)
          refVtan = limit_vel; //速度制限に引っかかっていた際の処理
        if ((refVtan - tan) / INT_TIME > LIMIT_ACC)
          refVtan = tan + LIMIT_ACC * INT_TIME; //再加速時の加速度制限
        acc_process = 5;
      }

      refVper = yokozurePID.getCmd(dist, 0.0,
                                   refvel[path_num] / 2); // 横(法線)方向速度

      // 旋回は以下の2種類を mode によって変える
      if (mode == FOLLOW_TANGENT) {
        if (mode_changed) {
          kakudoPID.PIDinit(gPosi.z, gPosi.z);
          mode_changed = false;
        }

        //機体の回転量が±180°以上になった際の旋回指令の修正値計算
        int cor_rotate = 0;
        while (fabs(gPosi.z - angle + refangle[path_num] +
                    cor_rotate * 2 * M_PI) > M_PI) {
          if (gPosi.z - angle + refangle[path_num] + cor_rotate * 2 * M_PI > 0)
            cor_rotate--;
          else
            cor_rotate++;
        }

        refVrot = kakudoPID.getCmd(
            angle - refangle[path_num] - cor_rotate * 2 * M_PI, gPosi.z, 2.90);

      } else {
        if (mode_changed) {
          kakudoPID.PIDinit(gPosi.z, gPosi.z);
          kakudo_filter.initPrevData(gPosi.z);
          mode_changed = false;
        }

        //機体の回転量が±180°以上になった際の旋回指令の修正値計算
        int cor_rotate = 0;
        while (fabs(gPosi.z + refangle[path_num] + cor_rotate * 2 * M_PI) >
               M_PI) {
          if (gPosi.z + refangle[path_num] + cor_rotate * 2 * M_PI > 0)
            cor_rotate--;
          else
            cor_rotate++;
        }

        // if (set_angle)
        //   refKakudo = kakudo_filter.SecondOrderLag(refKakudo_ - cor_rotate * 2 * M_PI);
        // else
        //   refKakudo = kakudo_filter.SecondOrderLag(-refangle[path_num] -
        //                                            cor_rotate * 2 * M_PI);
        if (set_angle)
          refKakudo = refKakudo_ - cor_rotate * 2 * M_PI;
        else
          refKakudo = -refangle[path_num] - cor_rotate * 2 * M_PI;

        refVrot = kakudoPID.getCmd(refKakudo, gPosi.z, 2.9);
      }

      // if ((refVrot - rot) / INT_TIME > limit_rot_acc){
      //     refVrot = rot + limit_rot_acc * INT_TIME;
      // }else if ((refVrot - rot) / INT_TIME < -limit_rot_acc){
      //     refVrot = rot - limit_rot_acc * INT_TIME;
      // }

      refVec = sqrt(pow(refVtan, 2) + pow(refVper, 2));
      if (refVec > limit_vel) {
        refVtan = refVtan * limit_vel / refVec;
        refVper = refVper * limit_vel / refVec;
      }

      tan = refVtan; //前回値の格納
      per = refVper;
      rot = refVrot;
      // ローカル座標系の指令速度(グローバル座標系のも込み込み)
      // refVxとrefVyをコメントアウトするとkakudoPIDのパラメータ調整が出来る(手で押してみて…)
      // refVperだけにするとyokozurePIDのパラメータ調整できる
      // refVtan = 0;
      // refVper = 0;
      // refVrot = 0;
      refVx = refVtan * cos(gPosi.z - angle) + refVper * sin(gPosi.z - angle);
      refVy = -refVtan * sin(gPosi.z - angle) + refVper * cos(gPosi.z - angle);
      refVz = refVrot;

      // yokozure 確認用
      // refVx = refVper * sin( gPosi.z - angle );
      // refVy = refVper * cos( gPosi.z - angle );

      // 収束判定して，収束していたら　1　を返す
      dist2goal =
          sqrt(pow(gPosi.x - curve_x_, 2.0) + pow(gPosi.y - curve_y_, 2.0));

      // 軌道追従制御なら，到達位置からの距離とベジエ曲線の t のどちらかの条件
      if (phi_error < fabs(phi_con * (1.0 - conv_tnum))) {
        // printf("e:%+.3lf p:%+.3lf\n", phi_error, fabs(phi_con * (1.0 -
        // conv_tnum))); printf("phi_con:%+.3lf phi_:%+.3lf\n", phi_con, phi_);
        flag_curve_dec = true;
        path_change = false;
        return 1;
      }

      // 収束していなかったら　0　を返す
      return 0;

    } else {
      // path_num が設定された max_pathnum を超えたら　-2　を返す
      return -2;
    }

  } else {
    // 初期化されていなかったら　-1　を返す
    return -1;
  }
}

double PathTracking::calcRefvelZ(double theta) {
  double joyVz = 0.0;
  static double pre_refVz = 0.0;
  static double limit_vel = sqrt(fabs(theta - gPosi.z) * TURN_ACC) * 0.8;
  if (limit_vel > TURN_MAX_VEL)
    limit_vel = TURN_MAX_VEL;
  static double pre_tar_z = 0.0;
  static int acc_count = 0;
  static int acc_count_ = limit_vel / TURN_ACC / INT_TIME;
  static int acc_mode = 0;
  static int acc_process = 0;
  static double dec_theta =
      pow(limit_vel, 2) / TURN_ACC / 2 + limit_vel * TURN_DEAD_TIME;
  int vel_sign = 1;
  if (theta < gPosi.z)
    vel_sign = -1;
  if (theta != pre_tar_z) {
    pre_refVz = 0.0;
    limit_vel = sqrt(fabs(theta - gPosi.z) * TURN_ACC) * 0.8;
    if (limit_vel > TURN_MAX_VEL)
      limit_vel = TURN_MAX_VEL;
    acc_count = 0;
    acc_count_ = limit_vel / TURN_ACC / INT_TIME;
    dec_theta = pow(limit_vel, 2) / TURN_ACC / 2 + limit_vel * TURN_DEAD_TIME;
    acc_mode = 0;
  }

  if (acc_mode == 1 || (fabs(theta - gPosi.z) < TURN_DEAD_BAND)) {
    joyVz = kakudoPID.getCmd((float)theta, (float)gPosi.z, limit_vel);
    if (theta < gPosi.z)
      joyVz = -joyVz;
    acc_process = 0;
    acc_mode = 1;
  } else if (fabs(theta - gPosi.z) <= dec_theta) {
    // acc_count--;
    // joyVz = TURN_ACC * INT_TIME * acc_count;
    if ((fabs(theta - gPosi.z) - limit_vel * TURN_DEAD_TIME) >
        TURN_DECC_THRESHOLD) {
      joyVz = pow(2 * TURN_ACC *
                      (fabs(theta - gPosi.z) - limit_vel * TURN_DEAD_TIME),
                  0.5);
    } else {
      joyVz = pow(2 * TURN_ACC * TURN_DECC_THRESHOLD, 0.5);
      acc_mode = 1;
    }
    if (joyVz > limit_vel)
      joyVz = limit_vel;
    acc_process = 2;
  } else if (acc_mode == 0 && (acc_count < acc_count_)) {
    acc_count++;
    joyVz = TURN_ACC * INT_TIME * acc_count;
    if (joyVz > limit_vel)
      joyVz = limit_vel;
    acc_process = 1;
  } else {
    joyVz = limit_vel;
    acc_process = 3;
  }
  stolen_turn_proc = acc_process;
  stolen_turn_theta = dec_theta;
  stolen_turn_e = fabs(theta - gPosi.z);

  // pidV.x = posiPID_x.getCmd((float)tar_pos.x, (float)gPosi.x, MAX_CMD_X);
  // pidV.y = posiPID_y.getCmd((float)tar_pos.y, (float)gPosi.y, MAX_CMD_Y);
  // joyVz = posiPID_z.getCmd((float)theta, (float)gPosi.z, lim_refVz);
  // pc.printf("%lf %lf\n",joyVz,limit_vel);
  // pc.printf("%d,%lf,%d,%d,%lf\n",acc_process,limit_vel,acc_count,acc_count_,dec_theta);

  pre_refVz = joyVz;
  pre_tar_z = theta;
  return joyVz * vel_sign;
}

double PathTracking::calcRefvel_posi(double tmp_x, double tmp_y, double temp_z, double max_vel, double conv_range){
    if (flag_pid) {
        posiPIDx.PIDinit(0.0, 0.0);
        posiPIDy.PIDinit(0.0, 0.0);
        posiPIDz.PIDinit(0.0, 0.0); //<----------------------
        flag_pid = false;
    }

    //機体の回転量が±180°以上になった際の旋回指令の修正値計算
    int cor_rotate = 0;
    while (fabs(-gPosi.z + temp_z + cor_rotate * 2 * M_PI) > M_PI) {
        if (-gPosi.z + temp_z + cor_rotate * 2 * M_PI > 0)
            cor_rotate--;
        else
            cor_rotate++;
    }
    refKakudo = temp_z + cor_rotate * 2 * M_PI;

    // PIDクラスを使って位置制御を行う(速度の指令値を得る)
    double refVxg = posiPIDx.getCmd(tmp_x, gPosi.x, max_vel);
    double refVyg = posiPIDy.getCmd(tmp_y, gPosi.y, max_vel);
    double refVzg = posiPIDz.getCmd(refKakudo, gPosi.z, 1.0);

    refVx = refVxg * cos(gPosi.z) + refVyg * sin(gPosi.z);
    refVy = -refVxg * sin(gPosi.z) + refVyg * cos(gPosi.z);
    refVz = refVzg;

    dist2goal = sqrt(pow(gPosi.x - tmp_x, 2.0) + pow(gPosi.y - tmp_y, 2.0));
    if(dist2goal < conv_range  && (fabs(refKakudo - gPosi.z) < 0.1)){
        flag_pid = true;
        return 0.0;
    }
    else{
        return dist2goal;
    }
}

void PathTracking::makePath(int num, double Pointx[4], double Pointy[4],
                            double angle) {
  if (num > max_pathnum) max_pathnum = num;
  for (int i = 0; i < 4; i++) {
    Px[4 * num + i] = Pointx[i];
    Py[4 * num + i] = Pointy[i];
  }

  refangle[num] = -angle;
  allL[num] = 0.0;
}
//円の線上にいるとき
void PathTracking::makePath_circle(int num, double Point2x[2], double Point2y[2], double angle){//pointには経路の最初と最後の値を入れる。
  double Pointx[4];
  double Pointy[4];
  double circlex[4];//O原点を基準としたときのベジェ曲線の円
  double circley[4];
  double goal_my_x = GOAL_MY_X;
  double goal_my_y = GOAL_MY_Y;
  double goal_ene_x = GOAL_ENE_X;
  double goal_ene_y = GOAL_ENE_Y;
  double goal_3circle_r = GOAL_3CIRCLE_R;
  
  int rotate_sign = 0;
  //int turn_mode = DEFENSE;
  
  if(num > max_pathnum) max_pathnum = num;
  //rotate_r = sqrt(pow(goal_ene_x - Point2x[0],2) + pow(goal_ene_y - Point2y[0],2));//現在の位置とGOALとの距離
  //味方側
  if(turn_mode){
        rotate_r = sqrt(pow(goal_my_x - Point2x[0],2) + pow(goal_my_y - Point2y[0],2));
  }else {
        rotate_r = sqrt(pow(goal_ene_x - Point2x[0],2) + pow(goal_ene_y - Point2y[0],2));
  }
  
  
  //rotate_r = goal_3circle_r;
  //theta_a = atan2(Point2x[0] - goal_ene_x, Point2y[0] - goal_ene_y);//敵フィールド
  //theta_a = atan2(Point2x[0] - goal_my_x, Point2y[0] - goal_my_y);
  //theta_b = -atan2(Point2x[1] - goal_ene_x, Point2y[1] - goal_ene_y);
  //theta_b = -atan2(Point2x[1] - goal_my_x, Point2y[1] - goal_my_y);

//theta_a = atan2(goal_my_x - gPosi.x, goal_my_y - gPosi.y);
//theta_b = -atan2(goal_my_x - 3.56, goal_my_y - 1.366);
  //味方側
  if(turn_mode){
        theta_a = -atan2(goal_my_x - Point2x[0], goal_my_y - Point2y[0]);
        theta_b = -atan2(goal_my_x - Point2x[1], goal_my_y - Point2y[1]);
  }else{
      theta_a = -atan2(goal_my_x - (15 - Point2x[0]), goal_my_y - (Point2y[0]));
      theta_b = -atan2(goal_my_x - (Point2x[1]), goal_my_y - (Point2y[1]));
  }

  //相手側
  



  rotate_theta = (theta_b - theta_a); //線と線の重なってる角度出す
  //rotate_theta = 1.50;
  add_theta = theta_a;
  //add_theta = 0.872;
  rotate_k = 4.0 / 3.0 * std::tan(rotate_theta / 4.0) * rotate_r;



//   circley[0] = Point2y[0];
//   circlex[0] = Point2x[0];
//   circley[1] = rotate_r + goal_my_y;
//   circlex[1] = rotate_k + goal_my_x;
//   circley[2] = rotate_r * cos(rotate_theta) + rotate_k * sin(rotate_theta) + goal_my_y;
//   circlex[2] = rotate_r * sin(rotate_theta) - rotate_k * cos(rotate_theta) + goal_my_x;
//   circley[3] = rotate_r * cos(rotate_theta) + goal_my_y;
//   circlex[3] = rotate_r * sin(rotate_theta) + goal_my_x;

//味方側時計回り もととなる式
  circley[0] = rotate_r;
  circlex[0] = 0;
  circley[1] = rotate_r;
  circlex[1] = rotate_k;
  circley[2] = rotate_r * cos(rotate_theta) + rotate_k * sin(rotate_theta);
  circlex[2] = rotate_r * sin(rotate_theta) - rotate_k * cos(rotate_theta);
  circley[3] = rotate_r * cos(rotate_theta);
  circlex[3] = rotate_r * sin(rotate_theta);

//式の変換
for(int i = 0; i < 4; i++) {
    Pointy[i] = circley[i] * cos(add_theta) - circlex[i] * sin(add_theta);
    Pointx[i] = circley[i] * sin(add_theta) + circlex[i] * cos(add_theta);
}

if(circley[0] > circley[3]) rotate_sign = 1;//反時計回り
else rotate_sign = 2;

  for (int i = 0; i < 4; i++) {
    if(turn_mode){//味方側
        Px[4 * num + i] = (Pointx[i] + goal_my_x);
        Py[4 * num + i] = 8 - (Pointy[i] + goal_my_y);
    }else{//相手側
        Px[4 * num + i] = 15 - (Pointx[i] + goal_my_x);
        Py[4 * num + i] = 8 - (Pointy[i] + goal_my_y);
    }
  }

  refangle[num] = -angle;
  allL[num] = 0.0;
}

int PathTracking::changePath(int num, double acc, double vec, double angle, coords tar_posi_, coords tar_posi2_, bool flag_) {
  int mode = 0;
  if ((num + 2) > max_pathnum)
    max_pathnum = num + 2;

  double epsilon_;
  int count_newton = 0;
  phi_con = 0.0;
  pre_phi = M_PI;
  tar_posi = tar_posi_;
  pre_refvel = vec;

  curve_r = pow(vec, 2) / acc;
  double inter_x = gPosi.x + gPosi.y * std::tan(angle);
  // printf("angle:%lf inter_x:%lf\n", angle, inter_x);
  if (tar_posi.x > inter_x) {
    curve_ox = gPosi.x + curve_r * cos(angle);
    curve_oy = gPosi.y + curve_r * sin(angle);
  } else if (tar_posi.x < inter_x) {
    curve_ox = gPosi.x - curve_r * cos(angle);
    curve_oy = gPosi.y - curve_r * sin(angle);
  }

  phi_ = atan2((gPosi.y - curve_oy), (gPosi.x - curve_ox));
  int sign = -1;
  if (fabs(phi_) > (M_PI / 2))
    sign = 1;
  pre_phi = phi_;

  double func_ = func2(pre_phi);
  double dfunc_ = dfunc2(pre_phi);
  double pre_dfunc = dfunc_;
  bool flag_newton = false;
  // pc.printf("%d,%d\n",(int)(func_/fabs(func_)), (int)(dfunc_/fabs(dfunc_)));
  while (!flag_newton) {
    pre_phi += sign * 0.03;
    func_ = func2(pre_phi);
    dfunc_ = dfunc2(pre_phi);
    if (sign * (func_ / fabs(func_)) != (dfunc_ / fabs(dfunc_))) {
      if (fabs(pre_dfunc - dfunc_) < 0.01) {
        flag_newton = true;
      }
    }
    // pc.printf("%d,%d\n",(int)(func_/fabs(func_)),
    // (int)(dfunc_/fabs(dfunc_)));
    pre_dfunc = dfunc_;
  }

  do {
    double temp_dfunc = dfunc2(pre_phi);
    if (temp_dfunc == 0)
      phi_con = pre_phi;
    else
      phi_con = pre_phi - func2(pre_phi) / temp_dfunc;

    epsilon_ = fabs((phi_con - pre_phi) / pre_phi);
    //   pc.printf("count:%d t:%lf\n",count_newton,phi_con);

    pre_phi = phi_con;
    count_newton++;
  } while (epsilon_ >= 1e-4 && count_newton <= 50);
  curve_x_ = curve_x(phi_con);
  curve_y_ = curve_y(phi_con);

  while (fabs(phi_con) > M_PI) {
    phi_con -= phi_con / fabs(phi_con) * 2 * M_PI;
  }

  double phi_error = fabs(phi_con - phi_);
  if (phi_ > M_PI / 2 && (phi_con < -M_PI / 2))
    phi_error = fabs(2 * M_PI + phi_con - phi_);
  else if (phi_ < -M_PI / 2 && (phi_con > M_PI / 2))
    phi_error = fabs(2 * M_PI - phi_con + phi_);

  double curve_f = phi_error * curve_r;
  double phi_dist = sqrt(pow(curve_ox - tar_posi.x, 2) + pow(curve_oy - tar_posi.y, 2));
  if (curve_f > 0.1)
    mode = 1;
  else
    mode = 2;
  if (curve_y_ < tar_posi.y)
    mode = 0;
  else if (phi_dist < curve_r)
    mode = 0;

  if(flag_) mode = 0;

  if(mode == 0 || mode == 2) num += 2;

  // printf("mode:%d\n",mode);

  double all_L = 0.0;
  double acc_L = 0.0;
  int path_sign = 1;
  if (Px[4 * path_num + 3] < tar_posi.x)
    path_sign = -1;
  dec_param[num] = dec_param[path_num];

  switch (mode) {
  case 0:
    // tar_posi = tar_posi2_;
    Px[4 * num] = Px[4 * path_num + 3];
    Py[4 * num] = Py[4 * path_num + 3];
    Px[4 * num + 1] = Px[4 * path_num + 3] - path_sign * 0.2;
    Py[4 * num + 1] = 0.5;
    Px[4 * num + 2] = tar_posi.x + path_sign * 0.2;
    Py[4 * num + 2] = 0.5;
    Px[4 * num + 3] = tar_posi.x;
    Py[4 * num + 3] = tar_posi.y;
    refangle[num] = -tar_posi.z;
    allL[num] = 0.0;
    // calcAccDecParam_re(num, num, true, 2.0, 4.0, 4.0);
    break;
  case 1:
    Px[4 * (num + 1)] = curve_x_;
    Py[4 * (num + 1)] = curve_y_;
    for (int i = 1; i < 4; i++) {
      Px[4 * (num + 1) + i] = tar_posi.x;
      Py[4 * (num + 1) + i] = tar_posi.y;
    }

    allL[num + 1] = sqrt(pow(Px[4 * (num + 1) + 3] - Px[4 * (num + 1)], 2) +
                         pow(Py[4 * (num + 1) + 3] - Py[4 * (num + 1)], 2));
    allL[num] = phi_error * curve_r;
    all_L = allL[num + 1] + allL[num];
    decL[num + 1] = (vec + refvel[path_num + 1]) *
                    (vec - refvel[path_num + 1]) / fabs(dec_param[path_num]) /
                    2;
    acc_mode[num] = MODE_NORMAL;
    acc_mode[num + 1] = MODE_NORMAL;
    dec_param[num + 1] = dec_param[path_num];
    if (allL[num + 1] >= decL[num + 1]) {
      decL[num] = 0.0;
      acc_count[num] = acc_count[num + 1] = 0;
      refvel[num + 1] = refvel[num] = vec;
      acc_param[num + 1] = acc_param[path_num];
      acc_L = (allL[num + 1] - decL[num + 1]) * 0.8;

      if (acc_L > 0) {
        accL[num + 1] = acc_L / 2;
        refvel[num + 1] =
            sqrt(2 * acc_param[num + 1] * accL[num + 1] + pow(pre_refvel, 2));
        decL[num + 1] = (refvel[num + 1] + refvel[path_num + 1]) *
                        (refvel[num + 1] - refvel[path_num + 1]) /
                        fabs(dec_param[path_num]) / 2;
        acc_count[num + 1] = (int)((refvel[num + 1] - pre_refvel) /
                                   acc_param[num + 1] / INT_TIME);
        count_acc = 0;
      }
    } else {
      decL[num] = decL[num + 1] - allL[num + 1];
      decL[num + 1] = allL[num + 1];
      refvel[num] = vec;
      refvel[num + 1] = sqrt(2 * decL[num + 1] * fabs(dec_param[path_num]) +
                             pow(refvel[path_num + 1], 2));
      acc_count[num] = acc_count[num + 1] = 0;
    }

    refangle[num + 1] = refangle[num] = -tar_posi.z;
    path_change = true;
    break;
  case 2:
    Px[4 * num] = gPosi.x;
    Py[4 * num] = gPosi.y;
    for (int i = 1; i < 4; i++) {
      Px[4 * num + i] = tar_posi.x;
      Py[4 * num + i] = tar_posi.y;
    }

    allL[num] = sqrt(pow(Px[4 * num + 3] - Px[4 * num], 2) +
                     pow(Py[4 * num + 3] - Py[4 * num], 2));
    all_L = allL[num];
    decL[num] = (vec + refvel[path_num + 1]) * (vec - refvel[path_num + 1]) /
                fabs(dec_param[path_num]) / 2;
    acc_mode[num] = MODE_NORMAL;
    if (allL[num] >= decL[num]) {
      refvel[num] = vec;
      acc_count[num] = 0;
      refangle[num] = -tar_posi.z;
      acc_param[num] = acc_param[path_num];
      acc_L = (allL[num] - decL[num]) * 0.8;

      if (acc_L > 0) {
        accL[num] = acc_L / 2;
        refvel[num] = sqrt(2 * acc_param[num] * accL[num] + pow(pre_refvel, 2));
        decL[num] = (refvel[num] + refvel[path_num + 1]) *
                    (refvel[num] - refvel[path_num + 1]) /
                    fabs(dec_param[path_num]) / 2;
        acc_count[num] =
            (int)((refvel[num] - pre_refvel) / acc_param[num] / INT_TIME);
        count_acc = 0;
      }
    } else {
      mode = 0;
      Px[4 * num] = Px[4 * path_num + 3];
      Py[4 * num] = Py[4 * path_num + 3];
      Px[4 * num + 1] = Px[4 * path_num + 3] - path_sign * 0.2;
      Py[4 * num + 1] = 0.5;
      Px[4 * num + 2] = tar_posi.x + path_sign * 0.2;
      Py[4 * num + 2] = 0.5;
      Px[4 * num + 3] = tar_posi.x;
      Py[4 * num + 3] = tar_posi.y;
      refangle[num] = -tar_posi.z;
      allL[num] = 0.0;
      // calcAccDecParam_re(num, num, true, 2.0, 4.0, 4.0);
    }
    break;
  }

//   printf("mode:%d phi:%lf,%lf coord:%lf,%lf,%lf x_:%lf y_:%lf\n", mode, phi_,
//          phi_con, curve_ox, curve_oy, curve_r, curve_x_, curve_y_);
//   for (int i = 0; i < 2; i++) {
//     printf("mode:%d,pre_v:%+.3lf,vel:%+.3lf,acc:%+.3lf,dec:%+.3lf\n",
//            acc_mode[num + i], pre_refvel, refvel[num + i], acc_param[num + i],
//            dec_param[num + i]);
//     printf("accL:%+.3lf,decL:%+.3lf,allL:%+.3lf\n", accL[num + i],
//            decL[num + i], allL[num + i]);
//   }
  pre_phi = phi_;
  return mode;
}

void PathTracking::changeDestPath(int num, int mode,double pre_vel,double Pointx[4],double Pointy[4],double angle, bool flag_param, double vel, double acc, double dec){
  makePath(num, Pointx, Pointy, angle);
  if(flag_param) 
    setAccDecParam(num, vel, acc, dec);
  else 
    setAccDecParam(num, refvel[path_num], acc_param[path_num], dec_param[path_num]);
  setPathNum(num, 0);
  acc_mode[num] = mode;
  //加速度から加速に必要なカウント数を計算
  // allL（経路のすべて）の計算
  if (allL[num] == 0.0) {
    for (double j = 0; j < CALC_L_RES; j++)
      allL[num] += 
      pow( pow(bezier_x(num, j / CALC_L_RES) - bezier_x(num, (j + 1) / CALC_L_RES), 2) + pow(bezier_y(num, j / CALC_L_RES) - bezier_y(num, (j + 1) / CALC_L_RES), 2), 0.5);
  }
  decL[num] = 0;
  accL[num] = 0;
  acc_count[num] = 0;
  pre_refvel = pre_vel;
  int sign = 1;

  //加速度から加速に必要なカウント数を計算
  //   acc_count[num] = refvel[num] / acc_param[num] / INT_TIME;
  //   printf("%d:%d\n", i, acc_count[num]);

  // decL，accL（減速，加速にかかる距離）の計算
  if(acc_mode[num] == MODE_STOP){
    decL[num] = pow(refvel[num], 2) / fabs(dec_param[num]) / 2;

    if (refvel[num] > pre_refvel) {
        //加速度から加速に必要なカウント数を計算
        acc_count[num] =
            (refvel[num] - pre_refvel) / acc_param[num] / INT_TIME;
        // printf("%d:%d\n", i, acc_count[num]);
        accL[num] =
            (refvel[num] + pre_refvel) * INT_TIME * acc_count[num] / 2;
        if (allL[num] < accL[num] + decL[num]) {
            decL[num] = decL[num] *
                        (allL[num] +
                            (pow(pre_refvel, 2) / 2 / fabs(acc_param[num]))) /
                        (accL[num] + decL[num] +
                            (pow(pre_refvel, 2) / 2 / fabs(acc_param[num])));
            accL[num] = allL[num] - decL[num];
            if(accL[num] < 0.0) accL[num] = 0.0;
            accL[num] = accL[num] * 0.8;
            decL[num] = decL[num] * 0.8;
            refvel[num] = sqrt(2 * dec_param[num] * decL[num]);
            acc_count[num] =
                accL[num] * 2 / (refvel[num] + pre_refvel) / INT_TIME;
            // printf("2 %d:%d:%lf\n", i, acc_count[num],refvel[num]);
        }
    }else if(decL[num] > allL[num]){
        acc_count[num] = 0;
        // printf("%d:%d\n", i, acc_count[num]);
    }

    dec_param[num] = pow(refvel[num], 2) / 2 / decL[num];
    //   printf("pathnum:%d,mode:%d,accL:%+.3lf,decL:%+.3lf,allL:%+.3lf,decParam:%+.3lf\n",
    //   i, acc_mode[num], accL[num], decL[num], allL[num], dec_param[num]);
  }
  else if(acc_mode[num] == MODE_NORMAL){
    if (refvel[num] > pre_refvel) {
      //加速度から加速に必要なカウント数を計算
      acc_count[num] =
          (refvel[num] - pre_refvel) / acc_param[num] / INT_TIME;
      // printf("%d:%d\n", i, acc_count[num]);
      accL[num] =
          (refvel[num] + pre_refvel) * INT_TIME * acc_count[num] / 2;
    }

    if (acc_mode[num + sign] == MODE_NORMAL ||
        acc_mode[num + sign] == MODE_STOP) {
      if (refvel[num] > refvel[num + sign]) {
        decL[num] = (refvel[num] + refvel[num + sign]) *
                    (refvel[num] - refvel[num + sign]) / fabs(dec_param[num]) /
                    2;
        if (allL[num] < accL[num] + decL[num]) {
          decL[num] = (decL[num] - (refvel[num + sign] *
                                    (refvel[num] - refvel[num + sign]) /
                                    fabs(dec_param[num]))) *
                          allL[num] / (accL[num] + decL[num]) +
                      (refvel[num + sign] * (refvel[num] - refvel[num + sign]) /
                       fabs(dec_param[num])) *
                          pow(allL[num] / (accL[num] + decL[num]), 2);
          accL[num] = allL[num] - decL[num];
          refvel[num] =
              sqrt(2 * dec_param[num] * decL[num] - pow(refvel[num + 1], 2));
          acc_count[num] = accL[num] * 2 / refvel[num] / INT_TIME;
          // printf("2 %d:%d:%lf\n", i, acc_count[num],refvel[num]);
          dec_param[num] = (refvel[num] + refvel[num + sign]) *
                           (refvel[num] - refvel[num + sign]) / 2 / decL[num];
        }
      } else {
        decL[num] = -1.0;
      }
    }
  }
  
  // dividNumの計算
  divideNum[num] =
      int(allL[num] / CURVE_DEC_UNIT_LENGTH); //全体の距離を0.1mで割っている
  flag_const = false;
}

// ベジエ曲線のパス番号をインクリメントする
void PathTracking::incrPathnum() {
  pre_posi = {Px[4 * path_num + 3], Py[4 * path_num + 3], refKakudo};
  path_num++;
  Px0 = gPosi.x;
  Py0 = gPosi.y;
  setParam();
  t_be = 0.0;
  pre_t_be = 0.1;
  count_acc = 0;
  pre_refvel = refvel[path_num - 1];
  get_resL = 1.0;
  dist2goal = 1.0;
}

void PathTracking::decrPathnum() {
  pre_posi = {Px[4 * path_num], Py[4 * path_num], refKakudo};
  path_num--;
  Px0 = gPosi.x;
  Py0 = gPosi.y;
  setParam_re();
  t_be = 0.0;
  pre_t_be = 0.1;
  count_acc = 0;
  pre_refvel = refvel[path_num + 1];
  get_resL = 1.0;
  dist2goal = 1.0;
}

void PathTracking::setParam() {
  pre_Px = Px[4 * path_num];
  pre_Py = Py[4 * path_num];
  path_mode = false;
  pre_refVz = 0.0;

  Ax[path_num] = Px[4 * path_num + 3] - 3 * Px[4 * path_num + 2] +
                 3 * Px[4 * path_num + 1] - gPosi.x;
  Ay[path_num] = Py[4 * path_num + 3] - 3 * Py[4 * path_num + 2] +
                 3 * Py[4 * path_num + 1] - gPosi.y;
  Bx[path_num] = Px[4 * path_num + 2] - 2 * Px[4 * path_num + 1] + gPosi.x;
  By[path_num] = Py[4 * path_num + 2] - 2 * Py[4 * path_num + 1] + gPosi.y;
  Cx[path_num] = Px[4 * path_num + 1] - gPosi.x;
  Cy[path_num] = Py[4 * path_num + 1] - gPosi.y;
  Dx[path_num] = gPosi.x;
  Dy[path_num] = gPosi.y;
  a_be[path_num] = pow(Ax[path_num], 2.0) + pow(Ay[path_num], 2.0);
  b_be[path_num] =
      5 * (Ax[path_num] * Bx[path_num] + Ay[path_num] * By[path_num]);
  c_be[path_num] =
      2 * ((3 * pow(Bx[path_num], 2.0) + 2 * Ax[path_num] * Cx[path_num]) +
           (3 * pow(By[path_num], 2.0) + 2 * Ay[path_num] * Cy[path_num]));
  d_be_[path_num] =
      9 * Bx[path_num] * Cx[path_num] + 9 * By[path_num] * Cy[path_num];
  e_be_[path_num] = 3 * pow(Cx[path_num], 2.0) + 3 * pow(Cy[path_num], 2.0);
  f_be_[path_num] = 0;
  //   printf("path:%d Ax:%lf Bx:%lf Cx:%lf
  //   Dx:%lf\n",path_num,Ax[path_num],Bx[path_num],Cx[path_num],Dx[path_num]);
  //   printf("path:%d a:%lf b:%lf c:%lf d:%lf e:%lf
  //   f:%lf\n",path_num,a_be[path_num],b_be[path_num],c_be[path_num],d_be[path_num],e_be_[path_num],f_be_[path_num]);
}

void PathTracking::setParam_re() {
  pre_Px = Px[4 * path_num + 3];
  pre_Py = Py[4 * path_num + 3];
  path_mode = true;
  pre_refVz = 0.0;

  Ax[path_num] = Px[4 * path_num] - 3 * Px[4 * path_num + 1] +
                 3 * Px[4 * path_num + 2] - gPosi.x;
  Ay[path_num] = Py[4 * path_num] - 3 * Py[4 * path_num + 1] +
                 3 * Py[4 * path_num + 2] - gPosi.y;
  Bx[path_num] = Px[4 * path_num + 1] - 2 * Px[4 * path_num + 2] + gPosi.x;
  By[path_num] = Py[4 * path_num + 1] - 2 * Py[4 * path_num + 2] + gPosi.y;
  Cx[path_num] = Px[4 * path_num + 2] - gPosi.x;
  Cy[path_num] = Py[4 * path_num + 2] - gPosi.y;
  Dx[path_num] = gPosi.x;
  Dy[path_num] = gPosi.y;
  a_be[path_num] = pow(Ax[path_num], 2.0) + pow(Ay[path_num], 2.0);
  b_be[path_num] =
      5 * (Ax[path_num] * Bx[path_num] + Ay[path_num] * By[path_num]);
  c_be[path_num] =
      2 * ((3 * pow(Bx[path_num], 2.0) + 2 * Ax[path_num] * Cx[path_num]) +
           (3 * pow(By[path_num], 2.0) + 2 * Ay[path_num] * Cy[path_num]));
  d_be_[path_num] =
      9 * Bx[path_num] * Cx[path_num] + 9 * By[path_num] * Cy[path_num];
  e_be_[path_num] = 3 * pow(Cx[path_num], 2.0) + 3 * pow(Cy[path_num], 2.0);
  f_be_[path_num] = 0;
  // while(1) printf("%lf %lf %lf
  // %lf\n",Ay[path_num],By[path_num],Cy[path_num],Dy[path_num]);
}

void PathTracking::setParam(int num) {
  Ax[num] =
      Px[4 * num + 3] - 3 * Px[4 * num + 2] + 3 * Px[4 * num + 1] - Px[4 * num];
  Ay[num] =
      Py[4 * num + 3] - 3 * Py[4 * num + 2] + 3 * Py[4 * num + 1] - Py[4 * num];
  Bx[num] = Px[4 * num + 2] - 2 * Px[4 * num + 1] + Px[4 * num];
  By[num] = Py[4 * num + 2] - 2 * Py[4 * num + 1] + Py[4 * num];
  Cx[num] = Px[4 * num + 1] - Px[4 * num];
  Cy[num] = Py[4 * num + 1] - Py[4 * num];
  Dx[num] = Px[4 * num];
  Dy[num] = Py[4 * num];
  a_be[num] = pow(Ax[num], 2.0) + pow(Ay[num], 2.0);
  b_be[num] = 5 * (Ax[num] * Bx[num] + Ay[num] * By[num]);
  c_be[num] = 2 * ((3 * pow(Bx[num], 2.0) + 2 * Ax[num] * Cx[num]) +
                   (3 * pow(By[num], 2.0) + 2 * Ay[num] * Cy[num]));
  d_be_[num] = 9 * Bx[num] * Cx[num] + 9 * By[num] * Cy[num];
  e_be_[num] = 3 * pow(Cx[num], 2.0) + 3 * pow(Cy[num], 2.0);
  f_be_[num] = 0;
}

void PathTracking::setParam_re(int num) {
  Ax[num] =
      Px[4 * num] - 3 * Px[4 * num + 1] + 3 * Px[4 * num + 2] - Px[4 * num + 3];
  Ay[num] =
      Py[4 * num] - 3 * Py[4 * num + 1] + 3 * Py[4 * num + 2] - Py[4 * num + 3];
  Bx[num] = Px[4 * num + 1] - 2 * Px[4 * num + 2] + Px[4 * num + 3];
  By[num] = Py[4 * num + 1] - 2 * Py[4 * num + 2] + Py[4 * num + 3];
  Cx[num] = Px[4 * num + 2] - Px[4 * num + 3];
  Cy[num] = Py[4 * num + 2] - Py[4 * num + 3];
  Dx[num] = Px[4 * num + 3];
  Dy[num] = Py[4 * num + 3];
  a_be[num] = pow(Ax[num], 2.0) + pow(Ay[num], 2.0);
  b_be[num] = 5 * (Ax[num] * Bx[num] + Ay[num] * By[num]);
  c_be[num] = 2 * ((3 * pow(Bx[num], 2.0) + 2 * Ax[num] * Cx[num]) +
                   (3 * pow(By[num], 2.0) + 2 * Ay[num] * Cy[num]));
  d_be_[num] = 9 * Bx[num] * Cx[num] + 9 * By[num] * Cy[num];
  e_be_[num] = 3 * pow(Cx[num], 2.0) + 3 * pow(Cy[num], 2.0);
  f_be_[num] = 0;
}

int PathTracking::getPathNum() { return path_num; }

void PathTracking::setPathNum(int num, bool mode) {
  path_num = num;
  t_be = 0.0;
  pre_t_be = 0.1;
  count_acc = 0;
  Px0 = gPosi.x;
  Py0 = gPosi.y;
  dist2goal = 1.0;
  if (mode)
    setParam_re();
  else
    setParam();
}

void PathTracking::setPathEnd(int num, double Px_, double Py_) {
  Px[4 * num + 3] = Px_;
  Py[4 * num + 3] = Py_;
  allL[num] = 0.0;
}

void PathTracking::setPathStart(int num, double Px_, double Py_) {
  Px[4 * num] = Px_;
  Py[4 * num] = Py_;
  allL[num] = 0.0;
}

// 収束判定に用いる距離などをセットする
void PathTracking::setConvPara(double xconv_length, double xconv_tnum = 0.997) {
  conv_length = xconv_length;
  conv_tnum = xconv_tnum;
}

void PathTracking::setAngle(double angle) {
  refKakudo_ = angle;
  set_angle = true;
}

void PathTracking::setConst(bool flag_const_) { flag_const = flag_const_; }

void PathTracking::setInseidPer(int flag_) { flag_inside_per = flag_; }

void PathTracking::setConstTurn(bool const_turn_) { const_turn = const_turn_; }

void PathTracking::closeFlagAngle() { set_angle = false; }

void PathTracking::addCondAngle() { flag_angle = true; }

void PathTracking::setMode(int xmode) {
  mode = xmode;
  mode_changed = true;
}

void PathTracking::setAccMode(int num, int mode) { acc_mode[num] = mode; }

void PathTracking::setCurveDec(bool flag_) { flag_curve_dec = flag_; }

int PathTracking::getMode() { return mode; }

int PathTracking::getAccMode() { return acc_mode[path_num]; }

void PathTracking::setPathAccMode(int num, int mode) { acc_mode[num] = mode; }

void PathTracking::setMaxPathnum(int num) { max_pathnum = num; }

void PathTracking::setPosiPIDxPara(float xKp, float xKi, float xKd) {
  posiPIDx.setPara(xKp, xKi, xKd);
}

void PathTracking::setPosiPIDyPara(float xKp, float xKi, float xKd) {
  posiPIDy.setPara(xKp, xKi, xKd);
}

void PathTracking::setPosiPIDzPara(float xKp, float xKi, float xKd) {
  posiPIDz.setPara(xKp, xKi, xKd);
}

void PathTracking::setYokozurePIDPara(float xKp, float xKi, float xKd) {
  yokozurePID.setPara(xKp, xKi, xKd);
}

void PathTracking::setKakudoPIDPara(float xKp, float xKi, float xKd) {
  kakudoPID.setPara(xKp, xKi, xKd);
}

void PathTracking::kakudoPIDinit() { kakudoPID.PIDinit(refKakudo, gPosi.z); }

void PathTracking::setRefKakudo() { refKakudo = refangle[path_num]; }

double PathTracking::getRefVper() { return per; }

double PathTracking::getRefVrot() { return rot; }

double PathTracking::getResL() { return get_resL; }

double PathTracking::getDist2goal() { return dist2goal; }

void PathTracking::getAutoPhase(int phase) { auto_phase = phase; }


//DWA----------------------------------------------------------
//運動モデル
State PathTracking::motion(State s, float v, float omega) {
    State ns = s;
    // s.x = gPosi.x;
    // s.y = gPosi.y;
    // s.omega = gPosi.z;
    ns.x += v * cos(gPosi.x) * INT_TIME;
    ns.y += v * sin(gPosi.y) * INT_TIME;
    ns.yaw += omega * INT_TIME;
    ns.v = v;
    ns.omega = omega;
    return ns;
}
// ゴールとの距離
float PathTracking::headingEval(State s, float gx, float gy) {
    float dx = gx - gPosi.x;
    float dy = gy - gPosi.y;
    float goalYaw = atan2(dy, dx);
    float diff = fabs(goalYaw - gPosi.z);
    return diff; // 角度差が小さいほど良い
}
// 障害物との距離
float PathTracking::distEval(State s, const std::vector<Obstacle>& obs, double R) {
    float minDist = 1e9;
    for (auto& o : obs) {
        float d = hypot(o.x - gPosi.x, o.y - gPosi.y) - R;
        if (d < minDist) minDist = d;
    }
    return minDist;
}

// DWA メイン
std::pair<float,float> PathTracking::dwaControl(State s, float gx, float gy,
                                  const std::vector<Obstacle>& obs) {
                                    
    float bestScore = -1e9;
    std::pair<float,float> bestU(0,0);

    // 動的ウィンドウ候補
    for (float v=0.0; v<=0.5; v+=0.05) {       // 並進速度候補
        for (float omega=-1.0; omega<=1.0; omega+=0.1) { // 角速度候補
            State pred = motion(s, v, omega);

            float h = headingEval(pred, gx, gy);
            float d = distEval(pred, obs, obstacleR);
            float vel = v;

            float score = 0.4*h + 0.4*d + 0.2*vel;

            if (score > bestScore) {
                bestScore = score;
                bestU = {v, omega};
            }
        }
    }
    return bestU;
}

int PathTracking::calcRefvel_DWA(){
    double refVxg, refVyg, refVzg; // グローバル座標系の指定速度
    double tmpPx, tmpPy;
    std::pair<double,double> ctrl = dwaControl(s, goalX, goalY, obs);
    double v = ctrl.first;
    double omega = ctrl.second;
    s = motion(s, v, omega);

    //printf("x=%.2f y=%.2f yaw=%.2f v=%.2f omega=%.2f\n", s.x, s.y, s.yaw, v, omega);

    // refVxg = v;
    // //refVyg = 0.0;
    // refVzg = omega;

    refVx = v;
    //refVyg = 0.0;
    refVz = omega;

    // refVx = refVxg * cos(gPosi.z) + refVyg * sin(gPosi.z);
    // refVy = -refVxg * sin(gPosi.z) + refVyg * cos(gPosi.z);
    // refVz = refVzg;
    
    // 収束判定して，収束していたら　1　を返す
    dist2goal = sqrt(pow(gPosi.x - goalX, 2.0) + pow(gPosi.y - goalY, 2.0));//距離

    if (dist2goal <= conv_length || t_be >= conv_tnum) {
        //printf("Arrived!\n");
        return 1;
    }

    return 0;//収束されていなかったら0を返す
}