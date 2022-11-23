/***
 *  一些基础的数据结构啥的
 * 
*/

#pragma once

#include <vector>
#include <cmath>
#include <Eigen/Core>
#include <ros/ros.h>

typedef Eigen::Matrix<double, 6, 3> Matrix6x3;

/* 无人机本身的物理参数 */
namespace UAVparam{
  double MaxVel = 2.0;
  double MaxAcc = 3.0;
  double MaxYawRate = M_PI;

  void LoadFromYaml(ros::NodeHandle& nh)
  {
    nh.param<double>("/UAVparam/MaxVel", MaxVel, 1.0);
    nh.param<double>("/UAVparam/MaxAcc", MaxAcc, 1.0);
    nh.param<double>("/UAVparam/MaxYawRate", MaxYawRate, M_PI);
  }
};

/* 无人的飞行状态 */
struct State
{
  Eigen::Vector3d pt;      // 全局的位置
  Eigen::Vector3d vel;     // 全局的速度  
  Eigen::Vector3d acc;     // 局部的加速度？
  Eigen::Vector3d rpy;     // Yaw是全局的，r p应该是机体的

  State(){
    pt  = Eigen::Vector3d::Zero();
    vel = Eigen::Vector3d::Zero();
    acc = Eigen::Vector3d::Zero();
    rpy = Eigen::Vector3d::Zero();
  }
};

/***
 * 多项式轨迹用于全局轨迹
 * 并且多项式轨迹不参与时间的分配
 * 时间分配的问题应该是上位机该考虑的问题
*/
class PolynomialTraj
{
public:
  PolynomialTraj(){
    order = 5;
    tt_t = 0;
    num_seg = 0;
    start_time = ros::Time::now().toSec();
    initConstant();
  }
  PolynomialTraj(int ord){
    order = ord;
    tt_t = 0;
    num_seg = 0;
    start_time = ros::Time::now().toSec();
    initConstant();
  }
  ~PolynomialTraj(){}

  void initConstant(){
    TCT_p_[0] = 1;
    TCT_v_[0] = 0; TCT_v_[1] = 1;
    TCT_a_[0] = 0; TCT_a_[0] = 0; TCT_a_[1] = 2;
  }

  void caluTime(double t){
    for(int i=1; i<6; i++){
      TCT_p_[i] = std::pow(t, i);
    }
    for(int i=2; i<6; i++){
      TCT_v_[i] = i * TCT_p_[i-1];
    }
    for(int i=3; i<6; i++){
      TCT_a_[i] = i * TCT_v_[i-1];
    }
  }

  void addSegment(Matrix6x3& coel, double time){
    coefs_.push_back(coel);
    times_.push_back(time);
    num_seg += 1;
    tt_t += time;

    // 每次都在刷新最后一个点
    Eigen::Matrix<double, 1, 6> TTC_p;
    for(int i = 0; i < 6; i++){
      TTC_p[i] = std::pow(time, i);
    }
    end_point = TTC_p * coel;
  }

  double getTotalTime(){return tt_t;}

  void setSeg(int segs){num_seg = segs;}

  void setTotalTime(double tt_time){tt_t = tt_time;}

  void setStartTime(){
    start_time = ros::Time::now().toSec();
  }

  // 这个时间 t 是相对时间 t 。 
  Eigen::Vector3d getPosition(double t){
    if(t < 0 || t > tt_t){
      ROS_ERROR("Time error");
      return;
    }

    Eigen::Vector3d position;



    return position;
  }

  // 这个时间 t 是相对时间 t 。
  Eigen::Vector3d getVelocity(double t){
    if(t < 0 || t > tt_t){
      ROS_ERROR("Time error");
      return;
    }
    Eigen::Vector3d velocity;



    return velocity;
  }

  Eigen::Vector3d getEndPoint(){
    return end_point;
  }

  bool evaluate(double t){
    if(t < start_time || t > start_time + tt_t){
      ROS_ERROR("Time error");
      return false;
    }

    double local_t = t - start_time;
  
    // step1: 选找哪一段。然后确定相对时间。 
    int index = 0;
    for(; index < num_seg; index++){
      if(local_t < times_[index]){
        break;
      }
      local_t -= times_[index];
    }

    caluTime(local_t);

    globalState_.pt  = TCT_p_ * coefs_[index];
    globalState_.vel = TCT_v_ * coefs_[index];
    globalState_.acc = TCT_a_ * coefs_[index];
    globalState_.rpy[2] = atan2(globalState_.vel[1], globalState_.vel[0]);
    return true;
  }

  double getPathLen(int index){
    double dis = 0;
    Eigen::Vector3d p1, p2;
    Eigen::Matrix<double, 1, 6> TTC_p;
    for(int i = 0; i < 6; i++){
      TTC_p[i] = std::pow(0, i);
    }
    p1 = TTC_p * coefs_[index];
    // 这一步可以多核加速一下。 // 根据电脑性能来选择吧步长吧
    for(double t = 0.02; t < times_[index] - 0.02; t += 0.02){
      for(int i=1; i<6; i++){
        TTC_p[i] = std::pow(t, i);
      }
      p2 = TTC_p * coefs_[index];
      dis += (p2 - p1).norm();
      p1 = p2;
    }
    return dis;
  }

  static PolynomialTraj minSnapTraj(const Eigen::MatrixXd &wps,      const Eigen::Vector3d &start_vel,
                                    const Eigen::Vector3d &end_vel,  const Eigen::Vector3d &start_acc,
                                    const Eigen::Vector3d &end_acc,  const Eigen::VectorXd &Time,
                                    const std::vector<double>& poses);

  static PolynomialTraj one_traj_gen(const State& start, const State& end, double totalTime);

public:
  std::vector<double> times_;           // 自己多注意一些，不能修改这些数字
  std::vector<Matrix6x3> coefs_;
  double last_progress_time_;           // 这个是处理的相对时间。 TODO: 

  // 这两个状态是让飞机跟随的点
  State globalState_;
  State loclaState_;

private:
  int order;                           // 多项式阶数, 默认5阶吧 
  int num_seg;                         // 多项式轨迹的分段数量
  double tt_t;                         // 轨迹所用的总用时
  double start_time;                   // 轨迹开始执行的时间
  Eigen::Vector3d end_point;
  Eigen::Matrix<double, 1, 6> TCT_p_;  // time coefficient table -> TCT
  Eigen::Matrix<double, 1, 6> TCT_v_;   
  Eigen::Matrix<double, 1, 6> TCT_a_;
};


PolynomialTraj PolynomialTraj::minSnapTraj(
  const Eigen::MatrixXd &wps,      const Eigen::Vector3d &start_vel,
  const Eigen::Vector3d &end_vel,  const Eigen::Vector3d &start_acc,
  const Eigen::Vector3d &end_acc,  const Eigen::VectorXd &Time,
  const std::vector<double>& poses)
{
  int seg_num = Time.size();
  // 每行包含了所有的 x,y,z 系数
  Eigen::MatrixXd poly_coeff(seg_num, 3 * 6);
  // 分解到每个轴上的系数
  Eigen::VectorXd Px(6 * seg_num), Py(6 * seg_num), Pz(6 * seg_num);

  int num_f, num_p; // number of fixed and free variables
  int num_d;        // number of all segments' derivatives

  // 定义了一个求阶乘的函数
  const static auto Factorial = [](int x) {
    int fac = 1;
    for (int i = x; i > 0; i--)
      fac = fac * i;
    return fac;
  };

  /* ---------- end point derivative ---------- */
  Eigen::VectorXd Dx = Eigen::VectorXd::Zero(seg_num * 6);
  Eigen::VectorXd Dy = Eigen::VectorXd::Zero(seg_num * 6);
  Eigen::VectorXd Dz = Eigen::VectorXd::Zero(seg_num * 6);

  // Exp:       6:[(1,   2,   3,   4,   5,   6  )]
  // VectorXd Dx: [(s_p, e_p, s_v, e_v, s_a, e_a)]
  /***
   * wps:k*6                             (k+1)*6
   *     [s_p, e_p, s_v, e_v, s_a, e_a], [s_p, e_p, s_v, e_v, s_a, e_a]
   * 0:x
   * 1:y
   * 2:z
  */
  // FIXME: 这里只约束了 wps 的point, 应该再约束一个 wps 的 vel 的方向
  for (int k = 0; k < seg_num; k++)
  {
    /* 位置点的滑窗约束 */
    Dx(k * 6) = wps(0, k);
    Dx(k * 6 + 1) = wps(0, k + 1);
    Dy(k * 6) = wps(1, k);
    Dy(k * 6 + 1) = wps(1, k + 1);
    Dz(k * 6) = wps(2, k);
    Dz(k * 6 + 1) = wps(2, k + 1);

    if(poses.at(k) > -200){
      // 中间点的速度应该是连续的
      double sin_pose = sin(poses.at(k) / 180 * M_PI);
      double cos_pose = cos(poses.at(k) / 180 * M_PI);
      Dx(k * 6 + 2) = UAVparam::MaxVel * cos_pose;            
      Dx(k * 6 + 3) = Dx(k * 6 + 2);
      Dy(k * 6 + 2) = UAVparam::MaxVel * sin_pose;
      Dy(k * 6 + 3) = Dy(k * 6 + 2);
      Dz(k * 6 + 2) = 0;
      Dz(k * 6 + 3) = 0;
    }

    /* 速度方向的约束条件 */
    /***
     * 分析一下：
     * 速度方向的约束和速度大小的约束不一样的:假设方向已定，设vx等于1,则vy、vz是确定的
     * 因此速度的约束是三个约束
     * 方向的约束是一个约束？
    */

    /* 收尾确定的量 */
    if (k == 0)
    {
      Dx(k * 6 + 2) = start_vel(0);
      Dy(k * 6 + 2) = start_vel(1);
      Dz(k * 6 + 2) = start_vel(2);

      Dx(k * 6 + 4) = start_acc(0);
      Dy(k * 6 + 4) = start_acc(1);
      Dz(k * 6 + 4) = start_acc(2);
    }
    else if (k == seg_num - 1)
    {
      Dx(k * 6 + 3) = end_vel(0);
      Dy(k * 6 + 3) = end_vel(1);
      Dz(k * 6 + 3) = end_vel(2);

      Dx(k * 6 + 5) = end_acc(0);
      Dy(k * 6 + 5) = end_acc(1);
      Dz(k * 6 + 5) = end_acc(2);
    }
  }

  /* ---------- Mapping Matrix A ---------- */
  Eigen::MatrixXd Ab;   // A 矩阵的 block
  // 分块对角矩阵,把多项式系数映射的实际的物理值。
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(seg_num * 6, seg_num * 6);

  // Dasheng定义的矩阵
  // 1, 0, 0,  0,   0,    0,
  // 0, 1, 0,  0,   0,    0,
  // 0, 0, 2,  0,   0,    0,
  // 1, t, t2, t3,  t4,   t5,
  // 0, 1, 2t, 3t2, 4t3,  5t4,
  // 0, 0, 2,  6t,  12t2, 20t3;

  // Fei_Gao定义的矩阵
  // 0    0     0    0   0  1
  // t5   t4    t3   t2  t  1
  // 0    0     0    0   1  0
  // 5t4  4t3   3t2  2t  1  0
  // 0    0     0    2   0  0
  // 20t3 12t2  6t   2   0  0

  // 代码中定义的矩阵
  // 1  0  0   0   0    0
  // 1  t  t2  t3  t4   t5
  // 0  1  0   0   0    0
  // 0  1  2t  3t2 4t3  5t4
  // 0  0  2   0   0    0
  // 0  0  2   6t  12t2 20t3

  for (int k = 0; k < seg_num; k++)
  {
    Ab = Eigen::MatrixXd::Zero(6, 6);
    for (int i = 0; i < 3; i++)   // Q:i为什么是3呢? A:因为p、v、a 三个量
    {
      Ab(2 * i, i) = Factorial(i);
      for (int j = i; j < 6; j++)
        Ab(2 * i + 1, j) = Factorial(j) / Factorial(j - i) * pow(Time(k), j - i);
    }
    A.block(k * 6, k * 6, 6, 6) = Ab;
  }

  /* ---------- Produce Selection Matrix C' ---------- */
  Eigen::MatrixXd Ct, C;

  num_f = 2 * seg_num + 4; // 3 + 3 + (seg_num - 1) * 2 = 2m + 4
  num_p = 2 * seg_num - 2; // (seg_num - 1) * 2 = 2m - 2
  num_d = 6 * seg_num;     // 
  Ct = Eigen::MatrixXd::Zero(num_d, num_f + num_p);  // 这个选择矩阵是十分庞大的
  Ct(0, 0) = 1;
  Ct(2, 1) = 1;
  Ct(4, 2) = 1; // stack the start point
  Ct(1, 3) = 1;
  Ct(3, 2 * seg_num + 4) = 1;
  Ct(5, 2 * seg_num + 5) = 1;

  Ct(6 * (seg_num - 1) + 0, 2 * seg_num + 0) = 1;
  Ct(6 * (seg_num - 1) + 1, 2 * seg_num + 1) = 1; // Stack the end point
  Ct(6 * (seg_num - 1) + 2, 4 * seg_num + 0) = 1;
  Ct(6 * (seg_num - 1) + 3, 2 * seg_num + 2) = 1; // Stack the end point
  Ct(6 * (seg_num - 1) + 4, 4 * seg_num + 1) = 1;
  Ct(6 * (seg_num - 1) + 5, 2 * seg_num + 3) = 1; // Stack the end point

  for (int j = 2; j < seg_num; j++)
  {
    Ct(6 * (j - 1) + 0, 2 + 2 * (j - 1) + 0) = 1;
    Ct(6 * (j - 1) + 1, 2 + 2 * (j - 1) + 1) = 1;
    Ct(6 * (j - 1) + 2, 2 * seg_num + 4 + 2 * (j - 2) + 0) = 1;
    Ct(6 * (j - 1) + 3, 2 * seg_num + 4 + 2 * (j - 1) + 0) = 1;
    Ct(6 * (j - 1) + 4, 2 * seg_num + 4 + 2 * (j - 2) + 1) = 1;
    Ct(6 * (j - 1) + 5, 2 * seg_num + 4 + 2 * (j - 1) + 1) = 1;
  }

  C = Ct.transpose();

  Eigen::VectorXd Dx1 = C * Dx;
  Eigen::VectorXd Dy1 = C * Dy;
  Eigen::VectorXd Dz1 = C * Dz;

  /* ---------- minimum snap matrix ---------- */
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(seg_num * 6, seg_num * 6);

  for (int k = 0; k < seg_num; k++)
  {
    for (int i = 3; i < 6; i++)
    {
      for (int j = 3; j < 6; j++)
      {
        Q(k * 6 + i, k * 6 + j) =
            i * (i - 1) * (i - 2) * j * (j - 1) * (j - 2) / (i + j - 5) * pow(Time(k), (i + j - 5));
      }
    }
  }

  /* ---------- R matrix ---------- */
  Eigen::MatrixXd R = C * A.transpose().inverse() * Q * A.inverse() * Ct;

  Eigen::VectorXd Dxf(2 * seg_num + 4), Dyf(2 * seg_num + 4), Dzf(2 * seg_num + 4);

  Dxf = Dx1.segment(0, 2 * seg_num + 4);
  Dyf = Dy1.segment(0, 2 * seg_num + 4);
  Dzf = Dz1.segment(0, 2 * seg_num + 4);

  Eigen::MatrixXd Rff(2 * seg_num + 4, 2 * seg_num + 4);
  Eigen::MatrixXd Rfp(2 * seg_num + 4, 2 * seg_num - 2);
  Eigen::MatrixXd Rpf(2 * seg_num - 2, 2 * seg_num + 4);
  Eigen::MatrixXd Rpp(2 * seg_num - 2, 2 * seg_num - 2);

  Rff = R.block(0, 0, 2 * seg_num + 4, 2 * seg_num + 4);
  Rfp = R.block(0, 2 * seg_num + 4, 2 * seg_num + 4, 2 * seg_num - 2);
  Rpf = R.block(2 * seg_num + 4, 0, 2 * seg_num - 2, 2 * seg_num + 4);
  Rpp = R.block(2 * seg_num + 4, 2 * seg_num + 4, 2 * seg_num - 2, 2 * seg_num - 2);

  /* ---------- close form solution ---------- */

  Eigen::VectorXd Dxp(2 * seg_num - 2), Dyp(2 * seg_num - 2), Dzp(2 * seg_num - 2);
  Dxp = -(Rpp.inverse() * Rfp.transpose()) * Dxf;
  Dyp = -(Rpp.inverse() * Rfp.transpose()) * Dyf;
  Dzp = -(Rpp.inverse() * Rfp.transpose()) * Dzf;

  Dx1.segment(2 * seg_num + 4, 2 * seg_num - 2) = Dxp;
  Dy1.segment(2 * seg_num + 4, 2 * seg_num - 2) = Dyp;
  Dz1.segment(2 * seg_num + 4, 2 * seg_num - 2) = Dzp;

  Px = (A.inverse() * Ct) * Dx1;
  Py = (A.inverse() * Ct) * Dy1;
  Pz = (A.inverse() * Ct) * Dz1;


  /* ---------- use polynomials ---------- */
  PolynomialTraj poly_traj;
  for (int i = 0; i < seg_num; i++)
  {
    Matrix6x3 coel;
    coel.col(0) = Px.segment(i * 6, 6).transpose();
    coel.col(1) = Py.segment(i * 6, 6).transpose();
    coel.col(2) = Pz.segment(i * 6, 6).transpose();
    double ts = Time(i);
    poly_traj.addSegment(coel, ts);
  }
  return poly_traj;
}

PolynomialTraj PolynomialTraj::one_traj_gen(const State& start, const State& end, double totalTime)
{
  // tt means totalTime
  double tt[6];
  for(int i = 0; i < 6; i++){
    tt[i] = pow(totalTime, i);
  }
  Eigen::Matrix<double, 6, 6> poly;
  poly << 1, 0, 0, 0, 0, 0,
          0, 1, 0, 0, 0, 0,
          0, 0, 2, 0, 0, 0,
          tt[0], tt[1], tt[2], tt[3], tt[4], tt[5],
          0, 1, 2*tt[1], 3*tt[2], 4*tt[3], 5*tt[4],
          0, 0, 2, 6*tt[1], 12*tt[2], 20*tt[3];
  Eigen::Matrix<double, 6, 6> Polynomial = poly.inverse();
  Eigen::Matrix<double, 6, 3> constant_b;
  constant_b.row(0) = start.pt.transpose();
  constant_b.row(1) = start.vel.transpose();
  constant_b.row(2) = start.acc.transpose();
  constant_b.row(3) = end.pt.transpose();
  constant_b.row(4) = end.vel.transpose();
  constant_b.row(5) = end.acc.transpose();

  PolynomialTraj poly_traj;
  Matrix6x3 coel = Polynomial * constant_b;
  poly_traj.addSegment(coel, totalTime);
  poly_traj.setSeg(1);
  poly_traj.setTotalTime(totalTime);
  return poly_traj;
}

