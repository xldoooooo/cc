#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/SO3Command.h>
#include <simulator/myModel.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <uav_utils/geometry_utils.h>


// 定义无人机的转速
typedef struct _Control
{
  double rpm[4];
} Control;

// 定义无人机的控制指令
typedef struct _Command
{
  float force[3];
  float omega[3];
  float qx, qy, qz, qw;
  float kR[3];
  float kOm[3];
  float corrections[3];
  float current_yaw;
  bool  use_external_yaw;
} Command;

// 定义力和力矩干扰
typedef struct _Disturbance
{
  Eigen::Vector3d f;
  Eigen::Vector3d m;
} Disturbance;

static Command     command;
static Disturbance disturbance;

void stateToOdomMsg(const Simulator::myModel::State& state, nav_msgs::Odometry& odom);
void quadToImuMsg(const Simulator::myModel& quad, sensor_msgs::Imu& imu);

// 输入期望力和期望角速度，输出电机转速
static Control getRateControl(const Simulator::myModel& quad, const Command& cmd)
{
  const double _kf = quad.getPropellerThrustCoefficient();
  const double _km = quad.getPropellerMomentCoefficient();
  const double kf  = _kf - cmd.corrections[0];
  const double km  = _km / _kf * kf;

  const double          d       = quad.getArmLength();
  const Eigen::Matrix3f J       = quad.getInertia().cast<float>();
  const float I[3][3] = { { J(0, 0), J(0, 1), J(0, 2) },
                          { J(1, 0), J(1, 1), J(1, 2) },
                          { J(2, 0), J(2, 1), J(2, 2) } };
  const Simulator::myModel::State state = quad.getState();

   // Rotation, may use external yaw
  Eigen::Vector3d _ypr = uav_utils::R_to_ypr(state.R);
  Eigen::Vector3d ypr  = _ypr;
  if (cmd.use_external_yaw)
    ypr[0] = cmd.current_yaw;
  Eigen::Matrix3d R;
  R = Eigen::AngleAxisd(ypr[0], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(ypr[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(ypr[2], Eigen::Vector3d::UnitX());
  float R11 = R(0, 0);
  float R12 = R(0, 1);
  float R13 = R(0, 2);
  float R21 = R(1, 0);
  float R22 = R(1, 1);
  float R23 = R(1, 2);
  float R31 = R(2, 0);
  float R32 = R(2, 1);
  float R33 = R(2, 2);

  float Om1 = state.omega(0);
  float Om2 = state.omega(1);
  float Om3 = state.omega(2);

  float force = 0;
  force = cmd.force[0] * R13 + cmd.force[1] * R23 + cmd.force[2] * R33;
  
  float eOm1 = Om1 - cmd.omega[0];
  float eOm2 = Om2 - cmd.omega[1];
  float eOm3 = Om3 - cmd.omega[2];
  // w x Iw 
  float in1 = Om2 * (I[2][0] * Om1 + I[2][1] * Om2 + I[2][2] * Om3) -
              Om3 * (I[1][0] * Om1 + I[1][1] * Om2 + I[1][2] * Om3);
  float in2 = Om3 * (I[0][0] * Om1 + I[0][1] * Om2 + I[0][2] * Om3) -
              Om1 * (I[2][0] * Om1 + I[2][1] * Om2 + I[2][2] * Om3);
  float in3 = Om1 * (I[1][0] * Om1 + I[1][1] * Om2 + I[1][2] * Om3) -
              Om2 * (I[0][0] * Om1 + I[0][1] * Om2 + I[0][2] * Om3);

  float M1 = -cmd.kOm[0] * eOm1 + in1; // - I[0][0]*muR1;
  float M2 = -cmd.kOm[1] * eOm2 + in2; // - I[1][1]*muR2;
  float M3 = -cmd.kOm[2] * eOm3 + in3; // - I[2][2]*muR3;

  float w_sq[4];
  w_sq[0] = force / (4 * kf) - M2 / (2 * d * kf) + M3 / (4 * km);
  w_sq[1] = force / (4 * kf) + M2 / (2 * d * kf) + M3 / (4 * km);
  w_sq[2] = force / (4 * kf) + M1 / (2 * d * kf) - M3 / (4 * km);
  w_sq[3] = force / (4 * kf) - M1 / (2 * d * kf) - M3 / (4 * km);

  Control control;
  for (int i = 0; i < 4; i++)
  {
    if (w_sq[i] < 0)
      w_sq[i] = 0;

    control.rpm[i] = sqrtf(w_sq[i]);
  }
  return control;

}

// 输入期望力和期望姿态，输出电机转速
static Control getControl(const Simulator::myModel& quad, const Command& cmd)
{
  const double _kf = quad.getPropellerThrustCoefficient();
  const double _km = quad.getPropellerMomentCoefficient();
  const double kf  = _kf - cmd.corrections[0];
  const double km  = _km / _kf * kf;

  const double          d       = quad.getArmLength();
  const Eigen::Matrix3f J       = quad.getInertia().cast<float>();
  const float I[3][3] = { { J(0, 0), J(0, 1), J(0, 2) },
                          { J(1, 0), J(1, 1), J(1, 2) },
                          { J(2, 0), J(2, 1), J(2, 2) } };
  const Simulator::myModel::State state = quad.getState();

  // Rotation, may use external yaw
  Eigen::Vector3d _ypr = uav_utils::R_to_ypr(state.R);
  Eigen::Vector3d ypr  = _ypr;
  if (cmd.use_external_yaw)
    ypr[0] = cmd.current_yaw;
  Eigen::Matrix3d R;
  R = Eigen::AngleAxisd(ypr[0], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(ypr[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(ypr[2], Eigen::Vector3d::UnitX());
  float R11 = R(0, 0);
  float R12 = R(0, 1);
  float R13 = R(0, 2);
  float R21 = R(1, 0);
  float R22 = R(1, 1);
  float R23 = R(1, 2);
  float R31 = R(2, 0);
  float R32 = R(2, 1);
  float R33 = R(2, 2);
  /*
    float R11 = state.R(0,0);
    float R12 = state.R(0,1);
    float R13 = state.R(0,2);
    float R21 = state.R(1,0);
    float R22 = state.R(1,1);
    float R23 = state.R(1,2);
    float R31 = state.R(2,0);
    float R32 = state.R(2,1);
    float R33 = state.R(2,2);
  */
  float Om1 = state.omega(0);
  float Om2 = state.omega(1);
  float Om3 = state.omega(2);
  //desired quaterion to desired rotation matrix transport
  float Rd11 =
    cmd.qw * cmd.qw + cmd.qx * cmd.qx - cmd.qy * cmd.qy - cmd.qz * cmd.qz;
  float Rd12 = 2 * (cmd.qx * cmd.qy - cmd.qw * cmd.qz);
  float Rd13 = 2 * (cmd.qx * cmd.qz + cmd.qw * cmd.qy);
  float Rd21 = 2 * (cmd.qx * cmd.qy + cmd.qw * cmd.qz);
  float Rd22 =
    cmd.qw * cmd.qw - cmd.qx * cmd.qx + cmd.qy * cmd.qy - cmd.qz * cmd.qz;
  float Rd23 = 2 * (cmd.qy * cmd.qz - cmd.qw * cmd.qx);
  float Rd31 = 2 * (cmd.qx * cmd.qz - cmd.qw * cmd.qy);
  float Rd32 = 2 * (cmd.qy * cmd.qz + cmd.qw * cmd.qx);
  float Rd33 =
    cmd.qw * cmd.qw - cmd.qx * cmd.qx - cmd.qy * cmd.qy + cmd.qz * cmd.qz;
  //attitude error function := 0.5 * tr(I_3 - R_des^T*R)
  float Psi = 0.5f * (3.0f - (Rd11 * R11 + Rd21 * R21 + Rd31 * R31 +
                              Rd12 * R12 + Rd22 * R22 + Rd32 * R32 +
                              Rd13 * R13 + Rd23 * R23 + Rd33 * R33));

  float force = 0;
  if (Psi < 1.0f) // Position control stability guaranteed only when Psi < 1
    force = -cmd.force[0] * R13 - cmd.force[1] * R23 - cmd.force[2] * R33;
  // ROS_INFO("The command thrust is %f", force);
  float eR1 = 0.5f * (R12 * Rd13 - R13 * Rd12 + R22 * Rd23 - R23 * Rd22 +
                      R32 * Rd33 - R33 * Rd32);
  float eR2 = 0.5f * (R13 * Rd11 - R11 * Rd13 - R21 * Rd23 + R23 * Rd21 -
                      R31 * Rd33 + R33 * Rd31);
  float eR3 = 0.5f * (R11 * Rd12 - R12 * Rd11 + R21 * Rd22 - R22 * Rd21 +
                      R31 * Rd32 - R32 * Rd31);

  float eOm1 = Om1;
  float eOm2 = Om2;
  float eOm3 = Om3;
  // w x Iw 
  float in1 = Om2 * (I[2][0] * Om1 + I[2][1] * Om2 + I[2][2] * Om3) -
              Om3 * (I[1][0] * Om1 + I[1][1] * Om2 + I[1][2] * Om3);
  float in2 = Om3 * (I[0][0] * Om1 + I[0][1] * Om2 + I[0][2] * Om3) -
              Om1 * (I[2][0] * Om1 + I[2][1] * Om2 + I[2][2] * Om3);
  float in3 = Om1 * (I[1][0] * Om1 + I[1][1] * Om2 + I[1][2] * Om3) -
              Om2 * (I[0][0] * Om1 + I[0][1] * Om2 + I[0][2] * Om3);
  /*
    // Robust Control --------------------------------------------
    float c2       = 0.6;
    float epsilonR = 0.04;
    float deltaR   = 0.1;
    float eA1 = eOm1 + c2 * 1.0/I[0][0] * eR1;
    float eA2 = eOm2 + c2 * 1.0/I[1][1] * eR2;
    float eA3 = eOm3 + c2 * 1.0/I[2][2] * eR3;
    float neA = sqrt(eA1*eA1 + eA2*eA2 + eA3*eA3);
    float muR1 = -deltaR*deltaR * eA1 / (deltaR * neA + epsilonR);
    float muR2 = -deltaR*deltaR * eA2 / (deltaR * neA + epsilonR);
    float muR3 = -deltaR*deltaR * eA3 / (deltaR * neA + epsilonR);
    // Robust Control --------------------------------------------
  */
  float M1 = -cmd.kR[0] * eR1 - cmd.kOm[0] * eOm1 + in1; // - I[0][0]*muR1;
  float M2 = -cmd.kR[1] * eR2 - cmd.kOm[1] * eOm2 + in2; // - I[1][1]*muR2;
  float M3 = -cmd.kR[2] * eR3 - cmd.kOm[2] * eOm3 + in3; // - I[2][2]*muR3;

  float w_sq[4];
  w_sq[0] = force / (4 * kf) - M2 / (2 * d * kf) + M3 / (4 * km);
  w_sq[1] = force / (4 * kf) + M2 / (2 * d * kf) + M3 / (4 * km);
  w_sq[2] = force / (4 * kf) + M1 / (2 * d * kf) - M3 / (4 * km);
  w_sq[3] = force / (4 * kf) - M1 / (2 * d * kf) - M3 / (4 * km);

  Control control;
  for (int i = 0; i < 4; i++)
  {
    if (w_sq[i] < 0)
      w_sq[i] = 0;

    control.rpm[i] = sqrtf(w_sq[i]);
  }
  return control;
}

// 输入cmd，赋值command
static void cmd_callback(const quadrotor_msgs::SO3Command::ConstPtr& cmd)
{
  command.force[0]         = cmd->force.x;
  command.force[1]         = cmd->force.y;
  command.force[2]         = cmd->force.z;
  command.omega[0]         = cmd->omega.x;
  command.omega[1]         = cmd->omega.y;
  command.omega[2]         = cmd->omega.z;
  command.qx               = cmd->orientation.x;
  command.qy               = cmd->orientation.y;
  command.qz               = cmd->orientation.z;
  command.qw               = cmd->orientation.w;
  command.kR[0]            = cmd->kR[0];
  command.kR[1]            = cmd->kR[1];
  command.kR[2]            = cmd->kR[2];
  command.kOm[0]           = cmd->kOm[0];
  command.kOm[1]           = cmd->kOm[1];
  command.kOm[2]           = cmd->kOm[2];
  command.corrections[0]   = cmd->aux.kf_correction;
  command.corrections[1]   = cmd->aux.angle_corrections[0];
  command.corrections[2]   = cmd->aux.angle_corrections[1];
  command.current_yaw      = cmd->aux.current_yaw;
  command.use_external_yaw = cmd->aux.use_external_yaw;
}

// 输入干扰力，赋值disturbance
static void force_disturbance_callback(const geometry_msgs::Vector3::ConstPtr& f)
{
  disturbance.f(0) = f->x;
  disturbance.f(1) = f->y;
  disturbance.f(2) = f->z;
}

// 输入干扰力矩，赋值disturbance
static void moment_disturbance_callback(const geometry_msgs::Vector3::ConstPtr& m)
{
  disturbance.m(0) = m->x;
  disturbance.m(1) = m->y;
  disturbance.m(2) = m->z;
}

// 节点程序，发布odom，imu，订阅cmd，force_disturbance，moment_disturbance，
int main(int argc, char** argv)
{
  // ros初始化，名称好像是随意选取的
  ros::init(argc, argv, "quadrotor_simulator_so3");

  // 定义节点
  ros::NodeHandle n("~");

  // 定义订阅和发布的消息
  ros::Publisher  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 100);
  ros::Publisher  imu_pub  = n.advertise<sensor_msgs::Imu>("imu", 10);
  ros::Subscriber cmd_sub =
    n.subscribe("cmd", 100, &cmd_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber f_sub =
    n.subscribe("force_disturbance", 100, &force_disturbance_callback,
                ros::TransportHints().tcpNoDelay());
  ros::Subscriber m_sub =
    n.subscribe("moment_disturbance", 100, &moment_disturbance_callback,
                ros::TransportHints().tcpNoDelay());

  // 定义模型
  Simulator::myModel quad;

  // 定义节点参数，初始化模型状态
  double  _init_x, _init_y, _init_z;
  n.param("simulator/init_state_x", _init_x, 0.0);
  n.param("simulator/init_state_y", _init_y, 0.0);
  n.param("simulator/init_state_z", _init_z, -1.0);
  Eigen::Vector3d position = Eigen::Vector3d(_init_x, _init_y, _init_z);
  quad.setStatePos(position);

  // 定义节点参数，设置仿真速率
  double simulation_rate;
  n.param("rate/simulation", simulation_rate, 1000.0);
  ROS_ASSERT(simulation_rate > 0);
  ros::Rate    ros_rate(simulation_rate);
  const double dt = 1 / simulation_rate;

  // 定义节点参数，设置里程发布速率
  double odom_rate;
  n.param("rate/odom", odom_rate, 100.0);
  const ros::Duration odom_pub_duration(1 / odom_rate);

  // 定义节点参数，设置四旋翼的名称
  std::string quad_name;
  n.param("quadrotor_name", quad_name, std::string("quadrotor"));

  // 获取当前状态
  Simulator::myModel::State state = quad.getState();

  // 定义四旋翼转速
  Control control;

  // 里程消息
  nav_msgs::Odometry odom_msg;
  odom_msg.header.frame_id = "/simulator";
  odom_msg.child_frame_id  = "/" + quad_name;

  // IMU消息
  sensor_msgs::Imu imu_msg;
  imu_msg.header.frame_id = "/simulator";

  ros::Time next_odom_pub_time = ros::Time::now();
  while (n.ok())
  {
    // 处理一次回调函数后，继续执行下面的语句
    ros::spinOnce();

    // 设置无人机的电机转速，完成一步仿真
    auto last = control;
    control   = getControl(quad, command); // attitude controller.
    //control   = getRateControl(quad, command); // attitude rate controller.
    // 当输入不存在时，采用上一次的转速
    for (int i = 0; i < 4; ++i)
    {
      if (std::isnan(control.rpm[i]))
        control.rpm[i] = last.rpm[i];
    }
    quad.setInput(control.rpm[0], control.rpm[1], control.rpm[2],
                  control.rpm[3]);
    quad.setExternalForce(disturbance.f);
    quad.setExternalMoment(disturbance.m);
    quad.step(dt);

    // 更新里程消息和IMU消息并发布
    ros::Time tnow = ros::Time::now();
    if (tnow >= next_odom_pub_time)
    {
      next_odom_pub_time += odom_pub_duration;
      odom_msg.header.stamp = tnow;
      state                 = quad.getState();
      ROS_INFO("The actual state is %f, %f, %f",
      state.x[0],state.x[1],state.x[2]);
      stateToOdomMsg(state, odom_msg);
      quadToImuMsg(quad, imu_msg);
      odom_pub.publish(odom_msg);
      imu_pub.publish(imu_msg);
    }
    
    // 休眠1/ros_rate秒
    ros_rate.sleep();
  }

  return 0;
}

// 将状态转为里程消息
void stateToOdomMsg(const Simulator::myModel::State& state, nav_msgs::Odometry& odom)
{
  odom.pose.pose.position.x = state.x(0);
  odom.pose.pose.position.y = state.x(1);
  odom.pose.pose.position.z = state.x(2);

  Eigen::Quaterniond q(state.R);
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();

  odom.twist.twist.linear.x = state.v(0);
  odom.twist.twist.linear.y = state.v(1);
  odom.twist.twist.linear.z = state.v(2);

  odom.twist.twist.angular.x = state.omega(0);
  odom.twist.twist.angular.y = state.omega(1);
  odom.twist.twist.angular.z = state.omega(2);
}

// 将状态转为IMU消息
void quadToImuMsg(const Simulator::myModel& quad, sensor_msgs::Imu& imu)
{
  Simulator::myModel::State state = quad.getState();
  Eigen::Quaterniond                   q(state.R);
  imu.orientation.x = q.x();
  imu.orientation.y = q.y();
  imu.orientation.z = q.z();
  imu.orientation.w = q.w();

  imu.angular_velocity.x = state.omega(0);
  imu.angular_velocity.y = state.omega(1);
  imu.angular_velocity.z = state.omega(2);

  imu.linear_acceleration.x = quad.getAcc()[0];
  imu.linear_acceleration.y = quad.getAcc()[1];
  imu.linear_acceleration.z = quad.getAcc()[2];
}
