#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <quadrotor_msgs/Corrections.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/mpc_ref_traj.h>
#include <quadrotor_msgs/SO3Command.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <SO3Control.h>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>

#include "mpc_wrapper.h"

#include "xld_control_nodelet.h"



//Position controller
void XldControlNodelet::publishSO3Command(void)
{
  controller_.calculateControl(des_pos_, des_vel_, des_acc_, des_yaw_,
                               des_yaw_dot_, kx_, kv_);

  // nmpc_controller_.nmpc_core(nmpc_controller_.nmpc_cmd_struct);

  const Eigen::Vector3d&    force       = controller_.getComputedForce();
  std::cout << "force: " << force.transpose() << std::endl; 
  // const Eigen::Vector3d&    force       = nmpc_controller_.getComputedForce();

  const Eigen::Quaterniond& orientation = controller_.getComputedOrientation();
  // const Eigen::Quaterniond& orientation = nmpc_controller_.getComputedOrientation();

}

void XldControlNodelet::position_cmd_callback(const quadrotor_msgs::PositionCommand::ConstPtr& cmd)
{
  ref_odom_ = *cmd;

  des_pos_ = Eigen::Vector3d(cmd->position.x, cmd->position.y, cmd->position.z);
  des_vel_ = Eigen::Vector3d(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
  des_acc_ = Eigen::Vector3d(cmd->acceleration.x, cmd->acceleration.y,
                             cmd->acceleration.z);

  if ( cmd->kx[0] > 1e-5 || cmd->kx[1] > 1e-5 || cmd->kx[2] > 1e-5 )
  {
    kx_ = Eigen::Vector3d(cmd->kx[0], cmd->kx[1], cmd->kx[2]);
  }
  if ( cmd->kv[0] > 1e-5 || cmd->kv[1] > 1e-5 || cmd->kv[2] > 1e-5 )
  {
    kv_ = Eigen::Vector3d(cmd->kv[0], cmd->kv[1], cmd->kv[2]);
  }

  des_yaw_              = cmd->yaw;
  des_yaw_dot_          = cmd->yaw_dot;
  position_cmd_updated_ = true;
  position_cmd_init_    = true;

  std::cout<< "des_pos: " << des_pos_.transpose() << std::endl;
}

void XldControlNodelet::traj_cmd_callback(const quadrotor_msgs::mpc_ref_traj::ConstPtr& msg)
{
   traj_msg = *msg;
  if(mpc_mode == AUTO_HOVER )
  {
    mpc_mode = AUTO_TRACKING;
    // fsm_switch = 1;
    ROS_INFO("Trajectory Received");
  }
}

void XldControlNodelet::odom_callback(const nav_msgs::Odometry::ConstPtr& odom)
{
  if(!odom_flag_)
  {
    start_odom_ = *odom;
    ROS_INFO("Odom Received");
  }
  odom_flag_ = 1;
  current_odom_ = *odom;

  const Eigen::Vector3d position(odom->pose.pose.position.x,
                                 odom->pose.pose.position.y,
                                 odom->pose.pose.position.z);
  const Eigen::Vector3d velocity(odom->twist.twist.linear.x,
                                 odom->twist.twist.linear.y,
                                 odom->twist.twist.linear.z);

  // current_yaw_ = tf::getYaw(odom->pose.pose.orientation);

  // current_pos_att_ =  std::vector<double> {odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z, 
  //                                       odom->pose.pose.orientation.x,odom->pose.pose.orientation.y, odom->pose.pose.orientation.z,
  //                                       odom->pose.pose.orientation.w};//new-add
  // current_vel_rate_ = std::vector<double> {odom->twist.twist.linear.x, odom->twist.twist.linear.y, odom->twist.twist.linear.z,
  //                      odom->twist.twist.angular.x, odom->twist.twist.angular.y, odom->twist.twist.angular.z};//new-add
  // std::cout << "pos: " << position.transpose() << std::endl;
  controller_.setPosition(position);
  controller_.setVelocity(velocity);
   
  
  // nmpc_controller_.nan_check_for_dist_estimates(distFx_,distFy_,distFz_);
  // nmpc_controller_.set_measurements(distFx_, distFy_, distFz_, current_pos_att_, current_vel_rate_, nmpc_struct);

  if (position_cmd_init_)
  {
    // We set position_cmd_updated_ = false and expect that the
    // position_cmd_callback would set it to true since typically a position_cmd
    // message would follow an odom message. If not, the position_cmd_callback
    // hasn't been called and we publish the so3 command ourselves
    // TODO: Fallback to hover if position_cmd hasn't been received for some
    // time
    if (!position_cmd_updated_)
      // publishSO3Command();
    position_cmd_updated_ = false;
  }
  else if ( init_z_ > -9999.0 )
  {
    des_pos_ = Eigen::Vector3d(init_x_, init_y_, init_z_);
    des_vel_ = Eigen::Vector3d(0,0,0);
    des_acc_ = Eigen::Vector3d(0,0,0); 
    ref_odom_.position.x = init_x_;
    ref_odom_.position.y = init_y_;
    ref_odom_.position.z = init_z_;

  }
  
}

void XldControlNodelet::enable_motors_callback(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data)
    ROS_INFO("Enabling motors");
  else
    ROS_INFO("Disabling motors");

  enable_motors_ = msg->data;
}

void XldControlNodelet::corrections_callback(const quadrotor_msgs::Corrections::ConstPtr& msg)
{
  corrections_[0] = msg->kf_correction;
  corrections_[1] = msg->angle_corrections[0];
  corrections_[2] = msg->angle_corrections[1];
}

void XldControlNodelet::imu_callback(const sensor_msgs::Imu& imu)
{
  const Eigen::Vector3d acc(imu.linear_acceleration.x,
                            imu.linear_acceleration.y,
                            imu.linear_acceleration.z);
  controller_.setAcc(acc);
}

// 相当于main函数
void XldControlNodelet::onInit(void)
{
  // 节点句柄
  ros::NodeHandle n("~");
  ROS_INFO("This is info message 1");
  // 节点参数赋值
  std::string quadrotor_name;
  n.param("quadrotor_name", quadrotor_name, std::string("quadrotor"));
  frame_id_ = "/" + quadrotor_name;
  mpc_mode = AUTO_HOVER;
  double mass;
  n.param("mass", mass, 0.98);
  controller_.setMass(mass);

  n.param("use_external_yaw", use_external_yaw_, true);

  n.param("gains/rot/x", kR_[0], 1.5);
  n.param("gains/rot/y", kR_[1], 1.5);
  n.param("gains/rot/z", kR_[2], 1.0);
  n.param("gains/ang/x", kOm_[0], 0.13);
  n.param("gains/ang/y", kOm_[1], 0.13);
  n.param("gains/ang/z", kOm_[2], 0.1);
  n.param("gains/kx/x", kx_[0], 5.7);
  n.param("gains/kx/y", kx_[1], 5.7);
  n.param("gains/kx/z", kx_[2], 6.2);
  n.param("gains/kv/x", kv_[0], 3.4);
  n.param("gains/kv/y", kv_[1], 3.4);
  n.param("gains/kv/z", kv_[2], 4.0);

  n.param("corrections/z", corrections_[0], 0.0);
  n.param("corrections/r", corrections_[1], 0.0);
  n.param("corrections/p", corrections_[2], 0.0);

  n.param("so3_control/init_state_x", init_x_, 0.0);
  n.param("so3_control/init_state_y", init_y_, 0.0);
  n.param("so3_control/init_state_z", init_z_, -10000.0);


  // 节点订阅与发布
  so3_command_pub_ = n.advertise<quadrotor_msgs::SO3Command>("so3_cmd", 10);

  odom_sub_ = n.subscribe("odom", 10, &XldControlNodelet::odom_callback, this,
                          ros::TransportHints().tcpNoDelay());

  position_cmd_sub_ = n.subscribe("position_cmd", 10, &XldControlNodelet::position_cmd_callback,
                this, ros::TransportHints().tcpNoDelay());

  enable_motors_sub_ = n.subscribe("motors", 2, &XldControlNodelet::enable_motors_callback, this,
                ros::TransportHints().tcpNoDelay());
  corrections_sub_ = n.subscribe("corrections", 10, &XldControlNodelet::corrections_callback,
                this, ros::TransportHints().tcpNoDelay());

  imu_sub_ = n.subscribe("imu", 10, &XldControlNodelet::imu_callback, this,
                         ros::TransportHints().tcpNoDelay());

  traj_cmd_sub_ = n.subscribe("traj_cmd", 10, &XldControlNodelet::traj_cmd_callback, this, ros::TransportHints().tcpNoDelay());

  // 100Hz
  ros::Rate rate(100);
  // 状态机
  while(ros::ok())
  {
    FSMProcess();
    ros::spinOnce();
    rate.sleep();
  }
}

// 状态机流程
void XldControlNodelet::FSMProcess()
{
  switch(mpc_mode)
  {
    case AUTO_HOVER:
    {
        reference = Eigen::MatrixXd::Zero(Nreference, N_horizon + 1);
        Eigen::Vector3d acc;
        // 加载参考加速度指令
        acc << ref_odom_.acceleration.x, ref_odom_.acceleration.y, ref_odom_.acceleration.z - 9.81;
        double m1 = 0.98;
        for(int i = 0; i < N_horizon + 1; ++i)
        {
          reference.col(i) << ref_odom_.position.x, ref_odom_.position.y, ref_odom_.position.z,
                                  ref_odom_.velocity.x, ref_odom_.velocity.y, ref_odom_.velocity.z,
                                  acc[0]*m1, acc[1]*m1, acc[2]*m1;
        }
        if(nmpc_wrapper_.getSolution(current_odom_, reference, control_))
          publishcontrol();
        else
          exit(0);
          break;
    }
    case AUTO_TRACKING:
    {
      getTrajRef();
      // if(reachgoal(current_odom, goal))
      // {
      //   mpc_mode = AUTO_HOVER;
      //   fsm_switch = 1;
      // }
          if(nmpc_wrapper_.getSolution(current_odom_, reference, control_))
            publishcontrol();
          else
          {
            exit(0);
            // mpc_mode = AUTO_HOVER;
            // fsm_switch = 1;
            // ROS_ERROR("NO_Solution. Turn to HOVER Mode");
          }
           break;
      }
    }
  
 
}

void XldControlNodelet::acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw, Eigen::Vector4d &quat) 
{
  Eigen::Vector3d zb_des, yb_des, xb_des, yc;
  Eigen::Matrix3d R;

  yc = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())*Eigen::Vector3d::UnitY();
  zb_des = vector_acc / vector_acc.norm();
  xb_des = yc.cross(zb_des) / ( yc.cross(zb_des) ).norm();
  yb_des = zb_des.cross(xb_des) / (zb_des.cross(xb_des)).norm();

  R << xb_des(0), yb_des(0), zb_des(0), 
       xb_des(1), yb_des(1), zb_des(1), 
       xb_des(2), yb_des(2), zb_des(2);

  double tr = R.trace();
  if (tr > 0.0) 
  {
    double S = sqrt(tr + 1.0) * 2.0; // S=4*qw
    quat[0] = 0.25 * S;
    quat[1] = (R(2, 1) - R(1, 2)) / S;
    quat[2] = (R(0, 2) - R(2, 0)) / S;
    quat[3] = (R(1, 0) - R(0, 1)) / S;
  } 
  else if ((R(0, 0) > R(1, 1)) & (R(0, 0) > R(2, 2))) 
  {
    double S = sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0; // S=4*qx
    quat[0] = (R(2, 1) - R(1, 2)) / S;
    quat[1] = 0.25 * S;
    quat[2] = (R(0, 1) + R(1, 0)) / S;
    quat[3] = (R(0, 2) + R(2, 0)) / S;
  } 
  else if (R(1, 1) > R(2, 2)) 
  {
    double S = sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0; // S=4*qy
    quat[0] = (R(0, 2) - R(2, 0)) / S;
    quat[1] = (R(0, 1) + R(1, 0)) / S;
    quat[2] = 0.25 * S;
    quat[3] = (R(1, 2) + R(2, 1)) / S;
  } 
  else 
  {
    double S = sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0; // S=4*qz
    quat[0] = (R(1, 0) - R(0, 1)) / S;
    quat[1] = (R(0, 2) + R(2, 0)) / S;
    quat[2] = (R(1, 2) + R(2, 1)) / S;
    quat[3] = 0.25 * S;
  }
}

void XldControlNodelet::getTrajRef()
{
  double last_px = current_odom_.pose.pose.position.x;
  double last_py = current_odom_.pose.pose.position.y;
  double next_px, next_py;
  double yaw = 0;
  double last_yaw = 0;
  Eigen::Vector4d quat;
  Eigen::Vector3d acc;
  int k = 0;
  double m1 = 0.98;                      
  for(auto point:traj_msg.mpc_ref_points)
  {
    // calculate orientation according to yaw and acc
    next_px = point.position.x;
    next_py = point.position.y;
    if(next_px == last_px && next_py == last_py)
      yaw = last_yaw;
    else
      yaw = acos((next_px - last_px)/sqrt((next_px - last_px)*(next_px - last_px) + (next_py - last_py)*(next_py - last_py)));
    last_yaw = yaw;
    last_px = next_px;
    last_py = next_py;
    acc << point.acceleration.x, point.acceleration.y, point.acceleration.z - 9.8066;
    acc2quaternion(acc, yaw, quat);
    // ref
    reference.col(k).setZero();
    reference.col(k) << point.position.x, point.position.y, point.position.z,
                        point.velocity.x, point.velocity.y, point.velocity.z,
                        acc[0]*m1, acc[1]*m1, acc[2]*m1;
    k++;
  }
}

void XldControlNodelet::publishcontrol()
{

  quadrotor_msgs::SO3Command::Ptr so3_command(
    new quadrotor_msgs::SO3Command); //! @note memory leak?
  so3_command->header.stamp    = ros::Time::now();
  so3_command->header.frame_id = frame_id_;
  so3_command->force.x         = control_[0];
  so3_command->force.y         = control_[1];
  so3_command->force.z         = control_[2];
  so3_command->omega.x         = 0;
  so3_command->omega.y         = 0;
  so3_command->omega.z         = 0;
  Eigen::Vector3f R3 = control_.normalized();
  R3[0] = -R3[0];
  R3[1] = -R3[1];
  R3[2] = -R3[2];
  double phid = -asin(R3[1]);
  double thetad = asin(R3[0]/cos(phid));
  double psid = 0;
  so3_command->orientation.w   =  cos(phid/2)*cos(thetad/2)*cos(psid/2)
                                + sin(phid/2)*sin(thetad/2)*sin(psid/2);
  so3_command->orientation.x   =  sin(phid/2)*cos(thetad/2)*cos(psid/2)
                                - cos(phid/2)*sin(thetad/2)*sin(psid/2);
  so3_command->orientation.y   =  cos(phid/2)*sin(thetad/2)*cos(psid/2)
                                + sin(phid/2)*cos(thetad/2)*sin(psid/2);
  so3_command->orientation.z   =  cos(phid/2)*cos(thetad/2)*sin(psid/2)
                                - sin(phid/2)*sin(thetad/2)*cos(psid/2);                                                                                      
  for (int i = 0; i < 3; i++)
  {
    so3_command->kR[i]  = kR_[i];
    so3_command->kOm[i] = kOm_[i];
  }
  so3_command->aux.current_yaw          = current_yaw_;
  so3_command->aux.kf_correction        = corrections_[0];
  so3_command->aux.angle_corrections[0] = corrections_[1];
  so3_command->aux.angle_corrections[1] = corrections_[2];
  so3_command->aux.enable_motors        = enable_motors_;
  so3_command->aux.use_external_yaw     = use_external_yaw_;
  so3_command_pub_.publish(so3_command);
}


