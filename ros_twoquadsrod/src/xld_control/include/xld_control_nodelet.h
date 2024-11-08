#ifndef __XLD_CONTROL_NODELET_H__
#define __XLD_CONTROL_NODELET_H__

#include <nodelet/nodelet.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <quadrotor_msgs/Corrections.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/mpc_ref_traj.h>
#include <quadrotor_msgs/SO3Command.h>
#include <pluginlib/class_list_macros.h>

#define N_horizon    20
#define Nreference   9
Eigen::MatrixXd reference(Nreference, N_horizon + 1);
enum MPCMode
{
  // AUTO_TAKEOFF,
  AUTO_HOVER,
  AUTO_TRACKING
};


class XldControlNodelet : public nodelet::Nodelet
{
public:
  XldControlNodelet()
    : position_cmd_updated_(false)
    , position_cmd_init_(false)
    , des_yaw_(0)
    , des_yaw_dot_(0)
    , current_yaw_(0)
    , enable_motors_(true)
    , // FIXME
    use_external_yaw_(false){}

  void onInit(void);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  void publishSO3Command(void);
  void position_cmd_callback(const quadrotor_msgs::PositionCommand::ConstPtr& cmd);
  void traj_cmd_callback(const quadrotor_msgs::mpc_ref_traj::ConstPtr& msg);
  void odom_callback(const nav_msgs::Odometry::ConstPtr& odom);
  void enable_motors_callback(const std_msgs::Bool::ConstPtr& msg);
  void corrections_callback(const quadrotor_msgs::Corrections::ConstPtr& msg);
  void imu_callback(const sensor_msgs::Imu& imu);
  void FSMProcess(void);
  void acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw, Eigen::Vector4d &quat);
  void publishcontrol(void);
  void getTrajRef();

  SO3Control  controller_;
  // NMPCControl nmpc_controller_;//new-add
  MPCWrapper nmpc_wrapper_; //new_add

  ros::Publisher  so3_command_pub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber position_cmd_sub_;
  ros::Subscriber enable_motors_sub_;
  ros::Subscriber corrections_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber traj_cmd_sub_;

  bool  position_cmd_updated_, position_cmd_init_;
  std::string frame_id_;

  Eigen::Vector3d des_pos_, des_vel_, des_acc_, kx_, kv_;
  std::vector<double> current_pos_att_;
  std::vector<double> current_vel_rate_;
  std::vector<double> distFx_;
  std::vector<double> distFy_;
  std::vector<double> distFz_;

  double     des_yaw_, des_yaw_dot_;
  double     current_yaw_;
  bool       enable_motors_;
  bool       use_external_yaw_;
  double     kR_[3], kOm_[3], corrections_[3];
  double     init_x_, init_y_, init_z_;

  // nmpc_struct_ nmpc_struct;

  bool odom_flag_ = 0; 
  // bool fsm_switch_ = 1;
  bool mpc_init_ = 0;  
  // Eigen::MatrixXd reference_;
  Eigen::Vector3f control_;
  nav_msgs::Odometry current_odom_, start_odom_;
  quadrotor_msgs::PositionCommand ref_odom_;
  quadrotor_msgs::mpc_ref_traj traj_msg;
  MPCMode mpc_mode;
};

//PLUGINLIB_DECLARE_CLASS(so3_control, SO3ControlNodelet, SO3ControlNodelet,
//                        nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(XldControlNodelet, nodelet::Nodelet);
#endif