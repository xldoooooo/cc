#ifndef MPCWrapper_H
#define MPCWrapper_H

#include <thread>
#include <chrono>
#include <stdio.h>
#include <stdlib.h>
// Eigen
#include <Eigen/Eigen>
// Ros
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
// Acados
#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados_solver_NMPC_model.h"


class MPCWrapper
{
  // MPC Para
  private:
    double cost_px, cost_py, cost_pz;
    double cost_vx, cost_vy, cost_vz;
    double cost_ux, cost_uy, cost_uz;
    double ux_max, ux_min, uy_max, uy_min, uz_max, uz_min;
    int status;
    NMPC_model_solver_capsule *acados_ocp_capsule;
    ocp_nlp_config *nlp_config;
    ocp_nlp_dims *nlp_dims;
    ocp_nlp_in *nlp_in;
    ocp_nlp_out *nlp_out;
    ocp_nlp_solver *nlp_solver;
    void *nlp_opts;

  public:
    MPCWrapper();
    ~MPCWrapper();
    bool getSolution(nav_msgs::Odometry& msg, const Eigen::MatrixXd& ref, Eigen::Vector3f& control);
    double x_current[6];
    double u_current[3];
    double param[18];
};


#endif