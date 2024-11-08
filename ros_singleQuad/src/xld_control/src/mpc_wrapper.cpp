#include "mpc_wrapper.h"


MPCWrapper::MPCWrapper()
{
  cost_px = 40;// nh.getParam("/cost/cost_px", cost_px); 
  cost_py = 40;// nh.getParam("/cost/cost_py", cost_py); 
  cost_pz = 60;// nh.getParam("/cost/cost_pz", cost_pz); 
  cost_vx = 40; // nh.getParam("/cost/cost_vx", cost_vx); 
  cost_vy = 40;// nh.getParam("/cost/cost_vy", cost_vy); 
  cost_vz = 50;// nh.getParam("/cost/cost_vz", cost_vz); 
  cost_ux = 5;// nh.getParam("/cost/cost_wx", cost_wx); 
  cost_uy = 5;// nh.getParam("/cost/cost_wy", cost_wy); 
  cost_uz = 5;// nh.getParam("/cost/cost_wz", cost_wz); 
  ux_max = 25;// nh.getParam("/boudings/wx_max", wx_max); 
  ux_min = -25;// nh.getParam("/boudings/wx_min", wx_min); 
  uy_max = 25;// nh.getParam("/boudings/wy_max", wy_max); 
  uy_min = -25;// nh.getParam("/boudings/wy_min", wy_min); 
  uz_max = 25;// nh.getParam("/boudings/wz_max", wz_max); 
  uz_min = -25;// nh.getParam("/boudings/wz_min", wz_min); 

  // create a capsule according to the pre-defined model
  acados_ocp_capsule = NMPC_model_acados_create_capsule();
  // optimizer
  status = NMPC_model_acados_create(acados_ocp_capsule);
  if (status)
  {
    printf("NMPC_model_acados_create() returned status %d. Exiting.\n", status);
    exit(1);
  }
  nlp_config = NMPC_model_acados_get_nlp_config(acados_ocp_capsule);
  nlp_dims = NMPC_model_acados_get_nlp_dims(acados_ocp_capsule);
  nlp_in = NMPC_model_acados_get_nlp_in(acados_ocp_capsule);
  nlp_out = NMPC_model_acados_get_nlp_out(acados_ocp_capsule);
  nlp_solver = NMPC_model_acados_get_nlp_solver(acados_ocp_capsule);
  nlp_opts = NMPC_model_acados_get_nlp_opts(acados_ocp_capsule);
}

MPCWrapper::~MPCWrapper()
{
  // free solver
  status = NMPC_model_acados_free(acados_ocp_capsule);
  if (status) {
      printf("NMPC_model_acados_free() returned status %d. \n", status);
  }
  // free solver capsule
  status = NMPC_model_acados_free_capsule(acados_ocp_capsule);
  if (status) {
      printf("NMPC_model_acados_free_capsule() returned status %d. \n", status);
  }
}

bool MPCWrapper::getSolution(nav_msgs::Odometry& msg, 
                            const Eigen::MatrixXd& ref,
                            Eigen::Vector3f& control)
{
  /* Update the state. */
	x_current[0] = msg.pose.pose.position.x;
  x_current[1] = msg.pose.pose.position.y;
  x_current[2] = msg.pose.pose.position.z;
  x_current[3] = msg.twist.twist.linear.x;
  x_current[4] = msg.twist.twist.linear.y;
  x_current[5] = msg.twist.twist.linear.z;
  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", x_current);
  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", x_current);

  // Update the reference and cost
  // Update the gains
  param[9] = cost_px;
  param[10] = cost_py;
  param[11] = cost_pz;
  param[12] = cost_vx;
  param[13] = cost_vy;
  param[14] = cost_vz;
  param[15] = cost_ux;
  param[16] = cost_uy;
  param[17] = cost_uz;
  // Update the reference
  int N = nlp_dims->N;
  int nx = *nlp_dims->nx;
  int nu = *nlp_dims->nu;
  for (int i = 0; i < N+1; ++i)
  {
    param[0] = ref.col(i)[0];
    param[1] = ref.col(i)[1];
    param[2] = ref.col(i)[2];
    param[3] = ref.col(i)[3];
    param[4] = ref.col(i)[4];
    param[5] = ref.col(i)[5];
    param[6] = ref.col(i)[6];
    param[7] = ref.col(i)[7];
    param[8] = ref.col(i)[8];
    NMPC_model_acados_update_params(acados_ocp_capsule, i, param, 18);
  }  
  // ROS_INFO("The desired reference traj is %f, %f, %f, %f, %f, %f, %f, %f, %f",
  // param[0],param[1],param[2],param[3],param[4],param[5],param[6],param[7],param[8]);
  /* Update the input boundings. */
  double lbu[] = {ux_min,uy_min,uz_min};
  double ubu[] = {ux_max,uy_max,uz_max};
  for(int i = 0; i < N; ++i)
  {
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lbu",lbu);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ubu",ubu);
  }

  // Initialize solution
  double m1 = 0.98;
  double u0[] = {0, 0, -m1*9.8066};
  for (int i = 0; i < N; i++)
  {
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x_current);
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0);
  }
  ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "x", x_current);
  /* Return Control Input Value */
  status = NMPC_model_acados_solve(acados_ocp_capsule);
  
  // Get the optimized control input
  ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", &u_current);
  control[0] = u_current[0];
  control[1] = u_current[1];
  control[2] = u_current[2];
  // ROS_INFO("The desired control is %f, %f, %f",
  // control[0],control[1],control[2]);

  return true;
}

