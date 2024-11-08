cmake_minimum_required(VERSION 2.8.3)
project(xld_control)
set(PROJECT_NAME xld_control)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O1 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  quadrotor_msgs
  tf
  nodelet
)

catkin_package()

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(./include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# include some more paths
include_directories($ENV{ACADOS_SOURCE_DIR}/include)
include_directories($ENV{ACADOS_SOURCE_DIR}/include/blasfeo/include)
include_directories($ENV{ACADOS_SOURCE_DIR}/include/hpipm/include)
include_directories($ENV{ACADOS_SOURCE_DIR}/include/acados)
include_directories(c_generated_code)

# load acados libs
link_directories($ENV{ACADOS_SOURCE_DIR}/lib)

file(GLOB ocp_solver
        c_generated_code/acados_solver_NMPC_model.c
        )

file(GLOB casadi_fun
        c_generated_code/NMPC_model_model/NMPC_model_expl_ode_fun.c
        c_generated_code/NMPC_model_model/NMPC_model_expl_ode_hess.c
        c_generated_code/NMPC_model_model/NMPC_model_expl_vde_forw.c
        c_generated_code/NMPC_model_model/NMPC_model_expl_vde_adj.c)
        
file(GLOB cost_solver
        c_generated_code/NMPC_model_cost/NMPC_model_cost_ext_cost_0_fun.c
        c_generated_code/NMPC_model_cost/NMPC_model_cost_ext_cost_0_fun_jac.c
        c_generated_code/NMPC_model_cost/NMPC_model_cost_ext_cost_0_fun_jac_hess.c
        c_generated_code/NMPC_model_cost/NMPC_model_cost_ext_cost_e_fun.c
        c_generated_code/NMPC_model_cost/NMPC_model_cost_ext_cost_e_fun_jac.c
        c_generated_code/NMPC_model_cost/NMPC_model_cost_ext_cost_e_fun_jac_hess.c
        c_generated_code/NMPC_model_cost/NMPC_model_cost_ext_cost_fun.c
        c_generated_code/NMPC_model_cost/NMPC_model_cost_ext_cost_fun_jac.c
        c_generated_code/NMPC_model_cost/NMPC_model_cost_ext_cost_fun_jac_hess.c
        )

add_library(ocp_shared_lib SHARED ${ocp_solver} ${casadi_fun} ${cost_solver})
target_link_libraries(ocp_shared_lib acados hpipm blasfeo)

add_library(nmpc_warpper src/mpc_wrapper.cpp)
target_link_libraries(nmpc_warpper ocp_shared_lib)

add_library(SO3Control src/SO3Control.cpp)
add_library(xld_control_nodelet src/xld_control_nodelet.cpp)

target_link_libraries(xld_control_nodelet 
    ${catkin_LIBRARIES}
    SO3Control
    nmpc_warpper
    ocp_shared_lib
)


