#include "traj_min_jerk.hpp"
#include "traj_min_snap.hpp"

#include <chrono>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <random>

#include <ros/ros.h>

#include <quadrotor_msgs/mpc_ref_point.h>
#include <quadrotor_msgs/mpc_ref_traj.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

using namespace std;
using namespace ros;
using namespace Eigen;

#define N_horizon 20
#define t_step 0.1

class RandomRouteGenerator
{
public:
    RandomRouteGenerator(Array3d l, Array3d u)
        : lBound(l), uBound(u), uniformReal(0.0, 1.0) {}

    inline MatrixXd generate(int N)
    {
        MatrixXd route(3, N + 1);
        Array3d temp;
        route.col(0).setZero();
        route.col(0) << 0, 0, 2;
        for (int i = 0; i < N; i++)
        {
            temp << uniformReal(gen), uniformReal(gen), uniformReal(gen);
            temp = (uBound - lBound) * temp + lBound;
            route.col(i + 1) << temp;
        }
        std::cout << "route" << route << std::endl;
        return route;
    }

private:
    Array3d lBound;
    Array3d uBound;
    std::mt19937_64 gen;
    std::uniform_real_distribution<double> uniformReal;
};

VectorXd allocateTime(const MatrixXd &wayPs,
                      double vel,
                      double acc)
{
    int N = (int)(wayPs.cols()) - 1;
    VectorXd durations(N);
    if (N > 0)
    {

        Eigen::Vector3d p0, p1;
        double dtxyz, D, acct, accd, dcct, dccd, t1, t2, t3;
        for (int k = 0; k < N; k++)
        {
            p0 = wayPs.col(k);
            p1 = wayPs.col(k + 1);
            D = (p1 - p0).norm();

            acct = vel / acc;
            accd = (acc * acct * acct / 2);
            dcct = vel / acc;
            dccd = acc * dcct * dcct / 2;

            if (D < accd + dccd)
            {
                t1 = sqrt(acc * D) / acc;
                t2 = (acc * t1) / acc;
                dtxyz = t1 + t2;
            }
            else
            {
                t1 = acct;
                t2 = (D - accd - dccd) / vel;
                t3 = dcct;
                dtxyz = t1 + t2 + t3;
            }

            durations(k) = dtxyz;
        }
    }

    return durations;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "example1_node");
    ros::NodeHandle nh_("~");
    ros::Publisher mpc_ref_pub;
    ros::Publisher traj_pub;

    int opt_method = 0;  // 0 for minimun jerk, 1 for minimun snap
    opt_method = nh_.param("opt_method", 0);
    std::cout << opt_method << std::endl;
    // 发布mpc_ref_traj和vis_traj话题
    mpc_ref_pub = nh_.advertise<quadrotor_msgs::mpc_ref_traj> ("/mpc_ref_traj", 1);
    traj_pub = nh_.advertise<visualization_msgs::Marker> ("/vis_trajectory", 1);
    // RandomRouteGenerator routeGen(Array3d(-16, -16, -16), Array3d(16, 16, 16));
    visualization_msgs::Marker traj_vis;


    min_jerk::JerkOpt jerkOpt;
    min_jerk::Trajectory minJerkTraj;

    min_snap::SnapOpt snapOpt;
    min_snap::Trajectory minSnapTraj;

    MatrixXd route;
    VectorXd ts;
    Matrix3d iS, fS;
    Eigen::Matrix<double, 3, 4> iSS, fSS;
    iS.setZero();
    fS.setZero();
    Vector3d zeroVec(0.0, 0.0, 0.0);
    Rate lp(100);

    std::chrono::high_resolution_clock::time_point tc0, tc1, tc2;
    double d0, d1;
    double t_duration;

    d0 = d1 = 0.0;
    // route = routeGen.generate(3);
    route = Eigen::MatrixXd::Zero(3, 13);
    route << 0,    0.5,  1,   1.5,   2,   2.5,   3,   3,   3,   3,   3,   3,   3,
             0,     0,   0,    0,    0,    0,    0,  0.5,  1,  1.5,  2,  2.5,  3,
            -0.3, -0.3,-0.3, -0.3, -0.3, -0.3, -0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3;

    iS.col(0) << route.leftCols<1>();
    fS.col(0) << route.rightCols<1>();
    ts = allocateTime(route, 0.5, 0.2);
    int wp_num = route.cols() - 1;
   
    iSS << iS, Eigen::MatrixXd::Zero(3, 1);
    fSS << fS, Eigen::MatrixXd::Zero(3, 1);

    // 0对应minJerk，1对应minSnap
    if(opt_method == 0)
    {
      tc0 = std::chrono::high_resolution_clock::now();
      jerkOpt.reset(iS, fS, route.cols() - 1);
      jerkOpt.generate(route.block(0, 1, 3, wp_num - 1), ts);
      jerkOpt.getTraj(minJerkTraj);
      t_duration = minJerkTraj.getTotalDuration();
      tc1 = std::chrono::high_resolution_clock::now();

      d0 += std::chrono::duration_cast<std::chrono::duration<double>>(tc1 - tc0).count();

      std::cout << "Piece Number: " << 2
                << " MinJerk Comp. Time: " << d0  << " s"<< std::endl;
    }
    else
    {
      tc1 = std::chrono::high_resolution_clock::now();
      snapOpt.reset(iSS, fSS, route.cols() - 1);
      snapOpt.generate(route.block(0, 1, 3, wp_num - 1), ts);
      snapOpt.getTraj(minSnapTraj);
      t_duration = minSnapTraj.getTotalDuration();
      tc2 = std::chrono::high_resolution_clock::now();

      d1 += std::chrono::duration_cast<std::chrono::duration<double>>(tc2 - tc1).count();

      std::cout << "Piece Number: " << 2
                << " MinSnap Comp. Time: " << d1  << " s" << std::endl;
    }

    double t = 0;
    double t_max = 0;
    ros::Time time_start;
    int start = 0;

    while(ros::ok())
    {
      if(start < 2)
      {
        time_start = ros::Time::now();
        start++;
      }
      ros::Time time_now = ros::Time::now();
      t = time_now.toSec() - time_start.toSec() + 0.01;
      quadrotor_msgs::mpc_ref_traj ref_traj;

      for(int i = 0; i < N_horizon + 1; ++i)
      {
        double rt = t + t_step * i;
        if(rt > t_duration)
          rt = t_max;
        else
          t_max = rt;

        Eigen::Vector3d p;
        Eigen::Vector3d v;
        Eigen::Vector3d a;

        if(opt_method == 0)
        {
          p = minJerkTraj.getPos(rt);
          v = minJerkTraj.getVel(rt);
          a = minJerkTraj.getAcc(rt);
        }
        else
        {
          p = minSnapTraj.getPos(rt);
          v = minSnapTraj.getVel(rt);
          a = minSnapTraj.getAcc(rt);
        }

        quadrotor_msgs::mpc_ref_point ref_point;
        ref_point.position.x = p[0];
        ref_point.position.y = p[1];
        ref_point.position.z = p[2];
        ref_point.velocity.x = v[0];
        ref_point.velocity.y = v[1];
        ref_point.velocity.z = v[2];
        ref_point.acceleration.x = a[0];
        ref_point.acceleration.y = a[1];
        ref_point.acceleration.z = a[2];
        ref_traj.mpc_ref_points.push_back(ref_point);

        traj_vis.header.stamp = ros::Time::now();
        traj_vis.header.frame_id = "world";
        traj_vis.ns = "traj_ns";
        traj_vis.id = 0;
        traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
        traj_vis.action = visualization_msgs::Marker::ADD;
        traj_vis.scale.x = 0.15;
        traj_vis.scale.y = 0.15;
        traj_vis.scale.z = 0.15;
        traj_vis.pose.orientation.x = 0.0;
        traj_vis.pose.orientation.y = 0.0;
        traj_vis.pose.orientation.z = 0.0;
        traj_vis.pose.orientation.w = 1.0;
        traj_vis.color.a = 1.0;
        traj_vis.color.r = 1.0;
        traj_vis.color.g = 0.0;
        traj_vis.color.b = 0.0;
        // traj_vis.points.clear();
        geometry_msgs::Point pt;
        pt.x = p[0];
        pt.y = p[1];
        pt.z = p[2];
        traj_vis.points.push_back(pt);
      }

      ref_traj.goal.x = 6;
      ref_traj.goal.y = 0;
      ref_traj.goal.z = -2;

      mpc_ref_pub.publish(ref_traj);
      traj_pub.publish(traj_vis);

      ros::spinOnce();
      lp.sleep();
    }
    
    return 0;
}
