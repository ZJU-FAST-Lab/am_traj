#ifndef FSTO_H
#define FSTO_H

#include "fsto/config.h"
#include "fsto/glbmap.h"
#include "fsto/r3planner.h"
#include "fsto/traj_gen.h"
#include "quadrotor_msgs/PolynomialTrajectory.h"

#include <iostream>
#include <memory>
#include <cmath>

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Joy.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class Visualization
{
public:
    Visualization(Config &conf, ros::NodeHandle &nh_);

    Config config;
    ros::NodeHandle nh;

    ros::Publisher routePub;
    ros::Publisher wayPointsPub;
    ros::Publisher appliedTrajectoryPub;

    void visualize(const Trajectory &appliedTraj, const std::vector<Eigen::Vector3d> &route, ros::Time timeStamp);
};

class MavGlobalPlanner
{
public:
    MavGlobalPlanner(Config &conf, ros::NodeHandle &nh_);
    ~MavGlobalPlanner();

    Config config;
    ros::NodeHandle nh;

    ros::Subscriber odomSub;
    void odomCallBack(const nav_msgs::Odometry::ConstPtr &msg);
    ros::Subscriber imuSub;
    void imuCallBack(const sensor_msgs::Imu::ConstPtr &msg);
    ros::Subscriber mapSub;
    void mapCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg);
    ros::Subscriber targetSub;
    void targetCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg);
    ros::Subscriber trajTriggerSub;
    void trajTriggerCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg);
    ros::Publisher trajPub;
    ros::Publisher autoManualPub;

    static void polynomialTrajConverter(const Trajectory &traj,
                                        quadrotor_msgs::PolynomialTrajectory &trajMsg,
                                        Eigen::Isometry3d tfR2L, ros::Time &iniStamp);

    Eigen::Isometry3d curOdomPose;
    ros::Time odomStamp;
    Eigen::Vector3d curOdomVel;
    bool odomInitialized;
    Eigen::Vector3d curOdomAcc;
    bool accInitialized;
    bool mapInitialized;

    std::shared_ptr<GlobalMap> glbMapPtr;
    R3Planner r3planner;
    TrajGen trajGen;
    Visualization visualization;
};
#endif