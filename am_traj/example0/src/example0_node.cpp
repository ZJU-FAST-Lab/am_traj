#include "am_traj/am_traj.hpp"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <random>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace ros;
using namespace Eigen;

class Config
{
public:
    Config(const ros::NodeHandle &nh_priv)
    {
        nh_priv.getParam("TrajectoryTopic", trajectoryTopic);
        nh_priv.getParam("WayPointsTopic", wayPointsTopic);
        nh_priv.getParam("RouteTopic", routeTopic);
        nh_priv.getParam("AccTopic", accTopic);
        nh_priv.getParam("FrameName", frameName);
        nh_priv.getParam("WeightT", weightT);
        nh_priv.getParam("WeightAcc", weightAcc);
        nh_priv.getParam("WeightJerk", weightJerk);
        nh_priv.getParam("MaxAccRate", maxAccRate);
        nh_priv.getParam("MaxVelRate", maxVelRate);
        nh_priv.getParam("Iterations", iterations);
        nh_priv.getParam("Epsilon", epsilon);
    }

    // Advertised Topics
    string trajectoryTopic;
    string wayPointsTopic;
    string routeTopic;
    string accTopic;

    // Frame Name
    std::string frameName;

    // Params
    double weightT;
    double weightAcc;
    double weightJerk;
    double maxAccRate;
    double maxVelRate;
    int iterations;
    double epsilon;
};

class Visualizer
{
public:
    Visualizer(const Config &conf, ros::NodeHandle &nh) : config(conf)
    {
        trajectoryPub = nh.advertise<visualization_msgs::Marker>(config.trajectoryTopic, 1);
        wayPointsPub = nh.advertise<visualization_msgs::Marker>(config.wayPointsTopic, 1);
        routePub = nh.advertise<visualization_msgs::Marker>(config.routeTopic, 1);
        accPub = nh.advertise<visualization_msgs::MarkerArray>(config.accTopic, 1);
    }

private:
    Config config;
    ros::Publisher routePub;
    ros::Publisher wayPointsPub;
    ros::Publisher trajectoryPub;
    ros::Publisher accPub;

public:
    void visualize(const Trajectory &traj, const vector<Vector3d> &route, int id = 0)
    {
        visualization_msgs::Marker routeMarker, wayPointsMarker, trajMarker, accMarker;
        visualization_msgs::MarkerArray accMarkers;

        routeMarker.id = id;
        routeMarker.type = visualization_msgs::Marker::LINE_LIST;
        routeMarker.header.stamp = ros::Time::now();
        routeMarker.header.frame_id = config.frameName;
        routeMarker.pose.orientation.w = 1.00;
        routeMarker.action = visualization_msgs::Marker::ADD;
        routeMarker.ns = "route";
        routeMarker.color.r = 1.00;
        routeMarker.color.g = 0.00;
        routeMarker.color.b = 0.00;
        routeMarker.color.a = 1.00;
        routeMarker.scale.x = 0.05;

        wayPointsMarker = routeMarker;
        wayPointsMarker.type = visualization_msgs::Marker::SPHERE_LIST;
        wayPointsMarker.ns = "waypoints";
        wayPointsMarker.color.r = 1.00;
        wayPointsMarker.color.g = 1.00;
        wayPointsMarker.color.b = 1.00;
        wayPointsMarker.scale.x = 0.30;
        wayPointsMarker.scale.y = 0.30;
        wayPointsMarker.scale.z = 0.30;

        trajMarker = routeMarker;
        trajMarker.ns = "trajectory";
        trajMarker.scale.x = 0.15;
        if (id == 0)
        {
            trajMarker.color.r = 1.00;
            trajMarker.color.g = 0.00;
            trajMarker.color.b = 0.00;
        }
        else if (id == 1)
        {
            trajMarker.color.r = 0.00;
            trajMarker.color.g = 1.00;
            trajMarker.color.b = 0.00;
        }
        else
        {
            trajMarker.color.r = 0.00;
            trajMarker.color.g = 0.00;
            trajMarker.color.b = 1.00;
        }

        accMarker = routeMarker;
        accMarker.type = visualization_msgs::Marker::ARROW;
        accMarker.header.stamp = ros::Time::now();
        accMarker.ns = "acc";
        if (id == 0)
        {
            accMarker.color.r = 255.0 / 255.0;
            accMarker.color.g = 20.0 / 255.0;
            accMarker.color.b = 147.0 / 255.0;
        }
        else if (id == 1)
        {
            accMarker.color.r = 60.0 / 255.0;
            accMarker.color.g = 179.0 / 255.0;
            accMarker.color.b = 113.0 / 255.0;
        }
        else
        {
            accMarker.color.r = 30.0 / 255.0;
            accMarker.color.g = 144.0 / 255.0;
            accMarker.color.b = 255.0 / 255.0;
        }
        accMarker.scale.x = 0.05;
        accMarker.scale.y = 0.15;
        accMarker.scale.z = 0.30;

        if (route.size() > 0)
        {
            bool first = true;
            Vector3d last;
            for (auto it : route)
            {
                if (first)
                {
                    first = false;
                    last = it;
                    continue;
                }
                geometry_msgs::Point point;

                point.x = last(0);
                point.y = last(1);
                point.z = last(2);
                routeMarker.points.push_back(point);
                point.x = it(0);
                point.y = it(1);
                point.z = it(2);
                routeMarker.points.push_back(point);
                last = it;

                wayPointsMarker.points.push_back(point);
            }

            routePub.publish(routeMarker);
        }

        if (route.size() > 0)
        {
            for (auto it : route)
            {
                geometry_msgs::Point point;
                point.x = it(0);
                point.y = it(1);
                point.z = it(2);
                wayPointsMarker.points.push_back(point);
            }

            wayPointsPub.publish(wayPointsMarker);
        }

        if (traj.getPieceNum() > 0)
        {
            double T = 0.01;
            Vector3d lastX = traj.getPos(0.0);
            for (double t = T; t < traj.getTotalDuration(); t += T)
            {
                geometry_msgs::Point point;
                Vector3d X = traj.getPos(t);
                point.x = lastX(0);
                point.y = lastX(1);
                point.z = lastX(2);
                trajMarker.points.push_back(point);
                point.x = X(0);
                point.y = X(1);
                point.z = X(2);
                trajMarker.points.push_back(point);
                lastX = X;
            }
            trajectoryPub.publish(trajMarker);
        }

        if (traj.getPieceNum() > 0)
        {
            if (id == 0)
            {
                accMarker.action = visualization_msgs::Marker::DELETEALL;
                accMarkers.markers.push_back(accMarker);
                accPub.publish(accMarkers);
                accMarkers.markers.clear();
                accMarker.action = visualization_msgs::Marker::ADD;
            }

            double T = 0.08;
            for (double t = 0; t < traj.getTotalDuration(); t += T)
            {
                accMarker.id += 3;
                accMarker.points.clear();
                geometry_msgs::Point point;
                Vector3d X = traj.getPos(t);
                point.x = X(0);
                point.y = X(1);
                point.z = X(2);
                accMarker.points.push_back(point);
                X += traj.getAcc(t);
                point.x = X(0);
                point.y = X(1);
                point.z = X(2);
                accMarker.points.push_back(point);
                accMarkers.markers.push_back(accMarker);
            }
            accPub.publish(accMarkers);
        }
    }
};

class RandomRouteGenerator
{
public:
    RandomRouteGenerator(Array3d l, Array3d u)
        : lBound(l), uBound(u), uniformReal(0.0, 1.0) {}

    inline vector<Vector3d> generate(int N)
    {
        vector<Vector3d> route;
        Array3d temp;
        route.emplace_back(0.0, 0.0, 0.0);
        for (int i = 0; i < N; i++)
        {
            temp << uniformReal(gen), uniformReal(gen), uniformReal(gen);
            temp = (uBound - lBound) * temp + lBound;
            route.emplace_back(temp);
        }
        return route;
    }

private:
    Array3d lBound;
    Array3d uBound;
    std::mt19937_64 gen;
    std::uniform_real_distribution<double> uniformReal;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "example0_node");
    ros::NodeHandle nh_, nh_priv("~");

    Config config(nh_priv);
    Visualizer viz(config, nh_);
    RandomRouteGenerator routeGen(Array3d(-16, -16, -16), Array3d(16, 16, 16));
    AmTraj amTrajOpt(config.weightT, config.weightAcc, config.weightJerk,
                     config.maxVelRate, config.maxAccRate, config.iterations, config.epsilon);

    vector<Vector3d> route;
    Vector3d zeroVec(0.0, 0.0, 0.0);
    Trajectory traj;
    Rate lp(0.25);
    int groupSize = 10;
    for (int i = 5; i < 11 && ok(); i++)
    {
        for (int j = 0; j < groupSize && ok(); j++)
        {
            route = routeGen.generate(i);
            traj = amTrajOpt.genOptimalTrajDTC(route, zeroVec, zeroVec, zeroVec, zeroVec);
            viz.visualize(traj, route, 0);
            std::cout << "---------------------------------------------------------------------------------------" << std::endl;
            std::cout << "RED:  Constrained Spatial-Temporal Optimal Trajectory" << std::endl
                      << "      Lap Time: " << traj.getTotalDuration() << " s" << std::endl
                      << "      Cost: " << amTrajOpt.evaluateObjective(traj) << std::endl
                      << "      Maximum Velocity Rate: " << traj.getMaxVelRate() << " m/s" << std::endl
                      << "      Maximum Acceleration Rate: " << traj.getMaxAccRate() << " m/s^2" << std::endl;

            traj = amTrajOpt.genOptimalTrajDC(route, zeroVec, zeroVec, zeroVec, zeroVec);
            viz.visualize(traj, route, 2);
            std::cout << "BLUE: Constrained Spatial Optimal Trajectory with Trapezoidal Time Allocation" << std::endl
                      << "      Lap Time: " << traj.getTotalDuration() << " s" << std::endl
                      << "      Cost: " << amTrajOpt.evaluateObjective(traj) << std::endl
                      << "      Maximum Velocity Rate: " << traj.getMaxVelRate() << " m/s" << std::endl
                      << "      Maximum Acceleration Rate: " << traj.getMaxAccRate() << " m/s^2" << std::endl;
            spinOnce();
            lp.sleep();
        }
    }

    return 0;
}
