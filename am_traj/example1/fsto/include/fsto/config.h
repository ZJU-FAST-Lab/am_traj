#ifndef CONFIG_H
#define CONFIG_H

#include <string>
#include <vector>

#include <ros/ros.h>

struct Config
{
    // Subscribed Topics
    std::string odomTopic;
    std::string imuTopic;
    std::string mapTopic;
    std::string targetTopic;
    std::string trajTriggerTopic;

    // Advertised Topics
    std::string trajectoryTopic;
    std::string autoManualTopic;

    // Frame Names
    std::string odomFrame;

    // Params
    double unitScaleInSI;
    double r3SafeRadius;
    double bodySafeRadius;
    double edfResolution;
    std::vector<double> r3Bound;
    double searchDuration;
    int tryOut;
    double maxAccRate;
    double maxVelRate;
    double weightT;
    double weightAcc;
    double weightJerk;
    int iterations;
    double temporalResolution;
    double spatialResolution;
    double epsilon;

    static void loadParameters(Config &conf, const ros::NodeHandle &nh_priv)
    {
        nh_priv.getParam("OdomTopic", conf.odomTopic);
        nh_priv.getParam("ImuTopic", conf.imuTopic);
        nh_priv.getParam("MapTopic", conf.mapTopic);
        nh_priv.getParam("TargetTopic", conf.targetTopic);
        nh_priv.getParam("TrajTriggerTopic", conf.trajTriggerTopic);
        nh_priv.getParam("TrajectoryTopic", conf.trajectoryTopic);
        nh_priv.getParam("AutoManualTopic", conf.autoManualTopic);
        nh_priv.getParam("OdomFrame", conf.odomFrame);
        nh_priv.getParam("UnitScaleInSI", conf.unitScaleInSI);
        nh_priv.getParam("R3SafeRadius", conf.r3SafeRadius);
        nh_priv.getParam("BodySafeRadius", conf.bodySafeRadius);
        nh_priv.getParam("EdfResolution", conf.edfResolution);
        nh_priv.getParam("R3Bound", conf.r3Bound);
        nh_priv.getParam("SearchDuration", conf.searchDuration);
        nh_priv.getParam("TryOut", conf.tryOut);
        nh_priv.getParam("MaxAccRate", conf.maxAccRate);
        nh_priv.getParam("MaxVelRate", conf.maxVelRate);
        nh_priv.getParam("WeightT", conf.weightT);
        nh_priv.getParam("WeightAcc", conf.weightAcc);
        nh_priv.getParam("WeightJerk", conf.weightJerk);
        nh_priv.getParam("Iterations", conf.iterations);
        nh_priv.getParam("TemporalResolution", conf.temporalResolution);
        nh_priv.getParam("SpatialResolution", conf.spatialResolution);
        nh_priv.getParam("Epsilon", conf.epsilon);
    }
};

#endif