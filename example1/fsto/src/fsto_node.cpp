#include "fsto/fsto.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fsto_node");
    ros::NodeHandle nh_;

    Config config;
    ros::NodeHandle nh_priv("~");

    Config::loadParameters(config, nh_priv);
    MavGlobalPlanner glbPlanner(config, nh_);

    ros::spin();

    return 0;
}
