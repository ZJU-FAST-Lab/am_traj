#include "router.hpp"

#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(RouterNode, nodelet::Nodelet);

RouterNode::RouterNode()
{
  setName("RouterNode");
  ROS_WARN("Construct router");
  yaw = 0.0;
  yaw_dot = 0.0;
}

RouterNode::~RouterNode()
{
}

void
RouterNode::run(double dt)
{
  if (!hasInput)
  {
    publishCommand(readAveragePosition(5), Eigen::Vector3d(),
                   Eigen::Vector3d(), yaw, 0.0);
  }
  else
  {
    publishCommand(position, velocity, acceleration, yaw, yaw_dot);
  }
}

void
RouterNode::clean()
{
  ROS_WARN("router clean");
  hasInput = false;
  MissionTemplateNode::clean();
}

bool
RouterNode::start()
{
  ROS_WARN("router start");
  initialPosition = readAveragePosition(1);
  MissionTemplateNode::start();
  return true;
}

void
RouterNode::listenerCallback(
  const quadrotor_msgs::PositionCommand::ConstPtr& ptr)
{
  hasInput    = true;
  position(0) = ptr->position.x;
  position(1) = ptr->position.y;
  position(2) = ptr->position.z;

  velocity(0) = ptr->velocity.x;
  velocity(1) = ptr->velocity.y;
  velocity(2) = ptr->velocity.z;

  acceleration(0) = ptr->acceleration.x;
  acceleration(1) = ptr->acceleration.y;
  acceleration(2) = ptr->acceleration.z;

  yaw = ptr->yaw;
  yaw_dot = ptr->yaw_dot;
}

Eigen::Vector3d
RouterNode::getInitialPosition() const
{
  return initialPosition;
}

void
RouterNode::onInit()
{
  ros::NodeHandle nh(getPrivateNodeHandle());
  nh.param("input", input, std::string("position_commond_input"));
  listener = nh.subscribe(input, 10, &RouterNode::listenerCallback, this,
                          ros::TransportHints().tcpNoDelay());
  MissionTemplateNode::onInit();
}
