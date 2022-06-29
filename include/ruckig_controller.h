#include "ros/ros.h"
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include "trajectory_msgs/JointTrajectory.h"

#pragma once

namespace ruckig_control {

std::vector<std::string> getStrings(const ros::NodeHandle& nh, const std::string& param_name)
{
  using namespace XmlRpc;
  XmlRpcValue xml_array;

  if (!nh.getParam(param_name, xml_array))
  {
    ROS_ERROR_STREAM("Could not find '" << param_name << "' parameter (namespace: " << nh.getNamespace() << ").");
    return std::vector<std::string>();
  }

  if (xml_array.getType() != XmlRpcValue::TypeArray)
  {
    ROS_ERROR_STREAM("The '" << param_name << "' parameter is not an array (namespace: " <<
                     nh.getNamespace() << ").");
    return std::vector<std::string>();
  }

  std::vector<std::string> out;
  for (int i = 0; i < xml_array.size(); ++i)
  {
    XmlRpc::XmlRpcValue& elem = xml_array[i];
    if (elem.getType() != XmlRpcValue::TypeString)
    {
      ROS_ERROR_STREAM("The '" << param_name << "' parameter contains a non-string element (namespace: " <<
                       nh.getNamespace() << ").");
      return std::vector<std::string>();
    }
    out.push_back(static_cast<std::string>(elem));
  }
  return out;
}

class RuckigController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
public:
  bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n)
  {
    jointNames = getStrings(n, "joints");

    if (jointNames.size() == 0)
    {
      ROS_ERROR("Must specify joints to control in controller parameters");
      return false;
    }

    for (auto & jointName : jointNames) {
      jointHandles.emplace_back(hw->getHandle(jointName));
      ROS_INFO("Added joint %s", jointName.c_str());
    }

    commandSub = n.subscribe<trajectory_msgs::JointTrajectory>("command", 1, &RuckigController::commandCB, this);

    if (commandSub)
    {
      ROS_INFO("Subscribed to the %s topic. Connected to %d publishers.", commandSub.getTopic().c_str(), commandSub.getNumPublishers());
    }
    else
    {
      ROS_ERROR("Failed to subscribe to the command topic");
      return false;
    }

    return true;
  }

  void update(const ros::Time& time, const ros::Duration& period)
  {
    for (int i = 0; i < jointHandles.size(); i++) {
      auto joint = jointHandles[i];
    }
  }

  void starting(const ros::Time& time) { }
  void stopping(const ros::Time& time) { }

  void commandCB(const trajectory_msgs::JointTrajectoryConstPtr& msg)
  {
    if (jointHandles.size() != msg->joint_names.size())
    {
      ROS_ERROR("Received incorrect number of points. Expected %lu but got %lu.", jointHandles.size(), msg->joint_names.size());
      return;
    }

    for (auto & pt : msg->points)
    {
      for (int i = 0; i < pt.positions.size(); i++)
      {
        jointHandles[i].setCommand(pt.positions[i]);
        ROS_INFO("\tset %s = %f", jointNames[i].c_str(), pt.positions[i]);
      }
    }
  }

private:
  ros::Subscriber commandSub;
  std::vector<std::string> jointNames;
  std::vector<hardware_interface::JointHandle> jointHandles;
};

}//namespace
