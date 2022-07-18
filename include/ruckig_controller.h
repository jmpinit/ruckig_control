#pragma once

#include "ros/ros.h"
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include "trajectory_msgs/JointTrajectory.h"
#include <ruckig/ruckig.hpp>

using namespace ruckig;

namespace ruckig_control {

constexpr double PI = 3.14159;
constexpr double PI_2 = PI / 2.0;

constexpr double radians(double degrees) {
  return 2 * PI * degrees / 360.0;
}

double clamp(double v, double min, double max) {
  if (v < min) {
    return min;
  }
  if (v > max) {
    return max;
  }
  return v;
}

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
    // Parameter reading

    jointNames = getStrings(n, "joints");

    if (jointNames.size() == 0)
    {
      ROS_ERROR("Must specify joints to control in controller parameters");
      return false;
    }

    for (auto & jointName : jointNames) {
      jointHandles.emplace_back(hw->getHandle(jointName));

      lastPositions.emplace_back(0);
      lastVelocities.emplace_back(0);
    }

    // Subscribe to the command topic

    commandSub = n.subscribe<trajectory_msgs::JointTrajectory>("command", 1, &RuckigController::commandCB, this);

    if (!commandSub)
    {
      ROS_ERROR("Failed to subscribe to the command topic");
      return false;
    }

    return true;
  }

  void update(const ros::Time& time, const ros::Duration& period)
  {
    if (!initialized)
    {
      initRuckig();
    }

    // Update the trajectory from Ruckig
    ruckig::Result updateResult = otg.update(input, output);

    if (updateResult == ruckig::Error) {
      ROS_ERROR("Ruckig update: Unclassified error");
    } else if (updateResult == ruckig::ErrorInvalidInput) {
      ROS_ERROR("Ruckig update: Error in the input parameter");
    } else if (updateResult == ruckig::ErrorTrajectoryDuration) {
      ROS_ERROR("Ruckig update: The trajectory exceeds the given positional limits");
    } else if (updateResult == ruckig::ErrorPositionalLimits) {
      ROS_ERROR("Ruckig update: The trajectory cannot be phase synchronized");
    } else if (updateResult == ruckig::ErrorExecutionTimeCalculation) {
      ROS_ERROR("Ruckig update: Error during the extremel time calculation (Step 1)");
    } else if (updateResult == ruckig::ErrorSynchronizationCalculation) {
      ROS_ERROR("Ruckig update: Error during the synchronization calculation (Step 2)");
    }

    if (updateResult < 0) {
      // Abort update if ruckig update failed
      return;
    }

    auto& p = output.new_position;
    output.pass_to_input(input);

    //ROS_INFO("Ruckig update (%f ms) -> %f, %f, %f, %f, %f, %f", period.toSec() * 1000, p[0], p[1], p[2], p[3], p[4], p[5]);

    // Command joints to next position
    for (int i = 0; i < p.size(); i++) {
      // HACK
      jointHandles[i].setCommand(i == 5 ? 0 : p[i]);
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

    // FIXME: take the waypoint times into account

    // FIXME: use the entire trajectory instead of the last point

    auto &lastPoint = msg->points.back();

    for (int i = 0; i < lastPoint.positions.size(); i++)
    {
      input.target_position[i] = lastPoint.positions[i];
      input.target_velocity[i] = 0;
      input.target_acceleration[i] = 0;
    }

    // HACK
    input.target_position[5] = 0;
    
    //ROS_INFO("Updated trajectory");
  }

private:
  // FIXME: Set size automatically
  bool initialized = false;
  Ruckig<6> otg {0.01}; // FIXME: set rate automatically
  InputParameter<6> input;
  OutputParameter<6> output;

  std::vector<float> lastPositions;
  std::vector<float> lastVelocities;

  ros::Subscriber commandSub;
  std::vector<std::string> jointNames;
  std::vector<hardware_interface::JointHandle> jointHandles;

  void initRuckig()
  {
    for (int i = 0; i < jointHandles.size(); i++)
    {
      auto joint = jointHandles[i];

      input.current_position[i] = joint.getPosition();
      // FIXME: don't assume that the robot starts from a stand-still
      input.current_velocity[i] = 0;
      input.current_acceleration[i] = 0;

      input.target_position[i] = input.current_position[i];
      input.target_velocity[i] = 0;
      input.target_acceleration[i] = 0;

      // TODO: make these configurable in the controller parameters
      input.max_velocity[i] = 2.0;
      input.max_acceleration[i] = 5.0;
      input.max_jerk[i] = 100.0;
    }

    initialized = true;
  }
};

} // namespace
