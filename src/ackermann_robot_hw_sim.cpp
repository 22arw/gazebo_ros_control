#include <gazebo_ros_control/ackermann_robot_hw_sim.h>

namespace gazebo_ros_control
{
bool AckermannRobotHWSim::initSim(
  const std::string& robot_namespace,
  ros::NodeHandle model_nh,
  gazebo::physics::ModelPtr parent_model,
  const urdf::Model *const urdf_model,
  std::vector<transmission_interface::TransmissionInfo> transmissions)
{
  ROS_DEBUG_STREAM_NAMED("AckermannRobotHWSim", "Initializing Robot HW Simulation");

  if(!DefaultRobotHWSim::initSim(robot_namespace, model_nh, parent_model, urdf_model, transmissions))
  {
    ROS_ERROR_STREAM_NAMED("AckermannRobotHWSim", "initSim failed for DefaultRobotHWSim derived class");
    return false;
  }
  else
  {
    ROS_DEBUG_STREAM_NAMED("AckermannRobotHWSim", "Default RobotHWSim derived class initialized correctly.");
  }
  ros::NodeHandle controller_nh(model_nh.getNamespace() + "/ackermann_controller");
  
  ROS_DEBUG_STREAM_NAMED("AckermannRobotHWSim", "Initializing Ackermann Control");

  if(!ackermann_sim_.init(this, model_nh, controller_nh))
  {
    ROS_ERROR_STREAM_NAMED("AckermannRobotHWSim", "init ackermann_sim_ failed.");
    return false;
  }
  else
  {
    ROS_DEBUG_STREAM_NAMED("AckermannRobotHWSim", "AckermannController Initialized Correctly");
  }

  ROS_DEBUG_STREAM_NAMED("AckermannRobotHWSim", "Starting Ackermann Control");

  if(!ackermann_sim_.startRequest(ros::Time::now()))
  {
    ROS_ERROR_STREAM_NAMED("AckermannRobotHWSim", "Starting Ackermann Control Failed");
    return false;
  }
  else
  {
    ROS_DEBUG_STREAM_NAMED("AckermannRobotHWSim", "Successfully started Ackerman Control");
  }
  return true;
}

void AckermannRobotHWSim::readSim(ros::Time time, ros::Duration period)
{
  DefaultRobotHWSim::readSim(time, period);
  ackermann_sim_.updateOdometry(time, period);  //updateOdometry used to be private, moved to public
}

void AckermannRobotHWSim::writeSim(ros::Time time, ros::Duration period)
{
  ackermann_sim_.moveRobot(time, period); //updateOdometry used to be private, moved to public
  DefaultRobotHWSim::writeSim(time, period);
}

}

PLUGINLIB_EXPORT_CLASS(gazebo_ros_control::AckermannRobotHWSim, gazebo_ros_control::RobotHWSim)