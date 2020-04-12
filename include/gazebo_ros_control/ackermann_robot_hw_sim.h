#ifndef _GAZEBO_ROS_CONTROL___ACKERMANN_ROBOT_HW_SIM_H_
#define _GAZEBO_ROS_CONTROL___ACKERMANN_ROBOT_HW_SIM_H_

#include <gazebo_ros_control/default_robot_hw_sim.h>    //Most class are nested
#include <ackermann_controller/ackermann_controller.h>

namespace gazebo_ros_control
{
class AckermannRobotHWSim : public DefaultRobotHWSim
{
public:
  virtual bool initSim(
    const std::string& robot_namespace,
    ros::NodeHandle model_nh,
    gazebo::physics::ModelPtr parent_model,
    const urdf::Model *const urdf_model,
    std::vector<transmission_interface::TransmissionInfo> transmissions);

  virtual void readSim(ros::Time time, ros::Duration period);

  virtual void writeSim(ros::Time time, ros::Duration period);

  //virtual void eStopActive(const bool active); //No changes at this time
private:
  ackermann_controller::AckermannController ackermann_sim_;
};

}

#endif // #ifndef __GAZEBO_ROS_CONTROL_PLUGIN_ACKERMANN_ROBOT_HW_SIM_H_