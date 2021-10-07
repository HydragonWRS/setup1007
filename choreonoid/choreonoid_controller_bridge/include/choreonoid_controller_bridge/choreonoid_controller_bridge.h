#ifndef __CHOREONOID_CONTROLLER_BRIDGE_H__
#define __CHOROENIOD_CONTROLLER_BRIDGE_H__

#include <cnoid/EigenUtil>
#include <cnoid/JointPath>
#include <cnoid/SharedJoystick>
#include <cnoid/SimpleController>
#include <iostream>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>

class ChoreonoidControllerBridge : public cnoid::SimpleController
{
public:
  ChoreonoidControllerBridge();

  virtual bool initialize(cnoid::SimpleControllerIO* io) override;
  virtual bool control() override;

  cnoid::Link* link(const char* name)
  {
    return body_->link(name);
  }

  void armControl();
  void trackControl();

  void init(cnoid::SimpleControllerIO* io);

  void cmdVelCallback(const geometry_msgs::Twist& msg);
  void jointTrajectoryCallback(const trajectory_msgs::JointTrajectory& msg);
  void modeStateCallback(const std_msgs::String& msg);
  void joyCallback(const sensor_msgs::Joy& msg);

  // ik config
  cnoid::BodyPtr* ik_body_;
  cnoid::Link* seven_dof_end_;
  cnoid::Link* five_dof_end_;
  std::shared_ptr<cnoid::JointPath> base_to_seven_dof_end_;
  std::shared_ptr<cnoid::JointPath> base_to_five_dof_end_;
  cnoid::Vector3 seven_dof_end_pos_;
  cnoid::Vector3 five_dof_end_pos_;
  cnoid::Vector3 seven_dof_end_rot_;
  cnoid::Vector3 five_dof_end_rot_;

  cnoid::Body* body_;
  cnoid::Link::ActuationMode main_actuation_mode_;

  cnoid::Link* track_l_;
  cnoid::Link* track_r_;

  std::string mode_state_;

  bool set_trajectory_;

  double dt_;
  double gain_;

  std::map<std::string, double> q_ref_;
  std::map<std::string, double> q_prev_;
  std::vector<cnoid::Link*> arm_joints_;

  sensor_msgs::Joy joy_;
  geometry_msgs::Twist twist_;
  trajectory_msgs::JointTrajectory trajectory_;
  std::map<std::string, std::vector<double>> trajectory_interpolation_target_;

  std::vector<std::shared_ptr<ros::Subscriber>> joint_trajectory_subscriber_;
  ros::Subscriber cmd_vel_subscriber_;
  ros::Subscriber mode_state_subscriber_;
  ros::Subscriber joy_subscriber_;

  boost::shared_ptr<ros::NodeHandle> ros_node_;
  boost::shared_ptr<ros::AsyncSpinner> async_ros_spin_;

  cnoid::SharedJoystickPtr joystick_;
};

#endif
