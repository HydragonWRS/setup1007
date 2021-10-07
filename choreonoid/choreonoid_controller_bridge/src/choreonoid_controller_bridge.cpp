#include <choreonoid_controller_bridge/choreonoid_controller_bridge.h>

std::vector<double> pgain = {
  /* MFRAME */ 200000, /* BLOCK */ 150000, /* BOOM */ 150000, /* ARM  */ 100000,
  /* PITCH  */ 30000,  /* ROLL  */ 20000,  /* TIP1 */ 500,    /* TIP2 */ 500,
  /* UFRAME */ 150000, /* SWING */ 50000,  /* BOOM */ 100000, /* ARM  */ 80000,
  /* ELBOW */ 30000,   /* YAW   */ 20000,  /* HAND */ 500,    /* ROD  */ 50000
};
std::vector<double> dgain = {
  /* MFRAME */ 20000, /* BLOCK */ 15000, /* BOOM */ 10000, /* ARM  */ 5000,
  /* PITCH  */ 500,   /* ROLL  */ 500,   /* TIP1 */ 50,    /* TIP2 */ 50,
  /* UFRAME */ 15000, /* SWING */ 1000,  /* BOOM */ 3000,  /* ARM  */ 2000,
  /* ELBOW */ 500,    /* YAW   */ 500,   /* HAND */ 20,    /* ROD  */ 5000
};

ChoreonoidControllerBridge::ChoreonoidControllerBridge()
{
  main_actuation_mode_ = cnoid::Link::JointDisplacement;
}

bool ChoreonoidControllerBridge::initialize(cnoid::SimpleControllerIO* io)
{
  body_ = io->body();

  seven_dof_end_ = body_->link("HANDBASE");
  five_dof_end_ = body_->link("TOHKU_TIP_01");
  cnoid::Link* base = body_->rootLink();
  base_to_five_dof_end_ = cnoid::JointPath::getCustomPath(body_, base, five_dof_end_);
  base_to_seven_dof_end_ = cnoid::JointPath::getCustomPath(body_, base, seven_dof_end_);
  base_to_five_dof_end_->calcForwardKinematics();
  base_to_seven_dof_end_->calcForwardKinematics();

  seven_dof_end_pos_ = seven_dof_end_->p();
  five_dof_end_pos_ = five_dof_end_->p();
  seven_dof_end_rot_ = cnoid::rpyFromRot(seven_dof_end_->attitude());
  five_dof_end_rot_ = cnoid::rpyFromRot(five_dof_end_->attitude());

  static bool initialized = false;
  int argc = 0;
  char** argv;

  set_trajectory_ = false;

  if (!ros::isInitialized()) {
    ros::init(argc, argv, "choreonoid");
  }

  init(io);

  std::string name = body_->name();
  std::replace(name.begin(), name.end(), '-', '_');
  ros_node_ = boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle(name));

  joy_subscriber_ = ros_node_->subscribe("/joy", 5, &ChoreonoidControllerBridge::joyCallback, this);
  cmd_vel_subscriber_ = ros_node_->subscribe("cmd_vel", 5, &ChoreonoidControllerBridge::cmdVelCallback, this);
  mode_state_subscriber_ = ros_node_->subscribe("/mode_state", 5, &ChoreonoidControllerBridge::modeStateCallback, this);

  std::vector<std::string> topic_list = { "/ARM7_controller/set_joint_trajectroy",
                                          "/ARM_controller/set_joint_trajectroy",
                                          "/DualArm_controller/set_joint_trajectroy" };
  joint_trajectory_subscriber_.resize(topic_list.size());
  for (std::size_t idx = 0; idx < topic_list.size(); ++idx) {
    joint_trajectory_subscriber_.at(idx).reset(new ros::Subscriber());

    *joint_trajectory_subscriber_.at(idx) =
      ros_node_->subscribe(topic_list.at(idx), 10, &ChoreonoidControllerBridge::jointTrajectoryCallback, this);
  }

  async_ros_spin_.reset(new ros::AsyncSpinner(4));
  async_ros_spin_->start();

  return true;
}

void ChoreonoidControllerBridge::init(cnoid::SimpleControllerIO* io)
{
  dt_ = io->timeStep();
  std::string option = io->optionString();
  if (option == "position") {
    main_actuation_mode_ = cnoid::Link::JointDisplacement;
  } else if (option == "torque") {
    main_actuation_mode_ = cnoid::Link::JointEffort;
  }

  // set up arm joint
  for (auto joint : body_->joints()) {
    if (joint->jointId() >= 0 && (joint->isRevoluteJoint() || joint->isPrismaticJoint())) {
      joint->setActuationMode(main_actuation_mode_);
      io->enableIO(joint);
      arm_joints_.push_back(joint);
      q_ref_[joint->name()] = joint->q();
    }
  }
  q_prev_ = q_ref_;

  // set up track joint
  gain_ = 3.0;
  track_l_ = link("WHEEL_L0");
  track_r_ = link("WHEEL_R0");
  if (!track_l_ || !track_r_) {
    gain_ = 1.0;
    track_l_ = link("TRACK_L");
    track_r_ = link("TRACK_R");
  }

  track_l_->setActuationMode(cnoid::Link::JointVelocity);
  track_r_->setActuationMode(cnoid::Link::JointVelocity);

  io->enableOutput(track_l_);
  io->enableOutput(track_r_);
}

bool ChoreonoidControllerBridge::control()
{
  static int trajectory_idx = 0;

  trackControl();

  if (mode_state_ == "GripperControl") {
    static double pushrod = 0.001;
    if(0 < joy_.axes.at(4)) {
      pushrod = -0.001;
    } else if(joy_.axes.at(4) < 0) {
      pushrod = 0.001;
    }
    double tohku_tip = -0.01 * joy_.axes.at(1);
    q_ref_[link("PUSHROD")->name()] = link("PUSHROD")->q() + pushrod;
    q_ref_[link("TOHKU_TIP_01")->name()] = q_ref_[link("TOHKU_TIP_02")->name()] = link("TOHKU_TIP_01")->q() + tohku_tip;
  } else if (mode_state_ == "TeleopIK_7DoF") {
    if (joy_.buttons.at(5) == 0) {
      seven_dof_end_pos_.x() += (0.01 * joy_.axes.at(4));
      seven_dof_end_pos_.y() += (0.01 * joy_.axes.at(3));
      seven_dof_end_pos_.z() += (0.01 * joy_.axes.at(1));
    } else if (joy_.buttons.at(5) == 1) {
      seven_dof_end_rot_.x() += (0.01 * joy_.axes.at(4));
      seven_dof_end_rot_.y() += (0.01 * joy_.axes.at(3));
      seven_dof_end_rot_.z() += (0.01 * joy_.axes.at(1));
    }

    bool result = base_to_seven_dof_end_->calcInverseKinematics(seven_dof_end_pos_, seven_dof_end_->calcRfromAttitude(cnoid::rotFromRpy(seven_dof_end_rot_)));
    if (result) {
      for (std::size_t idx = 0; idx < base_to_seven_dof_end_->numJoints(); ++idx) {
        cnoid::Link* joint = base_to_seven_dof_end_->joint(idx);
        q_ref_[joint->name()] = joint->q();
      }
    }

    base_to_seven_dof_end_->calcForwardKinematics();
    seven_dof_end_pos_ = seven_dof_end_->p();
    seven_dof_end_rot_ = cnoid::rpyFromRot(seven_dof_end_->attitude());
  } else if (mode_state_ == "TeleopIK_5DoF") {
    if (joy_.buttons.at(5) == 0) {
      five_dof_end_pos_.x() += (0.01 * joy_.axes.at(4));
      five_dof_end_pos_.y() += (0.01 * joy_.axes.at(3));
      five_dof_end_pos_.z() += (0.01 * joy_.axes.at(1));
    } else if (joy_.buttons.at(5) == 1) {
      five_dof_end_rot_.x() += (0.01 * joy_.axes.at(4));
      five_dof_end_rot_.y() += (0.01 * joy_.axes.at(3));
      five_dof_end_rot_.z() += (0.01 * joy_.axes.at(1));
    }

    bool result = base_to_five_dof_end_->calcInverseKinematics(five_dof_end_pos_, five_dof_end_->calcRfromAttitude(cnoid::rotFromRpy(five_dof_end_rot_)));
    if (result) {
      for (std::size_t idx = 0; idx < base_to_five_dof_end_->numJoints(); ++idx) {
        cnoid::Link* joint = base_to_five_dof_end_->joint(idx);
        q_ref_[joint->name()] = joint->q();
      }
    }

    base_to_five_dof_end_->calcForwardKinematics();
    five_dof_end_pos_ = five_dof_end_->p();
    five_dof_end_rot_ = cnoid::rpyFromRot(five_dof_end_->attitude());
  }

  if (set_trajectory_ && mode_state_ != "TeleopIK_5DoF" && mode_state_ != "TeleopIK_7DoF") {
    for (auto trajectory : trajectory_interpolation_target_) {
      q_ref_[trajectory.first] =
        trajectory.second.front() +
        (trajectory.second.back() - trajectory.second.front()) * (trajectory_idx * 0.01) /
          (trajectory_.points.back().time_from_start.toSec() - trajectory_.points.front().time_from_start.toSec());
    }

    trajectory_idx++;
    if (trajectory_.points.back().time_from_start.toSec() <= trajectory_idx * 0.01) {
      trajectory_idx = 0;
      set_trajectory_ = false;
      trajectory_interpolation_target_.clear();

      base_to_seven_dof_end_->calcForwardKinematics();
      base_to_five_dof_end_->calcForwardKinematics();
      seven_dof_end_pos_ = seven_dof_end_->p();
      five_dof_end_pos_ = five_dof_end_->p();
      seven_dof_end_rot_ = cnoid::rpyFromRot(seven_dof_end_->attitude());
      five_dof_end_rot_ = cnoid::rpyFromRot(five_dof_end_->attitude());
    }
  }

  if (main_actuation_mode_ == cnoid::Link::JointDisplacement) {
    for (std::size_t i = 0; i < arm_joints_.size(); ++i)
      arm_joints_[i]->q_target() = q_ref_[arm_joints_[i]->name()];
  } else {
    for (size_t i = 0; i < arm_joints_.size(); ++i) {
      auto joint = arm_joints_[i];
      auto q_current = joint->q();
      auto dq_current = (q_current - q_prev_[arm_joints_[i]->name()]) / dt_;
      joint->u() = pgain[i] * (q_ref_[arm_joints_[i]->name()] - q_current) + dgain[i] * (0.0 - dq_current);
      q_prev_[arm_joints_[i]->name()] = q_current;
    }
  }

  return true;
}

void ChoreonoidControllerBridge::cmdVelCallback(const geometry_msgs::Twist& msg)
{
  twist_ = msg;
}

void ChoreonoidControllerBridge::joyCallback(const sensor_msgs::Joy& msg)
{
  joy_ = msg;
}

void ChoreonoidControllerBridge::modeStateCallback(const std_msgs::String& msg)
{
  mode_state_ = msg.data;
}

void ChoreonoidControllerBridge::jointTrajectoryCallback(const trajectory_msgs::JointTrajectory& msg)
{
  if (!set_trajectory_) {
    trajectory_ = msg;
    set_trajectory_ = true;

    std::vector<std::string> joint_list;
    for (std::size_t idx = 0; idx < trajectory_.joint_names.size(); idx++) {
      joint_list.push_back(trajectory_.joint_names[idx]);
    }

    for (std::size_t j = 0; j < joint_list.size(); ++j) {
      for (std::size_t i = 0; i < trajectory_.points.size(); ++i) {
        trajectory_interpolation_target_[joint_list[j]].push_back(trajectory_.points[i].positions[j]);
      }
    }
  }
}

void ChoreonoidControllerBridge::trackControl()
{
  const double wheel_base = 1.44;
  const double wheel_radius = 0.44;
  track_l_->dq_target() = ((twist_.linear.x - (wheel_base / 2.0) * twist_.angular.z) / wheel_radius) * gain_;
  track_r_->dq_target() = ((twist_.linear.x + (wheel_base / 2.0) * twist_.angular.z) / wheel_radius) * gain_;
}

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(ChoreonoidControllerBridge)
