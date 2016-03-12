/*
* @copyright: Copyright[2015]<Svetlin Penkov>
*      @date: 2015-11-14
*     @email: s.v.penkov@sms.ed.ac.uk
*      @desc: Add file description...
*/
#include "baxter_cpp/baxter_arm.hpp"

#include <fstream>
#include <sstream>

#include <baxter_core_msgs/CameraControl.h>
#include <baxter_core_msgs/OpenCamera.h>
#include <baxter_core_msgs/SolvePositionIK.h>

using std::endl;
using std::ifstream;
using std::ofstream;
using std::string;
using std::stringstream;

BaxterArm::BaxterArm(ros::NodeHandle& nh, std::string name) :
  nh_(nh),
  name_(name) {

  stringstream topic_name;

  topic_name << "/robot/limb/" << name_ << "/joint_command";
  joint_command_pub_ = nh_.advertise<baxter_core_msgs::JointCommand>(
                         topic_name.str(), 10);
  topic_name.str("");

  topic_name << "/ExternalTools/" << name_ << "/PositionKinematicsNode/IKService";
  ik_solver_client_ = nh.serviceClient<baxter_core_msgs::SolvePositionIK>(
                        topic_name.str());
  topic_name.str("");

  topic_name << "/robot/limb/" << name_ << "/endpoint_state";
  endpoint_state_sub_  = nh_.subscribe<baxter_core_msgs::EndpointState>(
                           "/robot/limb/right/endpoint_state",
                           10,
                           &BaxterArm::EndpointStateCallback,
                           this);
  topic_name.str("");

  topic_name << "/robot/digital_io/" << name_ << "_lower_button/state";
  circle_cuff_button_sub_ = nh_.subscribe<baxter_core_msgs::DigitalIOState>(
                              topic_name.str(),
                              10,
                              &BaxterArm::CircleCuffButtonStateCallback,
                              this);
  circle_cuff_button_state_ = false;
  debounce_circle_cuff_button_ = false;
  topic_name.str("");

  topic_name << "/robot/digital_io/" << name_ << "_upper_button/state";
  dash_cuff_button_sub_ = nh_.subscribe<baxter_core_msgs::DigitalIOState>(
                            topic_name.str(),
                            10,
                            &BaxterArm::DashCuffButtonStateCallback,
                            this);
  dash_cuff_button_state_ = false;
  debounce_dash_cuff_button_ = false;
  topic_name.str("");

  topic_name << "/robot/range/" << name_ << "_hand_range/state";
  hand_distance_sub_ = nh.subscribe<sensor_msgs::Range>(
                         topic_name.str(),
                         10,
                         &BaxterArm::HandDistanceCallback,
                         this);
  hand_distance_ = 65.535; // Maximum value set by the baxter
  topic_name.str("");

  topic_name << "/robot/end_effector/" << name_ << "_gripper/command";
  end_effector_pub_ = nh_.advertise<baxter_core_msgs::EndEffectorCommand>(
                        topic_name.str(), 10, true);
  end_effector_cmd_.id = 65538;
  end_effector_cmd_.sender = ros::this_node::getName();
  topic_name.str("");

  open_camera_client_ = nh_.serviceClient<baxter_core_msgs::OpenCamera>(
                          "/cameras/open");

  led_pub_ = nh_.advertise<baxter_core_msgs::DigitalOutputCommand>(
               "/robot/digital_io/command", 10);

  // Initialisation Code
  EndEffector(baxter_core_msgs::EndEffectorCommand::CMD_CALIBRATE);
}

void BaxterArm::SetJointState(const sensor_msgs::JointState& joint_state) {
  joint_state_.header = joint_state.header;
  joint_state_.name.clear();
  joint_state_.position.clear();
  joint_state_.velocity.clear();
  joint_state_.effort.clear();

  for (size_t i = 0; i < joint_state.name.size(); ++i) {
    if (joint_state.name[i].find(name_) == 0) {
      joint_state_.name.push_back(joint_state.name[i]);
      joint_state_.position.push_back(joint_state.position[i]);
      joint_state_.velocity.push_back(joint_state.velocity[i]);
      joint_state_.effort.push_back(joint_state.effort[i]);
    }
  }
}

bool BaxterArm::CalibratePose(std::string pose_name) {
  baxter_core_msgs::JointCommand cmd;
  cmd.mode = cmd.POSITION_MODE;
  cmd.names = joint_state_.name;
  cmd.command = joint_state_.position;
  poses[pose_name] = cmd;

  SaveCalibratedPoses();

  return true;
}

bool BaxterArm::IsPoseCalibrated(std::string pose_name) {
  auto pose = poses.find(pose_name);
  return pose != poses.end();
}

void BaxterArm::LoadCalibratedPoses() {
  stringstream param_name, default_value;
  param_name << name_ << "_arm/calib_file";
  default_value << name_ << "_poses.cfg";

  nh_.param<string>(param_name.str(),
                    poses_filename_,
                    default_value.str());

  ifstream ifs;

  ifs.open(poses_filename_);
  if (!ifs.is_open()) {
    ROS_WARN_STREAM(name_ << " arm: Unable to load poses.");
    return;
  }

  int num_poses;
  ifs >> num_poses;
  for (int i = 0; i < num_poses; ++i) {
    string pose_name;
    ifs >> pose_name;

    baxter_core_msgs::JointCommand cmd;
    ifs >> cmd.mode;

    int num_names;
    ifs >> num_names;
    cmd.names.resize(num_names);
    for (size_t i = 0; i < cmd.names.size(); ++i) {
      ifs >> cmd.names[i];
    }

    int num_cmds;
    ifs >> num_cmds;
    cmd.command.resize(num_cmds);
    for (size_t i = 0; i < cmd.command.size(); ++i) {
      ifs >> cmd.command[i];
    }

    poses[pose_name] = cmd;
  }

  ifs.close();
}
void BaxterArm::SaveCalibratedPoses() {
  ofstream ofs;

  ofs.open (poses_filename_, std::ofstream::trunc);
  if (!ofs.is_open()) {
    ROS_WARN("Unable to save poses.");
  }

  ofs << poses.size() << endl;
  for (auto it = poses.begin(); it != poses.end(); ++it) {
    ofs << it->first << endl;
    baxter_core_msgs::JointCommand& cmd = it->second;
    ofs << cmd.mode << endl;

    ofs << cmd.names.size() << endl;
    for (size_t i = 0; i < cmd.names.size(); ++i) {
      ofs << cmd.names[i] << " ";
    }
    ofs << endl;

    ofs << cmd.command.size() << endl;
    for (size_t i = 0; i < cmd.command.size(); ++i) {
      ofs << cmd.command[i] << " ";
    }
    ofs << endl;
  }

  ofs.close();
}

int BaxterArm::MoveToPose(std::string pose_name, int accuracy_level) {
  auto pose = poses.find(pose_name);
  if (pose == poses.end()) {
    ROS_WARN_STREAM(name_ << " arm: Pose " << pose_name << " is not calibrated.");
    return -1;
  }

  joint_cmd_ = poses[pose_name];

  if (IsTargetPoseReached(joint_cmd_, joint_state_, accuracy_level)) {
    return 0;
  }

  joint_command_pub_.publish(joint_cmd_);

  return 1;
}

int BaxterArm::MoveToFrame(std::string frame_name,
                           tf2::Transform offset,
                           bool velocity_control) {
  geometry_msgs::PoseStamped offset_pose;
  offset_pose.header.stamp = ros::Time::now();
  offset_pose.header.frame_id = frame_name;
  offset_pose.pose.position.x = offset.getOrigin().x();
  offset_pose.pose.position.y = offset.getOrigin().y();
  offset_pose.pose.position.z = offset.getOrigin().z();
  offset_pose.pose.orientation.x = offset.getRotation().x();
  offset_pose.pose.orientation.y = offset.getRotation().y();
  offset_pose.pose.orientation.z = offset.getRotation().z();
  offset_pose.pose.orientation.w = offset.getRotation().w();

  baxter_core_msgs::SolvePositionIK ik_srv;
  ik_srv.request.pose_stamp.push_back(offset_pose);
  ik_srv.request.seed_mode = ik_srv.request.SEED_CURRENT;

  if (!ik_solver_client_.call(ik_srv)) {
    return -1;
  }

  if (!ik_srv.response.isValid[0])  {
    ROS_WARN("IK solver returned invalid solution.");
    return -1;
  }

  joint_cmd_.mode = joint_cmd_.POSITION_MODE;
  joint_cmd_.names = ik_srv.response.joints[0].name;
  joint_cmd_.command = ik_srv.response.joints[0].position;

  if (IsTargetPoseReached(joint_cmd_, joint_state_, 3)) {
    return 0;
  }

  if (velocity_control) {
    joint_cmd_.mode = joint_cmd_.VELOCITY_MODE;
    // Update the velocities to be with respect to the current joint position
    for (size_t i = 0; i < joint_cmd_.command.size(); ++i) {
      // Find the current state of hte corresponding joint
      for (size_t j = 0; j < joint_state_.name.size(); ++j) {
        if (joint_state_.name[j] == joint_cmd_.names[i]) {
          joint_cmd_.command[i] -= joint_state_.position[j];
          joint_cmd_.command[i] *= 0.5;
        }
      }
    }
  }

  joint_command_pub_.publish(joint_cmd_);

  return 1;
}

int BaxterArm::MoveTo(baxter_core_msgs::JointCommand pose, int accuracy_level) {
  joint_cmd_ = pose;

  if (IsTargetPoseReached(joint_cmd_, joint_state_, accuracy_level)) {
    return 0;
  }

  joint_command_pub_.publish(joint_cmd_);

  return 1;
}

baxter_core_msgs::JointCommand BaxterArm::CalcPose(std::string frame_name,
                                                   tf2::Transform offset) {
  bool solution_found = false;

  while (solution_found == false) {
    solution_found = true;
    geometry_msgs::PoseStamped offset_pose;
    offset_pose.header.stamp = ros::Time::now();
    offset_pose.header.frame_id = frame_name;
    offset_pose.pose.position.x = offset.getOrigin().x();
    offset_pose.pose.position.y = offset.getOrigin().y();
    offset_pose.pose.position.z = offset.getOrigin().z();
    offset_pose.pose.orientation.x = offset.getRotation().x();
    offset_pose.pose.orientation.y = offset.getRotation().y();
    offset_pose.pose.orientation.z = offset.getRotation().z();
    offset_pose.pose.orientation.w = offset.getRotation().w();
    ROS_INFO_STREAM("Pose: " << offset_pose);

    baxter_core_msgs::SolvePositionIK ik_srv;
    ik_srv.request.pose_stamp.push_back(offset_pose);
    ik_srv.request.seed_mode = ik_srv.request.SEED_CURRENT;

    if (!ik_solver_client_.call(ik_srv)) {
      ROS_WARN("IK solver not responding.");
      solution_found = false;
    }

    if (!ik_srv.response.isValid[0])  {
      ROS_WARN("IK solver returned invalid solution.");
      solution_found = false;
    }

    joint_cmd_.mode = joint_cmd_.POSITION_MODE;
    joint_cmd_.names = ik_srv.response.joints[0].name;
    joint_cmd_.command = ik_srv.response.joints[0].position;
  }
  return joint_cmd_;
}


bool BaxterArm::CircleCuffButtonState() {
  bool state = circle_cuff_button_state_;

  if (circle_cuff_button_state_) {
    debounce_circle_cuff_button_ = true;
    circle_cuff_button_state_ = false;
  }

  return state;
}

bool BaxterArm::DashCuffButtonState() {
  bool state = dash_cuff_button_state_;

  if (dash_cuff_button_state_) {
    debounce_dash_cuff_button_ = true;
    dash_cuff_button_state_ = false;
  }

  return state;
}

void BaxterArm::EndEffector(std::string cmd) {
  end_effector_cmd_.command = cmd;
  end_effector_pub_.publish(end_effector_cmd_);
  ros::Duration d(1); d.sleep();
}

bool BaxterArm::OpenCamera(int width, int height, double fps,
                           std::map<std::string, int> settings) {
  baxter_core_msgs::OpenCamera open_camera;

  stringstream camera_name;
  camera_name << name_ << "_hand_camera";
  open_camera.request.name = camera_name.str();
  open_camera.request.settings.width = width;
  open_camera.request.settings.height = height;
  open_camera.request.settings.fps = fps;


  for (auto it = settings.begin(); it != settings.end(); ++it) {
    baxter_core_msgs::CameraControl c;

    if (it->first == "exposure") {
      c.id = c.CAMERA_CONTROL_EXPOSURE;
    } else if (it->first == "gain") {
      c.id = c.CAMERA_CONTROL_GAIN;
    } else if (it->first == "white_balance_r") {
      c.id = c.CAMERA_CONTROL_WHITE_BALANCE_R;
    } else if (it->first == "white_balance_g") {
      c.id = c.CAMERA_CONTROL_WHITE_BALANCE_G;
    } else if (it->first == "white_balance_b") {
      c.id = c.CAMERA_CONTROL_WHITE_BALANCE_B;
    } else if (it->first == "window_x") {
      c.id = c.CAMERA_CONTROL_WINDOW_X;
    } else if (it->first == "window_y") {
      c.id = c.CAMERA_CONTROL_WINDOW_Y;
    } else if (it->first == "flip") {
      c.id = c.CAMERA_CONTROL_FLIP;
    } else if (it->first == "mirror") {
      c.id = c.CAMERA_CONTROL_MIRROR;
    } else if (it->first == "resolution_half") {
      c.id = c.CAMERA_CONTROL_RESOLUTION_HALF;
    }

    c.value = it->second;
    open_camera.request.settings.controls.push_back(c);
  }

  if (!open_camera_client_.call(open_camera))
    return false;

  if (open_camera.response.err != 0)
    return false;

  return true;
}

void BaxterArm::SetInnerLED(bool state) {
  stringstream output_name;
  output_name << name_ << "_inner_light";

  baxter_core_msgs::DigitalOutputCommand msg;
  msg.name = output_name.str();
  msg.value = state;

  led_pub_.publish(msg);
}

void BaxterArm::SetOuterLED(bool state) {
  stringstream output_name;
  output_name << name_ << "_outer_light";

  baxter_core_msgs::DigitalOutputCommand msg;
  msg.name = output_name.str();
  msg.value = state;

  led_pub_.publish(msg);
}

bool BaxterArm::IsTargetPoseReached(baxter_core_msgs::JointCommand cmd,
                                    sensor_msgs::JointState state,
                                    int accuracy_level) {
  double error = 0;
  for (size_t i = 0; i < cmd.command.size(); ++i) {
    // Find the current state of the corresponding joint
    for (size_t j = 0; j < state.name.size(); ++j) {
      if (state.name[j] == cmd.names[i]) {
        error += pow(cmd.command[i] - state.position[j], 2);
      }
    }
  }

  double ee_speed = 0;
  ee_speed += pow(endpoint_state_.twist.linear.x, 2);
  ee_speed += pow(endpoint_state_.twist.linear.y, 2);
  ee_speed += pow(endpoint_state_.twist.linear.z, 2);
  ee_speed += pow(endpoint_state_.twist.angular.x, 2);
  ee_speed += pow(endpoint_state_.twist.angular.y, 2);
  ee_speed += pow(endpoint_state_.twist.angular.z, 2);

  switch (accuracy_level) {
    case 1: // LOW
      return error < 0.5 && ee_speed < 0.5;
    case 2: // MID
      return error < 0.1 && ee_speed < 0.1;
    case 3:
      return error < 0.001 && ee_speed < 0.004;
    default:
      ROS_WARN("Unexpected accuracy level");
      return true;
  }
}


void BaxterArm::EndpointStateCallback(
  const baxter_core_msgs::EndpointStateConstPtr& endpoint_state) {
  endpoint_state_ = *endpoint_state;
}

void BaxterArm::CircleCuffButtonStateCallback(
  const baxter_core_msgs::DigitalIOStateConstPtr& cuff_button_state) {

  circle_cuff_button_state_ = cuff_button_state->state &&
                              !debounce_circle_cuff_button_;

  if (cuff_button_state->state != debounce_circle_cuff_button_) {
    debounce_circle_cuff_button_ = false;
  }
}

void BaxterArm::DashCuffButtonStateCallback(
  const baxter_core_msgs::DigitalIOStateConstPtr& cuff_button_state) {
  dash_cuff_button_state_ = cuff_button_state->state &&
                            !debounce_dash_cuff_button_;

  if (cuff_button_state->state != debounce_dash_cuff_button_) {
    debounce_dash_cuff_button_ = false;
  }
}

void BaxterArm::HandDistanceCallback(
  const sensor_msgs::RangeConstPtr& range_msg) {
  hand_distance_ = range_msg->range;
}
