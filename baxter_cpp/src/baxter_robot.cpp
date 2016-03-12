/*
* @copyright: Copyright[2015]<Svetlin Penkov>
*      @date: 2015-11-14
*     @email: s.v.penkov@sms.ed.ac.uk
*      @desc: Add file description...
*/
#include "baxter_cpp/baxter_robot.hpp"

BaxterRobot::BaxterRobot(ros::NodeHandle& nh, std::string name, double rate) :
  nh_(nh),
  name_(name),
  rate_(rate),
  left_arm_(nh, "left"),
  right_arm_(nh, "right"),
  head_(nh) {

  joint_state_sub_ = nh_.subscribe<sensor_msgs::JointState>(
                       "/robot/joint_states",
                       10,
                       &BaxterRobot::JointStateCallback,
                       this);
  left_arm_.LoadCalibratedPoses();
  right_arm_.LoadCalibratedPoses();
}

void BaxterRobot::Move() {

}

void BaxterRobot::CalibrateArmPose(ArmSide side,
                                   std::string pose_name,
                                   bool overwrite) {
  BaxterArm& arm = (side == kLeft) ? left_arm_ : right_arm_;

  if (!arm.IsPoseCalibrated(pose_name) || overwrite) {
    ROS_INFO_STREAM(arm.Name() << "_arm: Waiting to calibrate pose '" <<
                    pose_name << "'");
    while (ros::ok()) {
      if (arm.DashCuffButtonState()) {
        arm.CalibratePose(pose_name);
        head_.Nod();
        break;
      }
      ros::spinOnce();
      rate_.sleep();
    }
  }
}

void BaxterRobot::WaitForButtonPush(ArmSide side, ArmButton type) {
  BaxterArm& arm = (side == kLeft) ? left_arm_ : right_arm_;
  bool state = false;
  while (ros::ok() && !state) {
    switch (type) {
      case kCircle:
        state = arm.CircleCuffButtonState();
        break;
      case kDash:
        state = arm.DashCuffButtonState();
        break;
    }
    ros::spinOnce();
    rate_.sleep();
  }
  head_.Nod();
}

void BaxterRobot::MoveToPose(ArmSide side,
                             std::string pose_name,
                             bool look,
                             int accuracy_level) {
  BaxterArm& arm = (side == kLeft) ? left_arm_ : right_arm_;
  arm.SetInnerLED(true);
  while (ros::ok() && arm.MoveToPose(pose_name, accuracy_level) == 1) {
    if (look) {
      head_.PanToFrame((side == kLeft) ? "left_gripper" : "right_gripper");
    }
    ros::spinOnce();
    rate_.sleep();
  }
  arm.SetInnerLED(false);
}

void BaxterRobot::MoveToFrame(ArmSide side,
                              std::string frame_name,
                              tf2::Transform offset,
                              bool velocity_control) {
  BaxterArm& arm = (side == kLeft) ? left_arm_ : right_arm_;
  arm.SetInnerLED(true);
  while (ros::ok() &&
         arm.MoveToFrame(frame_name, offset, velocity_control) != 0) {
    ros::spinOnce();
    rate_.sleep();
  }
  arm.SetInnerLED(false);
}

void BaxterRobot::MoveTo(ArmSide side, baxter_core_msgs::JointCommand pose,
                         bool look, int accuracy_level) {
  BaxterArm& arm = (side == kLeft) ? left_arm_ : right_arm_;
  arm.SetInnerLED(true);
  while (ros::ok() && arm.MoveTo(pose, accuracy_level) == 1) {
    if (look) {
      head_.PanToFrame((side == kLeft) ? "left_gripper" : "right_gripper");
    }
    ros::spinOnce();
    rate_.sleep();
  }
  arm.SetInnerLED(false);
}

void BaxterRobot::MoveToward(ArmSide side, tf2::Transform offset,
                             bool look, int accuracy_level) {
  BaxterArm& arm = (side == kLeft) ? left_arm_ : right_arm_;
  baxter_core_msgs::JointCommand pose;
  pose = arm.CalcPose("right_gripper_base", offset);

  MoveTo(side, pose, look, accuracy_level);
}

void BaxterRobot::JointStateCallback(
  const sensor_msgs::JointStateConstPtr& joint_state) {
  // SDK 1.2.0 publishes gripper state separately, if so skip
  if (joint_state->name.size() != 17) { return; }
  joint_state_ = *joint_state;
  left_arm_.SetJointState(joint_state_);
  right_arm_.SetJointState(joint_state_);
  head_.SetJointState(joint_state_);
}

