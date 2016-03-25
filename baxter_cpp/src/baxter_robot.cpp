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
  led_pub_ = nh_.advertise<baxter_core_msgs::DigitalOutputCommand>(
               "/robot/digital_io/command", 10);
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

BaxterRobot::Result BaxterRobot::MoveToFrame(ArmSide side,
                                             std::string frame_name,
                                             tf2::Transform offset,
                                             bool velocity_control,
                                             int accuracy) {
  BaxterArm& arm = (side == kLeft) ? left_arm_ : right_arm_;
  arm.SetInnerLED(true);
  while (ros::ok() &&
         arm.MoveToFrame(frame_name, offset, velocity_control, accuracy) != 0) {
    ros::spinOnce();
    rate_.sleep();
    // Add check to see if Frame is still found, if not then break/frameLost
  }
  arm.SetInnerLED(false);
  return success;
}

tf2::Transform BaxterRobot::CheckBestApproach(ArmSide side,
                                              std::string frame_name,
                                              tf2::Transform offset) {
  BaxterArm& arm = (side == kLeft) ? left_arm_ : right_arm_;

  double max_error = 9999999;
  double error = 0;
  double z_orientation = -M_PI;
  tf2::Transform best_approach;
  while (z_orientation <= M_PI) {
    offset.getBasis().setRPY(M_PI, 0, z_orientation);
    baxter_core_msgs::JointCommand pose;

    pose = arm.CalcPose(frame_name, offset);
    error = arm.CalcError(pose);
    ROS_DEBUG_STREAM("Angle: " << z_orientation << " Error: " << error);
    if (error < max_error) {
      best_approach = offset;
      max_error = error;
    }
    z_orientation += M_PI / 2;
  }
  return best_approach;
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
  std::string gripper_frame_ = (side == kLeft) ? "left_gripper_base" :
                               "right_gripper_base";
  pose = arm.CalcPose(gripper_frame_, offset);

  MoveTo(side, pose, look, accuracy_level);
}

BaxterRobot::Result BaxterRobot::MoveToPoseNB(ArmSide side,
                                              std::string pose_name,
                                              bool look,
                                              int accuracy_level) {
  BaxterArm& arm = (side == kLeft) ? left_arm_ : right_arm_;
  // arm.SetInnerLED(true);
  // while (ros::ok() && arm.MoveToPose(pose_name, accuracy_level) == 1) {
  int result =  arm.MoveToPose(pose_name, accuracy_level);
  if (look) {
    head_.PanToFrame((side == kLeft) ? "left_gripper" : "right_gripper");
  }
  // ros::spinOnce();
  // rate_.sleep();
  // }
  // arm.SetInnerLED(false);
  if (result == 0) {
    return BaxterRobot::success;
  } else if (result == 1) {
    return BaxterRobot::working;
  } else {
    return BaxterRobot::error;
  }
}

BaxterRobot::Result BaxterRobot::MoveToFrameNB(ArmSide side,
                                               std::string frame_name,
                                               tf2::Transform offset,
                                               bool velocity_control,
                                               int accuracy) {
  BaxterArm& arm = (side == kLeft) ? left_arm_ : right_arm_;
  // arm.SetInnerLED(true);
  // while (ros::ok() &&
  int result = arm.MoveToFrame(frame_name, offset, velocity_control, accuracy);
  // ros::spinOnce();
  // rate_.sleep();
  // Add check to see if Frame is still found, if not then break/frameLost
  // }
  // arm.SetInnerLED(false);
  if (result == 0) {
    return BaxterRobot::success;
  } else if (result == 1) {
    return BaxterRobot::working;
  } else {
    return BaxterRobot::error;
  }
}

BaxterRobot::Result BaxterRobot::MoveToNB(ArmSide side,
                                          baxter_core_msgs::JointCommand pose,
                                          bool look, int accuracy_level) {
  BaxterArm& arm = (side == kLeft) ? left_arm_ : right_arm_;
  // arm.SetInnerLED(true);
  // while (ros::ok() && arm.MoveTo(pose, accuracy_level) == 1) {
  int result = arm.MoveTo(pose, accuracy_level);
  if (look) {
    head_.PanToFrame((side == kLeft) ? "left_gripper" : "right_gripper");
  }
  // ros::spinOnce();
  // rate_.sleep();
  // }
  // arm.SetInnerLED(false);
  if (result == 0) {
    return BaxterRobot::success;
  } else if (result == 1) {
    return BaxterRobot::working;
  } else {
    return BaxterRobot::error;
  }
}

baxter_core_msgs::JointCommand BaxterRobot::CalcOffsetPose(ArmSide side,
                                                           tf2::Transform offset) {
  BaxterArm& arm = (side == kLeft) ? left_arm_ : right_arm_;
  baxter_core_msgs::JointCommand pose;
  std::string gripper_frame_ = (side == kLeft) ? "left_gripper_base" :
                               "right_gripper_base";

  return pose = arm.CalcPose(gripper_frame_, offset);

  // return MoveToNB(side, pose, look, accuracy_level);
}

void BaxterRobot::SetLED(std::string led_name, bool state) {
  baxter_core_msgs::DigitalOutputCommand msg;
  msg.name = led_name;
  msg.value = state;

  led_pub_.publish(msg);
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
