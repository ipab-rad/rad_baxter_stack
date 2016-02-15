/*
* @copyright: Copyright[2015]<Svetlin Penkov>
*      @date: 2015-11-14
*     @email: s.v.penkov@sms.ed.ac.uk
*      @desc: Add file description...
*/
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "baxter_cpp/baxter_arm.hpp"
#include "baxter_cpp/baxter_head.hpp"

class BaxterRobot {
 public:
  enum ArmSide {kLeft, kRight};
  enum ArmButton {kCircle, kDash};

  BaxterRobot(ros::NodeHandle& nh, std::string name, double rate);

  void Move();

  BaxterArm& LeftArm() {return left_arm_; }
  BaxterArm& RightArm() {return right_arm_; }
  BaxterHead& Head() {return head_;}

  void CalibrateArmPose(ArmSide side,
                        std::string pose_name,
                        bool overwrite = false);
  void WaitForButtonPush(ArmSide side, ArmButton type);
  void MoveToPose(ArmSide side,
                  std::string pose_name,
                  bool look = false,
                  int accuracy_level = 3);
  void MoveToFrame(ArmSide side,
                   std::string frame_name,
                   tf2::Transform offset,
                   bool velocity_control = false);

 private:
  ros::NodeHandle nh_;
  std::string name_;
  ros::Rate rate_;

  BaxterArm left_arm_;
  BaxterArm right_arm_;
  BaxterHead head_;

  ros::Subscriber joint_state_sub_;
  sensor_msgs::JointState joint_state_;

  void JointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state);
};
