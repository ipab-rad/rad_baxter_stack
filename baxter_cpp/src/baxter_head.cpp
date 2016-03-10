/*
* @copyright: Copyright[2015]<Svetlin Penkov>
*      @date: 2015-11-15
*     @email: s.v.penkov@sms.ed.ac.uk
*      @desc: Add file description...
*/
#include "baxter_cpp/baxter_head.hpp"

#include <std_msgs/Bool.h>
#include <baxter_core_msgs/HeadPanCommand.h>

BaxterHead::BaxterHead(ros::NodeHandle nh) :
  nh_(nh) ,
  tf_listener_(tf_buffer_) {
  head_pan_pub_ = nh_.advertise<baxter_core_msgs::HeadPanCommand>(
                    "/robot/head/command_head_pan", 10);
  head_nod_pub_  = nh_.advertise<std_msgs::Bool>(
                     "/robot/head/command_head_nod", 10);

}

void BaxterHead::Nod() {
  std_msgs::Bool msg;
  msg.data = true;
  head_nod_pub_. publish(msg);
}

void BaxterHead::Pan(float target, float speed_ratio) {
  baxter_core_msgs::HeadPanCommand cmd;
  cmd.target = target;
  cmd.speed_ratio = speed_ratio;
  head_pan_pub_.publish(cmd);
}

bool BaxterHead::PanToFrame(std::string frame_name, float speed_ratio) {
  geometry_msgs::TransformStamped t;
  try {
    t = tf_buffer_.lookupTransform("base", frame_name, ros::Time());
  } catch (tf2::TransformException& ex) {
    return false;
  }

  float target = atan2(t.transform.translation.y, t.transform.translation.x);

  Pan(target, speed_ratio);

  return true;
}

void BaxterHead::SetJointState(const sensor_msgs::JointState& joint_state) {
  joint_state_.header = joint_state.header;
  joint_state_.name.clear();
  joint_state_.position.clear();
  joint_state_.velocity.clear();
  joint_state_.effort.clear();

  for (size_t i = 0; i < joint_state.name.size(); ++i) {
    if (joint_state.name[i].find("head") == 0) {
      joint_state_.name.push_back(joint_state.name[i]);
      joint_state_.position.push_back(joint_state.position[i]);
      joint_state_.velocity.push_back(joint_state.velocity[i]);
      joint_state_.effort.push_back(joint_state.effort[i]);
    }
  }
}
