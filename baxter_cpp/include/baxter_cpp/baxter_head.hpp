/*
* @copyright: Copyright[2015]<Svetlin Penkov>
*      @date: 2015-11-15
*     @email: s.v.penkov@sms.ed.ac.uk
*      @desc: Add file description...
*/
#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_listener.h>

class BaxterHead {
 public:
  BaxterHead(ros::NodeHandle nh);

  void Nod();

  void Pan(float target, float speed_ratio = 0.5);

  bool PanToFrame(std::string frame_name, float speed_ratio = 0.5);

  void SetJointState(const sensor_msgs::JointState& joint_state);

 private:
  ros::NodeHandle nh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Publisher head_nod_pub_;
  ros::Publisher head_pan_pub_;

  sensor_msgs::JointState joint_state_;
};
