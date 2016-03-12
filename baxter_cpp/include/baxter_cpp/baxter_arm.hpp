/*
* @copyright: Copyright[2015]<Svetlin Penkov>
*      @date: 2015-11-14
*     @email: s.v.penkov@sms.ed.ac.uk
*      @desc: Add file description...
*/

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>

#include <sensor_msgs/Range.h>
#include <sensor_msgs/JointState.h>

#include <baxter_core_msgs/DigitalIOState.h>
#include <baxter_core_msgs/DigitalOutputCommand.h>
#include <baxter_core_msgs/EndpointState.h>
#include <baxter_core_msgs/EndEffectorCommand.h>
#include <baxter_core_msgs/JointCommand.h>

class BaxterArm {
 public:
  BaxterArm(ros::NodeHandle& nh, std::string name);

  void SetJointState(const sensor_msgs::JointState& joint_state);

  bool CalibratePose(std::string pose_name);
  bool IsPoseCalibrated(std::string pose_name);
  void LoadCalibratedPoses();
  void SaveCalibratedPoses();

  int MoveToPose(std::string pose_name, int accuracy_level);
  int MoveToFrame(std::string frame_name, tf2::Transform offset,
                  bool velocity_control);
  int MoveTo(baxter_core_msgs::JointCommand pose, int accuracy_level);
  baxter_core_msgs::JointCommand CalcPose(std::string frame_name,
                                          tf2::Transform offset);

  bool CircleCuffButtonState();
  bool DashCuffButtonState();
  double HandDistance() { return hand_distance_; }

  void EndEffector(std::string cmd);

  bool OpenCamera(int width, int height, double fps,
                  std::map<std::string, int> settings);

  void SetInnerLED(bool state);
  void SetOuterLED(bool state);

  std::string Name() { return name_; }


 private:
  ros::NodeHandle nh_;
  std::string name_;

  // Joint State
  sensor_msgs::JointState joint_state_;

  ros::Publisher joint_command_pub_;
  baxter_core_msgs::JointCommand joint_cmd_;

  // Calibrated Poses
  std::map<std::string, baxter_core_msgs::JointCommand> poses;
  // File where the calibrated poses to be loaded from
  std::string poses_filename_;

  // Servoing
  ros::ServiceClient ik_solver_client_;

  // Endpoint State
  ros::Subscriber endpoint_state_sub_;
  baxter_core_msgs::EndpointState endpoint_state_;

  // Circle Cufflink Button
  ros::Subscriber circle_cuff_button_sub_;
  bool circle_cuff_button_state_;
  bool debounce_circle_cuff_button_;

  // Dash Cufflink Button
  ros::Subscriber dash_cuff_button_sub_;
  bool dash_cuff_button_state_;
  bool debounce_dash_cuff_button_;

  // Distance
  ros::Subscriber hand_distance_sub_;
  double hand_distance_;

  // End Effector
  ros::Publisher end_effector_pub_;
  baxter_core_msgs::EndEffectorCommand end_effector_cmd_;

  // Camera
  ros::ServiceClient open_camera_client_;


  // TODO (svepe): Add error thresholding based on the control mode
  // FK, position control, velocity control
  bool IsTargetPoseReached(baxter_core_msgs::JointCommand cmd,
                           sensor_msgs::JointState state,
                           int accuracy_level);
  // LEDs
  ros::Publisher led_pub_;

  // Subscribers
  void EndpointStateCallback(
    const baxter_core_msgs::EndpointStateConstPtr& endpoint_state);

  void CircleCuffButtonStateCallback(
    const baxter_core_msgs::DigitalIOStateConstPtr& cuff_button_state);

  void DashCuffButtonStateCallback(
    const baxter_core_msgs::DigitalIOStateConstPtr& cuff_button_state);

  void HandDistanceCallback(const sensor_msgs::RangeConstPtr& range_msg);
};
