// %Tag(FULLTEXT)%
#include <ros/ros.h>
#include <generic_control_toolbox/kdl_manager.hpp>


int main (int argc, char ** argv)
{
  ros::init(argc, argv, "load_chain");

  // Initialize a KDL manager instance with the 'torso' link configured as its
  // base frame.
  generic_control_toolbox::KDLManager manager("torso");

  // Load a kinematic chain which goes from the base frame ('torso') to a
  // user-configured end-effector ('left_hand')
  // %Tag(Init)%
  manager.initializeArm("left_hand");
  KDL::Frame pose;
  sensor_msgs::JointState state;
  // %EndTag(Init)%

  // The KDL manager parses the robot state information from a JointState message,
  // which robots using ROS will publish on the network.
  // %Tag(FK)%
  state = *ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");

  // Get the end-effector pose as a KDL::Frame.
  manager.getEefPose("left_hand", state, pose);
  // %EndTag(FK)%
  ROS_INFO_STREAM("The left hand end-effector position is " << pose.p.x() << ", " << pose.p.y() << ", " << pose.p.z());

  // Configure a gripping point on the robot
  // %Tag(Gripping)%
  manager.setGrippingPoint("left_hand", "left_gripper");

  // Get the pose of the gripping point
  manager.getGrippingPoint("left_hand", state, pose);
  // %EndTag(Gripping)%
  ROS_INFO_STREAM("The left hand gripping point position is " << pose.p.x() << ", " << pose.p.y() << ", " << pose.p.z());
}
// %EndTag(FULLTEXT)%
