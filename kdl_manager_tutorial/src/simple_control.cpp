// %Tag(FULLTEXT)%
#include <ros/ros.h>
#include <generic_control_toolbox/kdl_manager.hpp>

sensor_msgs::JointState state;

void stateCb(const sensor_msgs::JointState::ConstPtr &msg)
{
  state = *msg;
}

int main (int argc, char ** argv)
{
  ros::init(argc, argv, "simple_control");
  ros::NodeHandle nh;

  // Define the communications with the robot
  // %Tag(ROSCOMM)%
  ros::Subscriber state_sub = nh.subscribe("/joint_states", 1, &stateCb);
  ros::Publisher command_pub = nh.advertise<sensor_msgs::JointState>("/joint_command", 1);
  // %EndTag(ROSCOMM)%

  // Initialize a KDL manager on the robot's left arm
  generic_control_toolbox::KDLManager manager("torso");
  manager.initializeArm("left_hand");

  // %Tag(VARS)%
  KDL::Frame pose;
  KDL::Vector desired_position, error;
  KDL::JntArray q_dot(7);
  KDL::Twist command_vel = KDL::Twist::Zero();
  sensor_msgs::JointState command;

  desired_position.x(0.7);
  desired_position.y(0.2);
  desired_position.z(0.32);
  // %EndTag(VARS)%

  while (ros::ok())
  {
    // Get the end-effector pose as a KDL::Frame.
    // %Tag(LOOP)%
    if (manager.getEefPose("left_hand", state, pose))
    {
      // %Tag(ERROR)%
      error = desired_position - pose.p;
      command_vel.vel = 0.5*error;
      // %EndTag(ERROR)%
      ROS_INFO_THROTTLE(0.5,
                        "Position error: (%.2f, %.2f, %.2f)",
                        error.x(), error.y(), error.z());

      // Get the IK solution for the desired control command.
      // %Tag(IK)%
      manager.getVelIK("left_hand", state, command_vel, q_dot);
      command = state;
      manager.getJointState("left_hand", q_dot.data, command);
      command_pub.publish(command);
      // %EndTag(IK)%
    }
    // %EndTag(LOOP)%

    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  return 0;
}
// %EndTag(FULLTEXT)%
