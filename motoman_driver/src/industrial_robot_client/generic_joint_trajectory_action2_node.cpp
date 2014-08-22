#include "motoman_driver/industrial_robot_client/joint_trajectory_action2.h"

using industrial_robot_client::joint_trajectory_action2::JointTrajectoryAction2;

int main(int argc, char** argv)
{
  // initialize node
  ros::init(argc, argv, "joint_trajectory_action2");

  JointTrajectoryAction2 action;
  action.run();

  return 0;
}

