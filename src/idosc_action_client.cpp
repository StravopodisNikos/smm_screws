#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <smm_screws/idoscAction.h>

geometry_msgs::Vector3 desired_state_vel, desired_state_pos;

int main (int argc, char **argv)
{
  ros::init(argc, argv, "idosc_centralized_action_client");

  // 1. Create the action client
  actionlib::SimpleActionClient<smm_screws::idoscAction> ac("idosc_centralized_action_client", true);

  ROS_INFO("[ACTION CLIENT IDOSC] Waiting for action server to start.");
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("[ACTION CLIENT IDOSC] Action server started. Sending goal.");
  smm_screws::idoscActionGoal goal;
  desired_state_vel.x = 0.0f;
  desired_state_vel.y = 0.0f;
  desired_state_vel.z = 0.0f;
  desired_state_pos.x = 0.50f;
  desired_state_pos.y = 0.0f;
  desired_state_pos.z = 2.00f;
  goal.goal.desired_state_pos = desired_state_pos;
  goal.goal.desired_state_vel = desired_state_vel;
  ac.sendGoal(goal.goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(10.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("[ACTION CLIENT IDOSC] Action finished: %s",state.toString().c_str());
  }
  else
  {
    ROS_INFO("[ACTION CLIENT IDOSC] FAILED. Action did not finish before the time out.");
    ac.cancelGoal();
  }

  //exit
  return 0;
}