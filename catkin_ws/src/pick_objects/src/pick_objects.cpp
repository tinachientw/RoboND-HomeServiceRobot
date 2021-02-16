#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <cstring>
using namespace std;

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


struct position {
  float pos_x;
  float pos_y;
  float ori_w;
};

void send_move_msg(MoveBaseClient &ac, const string &msg_string, const position &pos) 
{
  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = pos.pos_x; 
  goal.target_pose.pose.position.y = pos.pos_y; 
  goal.target_pose.pose.orientation.w = pos.ori_w;

  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal (%s)", msg_string.c_str());
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, reached its goal (%s)", msg_string.c_str());
  else
    ROS_INFO("The base failed to reached its goal (%s) for some reason", msg_string.c_str());

}


int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  
  // pick up
  position pos;
  pos.pos_x = 4.0;
  pos.pos_y = 0.5;
  pos.ori_w = 1.0;
  
  string msg_string;
  msg_string = "pick up";
  send_move_msg(ac, msg_string, pos);


  sleep(1); // simulate the picking up


  // drop off
  pos.pos_x = -6.5;
  pos.pos_y = -2.0;
  pos.ori_w = 1.0;
  msg_string = "drop off";
  send_move_msg(ac, msg_string, pos);


  return 0;

}
