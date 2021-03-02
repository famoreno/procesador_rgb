#include "ros/ros.h"
#include "std_msgs/String.h"
#include "body_tracker_msgs/Skeleton.h"


void chatterCallback(const body_tracker_msgs::Skeleton::ConstPtr&  data)
{
  ROS_INFO("join_position_head_x is: %f", data->joint_position_head.x);
}


int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "listener"); //Inicializa el nodo
  ros::NodeHandle n;


  ros::Subscriber sub = n.subscribe("body_tracker/skeleton", 1000, chatterCallback); //Se suscribe al topic body_tracker/skeleton

  ros::spin();


  return 0;
}
