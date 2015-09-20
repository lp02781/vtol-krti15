#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include <geometry_msgs/Point.h>
#include <stdlib.h>
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
char stateData;
void chatterCallback(const geometry_msgs::Point& msg)
{
  ROS_INFO_STREAM( 
    "\nX Value = " << msg.x*2
	<< "\nY Value = " << msg.y*2) ;
}

void chatterCallback2(const std_msgs::Int16& integer)
{
  ROS_INFO_STREAM( "\nInteger Value = " << integer.data) ;
}

void chatterCallback3(const std_msgs::String& str)
{
	stateData = str.data[1];
	if(stateData == 'A'){
		ROS_INFO_STREAM( "\nString = Oke") ;
	}
	else if(stateData == 'l'){
		ROS_INFO_STREAM( "\nString = Not Oke") ;
	}
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "subscriber");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("krti15/topic_point", 1000, chatterCallback);
  ros::Subscriber sub2 = n.subscribe("krti15/flight_mode", 1000, chatterCallback2);
  ros::Subscriber sub3 = n.subscribe("krti15/string", 1000, chatterCallback3);
  ros::spin();
  return 0;
}
