#include "ros/ros.h"


int timer2_flag = 0;

void callback1(const ros::TimerEvent&)
{
	ROS_INFO("Callback 1 triggered");
	timer2_flag = 1;
}

void callback2(const ros::TimerEvent&)
{
  ROS_INFO("Callback 2 triggered");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  /**
   * Timers allow you to get a callback at a specified rate.  Here we create
   * two timers at different rates as a demonstration.
   */
  ros::Timer timer1 = n.createTimer(ros::Duration(5.0), callback1);
  ros::Timer timer2 = n.createTimer(ros::Duration(3.0), callback2,true);
  while (ros::ok()){
	
	if ( timer2_flag == 1){
		timer2.setPeriod(ros::Duration(3.0));
		timer2.start();
		timer2_flag = 0;
	}
    ros::spinOnce();
  }
  return 0;
}
