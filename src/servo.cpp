#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Int16.h"
#include <geometry_msgs/Point.h>
#include <stdlib.h>
#include <unistd.h>

void extmodeReceiver(const std_msgs::Int16& ext_mode_recv);
int ext_mode = 0;


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "servo");
  ros::NodeHandle servo;
  ros::Publisher pub_servo = servo.advertise<std_msgs::UInt16>("krti15/servo", 10);
  ros::Subscriber ext_mode_sub = servo.subscribe("/krti15/ext_mode", 10, extmodeReceiver);

  std_msgs::UInt16 servo_angle;

  while (ros::ok()){
	
	ros::spinOnce();
	if (ext_mode == 1){
		servo_angle.data = 120;
		pub_servo.publish(servo_angle);
		usleep(1000000);
		servo_angle.data = 50;
		pub_servo.publish(servo_angle);
		usleep(1000000);
	}
	
	else if (ext_mode == 2){
		servo_angle.data = 50;
		pub_servo.publish(servo_angle);
		break;
	}
  }


  return 0;
}

void extmodeReceiver(const std_msgs::Int16& ext_mode_recv){
	
	ext_mode = ext_mode_recv.data;
}
