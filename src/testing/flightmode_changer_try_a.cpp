#include <iostream>
#include "ros/ros.h"
#include "mavros/SetMode.h"
#include "mavros/State.h"
#include "mavros/OverrideRCIn.h"
#include "mavros/RCIn.h"
#include "std_msgs/Int16.h"
#include <unistd.h>

void flightmodeReceiver(const std_msgs::Int16& fm);
void rcinReceiver(const mavros::RCIn& rc_in_data);
int flightmode_data = 0;
int rc_in_channel_7 = 0;

int main(int argc, char **argv)
{	
	int last_flightmode_data = 0;
	int rc_taken = 0;
	//int channel_7_off = 987;
	int channel_7_mid = 1500;
	//int channel_7_on = 2010;

	
	ros::init(argc, argv, "flightmode_changer");
	ros::NodeHandle n;
	
	ros::Subscriber fm_sub = n.subscribe("/krti15/flight_mode", 100, flightmodeReceiver);
	ros::Subscriber rc_in_sub = n.subscribe("mavros/rc/in", 100, rcinReceiver);
	
	
	ROS_INFO("Starting Flight Mode Changer.");
	while (ros::ok()){
		
		ros::spinOnce();
		if(rc_taken == 1){
			
			if (flightmode_data == 0){
				last_flightmode_data = 1;
			}
			
			else if (flightmode_data == 1){
				last_flightmode_data = 0;
			}
			
		}
		
		if(rc_in_channel_7 > channel_7_mid){
			if(flightmode_data == 0 && last_flightmode_data == 1){
				
				system("rosrun mavros mavsys mode -c LOITER");
				ROS_INFO("LOITER");
				
				usleep(2000000);
			
				system("rosrun mavros mavsys mode -c AUTO");
				ROS_INFO("AUTO");
				
				last_flightmode_data = flightmode_data;
				
			}
			
			else if(flightmode_data == 1 && last_flightmode_data == 0){
				
				system("rosrun mavros mavsys mode -c LOITER");
				ROS_INFO("LOITER");
				
				usleep(2000000);
			
				system("rosrun mavros mavsys mode -c GUIDED");
				ROS_INFO("GUIDED");
				
				last_flightmode_data = flightmode_data;

			}
			rc_taken = 0;
		}
		
		else if ( rc_in_channel_7 < channel_7_mid ) {
			
			ROS_INFO("RC Take Over");
			rc_taken = 1;

		}
		
				
	}

  return 0;
}


void flightmodeReceiver(const std_msgs::Int16& fm){
	
	flightmode_data = fm.data;
}

void rcinReceiver(const mavros::RCIn& rc_in_data){
	
	rc_in_channel_7 = rc_in_data.channels[6];
}
