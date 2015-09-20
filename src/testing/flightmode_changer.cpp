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
	bool success;
	int last_flightmode_data = 0;
	int rc_taken = 0;
	//int channel_7_off = 987;
	int channel_7_mid = 1500;
	//int channel_7_on = 2010;

	
	ros::init(argc, argv, "flightmode_changer");
	ros::NodeHandle n;
	
	ros::Subscriber fm_sub = n.subscribe("/krti15/flight_mode", 100, flightmodeReceiver);
	ros::Subscriber rc_in_sub = n.subscribe("mavros/rc/in", 100, rcinReceiver);
	ros::Publisher pub_rc_override = n.advertise<mavros::OverrideRCIn>("mavros/rc/override", 100);
	
	// Call service to change flight mode
	ros::ServiceClient client = n.serviceClient<mavros::SetMode>("/mavros/set_mode");
	mavros::SetMode flight;
	
	
	// Call service to change flight mode
    mavros::OverrideRCIn rc_override_msg;
    rc_override_msg.channels[0] = 0;
    rc_override_msg.channels[1] = 0;
    rc_override_msg.channels[2] = 0;
    rc_override_msg.channels[3] = 0;
    rc_override_msg.channels[4] = 0;
    rc_override_msg.channels[5] = 0;
    rc_override_msg.channels[6] = 0;
    rc_override_msg.channels[7] = 0;
    
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
				
				flight.request.base_mode = 0;				//Set to 0 to use custom_mode
				flight.request.custom_mode = "LOITER";		//Set to '' to use base_mode
				success = client.call(flight);

				// Check for success and print out info.
				if(success){
					ROS_INFO_STREAM( "Flight Mode changed to "<< flight.request.custom_mode ) ;
				} 
				else {
					ROS_ERROR_STREAM( "Failed to changed." ) ;
				}
				
				usleep(2000000);
				
				flight.request.base_mode = 0;				//Set to 0 to use custom_mode
				flight.request.custom_mode = "AUTO";		//Set to '' to use base_mode
				success = client.call(flight);

				// Check for success and print out info.
				if(success){
					ROS_INFO_STREAM( "Flight Mode changed to "<< flight.request.custom_mode ) ;
				} 
				else {
					ROS_ERROR_STREAM( "Failed to changed." ) ;
				}
				
				last_flightmode_data = flightmode_data;
				
			}
			
			else if(flightmode_data == 1 && last_flightmode_data == 0){
				
				flight.request.base_mode = 0;				//Set to 0 to use custom_mode
				flight.request.custom_mode = "LOITER";		//Set to '' to use base_mode
				success = client.call(flight);

				// Check for success and print out info.
				if(success){
					ROS_INFO_STREAM( "Flight Mode changed to "<< flight.request.custom_mode ) ;
				} 
				else {
					ROS_ERROR_STREAM( "Failed to changed." ) ;
				}
				
				usleep(2000000);
				
				flight.request.base_mode = 0;				//Set to 0 to use custom_mode
				flight.request.custom_mode = "GUIDED";		//Set to '' to use base_mode
				success = client.call(flight);

				// Check for success and print out info.
				if(success){
					ROS_INFO_STREAM( "Flight Mode changed to "<< flight.request.custom_mode ) ;
				} 
				else {
					ROS_ERROR_STREAM( "Failed to changed." ) ;
				}
				
				last_flightmode_data = flightmode_data;

			}
			rc_taken = 0;
		}
		
		else if ( rc_in_channel_7 < channel_7_mid ) {
			
			rc_override_msg.channels[4] = 0;			// CHAN_RELEASE, Channel 5
			pub_rc_override.publish(rc_override_msg);
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
