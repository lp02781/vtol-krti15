#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "mavros/OverrideRCIn.h"
#include <vector>
#include <iostream>


int main(int argc, char **argv)
{
    int rc_override[8] = { 1,2,3,4,5,6,7,8};

	ros::init(argc, argv, "rcOverridePublisher");

	ros::NodeHandle n;

	ros::Publisher pub_rc_override = n.advertise<mavros::OverrideRCIn>("mavros/rc/override", 100);
	mavros::OverrideRCIn rc_override_msg;
	
	while (ros::ok())
	{
		
		//for loop, pushing data in the size of the array
		for (int i = 0; i < 8; i++){
			//assign array a random number between 0 and 255.
			rc_override_msg.channels[i] = rc_override[i];
			rc_override[i]++;
		}
		//Publish array
		pub_rc_override.publish(rc_override_msg);
		//Let the world know
		ROS_INFO("I published something!");
		//Do this.
		ros::spinOnce();
		//Added a delay so not to spam
		sleep(2);
	}

}
