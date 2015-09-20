#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "mavros/OverrideRCIn.h"
#include <vector>
#include <iostream>

int rc_override_received[8];
void arrayCallback(const mavros::OverrideRCIn& rc_override_data);

int main(int argc, char **argv)
{
	int j;
	ros::init(argc, argv, "rcOverrideSubscriber");

	ros::NodeHandle n;	

	ros::Subscriber rc_override_sub = n.subscribe("mavros/rc/override", 100, arrayCallback);


	while (ros::ok()){
		
	ros::spinOnce();
	for(j = 0; j < 8; j++){
		
		printf("%d, ", rc_override_received[j]);
	}
	
	
	printf("\n");
	}
	
	return 0;
}

void arrayCallback(const mavros::OverrideRCIn& rc_override_data)
{

	int i;
	for (i = 0; i < 8; i++){
		
			rc_override_received[i] = rc_override_data.channels[i];

		}
	
}
