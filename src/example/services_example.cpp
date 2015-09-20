#include "ros/ros.h"
#include "mavros/SetMode.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "set_mode_test");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<mavros::SetMode>("/mavros/set_mode");
  mavros::SetMode flight;
  
  flight.request.base_mode = 0;				//Set to 0 to use custom_mode
  flight.request.custom_mode = "GUIDED";	//Set to '' to use base_mode
  bool success = client.call(flight);

	// Check for success and use the response .
	if(success){
		ROS_INFO_STREAM( "Flight Mode changed to "<< flight.request.custom_mode ) ;
	} 
	else {
		ROS_ERROR_STREAM( "Failed to changed." ) ;
	}

  return 0;
}
