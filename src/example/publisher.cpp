#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include <geometry_msgs/Point.h>
#include <stdlib.h>


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "publisher");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Point>("krti15/topic_point", 1000);
  ros::Publisher pub_flightmode = n.advertise<std_msgs::Int16>("krti15/flight_mode", 1000);
  ros::Publisher pub_string = n.advertise<std_msgs::String>("krti15/string", 1000);
  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */

  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
	geometry_msgs::Point msg ;
	std_msgs::Int16 fm;
	std_msgs::String str;
	
	fm.data = 10+count;
	msg.x = count;
    msg.y = count+1;
    str.data = "Aldwin";
	chatter_pub.publish(msg);
	pub_string.publish(str);
	pub_flightmode.publish(fm);
	
    ROS_INFO_STREAM( 
    "\nX Value = " << msg.x
	<< "\nY Value = " << msg.y 
	<< "\nString = " << str.data
	<< "\nFM Value = " << fm.data ) ;
    
    
    ros::spinOnce();
    loop_rate.sleep();

    ++count;
  }


  return 0;
}
