#include "ros/ros.h"
#include "pid/plant_msg.h"
#include "pid/controller_msg.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Float64.h"

double t_IC = 0.0;
double setpoint_x = 320;
double setpoint_y = 240;
double setpoint_z = 2;

double cv_x = 0.0;
double cv_y = 0.0;
float rel_alt = 0;
double delta_t = 0.01; // control period in seconds
double x_pid_out = 0.0;
double y_pid_out = 0.0;
void pidXReceiver(const pid::controller_msg& x_msg);
void pidYReceiver(const pid::controller_msg& y_msg);
void cvReceiver(const geometry_msgs::Point& cv_msg); 
void altReceiver(const std_msgs::Float64& alt_msg);



int main(int argc, char **argv)
{
  ROS_INFO("Starting PID Controller.");
  ros::init(argc, argv, "pid_controller");
  
  ros::NodeHandle pid_controller;

  // Declare a new message variable
  pid::plant_msg  pid_x_in;
  pid::plant_msg  pid_y_in;
  geometry_msgs::TwistStamped quad_vel;
  
  // this is the area where the quadcopter starts descending.
  int zero_area_y = 50;
  int zero_area_x = 50;
  int zero_zone_x_l, zero_zone_y_l;
  int zero_zone_x_u, zero_zone_y_u;

  // Initial conditions -- these were defined in the header file
  pid_x_in.x = cv_x;
  pid_x_in.t = t_IC;
  
  pid_y_in.x = cv_y;
  pid_y_in.t = t_IC;
  
  // Set setpoint as target on the camera frame
  pid_x_in.setpoint = setpoint_x;
  pid_y_in.setpoint = setpoint_y;

  // Publish a plant.msg
  ros::Publisher pub_pid_x_in = pid_controller.advertise<pid::plant_msg>("/krti15/pid_x_in", 1);
  ros::Publisher pub_pid_y_in = pid_controller.advertise<pid::plant_msg>("/krti15/pid_y_in", 1);
  
  ros::Publisher pub_quad_vel = pid_controller.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 1000);

  // Subscribe to "control_effort" topic to get a controller_msg.msg
  // Change the topic to something that pid-ish
  ros::Subscriber sub_pid_x_out = pid_controller.subscribe("/krti15/pid_x_out", 1, pidXReceiver );
  ros::Subscriber sub_pid_y_out = pid_controller.subscribe("/krti15/pid_y_out", 1, pidYReceiver );
  
  ros::Subscriber sub_cv_point = pid_controller.subscribe("/krti15/cv_point", 1, cvReceiver );
  ros::Subscriber sub_rel_alt = pid_controller.subscribe("/mavros/global_position/rel_alt", 1, altReceiver );
  
  ros::Rate loop_rate(1/delta_t); // Control rate in Hz


  while (ros::ok())
  {
    
    // Determine the zero zone from the zero area
    zero_zone_x_l = setpoint_x - (zero_area_x/2);
    zero_zone_x_u = setpoint_x + (zero_area_x/2);
    zero_zone_y_l = setpoint_y - (zero_area_y/2);
    zero_zone_y_u = setpoint_y + (zero_area_y/2);
    
    // determine the zero zone to descend
    if ( (cv_x > zero_zone_x_l && cv_x < zero_zone_x_u) && (cv_y > zero_zone_y_l && cv_y < zero_zone_y_u)){
		
		if ( rel_alt > setpoint_z ){
			quad_vel.twist.linear.z = -0.5; // the speed of z axis
		}
		
		else {
			quad_vel.twist.linear.z = 0;
		}
	}
    
    else {
    
    quad_vel.twist.linear.z = 0;
	
	}
    
    // pid_x_in.x need to subscribe from image_processing
    pid_x_in.x = cv_x;
    pid_x_in.t = pid_x_in.t+delta_t;
  
    // pid_y_in.x need to subscribe from image_processing
	pid_y_in.x = cv_y;
    pid_y_in.t = pid_y_in.t+delta_t;
    
    pub_pid_x_in.publish(pid_x_in);
    pub_pid_y_in.publish(pid_y_in);
    
    ros::spinOnce();
    
    // x axis in quad frame means front and back movement, which in camera frame is in y axis
    // the axis reference for pid axis is the camera axis
    // negative value means forward motion, positive value means going backwards
    y_pid_out = 0 - y_pid_out;
    
    // y axis in quad frame means side movement, which in camera frame is in x axis
    // the axis reference for pid axis is the camera axis
    // negative value means going left, positive value means going right
    x_pid_out = 0 - x_pid_out;
    
    // publish to quad movement controller topic
    quad_vel.header.stamp = ros::Time::now();
    quad_vel.header.frame_id = "1";
    
    // x axis in quadcopter frame is y axis in camera frame, vice versa.
    quad_vel.twist.linear.x = y_pid_out;
    quad_vel.twist.linear.y = x_pid_out;
	
	pub_quad_vel.publish(quad_vel);

    loop_rate.sleep();
  }

  return 0;
}

// Callback when something is published on 'control_effort'
void pidXReceiver(const pid::controller_msg& x_msg)
{

  x_pid_out = x_msg.u;
}

void pidYReceiver(const pid::controller_msg& y_msg)
{

  y_pid_out = y_msg.u;
}

void cvReceiver(const geometry_msgs::Point& cv_msg)
{

  cv_x = cv_msg.x;
  cv_y = cv_msg.y;
}

void altReceiver(const std_msgs::Float64& alt_msg)
{

  rel_alt = alt_msg.data;
  
}


