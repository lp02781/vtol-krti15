#include "ros/ros.h"
#include "pid/plant_msg.h"
#include "pid/controller_msg.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float64.h"

double t_IC = 0.0;
double setpoint_x = 313;
double setpoint_y = 339;
double setpoint_z = 3;
int fm_data = 0;

double cv_x = 0.0;
double cv_y = 0.0;
int x_pos_flag = 0;
int y_pos_flag = 0;
float pos_x = 0;
float pos_y = 0;
float x_halogen = 0;
float y_halogen = 0;
float rel_alt = 0;
double delta_t = 0.01; // control period in seconds
double x_pid_out = 0.0;
double y_pid_out = 0.0;
void pidXReceiver(const pid::controller_msg& x_msg);
void pidYReceiver(const pid::controller_msg& y_msg);
void cvReceiver(const geometry_msgs::Point& cv_msg); 
void posReceiver(const geometry_msgs::PoseStamped& local_msg);
void fmReceiver(const std_msgs::Int16& fm_recv);



int main(int argc, char **argv)
{
  ROS_INFO("Starting PID Controller.");
  ros::init(argc, argv, "pid_controller");
  
  ros::NodeHandle pid_controller;

  // Declare a new message variable
  pid::plant_msg  pid_x_in;
  pid::plant_msg  pid_y_in;
  geometry_msgs::TwistStamped quad_vel;
  geometry_msgs::PoseStamped quad_pos;
  
  // this is the area where the quadcopter starts descending.
  int zero_area_y = 3;
  int zero_area_x = 3;
  double zero_zone_x_l, zero_zone_y_l;
  double zero_zone_x_u, zero_zone_y_u;

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
  ros::Publisher pub_quad_pos = pid_controller.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1000);

  // Subscribe to "control_effort" topic to get a controller_msg.msg
  // Change the topic to something that pid-ish
  ros::Subscriber sub_pid_x_out = pid_controller.subscribe("/krti15/pid_x_out", 10, pidXReceiver );
  ros::Subscriber sub_pid_y_out = pid_controller.subscribe("/krti15/pid_y_out", 10, pidYReceiver );
  
  ros::Subscriber sub_cv_point = pid_controller.subscribe("/krti15/cv_point", 10, cvReceiver );
  ros::Subscriber sub_pos = pid_controller.subscribe("mavros/local_position/local", 100, posReceiver );
  ros::Subscriber sub_fm = pid_controller.subscribe("/krti15/flight_mode", 1000, fmReceiver);
  
  ros::Rate loop_rate(1/delta_t); // Control rate in Hz


      // Determine the zero zone from the zero area
  zero_zone_x_l = setpoint_x - (zero_area_x/2);
  zero_zone_x_u = setpoint_x + (zero_area_x/2);
  zero_zone_y_l = setpoint_y - (zero_area_y/2);
  zero_zone_y_u = setpoint_y + (zero_area_y/2);
  
  while (ros::ok())
  {
    
    // determine the zero zone to descend
    if ( cv_x > zero_zone_x_l && cv_x < zero_zone_x_u && x_pos_flag == 0 && fm_data == 1){
		
		x_halogen = pos_x;
		x_pos_flag = 1;
		ROS_INFO_STREAM( "X Locked at " << x_halogen) ;
	}
    
    if ( cv_y > zero_zone_y_l && cv_y < zero_zone_y_u && y_pos_flag == 0 && fm_data == 1){
    
		y_halogen = pos_y;
		y_pos_flag = 1;
		ROS_INFO_STREAM( "Y Locked at " << y_halogen) ;
	}
	
	if ( x_pos_flag == 1 && y_pos_flag == 1 ){
    
			quad_pos.header.stamp = ros::Time::now();
			quad_pos.header.frame_id = "1";
			quad_pos.pose.position.x = x_halogen;	// x pos gps from camera
			quad_pos.pose.position.y = y_halogen; 	// y pos gps from camera
			quad_pos.pose.position.z = setpoint_z; 	// descend to 3 meter
			ROS_INFO_STREAM( "Publishing Position") ;
			pub_quad_pos.publish(quad_pos);
			
			while(rel_alt > setpoint_z+1 && ros::ok()){
				ros::spinOnce();
			}
			
			ROS_INFO_STREAM( "PID Ground Started!") ;
			
			while(fm_data == 1 && ros::ok()){	
				
				ros::spinOnce;
				pid_x_in.x = cv_x;
				pid_x_in.t = pid_x_in.t+delta_t;
			  
				pid_y_in.x = cv_y;
				pid_y_in.t = pid_y_in.t+delta_t;
				
				pub_pid_x_in.publish(pid_x_in);
				pub_pid_y_in.publish(pid_y_in);
				
				ros::spinOnce();
				
				y_pid_out = 0 - y_pid_out;
				x_pid_out = x_pid_out;
				
				quad_vel.header.stamp = ros::Time::now();
				quad_vel.header.frame_id = "1";
				quad_vel.twist.linear.x = x_pid_out;
				quad_vel.twist.linear.y = y_pid_out;
				
				pub_quad_vel.publish(quad_vel);
				loop_rate.sleep();
				
			}
			
			x_pos_flag = 0;
			y_pos_flag = 0;
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
    
    y_pid_out = 0 - y_pid_out;
    
    x_pid_out = x_pid_out;
    
    // publish to quad movement controller topic
    quad_vel.header.stamp = ros::Time::now();
    quad_vel.header.frame_id = "1";
    
    // x axis in quadcopter frame is y axis in camera frame, vice versa.
    quad_vel.twist.linear.x = x_pid_out;
    quad_vel.twist.linear.y = y_pid_out;
	
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

void posReceiver(const geometry_msgs::PoseStamped& local_msg)
{
	
	pos_x = local_msg.pose.position.x;
	pos_y = local_msg.pose.position.y;
	rel_alt = local_msg.pose.position.z;
	
}

void fmReceiver(const std_msgs::Int16& fm_recv){
	
	fm_data = fm_recv.data;

}

