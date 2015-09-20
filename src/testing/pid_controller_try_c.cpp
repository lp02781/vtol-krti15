#include "ros/ros.h"
#include "pid/plant_msg.h"
#include "pid/controller_msg.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Int16.h"
#include "mavros/State.h"
#include "std_msgs/Float64.h"
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

void pidXReceiver(const pid::controller_msg& x_msg);
void pidYReceiver(const pid::controller_msg& y_msg);
void cvReceiver(const geometry_msgs::Point& cv_msg); 
void posReceiver(const geometry_msgs::PoseStamped& local_msg);
void fmReceiver(const std_msgs::Int16& fm_recv);
void stateReceiver(const mavros::State& state_recv);
void altReceiver(const std_msgs::Float64& alt_msg);
void pidConvert();

int fm_data = 0;
double cv_x = 0.0;
double cv_y = 0.0;
float pos_x = 0;
float pos_y = 0;
float pos_z = 0;
float rel_alt = 0;
double x_pid_out = 0.0;
double y_pid_out = 0.0;
char state_char = 'A';

int main(int argc, char **argv){
	
	double t_IC = 0.0;
	double delta_t = 0.01;
	int zero_area_y = 3;
	int zero_area_x = 3;
	double zero_zone_x_l, zero_zone_y_l;
	double zero_zone_x_u, zero_zone_y_u;
	int x_pos_flag = 0;
	int y_pos_flag = 0;
	float x_halogen = 0;
	float y_halogen = 0;
	double setpoint_x = 313;
	double setpoint_y = 339;
	double setpoint_z = 3;
	
	ros::init(argc, argv, "pid_controller");
	  
	ros::NodeHandle pid_controller;

	ros::Publisher pub_pid_x_in = pid_controller.advertise<pid::plant_msg>("/krti15/pid_x_in", 1);
	ros::Publisher pub_pid_y_in = pid_controller.advertise<pid::plant_msg>("/krti15/pid_y_in", 1);
	ros::Publisher pub_quad_vel = pid_controller.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 1000);
	ros::Publisher pub_quad_pos = pid_controller.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1000);
	ros::Subscriber sub_pid_x_out = pid_controller.subscribe("/krti15/pid_x_out", 10, pidXReceiver );
	ros::Subscriber sub_pid_y_out = pid_controller.subscribe("/krti15/pid_y_out", 10, pidYReceiver );  
	ros::Subscriber sub_cv_point = pid_controller.subscribe("/krti15/cv_point", 10, cvReceiver );
	ros::Subscriber sub_pos = pid_controller.subscribe("mavros/local_position/local", 100, posReceiver );
	ros::Subscriber sub_fm = pid_controller.subscribe("/krti15/flight_mode", 1000, fmReceiver);
	ros::Subscriber sub_state = pid_controller.subscribe("mavros/state", 1000, stateReceiver);
	ros::Subscriber sub_rel_alt = pid_controller.subscribe("mavros/global_position/rel_alt", 1, altReceiver );
	ros::Rate loop_rate(1/delta_t); 

	pid::plant_msg  pid_x_in;
	pid::plant_msg  pid_y_in;
	geometry_msgs::TwistStamped quad_vel;
	geometry_msgs::PoseStamped quad_pos;
	  
	pid_x_in.x = cv_x;
	pid_x_in.t = t_IC;

	pid_y_in.x = cv_y;
	pid_y_in.t = t_IC;

	pid_x_in.setpoint = setpoint_x;
	pid_y_in.setpoint = setpoint_y;

	zero_zone_x_l = setpoint_x - (zero_area_x/2);
	zero_zone_x_u = setpoint_x + (zero_area_x/2);
	zero_zone_y_l = setpoint_y - (zero_area_y/2);
	zero_zone_y_u = setpoint_y + (zero_area_y/2);
  
	while ( rel_alt < 4 && ros::ok()){
	  ros::spinOnce();
	}
  
	ROS_INFO("Starting PID Controller.");
	while (ros::ok()){
    
		ros::spinOnce;		
		if ( cv_x > zero_zone_x_l && cv_x < zero_zone_x_u && x_pos_flag == 0 && state_char == 'G'){
			
			x_halogen = pos_x;
			x_pos_flag = 1;
			ROS_INFO_STREAM( "X Locked at " << x_halogen) ;
		}
		
		if ( cv_y > zero_zone_y_l && cv_y < zero_zone_y_u && y_pos_flag == 0 && state_char == 'G'){
		
			y_halogen = pos_y;
			y_pos_flag = 1;
			ROS_INFO_STREAM( "Y Locked at " << y_halogen) ;
		}
		
		if ( x_pos_flag == 1 && y_pos_flag == 1 && state_char == 'G'){
		
			quad_pos.header.stamp = ros::Time::now();
			quad_pos.header.frame_id = "1";
			quad_pos.pose.position.x = x_halogen;	
			quad_pos.pose.position.y = y_halogen; 	
			quad_pos.pose.position.z = setpoint_z; 	
			pub_quad_pos.publish(quad_pos);
			ROS_INFO_STREAM( "Publishing Position") ;
				
			while(rel_alt > setpoint_z && ros::ok()){
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
				
				pidConvert();
				
				quad_vel.header.stamp = ros::Time::now();
				quad_vel.header.frame_id = "1";
				quad_vel.twist.linear.x = x_pid_out;
				quad_vel.twist.linear.y = y_pid_out;
				
				pub_quad_vel.publish(quad_vel);
				loop_rate.sleep();
			}
				
			ROS_INFO_STREAM( "Job Complete") ;
			x_pos_flag = 0;
			y_pos_flag = 0;
		}		
		
		pid_x_in.x = cv_x;
		pid_x_in.t = pid_x_in.t+delta_t;
	  
		pid_y_in.x = cv_y;
		pid_y_in.t = pid_y_in.t+delta_t;
		
		pub_pid_x_in.publish(pid_x_in);
		pub_pid_y_in.publish(pid_y_in);
		
		ros::spinOnce();
		
		pidConvert();		
		
		quad_vel.header.stamp = ros::Time::now();
		quad_vel.header.frame_id = "1";
		quad_vel.twist.linear.x = x_pid_out;
		quad_vel.twist.linear.y = y_pid_out;
		
		pub_quad_vel.publish(quad_vel);

		loop_rate.sleep();
	}
	
	return 0;
}

void pidXReceiver(const pid::controller_msg& x_msg){

	x_pid_out = x_msg.u;
}

void pidYReceiver(const pid::controller_msg& y_msg){

    y_pid_out = y_msg.u;
}

void cvReceiver(const geometry_msgs::Point& cv_msg){

	cv_x = cv_msg.x;
    cv_y = cv_msg.y;
}

void posReceiver(const geometry_msgs::PoseStamped& local_msg){
	
	pos_x = local_msg.pose.position.x;
	pos_y = local_msg.pose.position.y;
	pos_z = local_msg.pose.position.z;
}

void fmReceiver(const std_msgs::Int16& fm_recv){
	
	fm_data = fm_recv.data;
}

void stateReceiver(const mavros::State& state_recv){
	
	state_char = state_recv.mode[0];
}

void altReceiver(const std_msgs::Float64& alt_msg){

	rel_alt = alt_msg.data;
}

void pidConvert(){

	y_pid_out = 0 - y_pid_out;
	x_pid_out = x_pid_out;
}
