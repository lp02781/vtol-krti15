#include "ros/ros.h"
#include "pid/plant_msg.h"
#include "pid/controller_msg.h"
#include "pid/pid_const_msg.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Int16.h"
#include "mavros/State.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int16.h"
#include "sensor_msgs/Imu.h"
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
void quadTimer(const ros::TimerEvent&);
void imuReceiver(const sensor_msgs::Imu& imu_msg);
void pidConvert();
int stableMotion();


int mission_count = 0;
int time_out = -1;
int fm_data = 0;
double cv_x = 0.0;
double cv_y = 0.0;
float pos_x = 0;
float pos_y = 0;
float pos_z = 0;
float accel_x = 0;
float accel_y = 0;
float accel_z = 0;
float rel_alt = 0;
double x_pid_out = 0.0;
double y_pid_out = 0.0;
char state_char = 'A';
#define ATHENA 1
#define GARUDA 2
#define YES 1
#define NO 0


int main(int argc, char **argv){
	
	// ##################################################
	// Value that needs to be changed when debugging
	
	float KpH = 0.0015 ;
	float KiH = 0 ;
	float KdH = 0 ;
	
	float KpL = 0.0005;
	float KiL = 0 ;
	float KdL = 0 ;

	#define QUAD GARUDA
	#define PIDG YES
	double setpoint_x = 358.6148;
	double setpoint_y = 275.4661;
	double setpoint_z = 4;
	int zero_area_y = 20;
	int zero_area_x = 20;
	int false_area_x = 320;
	int false_area_y = 240;
	int time_out_second = 5;
	
	// ##################################################
	
	int stay_here = 1;
	float const_cv_local = 0;
	int local_pos_cv_x = 0;
	int local_pos_cv_y = 0;
	int local_pos_flag = 0;
	double t_IC = 0.0;
	double delta_t = 0.01;
	double zero_zone_x_l, zero_zone_y_l;
	double zero_zone_x_u, zero_zone_y_u;
	double false_detec_zone_y_u, false_detec_zone_x_u;
	double false_detec_zone_y_l, false_detec_zone_x_l;
	int x_pos_flag = 0;
	int y_pos_flag = 0;
	float x_halogen = 0;
	float y_halogen = 0;
	
	ros::init(argc, argv, "pid_controller");
	  
	ros::NodeHandle pid_controller;
	
	ros::Timer timer_qc = pid_controller.createTimer(ros::Duration(1), quadTimer,true);

	ros::Publisher pub_pid_x_in = pid_controller.advertise<pid::plant_msg>("/krti15/pid_x_in", 1);
	ros::Publisher pub_pid_y_in = pid_controller.advertise<pid::plant_msg>("/krti15/pid_y_in", 1);
	ros::Publisher pub_pid_const = pid_controller.advertise<pid::pid_const_msg>("/krti15/pid_const", 1,true);
	ros::Publisher pub_quad_vel = pid_controller.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 1000);
	ros::Publisher pub_quad_pos = pid_controller.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1000);
	ros::Publisher pub_ext_mode = pid_controller.advertise<std_msgs::Int16>("krti15/ext_mode", 10,true);
	
	ros::Subscriber sub_pid_x_out = pid_controller.subscribe("/krti15/pid_x_out", 10, pidXReceiver );
	ros::Subscriber sub_pid_y_out = pid_controller.subscribe("/krti15/pid_y_out", 10, pidYReceiver );  
	ros::Subscriber sub_cv_point = pid_controller.subscribe("/krti15/cv_point", 10, cvReceiver );
	ros::Subscriber sub_pos = pid_controller.subscribe("mavros/local_position/local", 100, posReceiver );
	ros::Subscriber sub_fm = pid_controller.subscribe("/krti15/flight_mode", 10, fmReceiver);
	ros::Subscriber sub_state = pid_controller.subscribe("mavros/state", 10, stateReceiver);
	ros::Subscriber sub_rel_alt = pid_controller.subscribe("mavros/global_position/rel_alt", 1, altReceiver );
	ros::Subscriber sub_imu = pid_controller.subscribe("/mavros/imu/data", 10, imuReceiver );
	ros::Rate loop_rate(1/delta_t); 

	pid::plant_msg  pid_x_in;
	pid::plant_msg  pid_y_in;
	pid::pid_const_msg pid_const;
	geometry_msgs::TwistStamped quad_vel;
	geometry_msgs::PoseStamped quad_pos;
	std_msgs::Int16 ext_mode;
	  
	pid_const.p = KpH;
	pid_const.i = KiH;
	pid_const.d = KdH;
	pub_pid_const.publish(pid_const);
	
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
	
	false_detec_zone_x_l = 320 - (false_area_x/2);
	false_detec_zone_x_u = 320 + (false_area_x/2);
	false_detec_zone_y_l = 240 - (false_area_y/2);
	false_detec_zone_y_u = 240 + (false_area_y/2);
  
	#if QUAD == ATHENA
	ROS_INFO("Athena Quad! Go! Fight! Win!");
	#elif QUAD == GARUDA
	ROS_INFO("Garuda Quad! Go! Fight! Win!");
	#endif

	
	while ( rel_alt < 4 && ros::ok()){
	  ros::spinOnce();
	}
  
	ROS_INFO("Starting PID Controller.");
	while (ros::ok()){
    
		ros::spinOnce();		
		
		
		while ( fm_data == 1 && ros::ok() &&
				cv_x > false_detec_zone_x_l && 
				cv_x < false_detec_zone_x_u && 
				cv_y > false_detec_zone_y_l && 
				cv_y < false_detec_zone_y_u && state_char == 'G'){
			
			ros::spinOnce();
			if ( stay_here == 1){
				
				quad_pos.header.stamp = ros::Time::now();
				quad_pos.header.frame_id = "1";
				quad_pos.pose.position.x = pos_x;	
				quad_pos.pose.position.y = pos_y; 	
				quad_pos.pose.position.z = rel_alt; 	
				pub_quad_pos.publish(quad_pos);
				ROS_INFO_STREAM( "Stay Here") ;
				stay_here = 0;
				local_pos_flag = 0;
				
			}
			
			if ( stableMotion() && local_pos_flag == 0){
				
				const_cv_local = 0.019003268;
				local_pos_cv_x = 0 - ((setpoint_x-cv_x) * const_cv_local);
				local_pos_cv_y = 0 - ((setpoint_y-cv_y) * const_cv_local);
				
				
				x_halogen = pos_x + local_pos_cv_x;
				y_halogen = pos_y + local_pos_cv_y;
				quad_pos.header.stamp = ros::Time::now();
				quad_pos.header.frame_id = "1";
				quad_pos.pose.position.x = x_halogen;	
				quad_pos.pose.position.y = y_halogen; 	
				quad_pos.pose.position.z = rel_alt; 	
				pub_quad_pos.publish(quad_pos);
				ROS_INFO_STREAM( "Position Set") ;
				ROS_INFO_STREAM( "X Locked at " << x_halogen) ;
				ROS_INFO_STREAM( "Y Locked at " << y_halogen) ;
				usleep(2000000);
				while (!stableMotion()){
					ros::spinOnce();
				}
				
				ROS_INFO_STREAM( "Position Set Exited!") ;
	
				local_pos_flag = 1;
			}
			
			if (local_pos_flag == 1 && 
				cv_x < zero_zone_x_l && 
				cv_x > zero_zone_x_u && 
				cv_y < zero_zone_y_l && 
				cv_y > zero_zone_y_u && stableMotion()){
				ROS_INFO_STREAM( "Position Reset") ;
				local_pos_flag = 0;
			}
			
			if (cv_x > zero_zone_x_l && cv_x < zero_zone_x_u && cv_y > zero_zone_y_l && cv_y < zero_zone_y_u && stableMotion()){
				
				quad_pos.header.stamp = ros::Time::now();
				quad_pos.header.frame_id = "1";
				quad_pos.pose.position.z = setpoint_z; 	
				pub_quad_pos.publish(quad_pos);
				ROS_INFO_STREAM( "Publishing Position") ;
					
				while(rel_alt > setpoint_z+0.3 && ros::ok()){
				
					ros::spinOnce();
				}
				
				usleep(2000000);
				ROS_INFO_STREAM( "PID Ground Started!") ;
				
				pid_const.p = KpL;
				pid_const.i = KiL;
				pid_const.d = KdL;
				pub_pid_const.publish(pid_const);
				
				#if QUAD == ATHENA
				ext_mode.data = 1;
				pub_ext_mode.publish(ext_mode);
				ROS_INFO("Servo Start!");
				
				timer_qc.setPeriod(ros::Duration(time_out_second));
				timer_qc.start();
				#endif

				while(fm_data == 1 && ros::ok()){	
						
					ros::spinOnce();
					
					#if QUAD == ATHENA
					if ( time_out == 1){
						break;
					}
					#endif
					
					#if PIDG == YES
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
					#endif
				}
					
				#if QUAD == ATHENA
				time_out = 1;
				#endif
				
				ROS_INFO_STREAM( "Job Complete!") ;
				x_pos_flag = 0;
				y_pos_flag = 0;
				
				#if QUAD == GARUDA
				usleep(4000000);
				mission_count++;
				#endif
				
				pid_const.p = KpH;
				pid_const.i = KiH;
				pid_const.d = KdH;
				pub_pid_const.publish(pid_const);
				stay_here = 1;
				local_pos_flag = 0;
			}
	
			loop_rate.sleep();
		}		
		
		#if QUAD == ATHENA
		// If time out mission complete
		if (time_out == 1){
			ROS_INFO("Timed Out!");
			ext_mode.data = 2;
			pub_ext_mode.publish(ext_mode);
			break;
		}
		// If time out mission complete
		#elif QUAD == GARUDA
		if (mission_count == 3){
			ROS_INFO("Mission Done!");
			ext_mode.data = 2;
			pub_ext_mode.publish(ext_mode);
			break;
		}
		#endif
		
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
	
	#if QUAD == GARUDA
	y_pid_out = 0 - y_pid_out;
	x_pid_out = x_pid_out;

	#elif QUAD == ATHENA
	y_pid_out = y_pid_out;
	x_pid_out = 0 - x_pid_out;
	#endif
}

void quadTimer(const ros::TimerEvent&){
  
  time_out += 1;
}


void imuReceiver(const sensor_msgs::Imu& imu_msg)
{

  accel_x = imu_msg.linear_acceleration.x;
  accel_y = imu_msg.linear_acceleration.y;
  accel_z = imu_msg.linear_acceleration.z;
  
}

int stableMotion(){
	
	if (accel_z > 9 && accel_z < 10){
		return 1;
	}
	else{
		return 0;
	}
}
