#include "ros/ros.h"
#include "pid/plant_msg.h"
#include "pid/controller_msg.h"
#include "pid/pid_const_msg.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Int16.h"
#include "mavros/State.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int16.h"
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
void pidConvert();
int stableMotion();
void imuReceiver(const sensor_msgs::Imu& imu_msg);

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
	
	float KpH = 0.004 ;
	float KiH = 0 ;
	float KdH = 0 ;
	
	float KpL = 0.0005;
	float KiL = 0 ;
	float KdL = 0 ;

	#define QUAD GARUDA
	#define PIDG NO
	#define AGPS YES
	
	#if QUAD == ATHENA
	double setpoint_x = 325.5848;
	double setpoint_y = 282.8661;
	double setpoint_z = 5;
	int zero_area_y = 6; //pixel
	int zero_area_x = 6; //pixel
	#elif QUAD == GARUDA
	double setpoint_x = 358.6148;
	double setpoint_y = 275.4661;
	double setpoint_z = 3.5;
	int zero_area_y = 10; //pixel
	int zero_area_x = 10; //pixel
	#endif
	
	int false_area_x = 20; //meter
	int false_area_y = 20; //meter
	int halogen_area_x = 260;
	int halogen_area_y = 180;
	int time_out_second = 5;
	
	// ##################################################
	
	double t_IC = 0.0;
	double delta_t = 0.01;
	double zero_zone_x_l, zero_zone_y_l;
	double zero_zone_x_u, zero_zone_y_u;
	double false_detec_zone_y_u, false_detec_zone_x_u;
	double false_detec_zone_y_l, false_detec_zone_x_l;
	double halogen_zone_x_l,halogen_zone_x_u;
	double halogen_zone_y_l,halogen_zone_y_u;
	int x_pos_flag = 0;
	int y_pos_flag = 0;
	float x_halogen = 0;
	float y_halogen = 0;
	float x_halogen_home = 0;
	float y_halogen_home = 0;
	int home_flag = 0;
	float const_cv_local = 0;
	int local_pos_cv_x = 0;
	int local_pos_cv_y = 0;
	
	
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
  
	halogen_zone_x_l = 320 - (halogen_area_x/2);
	halogen_zone_x_u = 320 + (halogen_area_x/2);
	halogen_zone_y_l = 240 - (halogen_area_y/2);
	halogen_zone_y_u = 240 + (halogen_area_y/2);
  
  
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
		
		if ( fm_data == 1 && ros::ok() &&
				cv_x > halogen_zone_x_l && 
				cv_x < halogen_zone_x_u && 
				cv_y > halogen_zone_y_l && 
				cv_y < halogen_zone_y_u && home_flag == 0){
					
			x_halogen_home = pos_x;
			y_halogen_home = pos_y;
			ROS_INFO_STREAM( "Forced to Guided") ;
			ext_mode.data = 4;
			pub_ext_mode.publish(ext_mode);
			while(state_char != 'G' && ros::ok()){
				ros::spinOnce();
			}
			
			quad_pos.header.stamp = ros::Time::now();
			quad_pos.header.frame_id = "1";
			quad_pos.pose.position.x = x_halogen_home;	
			quad_pos.pose.position.y = y_halogen_home; 	
			quad_pos.pose.position.z = 6; 	
			pub_quad_pos.publish(quad_pos);
			ROS_INFO_STREAM( "Stay Here") ;
			ROS_INFO_STREAM( "Position Set") ;
			ROS_INFO_STREAM( "X Homed at " << x_halogen_home) ;
			ROS_INFO_STREAM( "Y Homed at " << y_halogen_home) ;
			usleep(4000000);
			while (!stableMotion() && ros::ok()){
				ros::spinOnce();
			}
			
			/*
			const_cv_local = 0; // set to zero | stay in first detection
			local_pos_cv_x = 0 - ((setpoint_x-cv_x) * const_cv_local); 
			local_pos_cv_y = 0 - ((setpoint_y-cv_y) * const_cv_local);
				
			x_halogen_home = pos_x + local_pos_cv_x;
			y_halogen_home = pos_y + local_pos_cv_y;
			quad_pos.header.stamp = ros::Time::now();
			quad_pos.header.frame_id = "1";
			quad_pos.pose.position.x = x_halogen_home;	
			quad_pos.pose.position.y = y_halogen_home; 	
			quad_pos.pose.position.z = 6; 	
			pub_quad_pos.publish(quad_pos);
			ROS_INFO_STREAM( "Position Set") ;
			ROS_INFO_STREAM( "X Homed at " << x_halogen_home) ;
			ROS_INFO_STREAM( "Y Homed at " << y_halogen_home) ;
			usleep(3000000);
			while (!stableMotion() && ros::ok()){
				ros::spinOnce();
			
			
			}
			*/
			
			home_flag = 1;
			
		}
		
		if (  fm_data == 0 && home_flag == 1 ){
			
			ROS_INFO_STREAM( "Position Lost") ;	
			quad_pos.header.stamp = ros::Time::now();
			quad_pos.header.frame_id = "1";
			quad_pos.pose.position.x = x_halogen_home;	
			quad_pos.pose.position.y = y_halogen_home; 	
			quad_pos.pose.position.z = 6; 	
			pub_quad_pos.publish(quad_pos);
			
			ROS_INFO_STREAM( "Position Set") ;
			ROS_INFO_STREAM( "X Homed at " << x_halogen_home) ;
			ROS_INFO_STREAM( "Y Homed at " << y_halogen_home) ;
			usleep(3000000);
			while (!stableMotion() && ros::ok()){
				ros::spinOnce();
			}
			
		}
		
		if ( cv_x > zero_zone_x_l && cv_x < zero_zone_x_u && x_pos_flag == 0 && state_char == 'G' && stableMotion()){
			
			x_halogen = pos_x;
			x_pos_flag = 1;
			
			false_detec_zone_x_l = x_halogen - (false_area_x/2);
			false_detec_zone_x_u = x_halogen + (false_area_x/2);			
			ROS_INFO_STREAM( "X Locked at " << x_halogen) ;
		}
		
		if ( cv_y > zero_zone_y_l && cv_y < zero_zone_y_u && y_pos_flag == 0 && state_char == 'G' && stableMotion()){
		
			y_halogen = pos_y;
			y_pos_flag = 1;
			
			false_detec_zone_y_l = y_halogen - (false_area_y/2);
			false_detec_zone_y_u = y_halogen + (false_area_y/2);
			ROS_INFO_STREAM( "Y Locked at " << y_halogen) ;
		}
		
		
		if ( (pos_x < false_detec_zone_x_l || pos_x > false_detec_zone_x_u) && x_pos_flag == 1 ){
			
			ROS_INFO_STREAM( "X Position Reseted! ") ;
			x_pos_flag = 0;			
		}

		if ( (pos_y < false_detec_zone_y_l || pos_y > false_detec_zone_y_u) && y_pos_flag == 1 ){
			
			ROS_INFO_STREAM( "Y Position Reseted! ") ;
			y_pos_flag = 0;			
		}
		
		
		if ( x_pos_flag == 1 && y_pos_flag == 1 && state_char == 'G'){
		
			quad_pos.header.stamp = ros::Time::now();
			quad_pos.header.frame_id = "1";
			quad_pos.pose.position.x = x_halogen;	
			quad_pos.pose.position.y = y_halogen; 	
			quad_pos.pose.position.z = setpoint_z; 	
			pub_quad_pos.publish(quad_pos);
			ROS_INFO_STREAM( "Publishing Position") ;
				
			while(rel_alt > setpoint_z+0.3 && ros::ok()){

				ros::spinOnce();
			}
			
			while (!stableMotion()){
				ros::spinOnce();
			}
			ROS_INFO_STREAM( "PID Ground Started!") ;
			
			pid_const.p = KpL;
			pid_const.i = KiL;
			pid_const.d = KdL;
			pub_pid_const.publish(pid_const);
			
			#if QUAD == ATHENA
			ext_mode.data = 1;	// not okay - will set flightmode_changer to normal
			pub_ext_mode.publish(ext_mode);
			ROS_INFO("Servo Start!");
			
			timer_qc.setPeriod(ros::Duration(time_out_second));
			timer_qc.start();
			#elif QUAD == GARUDA			
			timer_qc.setPeriod(ros::Duration(time_out_second*2));
			timer_qc.start();
			ROS_INFO("Timer Start! 10 Secs");
			#endif

			while(fm_data == 1 && ros::ok()){	
					
				ros::spinOnce();
				
				if ( time_out == 1){
					time_out = 3;
					break;
				}
					
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
				
					#if AGPS == YES
					//usleep(800000);	  //@ const P = 3
					usleep(300000); //@ const P = 4
					
					ros::spinOnce();
					quad_pos.header.stamp = ros::Time::now();
					quad_pos.header.frame_id = "1";
					quad_pos.pose.position.x = pos_x;	
					quad_pos.pose.position.y = pos_y; 	
					quad_pos.pose.position.z = rel_alt; 	
					pub_quad_pos.publish(quad_pos);
					
					usleep(700000);
					
					while (!stableMotion() && ros::ok()){
						ros::spinOnce();
					
					}
					
					#endif
							
				loop_rate.sleep();
				
				#endif
				
				
			}
				
				
				
			#if QUAD == GARUDA
			ext_mode.data = 3;
			pub_ext_mode.publish(ext_mode);
			usleep(6000000); // change too auto so that it can continue mission
			
			if (time_out == 3){
				mission_count++;
				time_out = 0;
			}
			else{
				mission_count++;
				timer_qc.setPeriod(ros::Duration(1));
				time_out = -1;
				timer_qc.start();
			}
			
			#elif QUAD == ATHENA
			time_out = 1;
			#endif
			
			ROS_INFO_STREAM( "Job Complete!") ;
			x_pos_flag = 0;
			y_pos_flag = 0;
			home_flag = 0;
			
			pid_const.p = KpH;
			pid_const.i = KiH;
			pid_const.d = KdH;
			pub_pid_const.publish(pid_const);
						
			// set to normal
			ext_mode.data = 1;
			pub_ext_mode.publish(ext_mode);
			
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
		
		#if AGPS == YES
		//usleep(800000);	  //@ const P = 3
		usleep(300000); //@ const P = 4
		
		ros::spinOnce();
		quad_pos.header.stamp = ros::Time::now();
		quad_pos.header.frame_id = "1";
		quad_pos.pose.position.x = pos_x;	
		quad_pos.pose.position.y = pos_y; 	
		quad_pos.pose.position.z = rel_alt; 	
		pub_quad_pos.publish(quad_pos);
		
		usleep(700000);
		
		while (!stableMotion() && ros::ok()){
			ros::spinOnce();
		
		}
		
		#endif
		
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
	y_pid_out = 0 - y_pid_out;
	x_pid_out = x_pid_out;
	#endif
}

void quadTimer(const ros::TimerEvent&){
  
  time_out ++;
}


void imuReceiver(const sensor_msgs::Imu& imu_msg)
{

  accel_x = imu_msg.linear_acceleration.x;
  accel_y = imu_msg.linear_acceleration.y;
  accel_z = imu_msg.linear_acceleration.z;
  
}

int stableMotion(){
	
	if (accel_z > 9.6 && accel_z < 10){
		return 1;
	}
	else{
		return 0;
	}
	
}

