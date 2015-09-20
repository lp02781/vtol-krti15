#include "ros/ros.h"
#include "pid/plant_msg.h"
#include "std_msgs/Int16.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float64.h"
#include "mavros/State.h"
#include <string>
#include <iostream>
#include <stdlib.h>
#include <unistd.h>


using namespace std;

void fmReceiver(const std_msgs::Int16& fm_recv);
void pidXin(const pid::plant_msg& x_recv);
void pidYin(const pid::plant_msg& y_recv);
void velReceiver(const geometry_msgs::TwistStamped& vel_recv);
void stateReceiver(const mavros::State& state_recv);
void posReceiver(const geometry_msgs::PoseStamped& local_msg);
void altReceiver(const std_msgs::Float64& alt_msg);
void imuReceiver(const sensor_msgs::Imu& imu_msg);


int fm_data = 0;
float pos_x = 0;
float pos_y = 0;
float pos_z = 0;
float accel_x = 0;
float accel_y = 0;
float accel_z = 0;
float cv_x = 0;
float cv_y = 0;
float sp_x = 0;
float sp_y = 0;
float vel_x = 0;
float vel_y = 0;
float vel_z = 0;
float rel_alt = 0;
string state_mode;
string forward_move;
string side_move;


int main(int argc, char **argv)
{

	ros::init(argc, argv, "status");
	ros::NodeHandle status;
	ros::Subscriber fm = status.subscribe("/krti15/flight_mode", 1000, fmReceiver);
	ros::Subscriber pidx = status.subscribe("/krti15/pid_x_in", 1000, pidXin);
	ros::Subscriber pidy = status.subscribe("/krti15/pid_y_in", 1000, pidYin);
	ros::Subscriber vel = status.subscribe("mavros/setpoint_velocity/cmd_vel", 1000, velReceiver);
	ros::Subscriber state = status.subscribe("mavros/state", 1000, stateReceiver);
	ros::Subscriber sub_pos = status.subscribe("mavros/local_position/local", 100, posReceiver );
	ros::Subscriber sub_rel_alt = status.subscribe("/mavros/global_position/rel_alt", 1, altReceiver );
	ros::Subscriber sub_imu = status.subscribe("/mavros/imu/data", 10, imuReceiver );
	
	while (ros::ok()){
		
		ros::spinOnce();
		// for forward movement, negative value means going forward
		if (vel_y < 0 ){
			forward_move = "Forward";
		}
		
		else {
			forward_move = "Backward";
		}
		
		if (vel_x < 0 ){
			side_move = "Right";
		}
		
		else {
			side_move = "Left";
		}
		
		cout << "== X and Y in Camera Frame Axis ==\n" << endl;	
		cout << "(Y) Forward Point = " << cv_y << endl;
		cout << "(X) Side Point = " << cv_x << endl;	
		
		cout << endl;
			
		cout << "(Y) Forward SetPoint = " << sp_y << endl;
		cout << "(X) Side SetPoint = " << sp_x << endl;	
			
		cout << "(Y) Local Point = " << pos_y << endl;
		cout << "(X) Local Point = " << pos_x << endl;	
		cout << "(Z) Local Point = " << pos_z << endl;	
			
		cout << "(Y) Accel Point = " << accel_y << endl;
		cout << "(X) Accel Point = " << accel_x << endl;	
		cout << "(Z) Accel Point = " << accel_z << endl;	
			
		cout << endl;	
		
		// x is foward in quad movement
		// y is side in quad movement
		cout << "Forward Velocity = " << vel_y << endl;	
		cout << "Side Velocity = " << vel_x << endl;	
		cout << "Altitude = " << rel_alt << endl;
		
		cout << endl;
		
		cout << "Forward Movement = " << forward_move << endl;	
		cout << "Side Movement = " << side_move << endl;	
		
		cout << endl;
			
		cout << "Halogen Detection = " << fm_data << endl;	
		cout << "Flight Mode = " << state_mode << endl;	
		
		usleep(90000);
		system("clear");
		

  }
  
  return 0;
}

void fmReceiver(const std_msgs::Int16& fm_recv){
	
	fm_data = fm_recv.data;

}
	
void pidXin(const pid::plant_msg& x_recv){
	
	cv_x = x_recv.x;
	sp_x = x_recv.setpoint;
	

}
	
void pidYin(const pid::plant_msg& y_recv){
	
	cv_y = y_recv.x;
	sp_y = y_recv.setpoint;
	
}
	
void velReceiver(const geometry_msgs::TwistStamped& vel_recv){
	
	vel_x = vel_recv.twist.linear.x;
	vel_y = vel_recv.twist.linear.y;
	vel_z = vel_recv.twist.linear.z;

}
	
void stateReceiver(const mavros::State& state_recv){
	
	state_mode = state_recv.mode;
}

void posReceiver(const geometry_msgs::PoseStamped& local_msg)
{
	
	pos_x = local_msg.pose.position.x;
	pos_y = local_msg.pose.position.y;
	pos_z = local_msg.pose.position.z;
	
}

void altReceiver(const std_msgs::Float64& alt_msg)
{

  rel_alt = alt_msg.data;
  
}

void imuReceiver(const sensor_msgs::Imu& imu_msg)
{

  accel_x = imu_msg.linear_acceleration.x;
  accel_y = imu_msg.linear_acceleration.y;
  accel_z = imu_msg.linear_acceleration.z;
  
}
