/*
Daman Bareiss
Algorithmic Robotics Lab @ University of Utah

This node generates a trajectory of goal positions and sends the necessary control input to the robot.
Derived from code from Parker Conroy.
*/

// THIS NEEDS TO BE UPDATED TO INCLUDE THE DYNAMICS MODEL OF GIVING A GOAL POSITION AND OUTPUTTING A CONTROL INPUT. CAN USE LQR ABOUT NEW DYNAMICS MODEL GIVEN A GENERATED TRAJECTORY AND SENDS IT TO THE ROBOT COMMANDS.

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Vector3.h>
#include <ardrone_autonomy/Navdata.h>
	
double max_speed = 1.0; //[m/s]
double max_speed_yaw= 1.0;
double des_altd= 1.0;

double cmd_x,cmd_y,cmd_z, cmd_yaw;
// state: {0 is failure, 2 is landed, 3 is flying, 4 is hovering, 6 taking off, 8 landing}
int drone_state =0; 
float drone_batt =100.0;

std_msgs::Empty emp_msg;
geometry_msgs::Vector3 v3_msg; //[x, y,z]
	

void nav_callback(const ardrone_autonomy::Navdata& msg_in)
{
	drone_state=msg_in.state;	
	drone_batt=msg_in.batteryPercent;
}	


int main(int argc, char** argv)
{
	ros::init(argc, argv,"ARDrone_Control");
    ros::NodeHandle node;
    ros::Rate loop_rate(50);

	ros::Publisher pub_empty_reset;
	ros::Publisher pub_empty_land;
	ros::Publisher pub_empty_takeoff;
	ros::Publisher pub_twist;
	ros::Publisher pub_v3;
	ros::Subscriber nav_sub;

	nav_sub = node.subscribe("ardrone/navdata", 1, nav_callback);

	pub_empty_reset = node.advertise<std_msgs::Empty>("ardrone/reset", 1);
	pub_empty_takeoff = node.advertise<std_msgs::Empty>("ardrone/takeoff", 1);
	pub_empty_land = node.advertise<std_msgs::Empty>("ardrone/land", 1);
	
    ROS_INFO("Starting AR-Drone Controller");
 	while (ros::ok()) {
	    if (drone_batt < 30.0)
	    {
		    ROS_ERROR("BATTERY IS CRITICAL LOW");
	    }
        // Reset the drone
        // Wait
        // Take off
        // Wait
        // Follow trajectory
        // Wait
        // Land

		//commands to change state of drone
		if (joy_a){
			while (drone_state ==2){
				ROS_INFO("Controller: Launching drone");
				pub_empty_takeoff.publish(emp_msg); //launches the drone
				ros::spinOnce();
				loop_rate.sleep();
			}//drone take off
		}	
		if (joy_b){
			while (drone_state ==3 || drone_state ==4){
				ROS_INFO("Controller: Landing drone");
				pub_empty_land.publish(emp_msg); //launches the drone
				ros::spinOnce();
				loop_rate.sleep();
			}//drone land
		}
		if (joy_xbox){
			double time_start=(double)ros::Time::now().toSec();
			while (drone_state == 0 ){
				ROS_INFO("Controller: Resetting drone");
				pub_empty_reset.publish(emp_msg); //resets the drone
				ros::spinOnce();
				loop_rate.sleep();
				if((double)ros::Time::now().toSec()> time_start+3.0){ 					
					ROS_ERROR("Controller: Time limit reached, unable reset ardrone");
					break; //exit loop
				}
			}//drone take off	
		}
		
		if (fabs(joy_x)<0.1) {joy_x =0;}
		if (fabs(joy_y)<0.1) {joy_y =0;}
		if (fabs(joy_z)<0.1) {joy_z =0;}
		if (fabs(joy_yaw)<0.1) {joy_yaw =0;}
		
		cmd_x= joy_x*max_speed;
		cmd_y= joy_y*max_speed;
		cmd_z= joy_z*max_speed;
		cmd_yaw=joy_yaw*max_speed_yaw;
		
		v3_msg.x=cmd_x;
		v3_msg.y=cmd_y;
		v3_msg.z=cmd_z;
		pub_v3.publish(v3_msg);
		
		geometry_msgs::Twist twist_msg;
		twist_msg.linear.x=cmd_x;
		twist_msg.linear.y=cmd_y;
		twist_msg.linear.z=cmd_z;
		twist_msg.angular.z=cmd_yaw;

		pub_twist.publish(twist_msg);
		

		ros::spinOnce();
		loop_rate.sleep();
		}//ros::ok
ROS_ERROR("AR_Drone_Controller has exited");
}//main
