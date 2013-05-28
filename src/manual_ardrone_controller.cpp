/*
Daman Bareiss
Algorithmic Robotics Lab @ University of Utah

This node takes in joystick messages related to position and outputs the proper control input for the robot.
Derived from code from Parker Conroy.
*/


#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Vector3.h>
#include <ardrone_autonomy/Navdata.h>
#include <Eigen/Dense>
#include <std_msgs/Float32.h>


double max_speed = 1.0; //[m/s]
double max_speed_yaw= 1.0;
double des_altd= 1.0;


geometry_msgs::Vector3 pcurr;
geometry_msgs::Vector3 pdes;
geometry_msgs::Vector3 u;
double Kx = 1.0; 
double Ky = 1.0; 
double Kz = 1.0; 
double K  = 1.0;
std_msgs::Float32 yaw_dummy;
double yaw;
double joy_x_,joy_y_,joy_z_,joy_yaw_;
double joy_x,joy_y,joy_z,joy_yaw;
int joy_a_,joy_b_,joy_xbox_;
int joy_a,joy_b,joy_xbox;
double cmd_x,cmd_y,cmd_z, cmd_yaw;
int drone_state =0; 
float drone_batt =100.0;
// state: {0 is failure, 2 is landed, 3 is flying, 4 is hovering, 6 taking off, 8 landing}


std_msgs::Empty emp_msg;
geometry_msgs::Vector3 v3_msg; //[x, y,z]
sensor_msgs::Joy joy_msg_in;

	
void joy_callback(const sensor_msgs::Joy& joy_msg_in)
{
	//Take in xbox controller
	joy_x_=joy_msg_in.axes[1]; //left stick up-down
	joy_y_=joy_msg_in.axes[0]; //left stick left-right
	joy_z_=joy_msg_in.axes[4]; //right stick up-down
	joy_yaw_=joy_msg_in.axes[3]; //right stick left-right ?????????
	joy_a_=joy_msg_in.buttons[0]; //a button
	joy_b_=joy_msg_in.buttons[1]; //b button
	joy_xbox_=joy_msg_in.buttons[8]; //xbox button
}	


void nav_callback(const ardrone_autonomy::Navdata& msg_in)
{
	drone_state=msg_in.state;	
	drone_batt=msg_in.batteryPercent;
}


void mocap_callback(const std_msgs::Float32& yaw_in)
{
    //yaw_dummy = yaw_in;
    yaw = yaw_in.data;
}


void merge_new_mgs(void)
{
	joy_x=joy_x_;
	joy_y=joy_y_;
	joy_z=joy_z_;
	joy_yaw=joy_yaw_;
	joy_a=joy_a_;
	joy_b=joy_b_;
	joy_xbox=joy_xbox_;
}


void u_callback(const geometry_msgs::Vector3& u_in)
{
    u.x = u_in.x;
    u.y = u_in.y;
    u.z = u_in.z;
}


void pos_callback(const geometry_msgs::Vector3& pcurr_in)
{
    pcurr.x = pcurr_in.x;
    pcurr.y = pcurr_in.y;
    pcurr.z = pcurr_in.z;
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
    ros::Publisher pdes_pub;
	ros::Subscriber joy_sub;
	ros::Subscriber nav_sub;
    ros::Subscriber pcurr_sub;
    ros::Subscriber u_sub;
    ros::Subscriber yaw_sub;

    pcurr_sub = node.subscribe("current_position",1,pos_callback);
	joy_sub = node.subscribe("joy", 1, joy_callback);
	nav_sub = node.subscribe("ardrone/navdata", 1, nav_callback);
    u_sub = node.subscribe("LQR_u", 1, u_callback);
    yaw_sub = node.subscribe("current_yaw",1,mocap_callback);

    pdes_pub = node.advertise<geometry_msgs::Vector3>("desired_position",1);
	pub_twist=node.advertise<geometry_msgs::Twist>("joy_vel_twist", 1); 
    pub_v3 = node.advertise<geometry_msgs::Vector3>("joy_vel", 1); 
	pub_empty_reset = node.advertise<std_msgs::Empty>("ardrone/reset", 1);
	pub_empty_takeoff = node.advertise<std_msgs::Empty>("ardrone/takeoff", 1);
	pub_empty_land = node.advertise<std_msgs::Empty>("ardrone/land", 1);
	
    ROS_INFO("Starting AR-Drone Controller");
 	while (ros::ok()) {
		merge_new_mgs();
	
	if (drone_batt < 30.0)
	{
		ROS_ERROR("BATTERY IS CRITICAL LOW");
	}
	
		//	system(chmod a+rw /dev/input/js0);
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
		
        pdes.x = pcurr.x + joy_x*Kx;
        pdes.y = pcurr.y + joy_y*Ky;
        pdes.z = pcurr.z + joy_z*Kz;
        pdes_pub.publish(pdes);
		
		geometry_msgs::Twist twist_msg;
		twist_msg.linear.x= u.x;
		twist_msg.linear.y= u.y;
		twist_msg.linear.z= u.z;
        //yaw = (double) yaw_dummy;
		twist_msg.angular.z= -1.0*K*yaw;   // CANNOT DO THE * OPERATOR WITH YAW. MUST FIX.

		pub_twist.publish(twist_msg);
		
		ros::spinOnce();
		loop_rate.sleep();
		}//ros::ok
        // Could have same issue as before where the control input is a function of the previous desired position based on the spinOnce timing().
    ROS_ERROR("AR_Drone_Controller has exited");
}//main
