/*
Daman Bareiss
Algorithmic Robotics Lab @ University of Utah

This node takes in the updated control input from invdynamics and outputs the message to the robot, including a global yaw controller to keep yaw as close as possible to zero.
*/
// state: {0 is failure, 2 is landed, 3 is flying, 4 is hovering, 6 taking off, 8 landing}

// Includes
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Vector3.h>
#include <ardrone_autonomy/Navdata.h>
#include <Eigen/Dense>
#include <std_msgs/Float32.h>

// General Variabl Initialization
double max_speed = 1.0; //[m/s]
double max_speed_yaw= 1.0;
double des_altd= 1.0;
geometry_msgs::Vector3 u;
double K  = 1.0;
double yaw;
int joy_a_,joy_b_,joy_xbox_;
int joy_a,joy_b,joy_xbox;
double cmd_x,cmd_y,cmd_z, cmd_yaw;
int drone_state =0; 
float drone_batt =100.0;
std_msgs::Empty emp_msg;
geometry_msgs::Vector3 v3_msg; //[x, y,z]

// Read in controller buttons	
void joy_callback(const sensor_msgs::Joy& joy_msg_in)
{
	joy_a_=joy_msg_in.buttons[0]; //a button
	joy_b_=joy_msg_in.buttons[1]; //b button
	joy_xbox_=joy_msg_in.buttons[8]; //xbox button
}	

// Read in sensor data from robot
void nav_callback(const ardrone_autonomy::Navdata& msg_in)
{
	drone_state=msg_in.state;	
	drone_batt=msg_in.batteryPercent;
}

// Read in yaw from mocap
void yaw_callback(const std_msgs::Float32& yaw_in)
{
    yaw = yaw_in.data;
}

// Combine updated controller buttons
void merge_new_mgs(void)
{
	joy_a=joy_a_;
	joy_b=joy_b_;
	joy_xbox=joy_xbox_;
}

// Read in new, safe u from projection node
void u_callback(const geometry_msgs::Vector3& u_in)
{
    u.x = u_in.x;
    u.y = u_in.y;
    u.z = u_in.z;
}

// Main Loop
int main(int argc, char** argv)
{
    // ROS initialization
	ros::init(argc, argv,"ARDrone_Control");
    ros::NodeHandle node;
    ros::Rate loop_rate(50);

    // ROS publishers
	ros::Publisher pub_empty_reset;
	pub_empty_reset = node.advertise<std_msgs::Empty>("ardrone/reset", 1);
    ros::Publisher pub_empty_land;
	pub_empty_land = node.advertise<std_msgs::Empty>("ardrone/land", 1);
	ros::Publisher pub_empty_takeoff;
	pub_empty_takeoff = node.advertise<std_msgs::Empty>("ardrone/takeoff", 1);
	ros::Publisher pub_twist;
	pub_twist=node.advertise<geometry_msgs::Twist>("joy_vel_twist", 1); 
	
    // ROS subscribers
	ros::Subscriber joy_sub;
	joy_sub = node.subscribe("joy", 1, joy_callback);
    ros::Subscriber nav_sub;
	nav_sub = node.subscribe("ardrone/navdata", 1, nav_callback);
    ros::Subscriber u_sub;
    u_sub = node.subscribe("new_u", 1, u_callback);
    ros::Subscriber yaw_sub;
    yaw_sub = node.subscribe("current_yaw",1,yaw_callback);

    // Starting main loop
    ROS_INFO("Starting AR-Drone Controller");
 	while (ros::ok()) 
    {
		merge_new_mgs();
        // Print message if battery low
	    if (drone_batt < 30.0)
	    {
		    ROS_ERROR("BATTERY IS CRITICAL LOW");
	    }
        // launch drone if button A is pressed
	    if (joy_a)
        {
		    while (drone_state ==2)
            {
			    ROS_INFO("Controller: Launching drone");
			    pub_empty_takeoff.publish(emp_msg);
			    ros::spinOnce();
			    loop_rate.sleep();
		    }
	    }
        // land drone if button B is pressed	
	    if (joy_b)
        {
		    while (drone_state ==3 || drone_state ==4)
            {
			    ROS_INFO("Controller: Landing drone");
			    pub_empty_land.publish(emp_msg);
			    ros::spinOnce();
			    loop_rate.sleep();
		    }//drone land
	    }
        // reset drone if xbox button is pressed
	    if (joy_xbox)
        {
		    double time_start=(double)ros::Time::now().toSec();
		    while (drone_state == 0 )
            {
			    ROS_INFO("Controller: Resetting drone");
			    pub_empty_reset.publish(emp_msg);
			    ros::spinOnce();
			    loop_rate.sleep();
			    if((double)ros::Time::now().toSec()> time_start+3.0){ 					
				    ROS_ERROR("Controller: Time limit reached, unable reset ardrone");
				    break;
			    }
		    }
	    }

        // Initialize, set, and send control output		
	    geometry_msgs::Twist twist_msg;
	    twist_msg.linear.x= u.x;
	    twist_msg.linear.y= u.y;
	    twist_msg.linear.z= u.z;
	    //twist_msg.angular.z= -1.0*K*yaw;
		twist_msg.angular.z = 0.0;
	    pub_twist.publish(twist_msg);
	
	    ros::spinOnce();
	    loop_rate.sleep();
    }//ros::ok
    ROS_ERROR("AR_Drone_Controller has exited");
}//main
