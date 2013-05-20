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

double joy_x_,joy_y_,joy_z_,joy_yaw_;
double joy_x,joy_y,joy_z,joy_yaw;
int joy_a_,joy_b_,joy_xbox_;
int joy_a,joy_b,joy_xbox;

float drone_batt = 100.0;
int drone_state = 0; 
// state: {0 is failure, 2 is landed, 3 is flying, 4 is hovering, 6 taking off, 8 landing}

std_msgs::Empty emp_msg;
geometry_msgs::Vector3 pdes;
Eigen::VectorXf u;
geometry_msgs::Twist cmd_out;

void nav_callback(const ardrone_autonomy::Navdata& msg_in)
{
	drone_state=msg_in.state;	
	drone_batt=msg_in.batteryPercent;
}

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

void merge_new_mgs(void){
		joy_x=joy_x_;
		joy_y=joy_y_;
		joy_z=joy_z_;
		joy_yaw=joy_yaw_;
		joy_a=joy_a_;
		joy_b=joy_b_;
		joy_xbox=joy_xbox_;
}

void LQR_callback(const Eigen::VectorXf& u_in)
{
    u(0) = u_in(0);
    u(1) = u_in(1);
    u(2) = u_in(2);
    u(3) = u_in(3);
}

double K = 1.0; // P-gain for yaw.

int main(int argc, char** argv)
{
    // ROS Initialization
    ros::init(argc, argv, "Trajectory_Node");
    ros::NodeHandle node;
    ros::Rate loop_rate(50);
    
    // Set up publisher and subscribers
    ros::Subscriber u_sub;
	ros::Subscriber nav_sub;
	ros::Subscriber joy_sub;
    ros::Publisher  pdes__pub;
	ros::Publisher  pub_empty_reset;
	ros::Publisher  pub_empty_land;
	ros::Publisher  pub_empty_takeoff;
	ros::Publisher  pub_twist;

    joy_sub = node.subscribe("joy", 1, joy_callback);
    nav_sub = node.subscribe("ardrone/navdata", 1, nav_callback);
    u_sub = node.subscribe("LQR_u",1,LQR_callback);

    pdes_pub = node.advertise<geometry_msgs::Vector3>("desired_position",1);
	pub_empty_reset = node.advertise<std_msgs::Empty>("ardrone/reset", 1);
	pub_empty_takeoff = node.advertise<std_msgs::Empty>("ardrone/takeoff", 1);
	pub_empty_land = node.advertise<std_msgs::Empty>("ardrone/land", 1);
    pub_twist = node.advertise<geometry_msgs::Twist>("cmd_vel",1);

    ROS_INFO("Starting AR-Drone Controller");
 	while (ros::ok()) 
    {
        if (drone_batt < 30.0) // First check battery
	    {
		    ROS_ERROR("BATTERY IS CRITICAL LOW");
	    }
        if (joy_a) // If button A is pressed, take-off
        {
			while (drone_state == 2)
            {   
				ROS_INFO("Controller: Launching drone");
				pub_empty_takeoff.publish(emp_msg);
				ros::spinOnce();
				loop_rate.sleep();
			}
        }
		if (joy_b) // If button B is pressed, land
        {
			while (drone_state ==3 || drone_state ==4)
            {
				ROS_INFO("Controller: Landing drone");
				pub_empty_land.publish(emp_msg);
				ros::spinOnce();
				loop_rate.sleep();
			}
		}
		if (joy_xbox) // If XBox button pressed, reset
        {
			double time_start=(double)ros::Time::now().toSec();
			while (drone_state == 0 )
            {
				ROS_INFO("Controller: Resetting drone");
				pub_empty_reset.publish(emp_msg); //resets the drone
				ros::spinOnce();
				loop_rate.sleep();
				if((double)ros::Time::now().toSec()> time_start+3.0)      
                { 					
					ROS_ERROR("Controller: Time limit reached, unable reset ardrone");
					break; //exit loop
				}
			}
		}

        // NEED TO ADD ONE MORE BUTTON WHICH INITIATES THE TRAJECTORY
        // WHILE TRAJECTORY IS GOING SHOULD GO THROUGH THE FOLLOWING CODE
        while (drone_state == 3)
        {
            // Set pdes, maybe tie several world position to several joystick positions.
            pdes.x = 0;
            pdes.y = 0;
            pdes.z = 1;
            pdes_pub.publish(pdes);
            cmd_out.linear.x  = u(0);
            cmd_out.linear.y  = u(1);
            cmd_out.linear.z  = u(2);
            cmd_out.angular.z = -1*K*drone_state.rotZ;
            pub_twist.publish(cmd_out);

            ros::spinOnce();
            loop_rate.sleep();
            // One thing that could be an issue, the received control input will be for the LQR_output of a previous cycle with an old pdes. should be close enough to be equal though. given small velocities and fast-time steps.
        }
    }
}
            
    

