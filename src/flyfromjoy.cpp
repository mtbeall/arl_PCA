/* 

	Daman Bareiss
	Algorithmic Robotics Lab
	
	This node taks in joystick commands and sends them to the ardrone_atuonomy as new_u for testing
*/

// Includes
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Joy.h>

// Variable Init
double joy_x_,joy_y_,joy_z_;
double joy_x,joy_y,joy_z;
sensor_msgs::Joy joy_msg_in;
geometry_msgs::Vector3 u_out;

// Read joystick postions
void joy_callback(const sensor_msgs::Joy& joy_msg_in)
{
	//Take in xbox controller
	joy_x_=joy_msg_in.axes[1]; //left stick up-down
	joy_y_=joy_msg_in.axes[0]; //left stick left-right
	joy_z_=joy_msg_in.axes[4]; //right stick up-down
}

// Combine joystick variables
void merge_new_msgs(void)
{
	joy_x=joy_x_;
	joy_y=joy_y_;
	joy_z=joy_z_;
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"flyfromjoy");
	ros::NodeHandle node;
	ros::Rate loop_rate(50);

	ros::Publisher u_pub;
	u_pub = node.advertise<geometry_msgs::Vector3>("new_u",1);
	
	ros::Subscriber joy_sub;
	joy_sub = node.subscribe("joy",1,joy_callback);

	double dead_zone = 0.1;
	while(ros::ok())
	{
		merge_new_msgs();
		u_out.x = joy_x;
		u_out.y = joy_y;
		u_out.z = joy_z;
		if(u_out.x < dead_zone && u_out.x > -dead_zone)
			u_out.x = 0.0;
		if(u_out.y < dead_zone && u_out.y > -dead_zone)
			u_out.y = 0.0;
		if(u_out.z < dead_zone && u_out.z > -dead_zone)
			u_out.z = 0.0;
		u_pub.publish(u_out);
		ros::spinOnce();
		loop_rate.sleep();
	}
}

	
	
	
