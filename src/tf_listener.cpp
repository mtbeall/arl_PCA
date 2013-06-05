/*

    Daman Bareiss
    Algorithmic Robotics Lab @ University of Utah
    
    This node listens to the tf from the mo-cap system and outputs a geometry_msgs::Vector3 of "current_position"

Edit: Matt Beall - Global Yaw and velocity computation

*/


#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Vector3.h>
#include "geometry_msgs/TransformStamped.h"
#include "ros/time.h"
#include <std_msgs/Float32.h>
#include <math.h>


#define yaw 0
#define pitch 1
#define roll 2
#define thrust 3


geometry_msgs::Vector3 pcurr;
geometry_msgs::Vector3 vcurr;
std_msgs::Float32 yaw_glob;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "TF_Listener_Node");
    ros::NodeHandle node;
       
    ros::Publisher pcurr_pub = node.advertise<geometry_msgs::Vector3>("current_position",1);
    ros::Publisher vcurr_pub = node.advertise<geometry_msgs::Vector3>("current_velocity",1);
    
    ros::Publisher yaw_pub = node.advertise<std_msgs::Float32>("current_yaw",1);
    tf::TransformListener listener;
    tf::StampedTransform stamped;
    int l_rate = 50;
    ros::Rate loop_rate(l_rate);

    double plast[] = {0, 0, 0};

    while(ros::ok())
    {
        listener.lookupTransform("/optitrak", "/quad",  ros::Time(0), stamped);
        pcurr.x = stamped.getOrigin().getX();
        pcurr.y = stamped.getOrigin().getY();
        pcurr.z = stamped.getOrigin().getZ();
	
	//Compute Velocities and store position for next step
	//(distance)*(Hz) = distance/second
	vcurr.x = l_rate * (pcurr.x - plast[0]);
	vcurr.y = l_rate * (pcurr.y - plast[1]);
	vcurr.z = l_rate * (pcurr.z - plast[3]);

	plast[0] = pcurr.x;
	plast[1] = pcurr.y;
	plast[2] = pcurr.z; 
        
	double euler_angles[3];
        pcurr_pub.publish(pcurr);
        vcurr_pub.publish(vcurr);

	//Get RPY angles, and compute global Yaw
	btMatrix3x3(stamped.getRotation()).getRPY(euler_angles[roll], euler_angles[pitch], euler_angles[yaw]);        
        
	yaw_glob.data = atan2(cos(euler_angles[roll])*sin(euler_angles[yaw]) + sin(euler_angles[roll])*sin(euler_angles[pitch])*cos(euler_angles[yaw]), cos(euler_angles[pitch])*cos(euler_angles[yaw]) );
	yaw_pub.publish(yaw_glob);
        ros::spinOnce();
        loop_rate.sleep();

    }
}

        
    
