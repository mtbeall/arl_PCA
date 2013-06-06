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
#include <Eigen/Dense>


#define yaw 0
#define pitch 1
#define roll 2
#define thrust 3


geometry_msgs::Vector3 pcurr;
geometry_msgs::Vector3 vcurr;
std_msgs::Float32 yaw_glob;
std_msgs::Float32 roll_glob;
std_msgs::Float32 pitch_glob;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mocap");
    ros::NodeHandle node;
       
    ros::Publisher pcurr_pub = node.advertise<geometry_msgs::Vector3>("current_position",1);
    ros::Publisher vcurr_pub = node.advertise<geometry_msgs::Vector3>("current_velocity",1);
    
    ros::Publisher yaw_pub = node.advertise<std_msgs::Float32>("current_yaw",1);
    ros::Publisher roll_pub = node.advertise<std_msgs::Float32>("current_roll",1);
    ros::Publisher pitch_pub = node.advertise<std_msgs::Float32>("current_pitch",1);
    tf::TransformListener listener;
    tf::StampedTransform stamped;
    int l_rate = 50;
    ros::Rate loop_rate(l_rate);

    //Eigen::Vector3f plast;
    //plast(0) = 0; plast(1) = 0; plast(2) = 0;
    //double plast[3] = {0,0,0};
    std::vector<double> plast(3);
    plast[0]=0;
    plast[1]=0;
    plast[2]=0;

    while(ros::ok())
    {
	listener.waitForTransform("optitrak", "quad", ros::Time(0), ros::Duration(2));
      	listener.lookupTransform("optitrak", "quad",  ros::Time(0), stamped);
        pcurr.x = stamped.getOrigin().getX();
        pcurr.y = stamped.getOrigin().getY();
        pcurr.z = stamped.getOrigin().getZ();
	
	//Compute Velocities and store position for next step
	//(distance)*(Hz) = distance/second
	vcurr.x = l_rate * (pcurr.x - plast[0]);
	vcurr.y = l_rate * (pcurr.y - plast[1]);
	vcurr.z = l_rate * (pcurr.z - plast[2]);

	plast[0] = pcurr.x;
	plast[1] = pcurr.y;
	plast[2] = pcurr.z; 
        
	double euler_angles[3];
        pcurr_pub.publish(pcurr);
        vcurr_pub.publish(vcurr);

	//Get RPY angles, and compute global Yaw
	btMatrix3x3(stamped.getRotation()).getRPY(euler_angles[roll], euler_angles[pitch], euler_angles[yaw]);        
        
	//yaw_glob.data = atan2(cos(euler_angles[roll])*sin(euler_angles[yaw]) + sin(euler_angles[roll])*sin(euler_angles[pitch])*cos(euler_angles[yaw]), cos(euler_angles[pitch])*cos(euler_angles[yaw]) );
	yaw_glob.data = euler_angles[yaw];
	roll_glob.data = euler_angles[roll];
	pitch_glob.data = euler_angles[pitch];
	yaw_pub.publish(yaw_glob);
	roll_pub.publish(roll_glob);
	pitch_pub.publish(pitch_glob);
	// These angles are measured as extrinsic(global). We now need to get the intrinsic(moving local frames) yaw or we can't pitch and roll. - Daman
        ros::spinOnce();
        loop_rate.sleep();

    }
}

        
    
