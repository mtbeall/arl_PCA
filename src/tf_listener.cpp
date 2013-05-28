/*

    Daman Bareiss
    Algorithmic Robotics Lab @ University of Utah
    
    This node listens to the tf from the mo-cap system and outputs a geometry_msgs::Vector3 of "current_position"

*/


#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Vector3.h>
#include "geometry_msgs/TransformStamped.h"
#include "ros/time.h"
#include <std_msgs/Float32.h>


#define yaw 0
#define pitch 1
#define roll 2
#define thrust 3


geometry_msgs::Vector3 pcurr;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "TF_Listener_Node");
    ros::NodeHandle node;
       
    ros::Publisher pcurr_pub = node.advertise<geometry_msgs::Vector3>("current_position",1);
    ros::Publisher yaw_pub = node.advertise<std_msgs::Float32>("current_yaw",1);
    tf::TransformListener listener;
    tf::StampedTransform stamped;

    ros::Rate loop_rate(50);
    while(ros::ok())
    {
        listener.lookupTransform("/quad", "/optitrak",  ros::Time(0), stamped);
        pcurr.x = stamped.getOrigin().getX();
        pcurr.y = stamped.getOrigin().getY();
        pcurr.z = stamped.getOrigin().getZ();
        double euler_angles[3];
        pcurr_pub.publish(pcurr);
        //btMatrix3x3(stamped.getRotation()).getRPY(euler_angles[roll], euler_angles[pitch], euler_angles[yaw]);        
        //yaw_pub.publish(euler_angles[yaw]);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

        
    
