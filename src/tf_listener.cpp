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

geometry_msgs::Vector3 pcurr;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "TF_Listener_Node");
    ros::NodeHandle node;
       
    ros::Publisher pcurr_pub = node.advertise<geometry_msgs::Vector3>("current_position",1);
    tf::TransformListener listener;
    tf::StampedTransform stamped;


    ros::Rate loop_rate(50);
    while(ros::ok())
    {
        //listener.waitForTransform("/quad", "/optitrak", ros::Time(0),ros::Duration(0.5));
        listener.lookupTransform("/quad", "/optitrak",  ros::Time(0), stamped);
        pcurr.x = stamped.getOrigin().getX();
        pcurr.y = stamped.getOrigin().getY();
        pcurr.z = stamped.getOrigin().getZ();
        pcurr_pub.publish(pcurr);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

        
    
