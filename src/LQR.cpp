/* 
    Daman Bareiss
    Algorithmic Robotics Lab @ University of Utah
    
    This node takes in a current position and velocity and runs the LQR on it about a given dynamics and outputs control input

*/

#include <ros/ros.h>
#include <Eigen/Dense>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>

// Define Dynamics Matrices
    // A,B,C,D linearized/discretized
// Find control matrices
    // S,T,L,E

std_msgs::Float32 u;                   // Control Input    
Eigen::MatrixXf L(1,6);     // L control matrix
Eigen::MatrixXf E(1,3);     // E control matrix
Eigen::VectorXf x(6);          // State
Eigen::VectorXf pd(3);      // Desired position
geometry_msgs::Vector3 pdes, pcurr, vcurr; // Current state

void read_pcurr(const geometry_msgs::Vector3& pcurr_in)
{
    pcurr.x = pcurr_in.x;
    pcurr.y = pcurr_in.y;
    pcurr.z = pcurr_in.z;
}
void read_vcurr(const geometry_msgs::Vector3& vcurr_in)
{
    vcurr.x = vcurr_in.x;
    vcurr.y = vcurr_in.y;
    vcurr.z = vcurr_in.z;
}
void read_pdes(const geometry_msgs::Vector3& pdes_in)
{
    pdes.x = pdes_in.x;
    pdes.y = pdes_in.y;
    pdes.z = pdes_in.z;
}

int main(int argc,char** argv)
{
    // ROS Initialization
    ros::init(argc, argv, "LQR_Node");
    ros::NodeHandle node;
    ros::Rate loop_rate(50);
    
    // Set up publisher and subscribers
    ros::Publisher  u_pub;
    ros::Subscriber pcurr_sub;
    ros::Subscriber vcurr_sub;
    ros::Subscriber pdes_sub;
    pcurr_sub = node.subscribe("current_position", 1, read_pcurr);
    vcurr_sub = node.subscribe("current_velocity", 1, read_vcurr);
    pdes_sub  = node.subscribe("new_desired_position", 1, read_pdes);
    u_pub = node.advertise<std_msgs::Float32>("control_input", 1);

    while(ros::ok())
    {
        /*
        x(0) = pcurr.x; 
        x(1) = pcurr.y; 
        x(2) = pcurr.z;
        x(3) = vcurr.x; 
        x(4) = vcurr.y; 
        x(5) = vcurr.z;
        pd(0) = pdes.x; 
        pd(1) = pdes.y; 
        pd(2) = pdes.z;
        u = -1*L*x + E*pd;
        */
        u = 0.0;
        u_pub.publish(u);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
    
    
