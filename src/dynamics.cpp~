/*
    Daman Bareiss
    Algorithmic Robotics Lab

    This node takes in the joystick inputs and outputs a desired position
*/

// Includes
#include <ros/ros.h>
#include <Eigen/Dense>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Joy.h>
#include <unsupported/Eigen/MatrixFunctions>

// Defines
#define xdim 6
#define udim 3
#define pdim 3

// Variable initialization
double joy_x_,joy_y_,joy_z_;
double joy_x,joy_y,joy_z;
double tau = 2.0;
double dt = 1.0/50.0;
int t = (int) tau/dt;
sensor_msgs::Joy joy_msg_in;
geometry_msgs::Vector3 pdes;
geometry_msgs::Vector3 pcurr;
geometry_msgs::Vector3 vcurr;

// Read joystick positions from controller
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

// Read position from mocap
void pos_callback(const geometry_msgs::Vector3& pos_in)
{
    pcurr.x = pos_in.x;
    pcurr.y = pos_in.y;
    pcurr.z = pos_in.z;
}

// Read velocity from mocap
void vel_callback(const geometry_msgs::Vector3& vel_in)
{
    vcurr.x = vel_in.x;
    vcurr.y = vel_in.y;
    vcurr.z = vel_in.z;
}
 
// Eigen initializations for dynamics math
Eigen::MatrixXf A(xdim,xdim);
Eigen::MatrixXf B(xdim,udim);
Eigen::MatrixXf F(xdim,xdim);
Eigen::MatrixXf G(xdim,udim);
Eigen::MatrixXf P(pdim,xdim);
Eigen::VectorXf u(udim);
Eigen::VectorXf pd(pdim);
Eigen::VectorXf x0(xdim);
Eigen::VectorXf x(xdim);

// Pre-Declarations
void setDynamics(Eigen::MatrixXf& A, Eigen::MatrixXf& B);
void findFG(const Eigen::MatrixXf& A, const Eigen::MatrixXf& B, Eigen::MatrixXf& F, Eigen::MatrixXf& G, const int& t);
Eigen::MatrixXf power(const Eigen::MatrixXf& M, const int& power);

// Main Loop
int main(int argc, char** argv)
{
    // ROS Initialization
    ros::init(argc, argv,"dynamics");
    ros::NodeHandle node;
    ros::Rate loop_rate(50);

    // ROS publishers
    ros::Publisher pdes_pub;
    pdes_pub = node.advertise<geometry_msgs::Vector3>("desired_position",1);

    // ROS subscribers
    ros::Subscriber joy_sub;
    joy_sub = node.subscribe("joy",1,joy_callback);
    ros::Subscriber pcurr_sub;
    pcurr_sub = node.subscribe("current_position",1,pos_callback);
    ros::Subscriber vcurr_sub;
    vcurr_sub = node.subscribe("current_velocity",1,vel_callback);

    // Maps state to position
    P = Eigen::MatrixXf::Zero(pdim,xdim);
    P(0,0) = 1; P(1,1) = 1; P(2,2) = 1;

    // Set A,B,F,G matrices for a given number of time steps, t.
    setDynamics(A,B);
    findFG(A,B,F,G,t);

    // Loop
    while(ros::ok())
    {
        // Set joystick
        merge_new_msgs();
        // Set u from joystick, May have to map from -1 to 1 later
        u[0] = joy_x; u[1] = joy_y; u[2] = joy_z;
        // Set x0
        x0[0] = pcurr.x; x0[1] = pcurr.y; x0[2] = pcurr.z;
        x0[3] = vcurr.x; x0[4] = vcurr.y; x0[5] = vcurr.z;
        // Calculate pdes
        pd = P*F*x0 + P*G*u;
        // Set and send pdes out
        pdes.x = pd[0]; pdes.y = pd[1]; pdes.z = pd[2];
        pdes_pub.publish(pdes);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

// Stores A and B matrices
void setDynamics(Eigen::MatrixXf& A, Eigen::MatrixXf& B)
{
    // Set dynamics here, A = ... B = ...
}

// Calculates and stores F,G matrices for a given number of time steps t
void findFG(const Eigen::MatrixXf& A, const Eigen::MatrixXf& B, Eigen::MatrixXf& F, Eigen::MatrixXf& G, const int& t)
{
    F = Eigen::MatrixXf::Identity(xdim,xdim);   // F = A^t
    G = Eigen::MatrixXf::Zero(xdim,udim);       // G = {summation from 0 to t-1} A^k*B
    F = power(F,t);
    for(int i=0; i < (t-1); i++) { G = G + power(A,i)*B; }
}

// Power function. Eigen .pow() only works for static matrices not dynamic matrices
Eigen::MatrixXf power(const Eigen::MatrixXf& M, const int& power)
{
    Eigen::MatrixXf Mp(M.rows(),M.cols());
    Mp = Eigen::MatrixXf::Identity(M.rows(),M.cols());
    for(int i = 1; i <= power; i++)
    {
        Mp = Mp*M;
    }
    return Mp;
}
