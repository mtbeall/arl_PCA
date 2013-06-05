/*
    Daman Bareiss
    Algorithmic Robotics Lab

    This node takes in the updated desired position and outputs an updated u.
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

// Time-related variables
double tau = 2.0;
double dt = 1.0/50.0;
int t = (int) tau/dt;

// Position, vel, etc variables
geometry_msgs::Vector3 pdes_new;
geometry_msgs::Vector3 pcurr;
geometry_msgs::Vector3 vcurr;
geometry_msgs::Vector3 u_out;

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

// Read des_pos from projection
void des_pos_callback(const geometry_msgs::Vector3& pos_in)
{
    pdes_new.x = pos_in.x;
    pdes_new.y = pos_in.y;
    pdes_new.z = pos_in.z;
}

// Eigen initialization for inv dynamics math
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

    // ROS publisher
    ros::Publisher u_pub;
    u_pub = node.advertise<geometry_msgs::Vector3>("new_u",1);

    // ROS subscribers
    ros::Subscriber pcurr_sub;
    pcurr_sub = node.subscribe("current_position",1,pos_callback);
    ros::Subscriber vcurr_sub;
    vcurr_sub = node.subscribe("current_velocity",1,vel_callback);
    ros::Subscriber pdes_sub;
    pdes_sub = node.subscribe("new_desired_position",1,des_pos_callback);

    // Maps state to position
    P = Eigen::MatrixXf::Zero(pdim,xdim);
    P(0,0) = 1; P(1,1) = 1; P(2,2) = 1;

    // Set A,B,F,G matrices for a given number of time steps, t.
    setDynamics(A,B);
    findFG(A,B,F,G,t);

    // Loop
    while(ros::ok())
    {
        // Set x0
        x0[0] = pcurr.x; x0[1] = pcurr.y; x0[2] = pcurr.z;
        x0[3] = vcurr.x; x0[4] = vcurr.y; x0[5] = vcurr.z;
        // Set pdes
        pd[0] = pdes_new.x; pd[1] = pdes_new.y; pd[2] = pdes_new.z;
        // Calculate u, may have to map to -1 and 1
        u = (P*G).inverse()*(pd - P*F*x0);
        // Set and send u out
        u_out.x = u[0]; u_out.y = u[1]; u_out.z = u[2];
        u_pub.publish(u_out);
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
    F = Eigen::MatrixXf::Identity(xdim,xdim); // F = A^t
    G = Eigen::MatrixXf::Zero(xdim,udim);     // G = {summation from 0 to t-1} A^k*B
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
