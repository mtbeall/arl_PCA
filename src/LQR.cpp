/* 
    Daman Bareiss
    Algorithmic Robotics Lab @ University of Utah
    
    This node takes in a current position and velocity and runs the LQR on it about a given dynamics and outputs control input

*/


#include <ros/ros.h>
#include <Eigen/Dense>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <Eigen/Transpose>


#define x_dim 6
#define u_dim 3
#define p_dim 3

Eigen::VectorXf u_out(u_dim+1);
Eigen::VectorXf u(u_dim);      		// Control Input 
Eigen::VectorXf x(x_dim);        	// State [x xdot y ydot z zdot]'
Eigen::VectorXf pd(p_dim);			// Desired position

Eigen::MatrixXf A(x_dim,x_dim);		// Dynamics Matrix A
Eigen::MatrixXf B(x_dim,u_dim);		// Dynamics Matrix B

Eigen::MatrixXf Q(p_dim,p_dim);		// LQR path Cost
Eigen::MatrixXf R(u_dim,u_dim);		// LQR input cost
Q = Eigen::MatrixXf::Zeros();
R = Eigen::MatrixXf::Zeros();

Eigen::MatrixXf L(u_dim,x_dim);     // L control matrix
Eigen::MatrixXf E(u_dim,p_dim);     // E control matrix
L = Eigen::MatrixXf::Zero();
E = Eigen::MatrixXf::Zero();

Eigen::MatrixXf S(x_dim,x_dim);		// Ricatti Solution S
Eigen::MatrixXf T(x_dim,p_dim);		// Ricatti T
S = Eigen::MatrixXf::Identity();
T = Eigen::MatrixXf::Identity();

Eigen::MatrixXf P(p_dim,x_dim);		// Map state to position
P = Eigen::MatrixXf::Zero();
P(0,0) = 1;  //|1 0 0 0 0 0|
P(1,2) = 1;  //|0 0 1 0 0 0|
P(2,4) = 1;  //|0 0 0 0 1 0|

geometry_msgs::Vector3 pdes, pcurr, vcurr; // Current state varaibles

// Define Dynamics Matrices
    // A,B,C,D linearized/discretized


for(int i = 0; i < 200; i++)
{
	S = P.transpose()*Q*P + A.transpose()*S*A - A.transpose()*S*B*(R+B.transpose()*S*B).inverse()*B.transpose()*S*A;
	T = P.transpose()*Q + A.transpose()*T - A.transpose()*S*B*(R+B.tranpose()*S*B).inverse()*B.transpose()*T;
}

L = (R + B.transpose()*S*B).inverse()*B.transpose()*S*A;
E = (R + B.transpose()*S*B).inverse()*B.transpose()*T;
// u = -Lx + Ep_d	

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

void findControlMatrices(Eigen::MatrixXf& S, Eigen::MatrixXf& T, Eigen::MatrixXf& L, Eigen::MatrixXf& E);

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
    u_pub = node.advertise<Eigen::VectorXf>("LQR_u", 1);

    while(ros::ok())
    {
        x(0) = pcurr.x; 
        x(1) = vcurr.x;
        x(2) = pcurr.y;
        x(3) = vcurr.y; 
        x(4) = pcurr.z; 
        x(5) = vcurr.z;
        pd(0) = pdes.x; 
        pd(1) = pdes.y; 
        pd(2) = pdes.z;
        u = -1*L*x + E*pd;
        u_out(0) = u(0);
        u_out(1) = u(1);
        u_out(2) = u(2);
        u_out(3) = 0.0; // maybe put a P controller about yaw here?? With orientation from the mo-cap as well.
        u_pub.publish(u_out);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
    
 
