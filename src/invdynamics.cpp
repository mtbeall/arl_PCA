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
#include <std_msgs/Float32.h>

// Defines
#define xdim 12
#define udim 3
#define pdim 3

// Time-related variables
// double tau = 1.25;
double tau;
//double dt = 1.0/50.0;
//int t = (int) tau/dt;

// Position, vel, etc variables
geometry_msgs::Vector3 pdes_new;
geometry_msgs::Vector3 pdes;
geometry_msgs::Vector3 pcurr;
geometry_msgs::Vector3 vcurr;
geometry_msgs::Vector3 u_out;
geometry_msgs::Vector3 rcurr;
geometry_msgs::Vector3 rdotcurr;

/*void tau_callback(const std_msgs::Float32& tau_in) {
	tau = tau_in.data;
}*/

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

// Read new_des_pos from projection
void new_des_pos_callback(const geometry_msgs::Vector3& pos_in)
{
    pdes_new.x = pos_in.x;
    pdes_new.y = pos_in.y;
    pdes_new.z = pos_in.z;
}

// read des_pos before collision check
void des_pos_callback(const geometry_msgs::Vector3& pos_in)
{
    pdes.x = pos_in.x;
    pdes.y = pos_in.y;
    pdes.z = pos_in.z;
}

void r_callback(const geometry_msgs::Vector3& r_in)
{
	rcurr.x = r_in.x;
	rcurr.y = r_in.y;
	rcurr.z = r_in.z;
}

void rdot_callback(const geometry_msgs::Vector3& rdot_in)
{
	rdotcurr.x = rdot_in.x;
	rdotcurr.y = rdot_in.y;
	rdotcurr.z = rdot_in.z;
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
void findFG(const Eigen::MatrixXf& A, const Eigen::MatrixXf& B, Eigen::MatrixXf& F, Eigen::MatrixXf& G, const double& tau);
Eigen::MatrixXf power(const Eigen::MatrixXf& M, const int& power);

// Main Loop
int main(int argc, char** argv)
{
    // ROS Initialization   
    ros::init(argc, argv,"dynamics");
    ros::NodeHandle node;
    ros::Rate loop_rate(50);

	if(node.getParam("/current_tau", tau)) {;}
	else {
		ROS_ERROR("NO TAU!!!!!!!!!");
	}

    // ROS publisher
    ros::Publisher u_pub;
    u_pub = node.advertise<geometry_msgs::Vector3>("new_u",1);

    // ROS subscribers
    ros::Subscriber pcurr_sub;
    pcurr_sub = node.subscribe("current_position",1,pos_callback);
    ros::Subscriber vcurr_sub;
    vcurr_sub = node.subscribe("current_velocity",1,vel_callback);
    ros::Subscriber pdes_new_sub;
    pdes_new_sub = node.subscribe("new_desired_position",1,new_des_pos_callback);
	ros::Subscriber pdes_sub;
	pdes_sub = node.subscribe("desired_position",1,des_pos_callback);
	ros::Subscriber rcurr_sub;
	rcurr_sub = node.subscribe("current_r",1,r_callback);
	ros::Subscriber rdotcurr_sub;
	rdotcurr_sub = node.subscribe("current_rdot",1,rdot_callback);
	/*ros::Subscriber tau_sub;
	tau_sub = node.subscribe("current_tau", 1, tau_callback);

	ros::spinOnce();*/

    // Maps state to position
    P = Eigen::MatrixXf::Zero(pdim,xdim);
    P(0,0) = 1; P(1,1) = 1; P(2,2) = 1;

    // Set A,B,F,G matrices for a given number of time steps, t.
    setDynamics(A,B);
	findFG(A,B,F,G,tau);

    // Loop
    while(ros::ok())
    {
		//double a = sqrt(pow(pdes.x - pcurr.x,2)+pow(pdes.y-pcurr.y,2)+pow(pdes.z-pcurr.z,2));
		//double b = sqrt(pow(pdes_new.x - pcurr.x,2)+pow(pdes_new.y-pcurr.y,2)+pow(pdes_new.z-pcurr.z,2));
		//double tau2 = tau*pow(b/a,1.0/3.0);
		//findFG(A,B,F,G,tau2);
        // Set x0
        x0[0] = pcurr.x; x0[1] = pcurr.y; x0[2] = pcurr.z;
        x0[3] = vcurr.x; x0[4] = vcurr.y; x0[5] = vcurr.z;
		x0[6] = rcurr.x; x0[7] = rcurr.y; x0[8] = rcurr.z;
		x0[9] = rdotcurr.x; x0[10] = rdotcurr.y; x0[11] = rdotcurr.z;
        // Set pdes
        pd[0] = pdes_new.x; pd[1] = pdes_new.y; pd[2] = pdes_new.z;
        // Calculate u, may have to map to -1 and 1
        u = (P*G).inverse()*(pd - P*F*x0);
        // Set and send u out
        //u_out.x = u[0]; u_out.y = u[1]; u_out.z = u[2];
		u_out.z = u[0]; u_out.x = -u[1]; u_out.y = -u[2];
        u_pub.publish(u_out);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

// Stores A and B matrices
void setDynamics(Eigen::MatrixXf& A, Eigen::MatrixXf& B)
{
	A<<	0,0,0,1,0,0,0,0,0,0,0,0,
	0,0,0,0,1,0,0,0,0,0,0,0,
	0,0,0,0,0,1,0,0,0,0,0,0,
	0,0,0,-0.25,0,0,0,9.812,0,0,0,0,
	0,0,0,0,-0.25,0,-9.812,0,0,0,0,0,
	0,0,0,0,0,-0.25,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,1,0,0,
	0,0,0,0,0,0,0,0,0,0,1,0,
	0,0,0,0,0,0,0,0,0,0,0,1,
	0,0,0,0,0,0,-1000,0,0,-63.246,0,0,
	0,0,0,0,0,0,0,-1000,0,0,-63.246,0,
	0,0,0,0,0,0,0,0,0,0,0,-0.1;
	 
	B<<	0,0,0,
	0,0,0,
	0,0,0,
	0,0,0,
	0,0,0,
	2.381,0,0,
	0,0,0,
	0,0,0,
	0,0,0,
	0,1000,0,
	0,0,1000,
	0,0,0;
}

// Calculates and stores F,G matrices for a given number of time steps t
void findFG(const Eigen::MatrixXf& A, const Eigen::MatrixXf& B, Eigen::MatrixXf& F, Eigen::MatrixXf& G, const double& t)
{
	Eigen::MatrixXf AB(xdim+udim,xdim+udim);
	AB << A, B, Eigen::MatrixXf::Constant(udim,xdim+udim, 0.0);
	AB = t*AB;
	AB = AB.exp();
	F << AB.topLeftCorner(xdim,xdim);
	G << AB.topRightCorner(xdim,udim);
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
