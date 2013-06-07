/*
    Daman Bareiss
    Algorithmic Robotics Lab @ University of Utah
    
    This node takes in a desired position and outputs a new, SAFE desired position.  
*/

// Includes
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <Eigen/Dense>
#include <math.h>

// Global variables for use in program
geometry_msgs::Vector3 pdes_old;
geometry_msgs::Vector3 pcurr;
geometry_msgs::Vector3 vcurr;
geometry_msgs::Vector3 pdes_new;

// Define + operator for geometry_msgs::Vector3
geometry_msgs::Vector3 operator+(geometry_msgs::Vector3 A, geometry_msgs::Vector3 B)
{
    geometry_msgs::Vector3 Sum;
    Sum.x = A.x + B.x;
    Sum.y = A.y + B.y;
    Sum.z = A.z + B.z;
    return Sum;
}

// Define - operator for geometry_msgs::Vector3
geometry_msgs::Vector3 operator-(geometry_msgs::Vector3 A, geometry_msgs::Vector3 B)
{
    geometry_msgs::Vector3 Diff;
    Diff.x = A.x - B.x;
    Diff.y = A.y - B.y;
    Diff.z = A.z - B.z;
    return Diff;
}   

// Define * operator for geometry_msgs::Vector3
geometry_msgs::Vector3 operator*(double a, geometry_msgs::Vector3 A)
{
    A.x = a*A.x;
    A.y = a*A.y;
    A.z = a*A.z;
    return A;
}

// Obstacle class: sets list of 3 vertices, normal vector, and a flag variable
class obstacle
{
    public:
    std::vector<geometry_msgs::Vector3> vertices; // List of 3 vertices
    geometry_msgs::Vector3 normal;                // Normal vector
    bool flag;                                    // Flag for later projection step
    double t;
    obstacle() {flag = false;t = 20.0;}                     // Default constructor with flag being set true
    obstacle(std::vector<geometry_msgs::Vector3> vertex, geometry_msgs::Vector3 norm) // Constructor setting all values
    {
        for(int i = 0; i<3; i++)                // Set the 3 vertices
        {
            vertices[i].x = vertex[i].x;
            vertices[i].y = vertex[i].y;
            vertices[i].z = vertex[i].z;
        }
        normal.x = norm.x;                      // Set the normal
        normal.y = norm.y;
        normal.z = norm.z;
        flag = false;                            // Set the flag
    }
};

  
// Reads in pdes from dynamics
void pdes_callback(const geometry_msgs::Vector3& pdes_in)
{
    pdes_old.x = pdes_in.x;
    pdes_old.y = pdes_in.y;
    pdes_old.z = pdes_in.z;
}

// Reads in pcurr from mo-cap system
void pos_callback(const geometry_msgs::Vector3& pcurr_in)
{
    pcurr.x = pcurr_in.x;
    pcurr.y = pcurr_in.y;
    pcurr.z = pcurr_in.z;
}

// Read in vcurr from mocap
void vel_callback(const geometry_msgs::Vector3& vcurr_in)
{
    vcurr.x = vcurr_in.x;
    vcurr.y = vcurr_in.y;
    vcurr.z = vcurr_in.z;
} 

// Pre-declarations
double dot(const geometry_msgs::Vector3& A, const geometry_msgs::Vector3& B);
double distance(const geometry_msgs::Vector3& A, const geometry_msgs::Vector3& B);
void check_seeable(const std::vector<obstacle>& obs_list, const geometry_msgs::Vector3& pcurr, std::vector<obstacle>& see_obs);
Eigen::Vector3f solveIntersection(const geometry_msgs::Vector3& pcurr, const geometry_msgs::Vector3& pdes, const obstacle& plane);
geometry_msgs::Vector3 intersectLine(const geometry_msgs::Vector3& pcurr, const geometry_msgs::Vector3& pdes, const Eigen::Vector3f tuv);
bool checkIntersection(const geometry_msgs::Vector3& pcurr, const geometry_msgs::Vector3& pdes, std::vector<obstacle>& see_obs);
geometry_msgs::Vector3 projectOntoPlane(const geometry_msgs::Vector3& p, const obstacle& plane);
geometry_msgs::Vector3 averageProject(const geometry_msgs::Vector3& intersect, const geometry_msgs::Vector3& project, const double& D, const obstacle& plane);
void buildObstacles(std::vector<obstacle>& full_obs_list);

// Main
int main(int argc, char** argv)
{
    // ROS Initialization
    ros::init(argc, argv, "Projection_Node");
    ros::NodeHandle node;
    ros::Rate loop_rate(50);
    
    // ROS publishers
    ros::Publisher  pdes_new_pub;
    pdes_new_pub = node.advertise<geometry_msgs::Vector3>("new_desired_pos", 1);

    // ROS subscribers
    ros::Subscriber pdes_old_sub;
    pdes_old_sub = node.subscribe("desired_position", 1, pdes_callback);
    ros::Subscriber pcurr_sub;
    pcurr_sub = node.subscribe("current_position", 1, pos_callback);
    ros::Subscriber vcurr_sub;
    vcurr_sub = node.subscribe("current_velocity", 1, vel_callback);

    // Obstacle list (full)
    std::vector<obstacle> full_obs_list;
    buildObstacles(full_obs_list);
    
    double t_min = 2.0;
    int t_argmin;
    // loop
    while(ros::ok()) {
        std::vector<obstacle> seeable_obs;              // declaration of list of seeable obstacles
        bool imminentCollision = false;                 // Set imminent collision to false initially
        check_seeable(full_obs_list,pcurr,seeable_obs); // Check normals to populate seeable obstacle list
        pdes_new.x = pdes_old.x;                        // Setting new pdes to the old one, will be updated if necessary
        pdes_new.y = pdes_old.y;
        pdes_new.z = pdes_old.z;
        if (seeable_obs.size() > 0)                     // Check if there are seeable obstacles
        {
            while(true)
            {
                imminentCollision = checkIntersection(pcurr,pdes_new,seeable_obs);
                if (imminentCollision)                      // Check if imminent collision
                {
                    for(int i = 0; i < seeable_obs.size(); i++)
                    {
                        if(seeable_obs[i].t < 2.0)
                        {
                            if(seeable_obs[i].t < t_min)
                            {
                                t_min = seeable_obs[i].t;
                                t_argmin = i;
                            }
                        }
                    }  
			/* FIX SOME BUG HERE - Daman                 
                    geometry_msgs::Vector3 intersect;
                    intersect = intersectLine(pcurr,pdes_new,solveIntersection(pcurr, pdes_new, seeable_obs[t_argmin]));
                    geometry_msgs::Vector3 project;
                    project = projectOntoPlane(pdes_new,seeable_obs[t_argmin]);
                    double dist;
                    dist = distance(pcurr, projectOntoPlane(pcurr,seeable_obs[t_argmin]));
                    pdes_new = averageProject(intersect, project, dist, seeable_obs[t_argmin]); */
		    for(int i = 0; i < seeable_obs.size(); i++)
		    {
		    	seeable_obs[i].t = 20.0;
		    }
                }
                else
                    break;
            }
	}
        pdes_new_pub.publish(pdes_new);                 // Publish new desired position
        ros::spinOnce();                                // Publish/subscribe to variables
	loop_rate.sleep();                              // Wait if necessary to reach desired loop_rate
    }
}

// function to find dot product of two geometry_msgs::Vector3's
double dot(const geometry_msgs::Vector3& A, const geometry_msgs::Vector3& B)
{
    return A.x*B.x + A.y*B.y + A.z*B.z;
}

// Returns distance betweent wo points
double distance(const geometry_msgs::Vector3& A, const geometry_msgs::Vector3& B)
{
    return std::sqrt( std::pow((A.x-B.x),2) + std::pow((A.y - B.y),2) + std::pow((A.z - B.z),2) );
}

// Checks if an obstacle is see-able
void check_seeable(const std::vector<obstacle>& obs_list, const geometry_msgs::Vector3& pcurr, std::vector<obstacle>& see_obs)
{
    geometry_msgs::Vector3 AB;
    for(int i = 0; i <  obs_list.size(); i++)
    {   
        AB = pcurr - obs_list[i].vertices[0];
        if (dot(AB,obs_list[i].normal) > 0)
            see_obs.push_back(obs_list[i]);
    }  
}

// Solve for t,u,v for intersection
Eigen::Vector3f solveIntersection(const geometry_msgs::Vector3& pcurr, const geometry_msgs::Vector3& pdes, const obstacle& plane)
{
    Eigen::Matrix3f A;
    Eigen::Matrix3f invA;
    Eigen::Vector3f b;
    Eigen::Vector3f x;
    A(0,0) = pcurr.x - pdes.x;
    A(0,1) = plane.vertices[1].x - plane.vertices[0].x;
    A(0,2) = plane.vertices[2].x - plane.vertices[0].x;
    A(1,0) = pcurr.y - pdes.y;
    A(1,1) = plane.vertices[1].y - plane.vertices[0].y;
    A(1,2) = plane.vertices[2].y - plane.vertices[0].y;
    A(2,0) = pcurr.z - pdes.z;
    A(2,1) = plane.vertices[1].z - plane.vertices[0].z;
    A(2,2) = plane.vertices[2].z - plane.vertices[0].z;
    return A.inverse()*b;
}

// Find the point a line intersects with a plane
geometry_msgs::Vector3 intersectLine(const geometry_msgs::Vector3& pcurr, const geometry_msgs::Vector3& pdes, const Eigen::Vector3f tuv)
{
    return pcurr + tuv(0)*(pdes - pcurr);
}

// Checks for an intersection between current position and desired position for all seeable obstacles
bool checkIntersection(const geometry_msgs::Vector3& pcurr, const geometry_msgs::Vector3& pdes, std::vector<obstacle>& see_obs)
{
    Eigen::Vector3f x;
    for(int i = 0; i < see_obs.size(); i++)
    {
        x = solveIntersection(pcurr, pdes, see_obs[i]);
        if(x(2) <= 1 && x(2) >= 0 && x(3) <= 1 && x(3) >= 0 && x(2)+x(3) <= 1)
        {
            see_obs[i].t = x(0);
            return true;
        }
    }
    return false;
}

// Projects a point onto a given plane through it's normal vector
geometry_msgs::Vector3 projectOntoPlane(const geometry_msgs::Vector3& p, const obstacle& plane)
{
    geometry_msgs::Vector3 p0;
    double D;
    p0 = p - plane.vertices[0];
    D = dot(p0,plane.normal);
    return p0 - D*plane.normal;
}
    
// Finds a point on the line proportional to the intersection or projection point.
geometry_msgs::Vector3 averageProject(const geometry_msgs::Vector3& intersect, const geometry_msgs::Vector3& project, const double& D, const obstacle& plane)
{
    double d = 1.0;
    geometry_msgs::Vector3 P;
    if (D<d) 
        P = ((d-D)/d) * (project - intersect) + intersect;
    else
        P = intersect;
    return P + 0.01*plane.normal;
}

// Builds full list of obstacles
void buildObstacles(std::vector<obstacle>& full_obs_list)
{
    // Build list of obstacles here.
    double width = 3.96;    // Width of enclosure (m)
    double height = 3.5;   // Height of enclosure (m)
    double length = 6.85;   // Length of enclosure (m)
    double radius = 0.559/0.5;   // Radius of quad sphere (m)
    std::vector<geometry_msgs::Vector3> vertex;
    geometry_msgs::Vector3 normal;
    obstacle newObs;
    // Wall by computers
    newObs.vertices[0].x = width/2.0-radius;
    newObs.vertices[0].y = -length/2.0+radius;
    newObs.vertices[0].z = 0.0+radius;
    newObs.vertices[1].x = -width/2.0+radius;
    newObs.vertices[1].y = -length/2.0+radius;
    newObs.vertices[1].z = 0.0+radius;
    newObs.vertices[2].x = -width/2.0+radius;
    newObs.vertices[2].y = -length/2.0+radius;
    newObs.vertices[2].z = height-radius;
    newObs.normal.x = 0.0;
    newObs.normal.y = 1.0;
    newObs.normal.z = 0.0;
    full_obs_list.push_back(newObs);
    newObs.vertices[0].x = -width/2.0+radius;
    newObs.vertices[0].y = -length/2.0+radius;
    newObs.vertices[0].z = height-radius;
    newObs.vertices[1].x = width/2.0-radius;
    newObs.vertices[1].y = -length/2.0+radius;
    newObs.vertices[1].z = height-radius;
    newObs.vertices[2].x = width/2.0-radius;
    newObs.vertices[2].y = -length/2.0+radius;
    newObs.vertices[2].z = 0.0+radius;
    newObs.normal.x = 0.0;
    newObs.normal.y = 1.0;
    newObs.normal.z = 0.0;
    full_obs_list.push_back(newObs);
    // Wall to right of computers
    newObs.vertices[0].x = width/2.0-radius;
    newObs.vertices[0].y = -length/2.0+radius;
    newObs.vertices[0].z = 0.0+radius;
    newObs.vertices[1].x = width/2.0-radius;
    newObs.vertices[1].y = length/2.0-radius;
    newObs.vertices[1].z = 0.0+radius;
    newObs.vertices[2].x = width/2.0-radius;
    newObs.vertices[2].y = -length/2.0+radius;
    newObs.vertices[2].z = height-radius;
    newObs.normal.x = -1.0;
    newObs.normal.y = 0.0;
    newObs.normal.z = 0.0;
    full_obs_list.push_back(newObs);
    newObs.vertices[0].x = width/2.0-radius;
    newObs.vertices[0].y = -length/2.0+radius;
    newObs.vertices[0].z = height-radius;
    newObs.vertices[1].x = width/2.0-radius;
    newObs.vertices[1].y = length/2.0-radius;
    newObs.vertices[1].z = height-radius;
    newObs.vertices[2].x = width/2.0-radius;
    newObs.vertices[2].y = length/2.0-radius;
    newObs.vertices[2].z = 0.0+radius;
    newObs.normal.x = -1.0;
    newObs.normal.y = 0.0;
    newObs.normal.z = 0.0;
    full_obs_list.push_back(newObs);
    // Wall to left of computers
    newObs.vertices[0].x = -width/2.0+radius;
    newObs.vertices[0].y = -length/2.0+radius;
    newObs.vertices[0].z = 0.0+radius;
    newObs.vertices[1].x = -width/2.0+radius;
    newObs.vertices[1].y = length/2.0-radius;
    newObs.vertices[1].z = 0.0+radius;
    newObs.vertices[2].x = -width/2.0+radius;
    newObs.vertices[2].y = -length/2.0+radius;
    newObs.vertices[2].z = height-radius;
    newObs.normal.x = 1.0;
    newObs.normal.y = 0.0;
    newObs.normal.z = 0.0;
    full_obs_list.push_back(newObs);
    newObs.vertices[0].x = -width/2.0+radius;
    newObs.vertices[0].y = -length/2.0+radius;
    newObs.vertices[0].z = height-radius;
    newObs.vertices[1].x = -width/2.0+radius;
    newObs.vertices[1].y = length/2.0-radius;
    newObs.vertices[1].z = height-radius;
    newObs.vertices[2].x = -width/2.0+radius;
    newObs.vertices[2].y = length/2.0-radius;
    newObs.vertices[2].z = 0.0+radius;
    newObs.normal.x = 1.0;
    newObs.normal.y = 0.0;
    newObs.normal.z = 0.0;
    full_obs_list.push_back(newObs);
    // Wall opposite computers
    newObs.vertices[0].x = width/2.0-radius;
    newObs.vertices[0].y = length/2.0-radius;
    newObs.vertices[0].z = 0.0+radius;
    newObs.vertices[1].x = -width/2.0+radius;
    newObs.vertices[1].y = length/2.0-radius;
    newObs.vertices[1].z = 0.0+radius;
    newObs.vertices[2].x = -width/2.0+radius;
    newObs.vertices[2].y = length/2.0-radius;
    newObs.vertices[2].z = height-radius;
    newObs.normal.x = 0.0;
    newObs.normal.y = -1.0;
    newObs.normal.z = 0.0;
    full_obs_list.push_back(newObs);
    newObs.vertices[0].x = -width/2.0+radius;
    newObs.vertices[0].y = length/2.0-radius;
    newObs.vertices[0].z = height-radius;
    newObs.vertices[1].x = width/2.0-radius;
    newObs.vertices[1].y = length/2.0-radius;
    newObs.vertices[1].z = height-radius;
    newObs.vertices[2].x = width/2.0-radius;
    newObs.vertices[2].y = length/2.0-radius;
    newObs.vertices[2].z = 0.0+radius;
    newObs.normal.x = 0.0;
    newObs.normal.y = -1.0;
    newObs.normal.z = 0.0;
    full_obs_list.push_back(newObs);
    // Floor
    newObs.vertices[0].x = -width/2.0+radius;
    newObs.vertices[0].y = -length/2.0+radius;
    newObs.vertices[0].z = 0.0+radius;
    newObs.vertices[1].x = width/2.0-radius;
    newObs.vertices[1].y = -length/2.0+radius;
    newObs.vertices[1].z = 0.0+radius;
    newObs.vertices[2].x = -width/2.0+radius;
    newObs.vertices[2].y = length/2.0-radius;
    newObs.vertices[2].z = 0.0+radius;
    newObs.normal.x = 0.0;
    newObs.normal.y = 0.0;
    newObs.normal.z = 1.0;
    full_obs_list.push_back(newObs);
    newObs.vertices[0].x = width/2.0-radius;
    newObs.vertices[0].y = -length/2.0+radius;
    newObs.vertices[0].z = 0.0+radius;
    newObs.vertices[1].x = width/2.0-radius;
    newObs.vertices[1].y = length/2.0-radius;
    newObs.vertices[1].z = 0.0+radius;
    newObs.vertices[2].x = -width/2.0+radius;
    newObs.vertices[2].y = length/2.0-radius;
    newObs.vertices[2].z = 0.0+radius;
    newObs.normal.x = 0.0;
    newObs.normal.y = 0.0;
    newObs.normal.z = 1.0;
    full_obs_list.push_back(newObs);
    // Ceiling
    newObs.vertices[0].x = -width/2.0+radius;
    newObs.vertices[0].y = -length/2.0+radius;
    newObs.vertices[0].z = 0.0+radius;
    newObs.vertices[1].x = width/2.0-radius;
    newObs.vertices[1].y = -length/2.0+radius;
    newObs.vertices[1].z = 0.0+radius;
    newObs.vertices[2].x = -width/2.0+radius;
    newObs.vertices[2].y = length/2.0-radius;
    newObs.vertices[2].z = 0.0+radius;
    newObs.normal.x = 0.0;
    newObs.normal.y = 0.0;
    newObs.normal.z = -1.0;
    full_obs_list.push_back(newObs);
    newObs.vertices[0].x = width/2.0-radius;
    newObs.vertices[0].y = -length/2.0+radius;
    newObs.vertices[0].z = 0.0+radius;
    newObs.vertices[1].x = width/2.0-radius;
    newObs.vertices[1].y = length/2.0-radius;
    newObs.vertices[1].z = 0.0+radius;
    newObs.vertices[2].x = -width/2.0+radius;
    newObs.vertices[2].y = length/2.0-radius;
    newObs.vertices[2].z = 0.0+radius;
    newObs.normal.x = 0.0;
    newObs.normal.y = 0.0;
    newObs.normal.z = -1.0;
    full_obs_list.push_back(newObs);
} 
