/*
    Daman Bareiss
    Algorithmic Robotics Lab @ University of Utah
    
    This node takes in a desired position (from joystick or trajectory generator) and outputs a new, SAFE desired position.  
*/

// -------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------------------


// Includes
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <Eigen/Dense>


// -------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------------------


// Global variables for use in program
geometry_msgs::Vector3 pdes_old;
geometry_msgs::Vector3 pcurr;
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
    obstacle() {flag = true;}                     // Default constructor with flag being set true
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
        flag = true;                            // Set the flag
    }
    void triggerFlag()
    {
        if(flag)
            flag = false;
        else
            flag = true;
    }
};

  
// Reads in pdes from joystick or trajectory planner
void read_pdes(const geometry_msgs::Vector3& pdes_in)
{
    pdes_old.x = pdes_in.x;
    pdes_old.y = pdes_in.y;
    pdes_old.z = pdes_in.z;
}


// Reads in pcurr from mo-cap system
void read_pcurr(const geometry_msgs::Vector3& pcurr_in)
{
    pcurr.x = pcurr_in.x;
    pcurr.y = pcurr_in.y;
    pcurr.z = pcurr_in.z;
}


// -------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------------------


// Pre-declarations
double dot(const geometry_msgs::Vector3& A, const geometry_msgs::Vector3& B);
Eigen::Vector3f solveIntersection(const geometry_msgs::Vector3& pcurr, const geometry_msgs::Vector3& pdes, const obstacle& plane);
bool checkIntersection(const geometry_msgs::Vector3& pcurr, const geometry_msgs::Vector3& pdes, const std::vector<obstacle>& see_obs);
geometry_msgs::Vector3 projectOntoPlane(const geometry_msgs::Vector3& p, const obstacle& plane);
geometry_msgs::Vector3 solveProjection(const geometry_msgs::Vector3& pcurr, const geometry_msgs::Vector3& pdes, std::vector<obstacle>& see_obs);
void check_seeable(const std::vector<obstacle>& obs_list, const geometry_msgs::Vector3& pcurr, std::vector<obstacle>& see_obs);
void buildObstacles(std::vector<obstacle>& full_obs_list);


// -------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------------------


int main(int argc, char** argv)
{
    // ROS Initialization
    ros::init(argc, argv, "Projection_Node");
    ros::NodeHandle node;
    ros::Rate loop_rate(50);
    
    // Set up publisher and subscribers
    ros::Publisher  pdes_new_pub;
    ros::Subscriber pdes_old_sub;
    ros::Subscriber pcurr_sub;
    pdes_old_sub = node.subscribe("desired_position", 1, read_pdes);
    pcurr_sub = node.subscribe("current_position", 1, read_pcurr);
    pdes_new_pub = node.advertise<geometry_msgs::Vector3>("new_desired_pos", 1);

    // Obstacle list (full)
    std::vector<obstacle> full_obs_list;
    buildObstacles(full_obs_list);

    // Main loop to run each time step
    while(ros::ok()) {
        std::vector<obstacle> seeable_obs;              // declaration of list of seeable obstacles
        bool imminentCollision = false;                 // Set imminent collision to false initially
        check_seeable(full_obs_list,pcurr,seeable_obs); // Check normals to populate seeable obstacle list
        pdes_new.x = pdes_old.x;                        // Setting new pdes to the old one, will be updated if necessary
        pdes_new.y = pdes_old.y;
        pdes_new.z = pdes_old.z;
        if (seeable_obs.size() > 0)                     // Check if there are seeable obstacles
        {
            imminentCollision = checkIntersection(pcurr, pdes_old,seeable_obs);
            if (imminentCollision)                      // Check if imminent collision
            {
                pdes_new = solveProjection(pcurr,pdes_old,seeable_obs); // Solve for new, SAFE, pdes
            }
        }
        pdes_new_pub.publish(pdes_new);                 // Publish new desired position
        ros::spinOnce();                                // Publish/subscribe to variables
		loop_rate.sleep();                              // Wait if necessary to reach desired loop_rate
    }
}


// -------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------------------


// function to find dot product
double dot(const geometry_msgs::Vector3& A, const geometry_msgs::Vector3& B)
{
    return A.x*B.x + A.y*B.y + A.z*B.z;
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
    invA = A.inverse();
    return invA*b;
}


// Checks for an intersection between current position and desired position for all seeable obstacles
bool checkIntersection(const geometry_msgs::Vector3& pcurr, const geometry_msgs::Vector3& pdes, const std::vector<obstacle>& see_obs)
{
    Eigen::Vector3f x;
    for(int i = 0; i < see_obs.size(); i++)
    {
        x = solveIntersection(pcurr, pdes, see_obs[i]);
        if(x(2) <= 1 && x(2) >= 0 && x(3) <= 1 && x(3) >= 0 && x(2)+x(3) <= 1)
            return true;
    }
    return false;
}

geometry_msgs::Vector3 projectOntoPlane(const geometry_msgs::Vector3& p, const obstacle& plane)
{
    geometry_msgs::Vector3 p0;
    double D;
    p0 = p - plane.vertices[0];
    D = dot(p0,plane.normal);
    return p0 + D*plane.normal;
}


geometry_msgs::Vector3 solveProjection(const geometry_msgs::Vector3& pcurr, const geometry_msgs::Vector3& pdes, std::vector<obstacle>& see_obs)
{
    geometry_msgs::Vector3 pdes_new;    
    geometry_msgs::Vector3 p;
    double min_dist, dist;
    int    min_dist_index;

    pdes_new.x = pdes.x; pdes_new.y = pdes.y; pdes_new.z = pdes.z;
    p = pdes_new - see_obs[0].vertices[0];
    min_dist = dot(p,see_obs[0].normal);
    min_dist_index = 0;
    while(true)     // Loop until break
    {    
        for(int i = 0; i < see_obs.size(); i++) // Loop over see-able obstacles
        {
            if(see_obs[i].flag) // Check if plane has already been projected onto
            {
                p = pdes_new - see_obs[i].vertices[0];  // Vector from vertex to robot
                dist = dot(p,see_obs[i].normal);        // Distance from robot to plane
                if ( dist < min_dist)                   // Check if new dist is small
                {
                    min_dist = dist;                    // Reset min dist if it is
                    min_dist_index = i;                 // Update argmin if smaller
                }
            }
        }
        see_obs[min_dist_index].flag = false;           // Update flag so same plane isn't checked again
        pdes_new = projectOntoPlane(pdes_new,see_obs[min_dist_index]) + 0.001*see_obs[min_dist_index].normal; // Update pdes
        if(checkIntersection(pcurr,pdes_new,see_obs) == false)  // Check if new pdes intersect another see-able obstacle
            break;                                              // If no, break from while, else continue on to project again
    }
    for(int i = 0; i < see_obs.size(); i++)             // Reset flags for see-able obstacles
        see_obs[i].flag = true;                         // !!!!! May eliminate later. Seeable obstacles variable is initialized each time sot his may be an unnecessary step
    return pdes_new;                                    // Output new, SAFE pdes
}
  
      
// Builds full list of obstacles
void buildObstacles(std::vector<obstacle>& full_obs_list)
{
    // Build list of obstacles here.
    double width = 3.0;    // Width of enclosure (m)
    double height = 3.0;   // Height of enclosure (m)
    double length = 4.0;   // Length of enclosure (m)
    double radius = 0.25;   // Radius of quad sphere (m)
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
