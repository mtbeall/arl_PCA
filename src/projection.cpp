/*
    Daman Bareiss
    Algorithmic Robotics Lab @ University of Utah
    
    This node takes in a desired position and outputs a new, SAFE desired position.  
*/

// Includes
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <math.h>

// Global variables for use in program
geometry_msgs::Vector3 pdes_old;	// Original desired position
geometry_msgs::Vector3 pdes_new;	// Updated desired position
geometry_msgs::Vector3 pcurr;		// Current position
geometry_msgs::Vector3 vcurr;		// Current velocity

// Define + operator for geometry_msgs::Vector3
geometry_msgs::Vector3 operator+(geometry_msgs::Vector3 A, geometry_msgs::Vector3 B) {
    geometry_msgs::Vector3 Sum;
    Sum.x = A.x + B.x;
    Sum.y = A.y + B.y;
    Sum.z = A.z + B.z;
    return Sum;
}

// Define - operator for geometry_msgs::Vector3
geometry_msgs::Vector3 operator-(geometry_msgs::Vector3 A, geometry_msgs::Vector3 B) {
    geometry_msgs::Vector3 Diff;
    Diff.x = A.x - B.x;
    Diff.y = A.y - B.y;
    Diff.z = A.z - B.z;
    return Diff;
}   

// Define * operator for geometry_msgs::Vector3
geometry_msgs::Vector3 operator*(double a, geometry_msgs::Vector3 A) {
    A.x = a*A.x;
    A.y = a*A.y;
    A.z = a*A.z;
    return A;
}

// Obstacle class: sets list of 3 vertices, normal vector, and a flag variable
class obstacle {
    public:
    	std::vector<geometry_msgs::Vector3> vertices; // List of 3 vertices
	    geometry_msgs::Vector3 normal;                // Normal vector
	    bool flag;                                    // Flag for later projection step
		double t;	// Intersection parameter
	    obstacle() // Default constructor with flag being set false
		{
			t = 120.0;	// Set t large at first
			vertices.resize(3); // set vertices std::vector to size 3
		}                     
	    obstacle(std::vector<geometry_msgs::Vector3> vertex, geometry_msgs::Vector3 norm) // Constructor setting all values
	    {
	        vertices.resize(3);	// set std::vector of vertices to size 3
			for(int i = 0; i<3; i++)  // Set the 3 vertices
	        {
				vertices[i].x = vertex[i].x;
				vertices[i].y = vertex[i].y;
				vertices[i].z = vertex[i].z;
	        }
	        normal.x = norm.x;  // Set the normal
	        normal.y = norm.y;
	        normal.z = norm.z;
			t = 120.0;			// initialize large t
    	}
};

  
// Reads in pdes from dynamics
void pdes_callback(const geometry_msgs::Vector3& pdes_in) {
    pdes_old.x = pdes_in.x;
    pdes_old.y = pdes_in.y;
    pdes_old.z = pdes_in.z;
}

// Reads in pcurr from mo-cap system
void pos_callback(const geometry_msgs::Vector3& pcurr_in) {
    pcurr.x = pcurr_in.x;
    pcurr.y = pcurr_in.y;
    pcurr.z = pcurr_in.z;
}

// Read in vcurr from mocap
void vel_callback(const geometry_msgs::Vector3& vcurr_in) {
    vcurr.x = vcurr_in.x;
    vcurr.y = vcurr_in.y;
    vcurr.z = vcurr_in.z;
} 

// Pre-declarations
double dot(const geometry_msgs::Vector3& A, const geometry_msgs::Vector3& B);
double findDistance(const geometry_msgs::Vector3& A, const obstacle& obs);
void checkSeeable(const std::vector<obstacle>& real_obs_list, const std::vector<obstacle>& trans_obs_list, const geometry_msgs::Vector3& pcurr, std::vector<obstacle>& see_obs);
Eigen::Vector3f solveIntersection(const geometry_msgs::Vector3& pcurr, const geometry_msgs::Vector3& pdes, const obstacle& plane);
geometry_msgs::Vector3 intersectLine(const geometry_msgs::Vector3& pcurr, const geometry_msgs::Vector3& pdes, const Eigen::Vector3f& t);
bool checkIntersection(const geometry_msgs::Vector3& pcurr, const geometry_msgs::Vector3& pdes, std::vector<obstacle>& see_obs);
geometry_msgs::Vector3 projectOntoPlane(const geometry_msgs::Vector3& p, const obstacle& plane);
geometry_msgs::Vector3 averageProject(const geometry_msgs::Vector3& intersect, const geometry_msgs::Vector3& project, const double& D, const obstacle& plane);
void buildRealObstacles(std::vector<obstacle>& real_obs_list);
void buildTranslatedObstacles(std::vector<obstacle>& full_obs_list);
double determ(const Eigen::MatrixXf& A);
double joyError = 0.01; // controller dead-zone

// Main
int main(int argc, char** argv) {
    // ROS Initialization
    ros::init(argc, argv, "Projection_Node");
    ros::NodeHandle node;
    ros::Rate loop_rate(50);
    
    // ROS publishers
    ros::Publisher  pdes_new_pub;
    pdes_new_pub = node.advertise<geometry_msgs::Vector3>("new_desired_position", 1);

    // ROS subscribers
    ros::Subscriber pdes_old_sub;
    pdes_old_sub = node.subscribe("desired_position", 1, pdes_callback);
    ros::Subscriber pcurr_sub;
    pcurr_sub = node.subscribe("current_position", 1, pos_callback);
    ros::Subscriber vcurr_sub;
    vcurr_sub = node.subscribe("current_velocity", 1, vel_callback);

    // real obstacle list
    std::vector<obstacle> real_obs_list;
    buildRealObstacles(real_obs_list);

	// translated obstacle list
	std::vector<obstacle> full_obs_list;
	buildTranslatedObstacles(full_obs_list);
    

    double t_min = 2.0;
    int t_argmin;
    while(ros::ok()) { // main loop
        ros::spinOnce();	// read in data
		std::vector<obstacle> seeable_obs;	// make seeable obstacles list
        bool imminentCollision = false;                 // Set imminent collision to false, initially
        checkSeeable(real_obs_list,full_obs_list,pcurr,seeable_obs); // Check normals to populate seeable obstacle list
		//ROS_INFO("Full Obstacle %d", full_obs_list.size());
		//ROS_INFO("Seeable Obstacle %d", seeable_obs.size());
        pdes_new.x = pdes_old.x;                        // Setting new pdes to the old one, will be updated if necessary
        pdes_new.y = pdes_old.y;
        pdes_new.z = pdes_old.z;
	    if (seeable_obs.size() > 0) {                    // Check if there are seeable obstacles
			//ROS_INFO("Checking for collision!!!");
	        while(true) {
				t_min = 100.0;
				imminentCollision = false;
	            imminentCollision = checkIntersection(pcurr,pdes_new,seeable_obs);
	            if (imminentCollision) {                      // Check if imminent collision
					//ROS_INFO("Collision");
	                for(int i = 0; i < seeable_obs.size(); i++) {
                        if(seeable_obs[i].t < t_min) {
                            t_min = seeable_obs[i].t;
                            t_argmin = i;
	                    }
	                }                 
	                geometry_msgs::Vector3 intersect;
	                intersect = intersectLine(pcurr,pdes_new,solveIntersection(pcurr, pdes_new, seeable_obs[t_argmin]));
	                geometry_msgs::Vector3 project;
	                project = projectOntoPlane(pdes_new,seeable_obs[t_argmin]);
	                double dist = findDistance(pcurr, seeable_obs[t_argmin]); //projectOntoPlane(pcurr,seeable_obs[t_argmin]));
	                pdes_new = averageProject(intersect, project, dist, seeable_obs[t_argmin]); 
					for(int i = 0; i < seeable_obs.size(); i++) {
						seeable_obs[i].t = 120.0;
					}
	            }
	            else {
					//ROS_ERROR("NO Collision");
	                break;
				}
	        }
			//ROS_ERROR("EXITING WHILE");
		}
        pdes_new_pub.publish(pdes_new);                 // Publish new desired position
		loop_rate.sleep();                              // Wait if necessary to reach desired loop_rate
    }
	ROS_ERROR("Exiting");
}

// function to find dot product of two geometry_msgs::Vector3's
double dot(const geometry_msgs::Vector3& A, const geometry_msgs::Vector3& B) {
    return A.x*B.x + A.y*B.y + A.z*B.z;
}

// Returns distance between two points
double findDistance(const geometry_msgs::Vector3& pc, const obstacle& obs) {
	geometry_msgs::Vector3 p;
	p = pc - obs.vertices[0];
	return abs(dot(p,obs.normal));
	//return dot(p,obs.normal);
}

// Checks if an obstacle is see-able
void checkSeeable(const std::vector<obstacle>& real_obs_list, const std::vector<obstacle>& trans_obs_list, const geometry_msgs::Vector3& pcurr, std::vector<obstacle>& see_obs) {
    geometry_msgs::Vector3 AB;
    for(int i = 0; i <  real_obs_list.size(); i++)
    {   
        AB = pcurr - real_obs_list[i].vertices[0];
        if (dot(AB,real_obs_list[i].normal) > 0.0)
            see_obs.push_back(trans_obs_list[i]);
    }
}

// Solve for t,u,v for intersection
Eigen::Vector3f solveIntersection(const geometry_msgs::Vector3& pcurr, const geometry_msgs::Vector3& pdes, const obstacle& plane) {
    Eigen::Matrix3f A;
    Eigen::Vector3f b;
	Eigen::Vector3f Answer;
    A(0,0) = pcurr.x - pdes.x;
    A(0,1) = plane.vertices[1].x - plane.vertices[0].x;
    A(0,2) = plane.vertices[2].x - plane.vertices[0].x;
    A(1,0) = pcurr.y - pdes.y;
    A(1,1) = plane.vertices[1].y - plane.vertices[0].y;
    A(1,2) = plane.vertices[2].y - plane.vertices[0].y;
    A(2,0) = pcurr.z - pdes.z;
    A(2,1) = plane.vertices[1].z - plane.vertices[0].z;
    A(2,2) = plane.vertices[2].z - plane.vertices[0].z;
	b(0) = pcurr.x - plane.vertices[0].x;
	b(1) = pcurr.y - plane.vertices[0].y;
	b(2) = pcurr.z - plane.vertices[0].z;
	if(A.determinant() < 0.001 && A.determinant() > -0.001)
	{
		Answer(0) = 5.0; Answer(1) = 5.0; Answer(2) = 5.0;
		return Answer;
	}
	Answer = A.inverse()*b;
	return Answer;
}

// Find the point a line intersects with a plane
geometry_msgs::Vector3 intersectLine(const geometry_msgs::Vector3& pcurr, const geometry_msgs::Vector3& pdes, const Eigen::Vector3f& tuv) {
    return pcurr + tuv(0)*(pdes - pcurr);
}

bool checkIntersection(const geometry_msgs::Vector3& pcurr, const geometry_msgs::Vector3& pdes, std::vector<obstacle>& see_obs) {
    Eigen::Vector3f x;
	bool flag = false;
    for(int i = 0; i < see_obs.size(); i++)
    {
        x = solveIntersection(pcurr, pdes, see_obs[i]);
        if(x(0) <= 1.0 && x(0) >= 0.0 && x(1) <= 1.0 && x(1) >= 0.0 && x(2) <= 1.0 && x(2) >= 0.0 && x(1)+x(2) <= 1.0 && dot(pcurr-see_obs[i].vertices[0],see_obs[i].normal)>0.0)
        {
            see_obs[i].t = x(0);
            flag = true;
			//return true;
        }
		else if(x(1) >= -1.0 && x(0) <= 0.0 && x(1) <= 1.0 && x(1) >= 0.0 && x(2) <= 1.0 && x(2) >= 0.0 && x(1)+x(2) <= 1.0 && dot(pcurr-see_obs[i].vertices[0],see_obs[i].normal)<0.0)
		{
            see_obs[i].t = x(0);
            flag = true;
			//return true;
        }
    }
    return flag;
}

// Projects a point onto a given plane through it's normal vector
geometry_msgs::Vector3 projectOntoPlane(const geometry_msgs::Vector3& pdes, const obstacle& plane) {
    geometry_msgs::Vector3 p0;
    double D;
    p0 = pdes - plane.vertices[0];
    D = dot(p0,plane.normal);
	return pdes - D*plane.normal;
}
    
// Finds a point on the line proportional to the intersection or projection point.
geometry_msgs::Vector3 averageProject(const geometry_msgs::Vector3& intersect, const geometry_msgs::Vector3& project, const double& D, const obstacle& plane) {
    float d = 1.0;
	float f = 0.01;
	float v = 1.0;

    geometry_msgs::Vector3 P;
    if (D<d) 
        P = ((d-D)/d) * (project - intersect) + intersect;
    else
        P = intersect;

	return P + f*plane.normal;
}

// Builds full list of obstacles
void buildRealObstacles(std::vector<obstacle>& full_obs_list) {
	// Build list of obstacles here.
	double width = 3.96;    // Width of enclosure (m)
	double height = 3.2;   // Height of enclosure (m)
	double length = 6.85;   // Length of enclosure (m)
	obstacle newObs;
	geometry_msgs::Vector3 vertex;
	// Wall by computers
	vertex.x = width/2.0;
	vertex.y = -length/2.0;
	vertex.z = 0.0;
	newObs.vertices[0] = vertex;
	vertex.x = -width/2.0;
	vertex.y = -length/2.0;
	vertex.z = 0.0;
	newObs.vertices[1] = vertex;
	vertex.x = -width/2.0;
	vertex.y = -length/2.0;
	vertex.z = height;
	newObs.vertices[2] = vertex;
	newObs.normal.x = 0.0;
	newObs.normal.y = 1.0;
	newObs.normal.z = 0.0;
	full_obs_list.push_back(newObs);
	vertex.x = -width/2.0;
	vertex.y = -length/2.0;
	vertex.z = height;
	newObs.vertices[0] = vertex;
	vertex.x = width/2.0;
	vertex.y = -length/2.0;
	vertex.z = height;
	newObs.vertices[1] = vertex;
	vertex.x = width/2.0;
	vertex.y = -length/2.0;
	vertex.z = 0.0;
	newObs.vertices[2] = vertex;
	newObs.normal.x = 0.0;
	newObs.normal.y = 1.0;
	newObs.normal.z = 0.0;
	full_obs_list.push_back(newObs);
	// Wall to right of computers
	vertex.x = width/2.0;
	vertex.y = -length/2.0;
	vertex.z = 0.0;
	newObs.vertices[0] = vertex;
	vertex.x = width/2.0;
	vertex.y = length/2.0;
	vertex.z = 0.0;
	newObs.vertices[1] = vertex;
	vertex.x = width/2.0;
	vertex.y = -length/2.0;
	vertex.z = height;
	newObs.vertices[2] = vertex;
	newObs.normal.x = -1.0;
	newObs.normal.y = 0.0;
	newObs.normal.z = 0.0;
	full_obs_list.push_back(newObs);
	vertex.x = width/2.0;
	vertex.y = -length/2.0;
	vertex.z = height;
	newObs.vertices[0] = vertex;
	vertex.x = width/2.0;
	vertex.y = length/2.0;
	vertex.z = height;
	newObs.vertices[1] = vertex;
	vertex.x = width/2.0;
	vertex.y = length/2.0;
	vertex.z = 0.0;
	newObs.vertices[2] = vertex;
	newObs.normal.x = -1.0;
	newObs.normal.y = 0.0;
	newObs.normal.z = 0.0;
	full_obs_list.push_back(newObs);
	// Wall to left of computers
	vertex.x = -width/2.0;
	vertex.y = -length/2.0;
	vertex.z = 0.0;
	newObs.vertices[0] = vertex;
	vertex.x = -width/2.0;
	vertex.y = length/2.0;
	vertex.z = 0.0;
	newObs.vertices[1] = vertex;
	vertex.x = -width/2.0;
	vertex.y = -length/2.0;
	vertex.z = height;
	newObs.vertices[2] = vertex;
	newObs.normal.x = 1.0;
	newObs.normal.y = 0.0;
	newObs.normal.z = 0.0;
	full_obs_list.push_back(newObs);
	vertex.x = -width/2.0;
	vertex.y = -length/2.0;
	vertex.z = height;
	newObs.vertices[0] = vertex;
	vertex.x = -width/2.0;
	vertex.y = length/2.0;
	vertex.z = height;
	newObs.vertices[1] = vertex;
	vertex.x = -width/2.0;
	vertex.y = length/2.0;
	vertex.z = 0.0;
	newObs.vertices[2] = vertex;
	newObs.normal.x = 1.0;
	newObs.normal.y = 0.0;
	newObs.normal.z = 0.0;
	full_obs_list.push_back(newObs);
	// Wall opposite computers
	vertex.x = width/2.0;
	vertex.y = length/2.0;
	vertex.z = 0.0;
	newObs.vertices[0] = vertex;
	vertex.x = -width/2.0;
	vertex.y = length/2.0;
	vertex.z = 0.0;
	newObs.vertices[1] = vertex;
	vertex.x = -width/2.0;
	vertex.y = length/2.0;
	vertex.z = height;
	newObs.vertices[2] = vertex;
	newObs.normal.x = 0.0;
	newObs.normal.y = -1.0;
	newObs.normal.z = 0.0;
	full_obs_list.push_back(newObs);
	vertex.x = -width/2.0;
	vertex.y = length/2.0;
	vertex.z = height;
	newObs.vertices[0] = vertex;
	vertex.x = width/2.0;
	vertex.y = length/2.0;
	vertex.z = height;
	newObs.vertices[1] = vertex;
	vertex.x = width/2.0;
	vertex.y = length/2.0;
	vertex.z = 0.0;
	newObs.vertices[2] = vertex;
	newObs.normal.x = 0.0;
	newObs.normal.y = -1.0;
	newObs.normal.z = 0.0;
	full_obs_list.push_back(newObs);
	// Floor
	vertex.x = -width/2.0;
	vertex.y = -length/2.0;
	vertex.z = 0.0;
	newObs.vertices[0] = vertex;
	vertex.x = width/2.0;
	vertex.y = -length/2.0;
	vertex.z = 0.0;
	newObs.vertices[1] = vertex;
	vertex.x = -width/2.0;
	vertex.y = length/2.0;
	vertex.z = 0.0;
	newObs.vertices[2] = vertex;
	newObs.normal.x = 0.0;
	newObs.normal.y = 0.0;
	newObs.normal.z = 1.0;
	full_obs_list.push_back(newObs);
	vertex.x = width/2.0;
	vertex.y = -length/2.0;
	vertex.z = 0.0;
	newObs.vertices[0] = vertex;
	vertex.x = width/2.0;
	vertex.y = length/2.0;
	vertex.z = 0.0;
	newObs.vertices[1] = vertex;
	vertex.x = -width/2.0;
	vertex.y = length/2.0;
	vertex.z = 0.0;
	newObs.vertices[2] = vertex;
	newObs.normal.x = 0.0;
	newObs.normal.y = 0.0;
	newObs.normal.z = 1.0;
	full_obs_list.push_back(newObs);
	// Ceiling
	vertex.x = -width/2.0;
	vertex.y = -length/2.0;
	vertex.z = height;
	newObs.vertices[0] = vertex;
	vertex.x = width/2.0;
	vertex.y = -length/2.0;
	vertex.z = height;
	newObs.vertices[1] = vertex;
	vertex.x = -width/2.0;
	vertex.y = length/2.0;
	vertex.z = height;
	newObs.vertices[2] = vertex;
	newObs.normal.x = 0.0;
	newObs.normal.y = 0.0;
	newObs.normal.z = -1.0;
	full_obs_list.push_back(newObs);
	vertex.x = width/2.0;
	vertex.y = -length/2.0;
	vertex.z = height;
	newObs.vertices[0] = vertex;
	vertex.x = width/2.0;
	vertex.y = length/2.0;
	vertex.z = height;
	newObs.vertices[1] = vertex;
	vertex.x = -width/2.0;
	vertex.y = length/2.0;
	vertex.z = height;
	newObs.vertices[2] = vertex;
	newObs.normal.x = 0.0;
	newObs.normal.y = 0.0;
	newObs.normal.z = -1.0;
	full_obs_list.push_back(newObs);
}

// Builds full list of obstacles
void buildTranslatedObstacles(std::vector<obstacle>& full_obs_list) {
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// I set the radius to 0 and the offset to 1 to get a better idea of where the walls are
	// Build list of obstacles here.
	double width = 3.96;    // Width of enclosure (m)
	double height = 3.2;   // Height of enclosure (m)
	//double length = 6.85;   // Length of enclosure (m)
	double length = 6.0;
	//double length = 6.0;
	double radius = 0.559/2.0;   // Radius of quad sphere (m)
	radius = 0.0*radius;
	/*double offsetX = 1.0;
	double offsetY = 1.5;
	double offsetZ = 0.5;*/
	double offset = 1.0 + radius;
	obstacle newObs;
	geometry_msgs::Vector3 vertex;
	
	// Wall by computers
	newObs.normal.x = 0.0;
	newObs.normal.y = 1.0;
	newObs.normal.z = 0.0;
	vertex.x = width/2.0;
	vertex.y = -length/2.0;
	vertex.z = 0.0;
	newObs.vertices[0] = vertex + offset*newObs.normal;
	vertex.x = -width/2.0;
	vertex.y = -length/2.0;
	vertex.z = 0.0;
	newObs.vertices[1] = vertex + offset*newObs.normal;
	vertex.x = -width/2.0;
	vertex.y = -length/2.0;
	vertex.z = height;
	newObs.vertices[2] = vertex + offset*newObs.normal;
	full_obs_list.push_back(newObs);
	vertex.x = -width/2.0;
	vertex.y = -length/2.0;
	vertex.z = height;
	newObs.vertices[0] = vertex + offset*newObs.normal;
	vertex.x = width/2.0;
	vertex.y = -length/2.0;
	vertex.z = height;
	newObs.vertices[1] = vertex + offset*newObs.normal;
	vertex.x = width/2.0;
	vertex.y = -length/2.0;
	vertex.z = 0.0;
	newObs.vertices[2] = vertex + offset*newObs.normal;
	full_obs_list.push_back(newObs);
	
	// Wall to right of computers
	newObs.normal.x = -1.0;
	newObs.normal.y = 0.0;
	newObs.normal.z = 0.0;
	vertex.x = width/2.0;
	vertex.y = -length/2.0;
	vertex.z = 0.0;
	newObs.vertices[0] = vertex + offset*newObs.normal;
	vertex.x = width/2.0;
	vertex.y = length/2.0;
	vertex.z = 0.0;
	newObs.vertices[1] = vertex + offset*newObs.normal;
	vertex.x = width/2.0;
	vertex.y = -length/2.0;
	vertex.z = height;
	newObs.vertices[2] = vertex + offset*newObs.normal;
	full_obs_list.push_back(newObs);
	vertex.x = width/2.0;
	vertex.y = -length/2.0;
	vertex.z = height;
	newObs.vertices[0] = vertex + offset*newObs.normal;
	vertex.x = width/2.0;
	vertex.y = length/2.0;
	vertex.z = height;
	newObs.vertices[1] = vertex + offset*newObs.normal;
	vertex.x = width/2.0;
	vertex.y = length/2.0;
	vertex.z = 0.0;
	newObs.vertices[2] = vertex + offset*newObs.normal;
	full_obs_list.push_back(newObs);
	
	// Wall to left of computers
	newObs.normal.x = 1.0;
	newObs.normal.y = 0.0;
	newObs.normal.z = 0.0;
	vertex.x = -width/2.0;
	vertex.y = -length/2.0;
	vertex.z = 0.0;
	newObs.vertices[0] = vertex + offset*newObs.normal;
	vertex.x = -width/2.0;
	vertex.y = length/2.0;
	vertex.z = 0.0;
	newObs.vertices[1] = vertex + offset*newObs.normal;
	vertex.x = -width/2.0;
	vertex.y = -length/2.0;
	vertex.z = height;
	newObs.vertices[2] = vertex + offset*newObs.normal;
	full_obs_list.push_back(newObs);
	vertex.x = -width/2.0;
	vertex.y = -length/2.0;
	vertex.z = height;
	newObs.vertices[0] = vertex + offset*newObs.normal;
	vertex.x = -width/2.0;
	vertex.y = length/2.0;
	vertex.z = height;
	newObs.vertices[1] = vertex + offset*newObs.normal;
	vertex.x = -width/2.0;
	vertex.y = length/2.0;
	vertex.z = 0.0;
	newObs.vertices[2] = vertex + offset*newObs.normal;
	full_obs_list.push_back(newObs);
	
	// Wall opposite computers
	offset = 1.5+radius;
	newObs.normal.x = 0.0;
	newObs.normal.y = -1.0;
	newObs.normal.z = 0.0;
	vertex.x = width/2.0;
	vertex.y = length/2.0;
	vertex.z = 0.0;
	newObs.vertices[0] = vertex + offset*newObs.normal;
	vertex.x = -width/2.0;
	vertex.y = length/2.0;
	vertex.z = 0.0;
	newObs.vertices[1] = vertex + offset*newObs.normal;
	vertex.x = -width/2.0;
	vertex.y = length/2.0;
	vertex.z = height;
	newObs.vertices[2] = vertex + offset*newObs.normal;
	full_obs_list.push_back(newObs);
	vertex.x = -width/2.0;
	vertex.y = length/2.0;
	vertex.z = height;
	newObs.vertices[0] = vertex + offset*newObs.normal;
	vertex.x = width/2.0;
	vertex.y = length/2.0;
	vertex.z = height;
	newObs.vertices[1] = vertex + offset*newObs.normal;
	vertex.x = width/2.0;
	vertex.y = length/2.0;
	vertex.z = 0.0;
	newObs.vertices[2] = vertex + offset*newObs.normal;
	full_obs_list.push_back(newObs);
	
	// Floor
	offset = 0.50+radius;
	newObs.normal.x = 0.0;
	newObs.normal.y = 0.0;
	newObs.normal.z = 1.0;
	vertex.x = -width/2.0;
	vertex.y = -length/2.0;
	vertex.z = 0.0;
	newObs.vertices[0] = vertex + offset*newObs.normal;
	vertex.x = width/2.0;
	vertex.y = -length/2.0;
	vertex.z = 0.0;
	newObs.vertices[1] = vertex + offset*newObs.normal;
	vertex.x = -width/2.0;
	vertex.y = length/2.0;
	vertex.z = 0.0;
	newObs.vertices[2] = vertex + offset*newObs.normal;
	full_obs_list.push_back(newObs);
	vertex.x = width/2.0;
	vertex.y = -length/2.0;
	vertex.z = 0.0;
	newObs.vertices[0] = vertex + offset*newObs.normal;
	vertex.x = width/2.0;
	vertex.y = length/2.0;
	vertex.z = 0.0;
	newObs.vertices[1] = vertex + offset*newObs.normal;
	vertex.x = -width/2.0;
	vertex.y = length/2.0;
	vertex.z = 0.0+radius;
	newObs.vertices[2] = vertex + offset*newObs.normal;
	full_obs_list.push_back(newObs);
	
	// Ceiling
	offset = radius + 1.5;
	newObs.normal.x = 0.0;
	newObs.normal.y = 0.0;
	newObs.normal.z = -1.0;
	vertex.x = -width/2.0;
	vertex.y = -length/2.0;
	vertex.z = height;
	newObs.vertices[0] = vertex + offset*newObs.normal;
	vertex.x = width/2.0;
	vertex.y = -length/2.0;
	vertex.z = height;
	newObs.vertices[1] = vertex + offset*newObs.normal;
	vertex.x = -width/2.0;
	vertex.y = length/2.0;
	vertex.z = height;
	newObs.vertices[2] = vertex + offset*newObs.normal;
	full_obs_list.push_back(newObs);
	vertex.x = width/2.0;
	vertex.y = -length/2.0;
	vertex.z = height;
	newObs.vertices[0] = vertex + offset*newObs.normal;
	vertex.x = width/2.0;
	vertex.y = length/2.0;
	vertex.z = height;
	newObs.vertices[1] = vertex + offset*newObs.normal;
	vertex.x = -width/2.0;
	vertex.y = length/2.0;
	vertex.z = height;
	newObs.vertices[2] = vertex + offset*newObs.normal;
	full_obs_list.push_back(newObs);
} 
