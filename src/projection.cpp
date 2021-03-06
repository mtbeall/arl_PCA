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

geometry_msgs::Vector3 cross(const geometry_msgs::Vector3& A, const geometry_msgs::Vector3& B);

double findDistance(const geometry_msgs::Vector3& A, const obstacle& obs);
void checkSeeable(const std::vector<obstacle>& real_obs_list, const std::vector<obstacle>& trans_obs_list, const geometry_msgs::Vector3& pcurr, std::vector<obstacle>& see_obs);
Eigen::Vector3f solveIntersection(const geometry_msgs::Vector3& pcurr, const geometry_msgs::Vector3& pdes, const obstacle& plane);
geometry_msgs::Vector3 intersectLine(const geometry_msgs::Vector3& pcurr, const geometry_msgs::Vector3& pdes, const Eigen::Vector3f& t);
bool checkIntersection(const geometry_msgs::Vector3& pcurr, const geometry_msgs::Vector3& pdes, std::vector<obstacle>& see_obs);
geometry_msgs::Vector3 projectOntoPlane(const geometry_msgs::Vector3& p, const obstacle& plane);
geometry_msgs::Vector3 averageProject(const geometry_msgs::Vector3& intersect, const geometry_msgs::Vector3& project, const double& D, const obstacle& plane);


void translate(std::vector<geometry_msgs::Vector3>& obsVerts, geometry_msgs::Vector3 trans);
void rotY(std::vector<geometry_msgs::Vector3>& obsVerts, double angle);
void rotZ(std::vector<geometry_msgs::Vector3>& obsVerts, double angle);
void createTriPrism(std::vector<geometry_msgs::Vector3>& obsVerts, double w, double l);
void buildTriPrism(std::vector<obstacle>& obs_list,std::vector<geometry_msgs::Vector3>& obsVerts);
void createRectPrism(std::vector<geometry_msgs::Vector3>& obsVerts, double h, double w, double l);
void buildRectPrism(std::vector<obstacle>& obs_list,std::vector<geometry_msgs::Vector3>& obsVerts);
void createHexPrism(std::vector<geometry_msgs::Vector3>& obsVerts, double w, double l);
void buildHexPrism(std::vector<obstacle>& obs_list,std::vector<geometry_msgs::Vector3>& obsVerts);
std::vector<geometry_msgs::Vector3> expandPrism(const std::vector<geometry_msgs::Vector3>& obsVerts, double buffer);
void buildInternalObs(std::vector<obstacle>& real_obs_list, std::vector<obstacle>& full_obs_list);

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
    std::vector<obstacle> full_obs_list;

    buildRealObstacles(real_obs_list);
    // translated obstacle list
    buildTranslatedObstacles(full_obs_list);
    buildInternalObs(real_obs_list, full_obs_list);
    //buildFullInternalObs(full_obs_list);
        

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
	                for(unsigned int i = 0; i < seeable_obs.size(); i++) {
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
					for(unsigned int i = 0; i < seeable_obs.size(); i++) {
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

// function to find cross product of two geometry_msgs::Vector3's
geometry_msgs::Vector3 cross(const geometry_msgs::Vector3& A, const geometry_msgs::Vector3& B) {
	geometry_msgs::Vector3 answer;
	answer.x = A.y*B.z - A.z*B.y;
	answer.y = A.z*B.x - A.x*B.z;
	answer.z = A.x*B.y - A.y*B.x;
	return answer;
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
    for(unsigned int i = 0; i <  real_obs_list.size(); i++)
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
    for(unsigned int i = 0; i < see_obs.size(); i++)
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



//Matt Beall
//takes a list of vertices that represent any internal obstacle and translates them as per the trans vector
void translate(std::vector<geometry_msgs::Vector3>& obsVerts, geometry_msgs::Vector3 trans){
	int numVerts = obsVerts.size();
	for(unsigned int i = 0; i<numVerts; i++){
		obsVerts[i].x += trans.x;
		obsVerts[i].y += trans.y;
		obsVerts[i].z += trans.z;
	}
}

//takes a list of vertices that represent any internal obstacle and rotates them about the y axis by angle in rads
void rotY(std::vector<geometry_msgs::Vector3>& obsVerts, double angle){
	int numVerts = obsVerts.size();
	geometry_msgs::Vector3 vertex;	
	for(int i = 0; i < numVerts; i++){
		vertex = obsVerts[i];
		obsVerts[i].x = cos(angle)*vertex.x + sin(angle)*vertex.z;
		obsVerts[i].y = vertex.y;
		obsVerts[i].z = -sin(angle)*vertex.x + cos(angle)*vertex.z;
	}
}

//takes a list of vertices that represent any internal obstacle and rotates them about the z axis by angle in rads
void rotZ(std::vector<geometry_msgs::Vector3>& obsVerts, double angle){
	int numVerts = obsVerts.size();
	geometry_msgs::Vector3 vertex;	
	for(int i = 0; i < numVerts; i++){
		vertex = obsVerts[i];
		obsVerts[i].x = cos(angle)*vertex.x - sin(angle)*vertex.y;
		obsVerts[i].y = sin(angle)*vertex.x + cos(angle)*vertex.y;
		obsVerts[i].z = vertex.z;
	}
}

// NO RESIZE
//create a triangular prism shaped internal obstacle. creates a list of vertices to then be manipulated.
void createTriPrism(std::vector<geometry_msgs::Vector3>& obsVerts, double w, double l){
	//Triangular Prism
	//    0__________3
	//   . .          . 
	//  1__ 2_____4____5
	//assumes equilateral. width is the triangle side (eg, 0-1) and length is the prism length (eg 0-3)
	double h = w*sqrt(3.0)/2.0;
	geometry_msgs::Vector3 vertex;
	
	
	vertex.x = 0.0;
	vertex.y = w/2.0;
	vertex.z = h;
	obsVerts.push_back(vertex);

	vertex.x = 0.0;
	vertex.y = w;
	vertex.z = 0.0;
	obsVerts.push_back(vertex);

	vertex.x = 0.0;
	vertex.y = 0.0;
	vertex.z = 0.0;
	obsVerts.push_back(vertex);
	
	vertex.x = l;
	vertex.y = w/2.0;
	vertex.z = h;
	obsVerts.push_back(vertex);
	
	vertex.x = l;
	vertex.y = w;
	vertex.z = 0.0;
	obsVerts.push_back(vertex);
	
	vertex.x = l;
	vertex.y = 0.0;
	vertex.z = 0.0;
	obsVerts.push_back(vertex);
	
}

//Takes a list of vertices that represent a triangular prism and turns them into triangle obstacles.
void buildTriPrism(std::vector<obstacle>& obs_list,std::vector<geometry_msgs::Vector3>& obsVerts){
	
	double mag;
	obstacle newObs;
	geometry_msgs::Vector3 vertex;
	

	newObs.vertices[0] = obsVerts[0];
	newObs.vertices[1] = obsVerts[1];
	newObs.vertices[2] = obsVerts[2];
	vertex = cross(newObs.vertices[1]-newObs.vertices[0],newObs.vertices[2]-newObs.vertices[0]);	
	mag = sqrt( pow(vertex.x,2.0) + pow(vertex.y,2.0) + pow(vertex.z,2.0) );
	newObs.normal.x = vertex.x/mag;
	newObs.normal.y = vertex.y/mag;
	newObs.normal.z = vertex.z/mag;
	obs_list.push_back(newObs);

	newObs.vertices[0] = obsVerts[0];
	newObs.vertices[1] = obsVerts[4];
	newObs.vertices[2] = obsVerts[1];
	vertex = cross(newObs.vertices[1]-newObs.vertices[0],newObs.vertices[2]-newObs.vertices[0]);	
	mag = sqrt( pow(vertex.x,2.0) + pow(vertex.y,2.0) + pow(vertex.z,2.0) );
	newObs.normal.x = vertex.x/mag;
	newObs.normal.y = vertex.y/mag;
	newObs.normal.z = vertex.z/mag;
	obs_list.push_back(newObs);
	
	newObs.vertices[0] = obsVerts[0];
	newObs.vertices[1] = obsVerts[3];
	newObs.vertices[2] = obsVerts[4];
	vertex = cross(newObs.vertices[1]-newObs.vertices[0],newObs.vertices[2]-newObs.vertices[0]);	
	mag = sqrt( pow(vertex.x,2.0) + pow(vertex.y,2.0) + pow(vertex.z,2.0) );
	newObs.normal.x = vertex.x/mag;
	newObs.normal.y = vertex.y/mag;
	newObs.normal.z = vertex.z/mag;
	obs_list.push_back(newObs);
	
	newObs.vertices[0] = obsVerts[0];
	newObs.vertices[1] = obsVerts[5];
	newObs.vertices[2] = obsVerts[3];
	vertex = cross(newObs.vertices[1]-newObs.vertices[0],newObs.vertices[2]-newObs.vertices[0]);	
	mag = sqrt( pow(vertex.x,2.0) + pow(vertex.y,2.0) + pow(vertex.z,2.0) );
	newObs.normal.x = vertex.x/mag;
	newObs.normal.y = vertex.y/mag;
	newObs.normal.z = vertex.z/mag; 
	obs_list.push_back(newObs);
	
	newObs.vertices[0] = obsVerts[0];
	newObs.vertices[1] = obsVerts[2];
	newObs.vertices[2] = obsVerts[5];
	vertex = cross(newObs.vertices[1]-newObs.vertices[0],newObs.vertices[2]-newObs.vertices[0]);	
	mag = sqrt( pow(vertex.x,2.0) + pow(vertex.y,2.0) + pow(vertex.z,2.0) );
	newObs.normal.x = vertex.x/mag;
	newObs.normal.y = vertex.y/mag;
	newObs.normal.z = vertex.z/mag;
	obs_list.push_back(newObs);
	
	newObs.vertices[0] = obsVerts[1];
	newObs.vertices[1] = obsVerts[4];
	newObs.vertices[2] = obsVerts[5];
	vertex = cross(newObs.vertices[1]-newObs.vertices[0],newObs.vertices[2]-newObs.vertices[0]);	
	mag = sqrt( pow(vertex.x,2.0) + pow(vertex.y,2.0) + pow(vertex.z,2.0) );
	newObs.normal.x = vertex.x/mag;
	newObs.normal.y = vertex.y/mag;
	newObs.normal.z = vertex.z/mag;
	obs_list.push_back(newObs);
	
	newObs.vertices[0] = obsVerts[1];
	newObs.vertices[1] = obsVerts[5];
	newObs.vertices[2] = obsVerts[2];
	vertex = cross(newObs.vertices[1]-newObs.vertices[0],newObs.vertices[2]-newObs.vertices[0]);	
	mag = sqrt( pow(vertex.x,2.0) + pow(vertex.y,2.0) + pow(vertex.z,2.0) );
	newObs.normal.x = vertex.x/mag;
	newObs.normal.y = vertex.y/mag;
	newObs.normal.z = vertex.z/mag;
	obs_list.push_back(newObs);
	
	newObs.vertices[0] = obsVerts[3];
	newObs.vertices[1] = obsVerts[5];
	newObs.vertices[2] = obsVerts[4];
	vertex = cross(newObs.vertices[1]-newObs.vertices[0],newObs.vertices[2]-newObs.vertices[0]);	
	mag = sqrt( pow(vertex.x,2.0) + pow(vertex.y,2.0) + pow(vertex.z,2.0) );
	newObs.normal.x = vertex.x/mag;
	newObs.normal.y = vertex.y/mag;
	newObs.normal.z = vertex.z/mag;
	obs_list.push_back(newObs);

}

//create a rectangular prism shaped internal obstacle. creates a list of vertices to then be manipulated.
void createRectPrism(std::vector<geometry_msgs::Vector3>& obsVerts, double h, double w, double l){
	//Rect Prism
	//   0___3 .....4 ___7
	//  h|   |      .    |
	//   1__ 2 .....5____6
	//     w      l
	geometry_msgs::Vector3 vertex;

	vertex.x = 0.0;
	vertex.y = w;
	vertex.z = h;
	obsVerts.push_back(vertex);

	vertex.x = 0.0;
	vertex.y = w;
	vertex.z = 0.0;
	obsVerts.push_back(vertex);

	vertex.x = 0.0;
	vertex.y = 0.0;
	vertex.z = 0.0;
	obsVerts.push_back(vertex);
	
	vertex.x = 0.0;
	vertex.y = 0.0;
	vertex.z = h;
	obsVerts.push_back(vertex);
	
	vertex.x = l;
	vertex.y = w;
	vertex.z = h;
	obsVerts.push_back(vertex);

	vertex.x = l;
	vertex.y = w;
	vertex.z = 0.0;
	obsVerts.push_back(vertex);
	
	vertex.x = l;
	vertex.y = 0.0;
	vertex.z = 0.0;
	obsVerts.push_back(vertex);

	vertex.x = l;
	vertex.y = 0.0;
	vertex.z = h;
	obsVerts.push_back(vertex);
	
}

//Takes a list of vertices that represent a rectangular prism and turns them into triangle obstacles.
void buildRectPrism(std::vector<obstacle>& obs_list,std::vector<geometry_msgs::Vector3>& obsVerts){
	
	double mag;
	obstacle newObs;
	geometry_msgs::Vector3 vertex;
	
	//Side 1
	newObs.vertices[0] = obsVerts[0];
	newObs.vertices[1] = obsVerts[1];
	newObs.vertices[2] = obsVerts[2];
	vertex = cross(newObs.vertices[1]-newObs.vertices[0],newObs.vertices[2]-newObs.vertices[0]);	
	mag = sqrt( pow(vertex.x,2.0) + pow(vertex.y,2.0) + pow(vertex.z,2.0) );
	newObs.normal.x = vertex.x/mag;
	newObs.normal.y = vertex.y/mag;
	newObs.normal.z = vertex.z/mag;
	obs_list.push_back(newObs);

	newObs.vertices[0] = obsVerts[0];
	newObs.vertices[1] = obsVerts[2];
	newObs.vertices[2] = obsVerts[3];
	vertex = cross(newObs.vertices[1]-newObs.vertices[0],newObs.vertices[2]-newObs.vertices[0]);	
	mag = sqrt( pow(vertex.x,2.0) + pow(vertex.y,2.0) + pow(vertex.z,2.0) );
	newObs.normal.x = vertex.x/mag;
	newObs.normal.y = vertex.y/mag;
	newObs.normal.z = vertex.z/mag;
	obs_list.push_back(newObs);

	newObs.vertices[0] = obsVerts[0];
	newObs.vertices[1] = obsVerts[1];
	newObs.vertices[2] = obsVerts[2];
	vertex = cross(newObs.vertices[1]-newObs.vertices[0],newObs.vertices[2]-newObs.vertices[0]);	
	mag = sqrt( pow(vertex.x,2.0) + pow(vertex.y,2.0) + pow(vertex.z,2.0) );
	newObs.normal.x = vertex.x/mag;
	newObs.normal.y = vertex.y/mag;
	newObs.normal.z = vertex.z/mag;
	obs_list.push_back(newObs);

	//Panels
	newObs.vertices[0] = obsVerts[0];
	newObs.vertices[1] = obsVerts[4];
	newObs.vertices[2] = obsVerts[5];
	vertex = cross(newObs.vertices[1]-newObs.vertices[0],newObs.vertices[2]-newObs.vertices[0]);	
	mag = sqrt( pow(vertex.x,2.0) + pow(vertex.y,2.0) + pow(vertex.z,2.0) );
	newObs.normal.x = vertex.x/mag;
	newObs.normal.y = vertex.y/mag;
	newObs.normal.z = vertex.z/mag;
	obs_list.push_back(newObs);

	newObs.vertices[0] = obsVerts[0];
	newObs.vertices[1] = obsVerts[5];
	newObs.vertices[2] = obsVerts[1];
	vertex = cross(newObs.vertices[1]-newObs.vertices[0],newObs.vertices[2]-newObs.vertices[0]);	
	mag = sqrt( pow(vertex.x,2.0) + pow(vertex.y,2.0) + pow(vertex.z,2.0) );
	newObs.normal.x = vertex.x/mag;
	newObs.normal.y = vertex.y/mag;
	newObs.normal.z = vertex.z/mag;
	obs_list.push_back(newObs);

	newObs.vertices[0] = obsVerts[1];
	newObs.vertices[1] = obsVerts[5];
	newObs.vertices[2] = obsVerts[6];
	vertex = cross(newObs.vertices[1]-newObs.vertices[0],newObs.vertices[2]-newObs.vertices[0]);	
	mag = sqrt( pow(vertex.x,2.0) + pow(vertex.y,2.0) + pow(vertex.z,2.0) );
	newObs.normal.x = vertex.x/mag;
	newObs.normal.y = vertex.y/mag;
	newObs.normal.z = vertex.z/mag;
	obs_list.push_back(newObs);

	newObs.vertices[0] = obsVerts[1];
	newObs.vertices[1] = obsVerts[6];
	newObs.vertices[2] = obsVerts[2];
	vertex = cross(newObs.vertices[1]-newObs.vertices[0],newObs.vertices[2]-newObs.vertices[0]);	
	mag = sqrt( pow(vertex.x,2.0) + pow(vertex.y,2.0) + pow(vertex.z,2.0) );
	newObs.normal.x = vertex.x/mag;
	newObs.normal.y = vertex.y/mag;
	newObs.normal.z = vertex.z/mag;
	obs_list.push_back(newObs);

	newObs.vertices[0] = obsVerts[2];
	newObs.vertices[1] = obsVerts[6];
	newObs.vertices[2] = obsVerts[7];
	vertex = cross(newObs.vertices[1]-newObs.vertices[0],newObs.vertices[2]-newObs.vertices[0]);	
	mag = sqrt( pow(vertex.x,2.0) + pow(vertex.y,2.0) + pow(vertex.z,2.0) );
	newObs.normal.x = vertex.x/mag;
	newObs.normal.y = vertex.y/mag;
	newObs.normal.z = vertex.z/mag;
	obs_list.push_back(newObs);

	newObs.vertices[0] = obsVerts[2];
	newObs.vertices[1] = obsVerts[7];
	newObs.vertices[2] = obsVerts[3];
	vertex = cross(newObs.vertices[1]-newObs.vertices[0],newObs.vertices[2]-newObs.vertices[0]);	
	mag = sqrt( pow(vertex.x,2.0) + pow(vertex.y,2.0) + pow(vertex.z,2.0) );
	newObs.normal.x = vertex.x/mag;
	newObs.normal.y = vertex.y/mag;
	newObs.normal.z = vertex.z/mag;
	obs_list.push_back(newObs);

	newObs.vertices[0] = obsVerts[3];
	newObs.vertices[1] = obsVerts[7];
	newObs.vertices[2] = obsVerts[4];
	vertex = cross(newObs.vertices[1]-newObs.vertices[0],newObs.vertices[2]-newObs.vertices[0]);	
	mag = sqrt( pow(vertex.x,2.0) + pow(vertex.y,2.0) + pow(vertex.z,2.0) );
	newObs.normal.x = vertex.x/mag;
	newObs.normal.y = vertex.y/mag;
	newObs.normal.z = vertex.z/mag;
	obs_list.push_back(newObs);

	newObs.vertices[0] = obsVerts[3];
	newObs.vertices[1] = obsVerts[4];
	newObs.vertices[2] = obsVerts[0];
	vertex = cross(newObs.vertices[1]-newObs.vertices[0],newObs.vertices[2]-newObs.vertices[0]);	
	mag = sqrt( pow(vertex.x,2.0) + pow(vertex.y,2.0) + pow(vertex.z,2.0) );
	newObs.normal.x = vertex.x/mag;
	newObs.normal.y = vertex.y/mag;
	newObs.normal.z = vertex.z/mag;
	obs_list.push_back(newObs);

	//Side 2
	newObs.vertices[0] = obsVerts[4];
	newObs.vertices[1] = obsVerts[7];
	newObs.vertices[2] = obsVerts[6];
	vertex = cross(newObs.vertices[1]-newObs.vertices[0],newObs.vertices[2]-newObs.vertices[0]);	
	mag = sqrt( pow(vertex.x,2.0) + pow(vertex.y,2.0) + pow(vertex.z,2.0) );
	newObs.normal.x = vertex.x/mag;
	newObs.normal.y = vertex.y/mag;
	newObs.normal.z = vertex.z/mag;
	obs_list.push_back(newObs);

	newObs.vertices[0] = obsVerts[4];
	newObs.vertices[1] = obsVerts[6];
	newObs.vertices[2] = obsVerts[5];
	vertex = cross(newObs.vertices[1]-newObs.vertices[0],newObs.vertices[2]-newObs.vertices[0]);	
	mag = sqrt( pow(vertex.x,2.0) + pow(vertex.y,2.0) + pow(vertex.z,2.0) );
	newObs.normal.x = vertex.x/mag;
	newObs.normal.y = vertex.y/mag;
	newObs.normal.z = vertex.z/mag;
	obs_list.push_back(newObs);
	
}

// NO RESIZE
//create an equilateral hexagonal prism shaped internal obstacle. creates a list of vertices to then be manipulated.
void createHexPrism(std::vector<geometry_msgs::Vector3>& obsVerts, double w, double l){
	//Hexagonal Prism
	//assumes equilateral. width is the hexagon side (eg, 0-1) and length is the prism length (eg 0-6)
	geometry_msgs::Vector3 vertex;
	
	//side 1
	vertex.x = 0.0;
	vertex.y = w/2.0;
	vertex.z = w*sqrt(3.0);
	obsVerts.push_back(vertex);
	
	vertex.x = 0.0;
	vertex.y = 0.0;
	vertex.z = w*sqrt(3.0)/2.0;
	obsVerts.push_back(vertex);
	
	vertex.x = 0.0;
	vertex.y = w/2.0;
	vertex.z = 0.0;
	obsVerts.push_back(vertex);

	vertex.x = 0.0;
	vertex.y = 3.0*w/2.0;
	vertex.z = 0.0;
	obsVerts.push_back(vertex);
	
	vertex.x = 0.0;
	vertex.y = w*2.0;
	vertex.z = w*sqrt(3.0)/2.0;
	obsVerts.push_back(vertex);
	
	vertex.x = 0.0;
	vertex.y = 3.0*w/2.0;
	vertex.z = w*sqrt(3.0);
	obsVerts.push_back(vertex);
	
	//side 2
	vertex.x = l;
	vertex.y = w/2.0;
	vertex.z = w*sqrt(3.0);
	obsVerts.push_back(vertex);

	vertex.x = l;
	vertex.y = 0.0;
	vertex.z = w*sqrt(3.0)/2.0;
	obsVerts.push_back(vertex);
	
	vertex.x = l;
	vertex.y = w/2.0;
	vertex.z = 0.0;
	obsVerts.push_back(vertex);

	vertex.x = l;
	vertex.y = 3.0*w/2.0;
	vertex.z = 0.0;
	obsVerts.push_back(vertex);
	
	vertex.x = l;
	vertex.y = w*2.0;
	vertex.z = w*sqrt(3.0)/2.0;
	obsVerts.push_back(vertex);
	
	vertex.x = l;
	vertex.y = 3.0*w/2.0;
	vertex.z = w*sqrt(3.0);
	obsVerts.push_back(vertex);

}

//Takes a list of vertices that represent an equilateral hexagonal prism and turns them into triangle obstacles.
void buildHexPrism(std::vector<obstacle>& obs_list,std::vector<geometry_msgs::Vector3>& obsVerts){
	
	double mag;
	obstacle newObs;
	geometry_msgs::Vector3 vertex;
	
	//side 1
	newObs.vertices[0] = obsVerts[0];
	newObs.vertices[1] = obsVerts[1];
	newObs.vertices[2] = obsVerts[2];
	vertex = cross(newObs.vertices[1]-newObs.vertices[0],newObs.vertices[2]-newObs.vertices[0]);	
	mag = sqrt( pow(vertex.x,2.0) + pow(vertex.y,2.0) + pow(vertex.z,2.0) );
	newObs.normal.x = vertex.x/mag;
	newObs.normal.y = vertex.y/mag;
	newObs.normal.z = vertex.z/mag;
	obs_list.push_back(newObs);
	
	newObs.vertices[0] = obsVerts[0];
	newObs.vertices[1] = obsVerts[2];
	newObs.vertices[2] = obsVerts[3];
	vertex = cross(newObs.vertices[1]-newObs.vertices[0],newObs.vertices[2]-newObs.vertices[0]);	
	mag = sqrt( pow(vertex.x,2.0) + pow(vertex.y,2.0) + pow(vertex.z,2.0) );
	newObs.normal.x = vertex.x/mag;
	newObs.normal.y = vertex.y/mag;
	newObs.normal.z = vertex.z/mag;
	obs_list.push_back(newObs);
	
	newObs.vertices[0] = obsVerts[0];
	newObs.vertices[1] = obsVerts[3];
	newObs.vertices[2] = obsVerts[4];
	vertex = cross(newObs.vertices[1]-newObs.vertices[0],newObs.vertices[2]-newObs.vertices[0]);	
	mag = sqrt( pow(vertex.x,2.0) + pow(vertex.y,2.0) + pow(vertex.z,2.0) );
	newObs.normal.x = vertex.x/mag;
	newObs.normal.y = vertex.y/mag;
	newObs.normal.z = vertex.z/mag;
	obs_list.push_back(newObs);
	
	newObs.vertices[0] = obsVerts[0];
	newObs.vertices[1] = obsVerts[4];
	newObs.vertices[2] = obsVerts[5];
	vertex = cross(newObs.vertices[1]-newObs.vertices[0],newObs.vertices[2]-newObs.vertices[0]);	
	mag = sqrt( pow(vertex.x,2.0) + pow(vertex.y,2.0) + pow(vertex.z,2.0) );
	newObs.normal.x = vertex.x/mag;
	newObs.normal.y = vertex.y/mag;
	newObs.normal.z = vertex.z/mag;
	obs_list.push_back(newObs);
	
	//Panels
	newObs.vertices[0] = obsVerts[0];
	newObs.vertices[1] = obsVerts[6];
	newObs.vertices[2] = obsVerts[7];
	vertex = cross(newObs.vertices[1]-newObs.vertices[0],newObs.vertices[2]-newObs.vertices[0]);	
	mag = sqrt( pow(vertex.x,2.0) + pow(vertex.y,2.0) + pow(vertex.z,2.0) );
	newObs.normal.x = vertex.x/mag;
	newObs.normal.y = vertex.y/mag;
	newObs.normal.z = vertex.z/mag;
	obs_list.push_back(newObs);
	
	newObs.vertices[0] = obsVerts[0];
	newObs.vertices[1] = obsVerts[7];
	newObs.vertices[2] = obsVerts[1];
	vertex = cross(newObs.vertices[1]-newObs.vertices[0],newObs.vertices[2]-newObs.vertices[0]);	
	mag = sqrt( pow(vertex.x,2.0) + pow(vertex.y,2.0) + pow(vertex.z,2.0) );
	newObs.normal.x = vertex.x/mag;
	newObs.normal.y = vertex.y/mag;
	newObs.normal.z = vertex.z/mag;
	obs_list.push_back(newObs);
	
	newObs.vertices[0] = obsVerts[1];
	newObs.vertices[1] = obsVerts[7];
	newObs.vertices[2] = obsVerts[8];
	vertex = cross(newObs.vertices[1]-newObs.vertices[0],newObs.vertices[2]-newObs.vertices[0]);	
	mag = sqrt( pow(vertex.x,2.0) + pow(vertex.y,2.0) + pow(vertex.z,2.0) );
	newObs.normal.x = vertex.x/mag;
	newObs.normal.y = vertex.y/mag;
	newObs.normal.z = vertex.z/mag;
	obs_list.push_back(newObs);
	
	newObs.vertices[0] = obsVerts[1];
	newObs.vertices[1] = obsVerts[8];
	newObs.vertices[2] = obsVerts[2];
	vertex = cross(newObs.vertices[1]-newObs.vertices[0],newObs.vertices[2]-newObs.vertices[0]);	
	mag = sqrt( pow(vertex.x,2.0) + pow(vertex.y,2.0) + pow(vertex.z,2.0) );
	newObs.normal.x = vertex.x/mag;
	newObs.normal.y = vertex.y/mag;
	newObs.normal.z = vertex.z/mag;
	obs_list.push_back(newObs);
	
	newObs.vertices[0] = obsVerts[2];
	newObs.vertices[1] = obsVerts[8];
	newObs.vertices[2] = obsVerts[9];
	vertex = cross(newObs.vertices[1]-newObs.vertices[0],newObs.vertices[2]-newObs.vertices[0]);	
	mag = sqrt( pow(vertex.x,2.0) + pow(vertex.y,2.0) + pow(vertex.z,2.0) );
	newObs.normal.x = vertex.x/mag;
	newObs.normal.y = vertex.y/mag;
	newObs.normal.z = vertex.z/mag;
	obs_list.push_back(newObs);
	
	newObs.vertices[0] = obsVerts[2];
	newObs.vertices[1] = obsVerts[9];
	newObs.vertices[2] = obsVerts[3];
	vertex = cross(newObs.vertices[1]-newObs.vertices[0],newObs.vertices[2]-newObs.vertices[0]);	
	mag = sqrt( pow(vertex.x,2.0) + pow(vertex.y,2.0) + pow(vertex.z,2.0) );
	newObs.normal.x = vertex.x/mag;
	newObs.normal.y = vertex.y/mag;
	newObs.normal.z = vertex.z/mag;
	obs_list.push_back(newObs);
	
	newObs.vertices[0] = obsVerts[3];
	newObs.vertices[1] = obsVerts[9];
	newObs.vertices[2] = obsVerts[10];
	vertex = cross(newObs.vertices[1]-newObs.vertices[0],newObs.vertices[2]-newObs.vertices[0]);	
	mag = sqrt( pow(vertex.x,2.0) + pow(vertex.y,2.0) + pow(vertex.z,2.0) );
	newObs.normal.x = vertex.x/mag;
	newObs.normal.y = vertex.y/mag;
	newObs.normal.z = vertex.z/mag;
	obs_list.push_back(newObs);
	
	newObs.vertices[0] = obsVerts[3];
	newObs.vertices[1] = obsVerts[10];
	newObs.vertices[2] = obsVerts[4];
	vertex = cross(newObs.vertices[1]-newObs.vertices[0],newObs.vertices[2]-newObs.vertices[0]);	
	mag = sqrt( pow(vertex.x,2.0) + pow(vertex.y,2.0) + pow(vertex.z,2.0) );
	newObs.normal.x = vertex.x/mag;
	newObs.normal.y = vertex.y/mag;
	newObs.normal.z = vertex.z/mag;
	obs_list.push_back(newObs);
	
	newObs.vertices[0] = obsVerts[4];
	newObs.vertices[1] = obsVerts[10];
	newObs.vertices[2] = obsVerts[11];
	vertex = cross(newObs.vertices[1]-newObs.vertices[0],newObs.vertices[2]-newObs.vertices[0]);	
	mag = sqrt( pow(vertex.x,2.0) + pow(vertex.y,2.0) + pow(vertex.z,2.0) );
	newObs.normal.x = vertex.x/mag;
	newObs.normal.y = vertex.y/mag;
	newObs.normal.z = vertex.z/mag;
	obs_list.push_back(newObs);
	
	newObs.vertices[0] = obsVerts[4];
	newObs.vertices[1] = obsVerts[11];
	newObs.vertices[2] = obsVerts[5];
	vertex = cross(newObs.vertices[1]-newObs.vertices[0],newObs.vertices[2]-newObs.vertices[0]);	
	mag = sqrt( pow(vertex.x,2.0) + pow(vertex.y,2.0) + pow(vertex.z,2.0) );
	newObs.normal.x = vertex.x/mag;
	newObs.normal.y = vertex.y/mag;
	newObs.normal.z = vertex.z/mag;
	obs_list.push_back(newObs);
	
	newObs.vertices[0] = obsVerts[5];
	newObs.vertices[1] = obsVerts[11];
	newObs.vertices[2] = obsVerts[6];
	vertex = cross(newObs.vertices[1]-newObs.vertices[0],newObs.vertices[2]-newObs.vertices[0]);	
	mag = sqrt( pow(vertex.x,2.0) + pow(vertex.y,2.0) + pow(vertex.z,2.0) );
	newObs.normal.x = vertex.x/mag;
	newObs.normal.y = vertex.y/mag;
	newObs.normal.z = vertex.z/mag;
	obs_list.push_back(newObs);
	
	newObs.vertices[0] = obsVerts[5];
	newObs.vertices[1] = obsVerts[6];
	newObs.vertices[2] = obsVerts[0];
	vertex = cross(newObs.vertices[1]-newObs.vertices[0],newObs.vertices[2]-newObs.vertices[0]);	
	mag = sqrt( pow(vertex.x,2.0) + pow(vertex.y,2.0) + pow(vertex.z,2.0) );
	newObs.normal.x = vertex.x/mag;
	newObs.normal.y = vertex.y/mag;
	newObs.normal.z = vertex.z/mag;
	obs_list.push_back(newObs);
	
	//Side 2
	newObs.vertices[0] = obsVerts[6];
	newObs.vertices[1] = obsVerts[11];
	newObs.vertices[2] = obsVerts[10];
	vertex = cross(newObs.vertices[1]-newObs.vertices[0],newObs.vertices[2]-newObs.vertices[0]);	
	mag = sqrt( pow(vertex.x,2.0) + pow(vertex.y,2.0) + pow(vertex.z,2.0) );
	newObs.normal.x = vertex.x/mag;
	newObs.normal.y = vertex.y/mag;
	newObs.normal.z = vertex.z/mag;
	obs_list.push_back(newObs);
	
	newObs.vertices[0] = obsVerts[6];
	newObs.vertices[1] = obsVerts[10];
	newObs.vertices[2] = obsVerts[9];
	vertex = cross(newObs.vertices[1]-newObs.vertices[0],newObs.vertices[2]-newObs.vertices[0]);	
	mag = sqrt( pow(vertex.x,2.0) + pow(vertex.y,2.0) + pow(vertex.z,2.0) );
	newObs.normal.x = vertex.x/mag;
	newObs.normal.y = vertex.y/mag;
	newObs.normal.z = vertex.z/mag;
	obs_list.push_back(newObs);
	
	newObs.vertices[0] = obsVerts[6];
	newObs.vertices[1] = obsVerts[9];
	newObs.vertices[2] = obsVerts[8];
	vertex = cross(newObs.vertices[1]-newObs.vertices[0],newObs.vertices[2]-newObs.vertices[0]);	
	mag = sqrt( pow(vertex.x,2.0) + pow(vertex.y,2.0) + pow(vertex.z,2.0) );
	newObs.normal.x = vertex.x/mag;
	newObs.normal.y = vertex.y/mag;
	newObs.normal.z = vertex.z/mag;
	obs_list.push_back(newObs);
	
	newObs.vertices[0] = obsVerts[6];
	newObs.vertices[1] = obsVerts[8];
	newObs.vertices[2] = obsVerts[7];
	vertex = cross(newObs.vertices[1]-newObs.vertices[0],newObs.vertices[2]-newObs.vertices[0]);	
	mag = sqrt( pow(vertex.x,2.0) + pow(vertex.y,2.0) + pow(vertex.z,2.0) );
	newObs.normal.x = vertex.x/mag;
	newObs.normal.y = vertex.y/mag;
	newObs.normal.z = vertex.z/mag;
	obs_list.push_back(newObs);

}


//takes a list of vertices that represent an internal obstacle (prism created with one of the prism functions
//and expands the vertices out from the centroid of each side, and then along the length.
std::vector<geometry_msgs::Vector3> expandPrism(const std::vector<geometry_msgs::Vector3>& obsVerts, double buffer){
	
	int numVerts, halfVerts;
	double mag;
	geometry_msgs::Vector3 vertex;
	geometry_msgs::Vector3 center, center1, center2, norm1, norm2, diff;
	std::vector<geometry_msgs::Vector3> obsVertsExpanded;
	//obsVertsExpanded.resize(obsVerts.size());
	center.x = 0.0;
	center.y = 0.0;
	center.z = 0.0;

	center1.x = 0.0;
	center1.y = 0.0;
	center1.z = 0.0;

	center2.x = 0.0;
	center2.y = 0.0;
	center2.z = 0.0;

	numVerts = obsVerts.size();
	halfVerts = numVerts/2;
	for(int i = 0; i<halfVerts; i++){
		center1 = center1 + obsVerts[i];
	}

	for(int i = halfVerts; i<numVerts;i++){
		center2 = center2 + obsVerts[i];

	}
	center1.x = center1.x/halfVerts;
	center1.y = center1.y/halfVerts;
	center1.z = center1.z/halfVerts;
	
	center2.x = center2.x/halfVerts;
	center2.y = center2.y/halfVerts;
	center2.z = center2.z/halfVerts;
	
	center.x = (center1.x + center2.x)/2.0;
	center.y = (center1.y + center2.y)/2.0;
	center.z = (center1.z + center2.z)/2.0;
	
	norm1 = center1 - center;
	mag = sqrt( pow(norm1.x,2.0) + pow(norm1.y,2.0) + pow(norm1.z,2.0) );
	norm1.x = (buffer/2.0)*(norm1.x/mag);
	norm1.y = (buffer/2.0)*(norm1.y/mag);
	norm1.z = (buffer/2.0)*(norm1.z/mag);

	norm2 = center2 - center;
	mag = sqrt( pow(norm2.x,2.0) + pow(norm2.y,2.0) + pow(norm2.z,2.0) );
	norm2.x = (buffer/2.0)*(norm2.x/mag);
	norm2.y = (buffer/2.0)*(norm2.y/mag);
	norm2.z = (buffer/2.0)*(norm2.z/mag);
	
	for(int i = 0; i<halfVerts; i++){
		diff = obsVerts[i] - center1;
		
		mag = sqrt( pow(diff.x,2.0) + pow(diff.y,2.0) + pow(diff.z,2.0) );
		diff.x = buffer*(diff.x/mag);
		diff.y = buffer*(diff.y/mag);
		diff.z = buffer*(diff.z/mag);
		
		//obsVertsExpanded[i] = obsVerts[i] + diff + norm1;
		obsVertsExpanded.push_back(obsVerts[i] + diff + norm1);
	}
	
	for(int i = halfVerts; i<numVerts; i++){
		diff = obsVerts[i] - center2;
		
		mag = sqrt( pow(diff.x,2.0) + pow(diff.y,2.0) + pow(diff.z,2.0) );
		diff.x = buffer*(diff.x/mag);
		diff.y = buffer*(diff.y/mag);
		diff.z = buffer*(diff.z/mag);
		
		//obsVertsExpanded[i] = obsVerts[i] + diff + norm2;
		obsVertsExpanded.push_back(obsVerts[i] + diff + norm2);
	}
	return obsVertsExpanded;

}
//adds real internal obstacle to obs list
void buildInternalObs(std::vector<obstacle>& real_obs_list, std::vector<obstacle>& full_obs_list){
	double angleY,angleZ, d, h, w, l, buffer;
	std::vector<geometry_msgs::Vector3> obsVerts, obsVertsExpanded;
	geometry_msgs::Vector3 vertex;
	geometry_msgs::Vector3 trans;
	obstacle newObs;		
	const double PI = 4.0*atan(1.0);
	
	//Triangular Prism
	/*
	w = .6;
	l =3.0;
	//l = 2.0;	
	trans.x = w/2.0;
	trans.y = 0.0;
	trans.z = 0.0;
	angleY = -PI/2.0;
	angleZ = PI/2.0;
	createTriPrism(obsVerts,w,l);
	
	rotY(obsVerts, angleY);
	rotZ(obsVerts, angleZ);
	translate(obsVerts,trans);
	
	buildTriPrism(real_obs_list, obsVerts);
	*/

	//Rectangular Prism
	
	h = 3.0;
	w = 1.0;
	l = 3.2;

	angleY = -PI/2.0;
	angleZ = PI/4.0;
	
	
	createRectPrism(obsVerts,h,w,l);
	
	trans.x = -l/2.0;
	trans.y = -w/2.0;
	trans.z = -h/2.0;
	translate(obsVerts,trans);

	rotY(obsVerts,angleY);
	rotZ(obsVerts,angleZ);
	
	trans.x = 1.2;
	trans.y = 0.75;
	trans.z = l/2.0;
	translate(obsVerts,trans);	
	
	buildRectPrism(real_obs_list, obsVerts);
	
	obsVertsExpanded = expandPrism(obsVerts, sqrt(2.0)*0.5);
	
	buildRectPrism(full_obs_list, obsVertsExpanded);
	

	//Hard Coded - rectangular column in middle of room
	/*
	//REAL OBS LIST
	//h = 0.3937;
	//w = 0.4572;
	h = 5.0;
	w = 1.0;		
	l = 3.2;	
	//facing computers	
	vertex.x = -h/2.0;
	vertex.y = -w/2.0;
	vertex.z = l;
	newObs.vertices[0] = vertex;
	vertex.x = -h/2.0;
	vertex.y = -w/2.0;
	vertex.z = 0.0;
	newObs.vertices[1] = vertex;
	vertex.x = h/2.0;
	vertex.y = -w/2.0;
	vertex.z = l;
	newObs.vertices[2] = vertex;
	newObs.normal.x = 0.0;
	newObs.normal.y = -1.0;
	newObs.normal.z = 0.0;
	real_obs_list.push_back(newObs);
	vertex.x = h/2.0;
	vertex.y = -w/2.0;
	vertex.z = l;
	newObs.vertices[0] = vertex;
	vertex.x = -h/2.0;
	vertex.y = -w/2.0;
	vertex.z = 0.0;
	newObs.vertices[1] = vertex;
	vertex.x = h/2.0;
	vertex.y = -w/2.0;
	vertex.z = 0.0;
	newObs.vertices[2] = vertex;
	newObs.normal.x = 0.0;
	newObs.normal.y = -1.0;
	newObs.normal.z = 0.0;
	real_obs_list.push_back(newObs);
	//facing door
	vertex.x = -h/2.0;
	vertex.y = w/2.0;
	vertex.z = l;
	newObs.vertices[0] = vertex;
	vertex.x = -h/2.0;
	vertex.y = w/2.0;
	vertex.z = 0.0;
	newObs.vertices[1] = vertex;
	vertex.x = h/2.0;
	vertex.y = w/2.0;
	vertex.z = l;
	newObs.vertices[2] = vertex;
	newObs.normal.x = 0.0;
	newObs.normal.y = 1.0;
	newObs.normal.z = 0.0;
	real_obs_list.push_back(newObs);
	vertex.x = h/2.0;
	vertex.y = w/2.0;
	vertex.z = l;
	newObs.vertices[0] = vertex;
	vertex.x = -h/2.0;
	vertex.y = w/2.0;
	vertex.z = 0.0;
	newObs.vertices[1] = vertex;
	vertex.x = h/2.0;
	vertex.y = w/2.0;
	vertex.z = 0.0;
	newObs.vertices[2] = vertex;
	newObs.normal.x = 0.0;
	newObs.normal.y = 1.0;
	newObs.normal.z = 0.0;
	real_obs_list.push_back(newObs);
	//facing right
	vertex.x = h/2.0;
	vertex.y = -w/2.0;
	vertex.z = l;
	newObs.vertices[0] = vertex;
	vertex.x = h/2.0;
	vertex.y = -w/2.0;
	vertex.z = 0.0;
	newObs.vertices[1] = vertex;
	vertex.x = h/2.0;
	vertex.y = w/2.0;
	vertex.z = l;
	newObs.vertices[2] = vertex;
	newObs.normal.x = 1.0;
	newObs.normal.y = 0.0;
	newObs.normal.z = 0.0;
	real_obs_list.push_back(newObs);
	vertex.x = h/2.0;
	vertex.y = w/2.0;
	vertex.z = l;
	newObs.vertices[0] = vertex;
	vertex.x = h/2.0;
	vertex.y = w/2.0;
	vertex.z = 0.0;
	newObs.vertices[1] = vertex;
	vertex.x = h/2.0;
	vertex.y = -w/2.0;
	vertex.z = 0.0;
	newObs.vertices[2] = vertex;
	newObs.normal.x = 1.0;
	newObs.normal.y = 0.0;
	newObs.normal.z = 0.0;
	real_obs_list.push_back(newObs);
	//facing left
	vertex.x = -h/2.0;
	vertex.y = -w/2.0;
	vertex.z = l;
	newObs.vertices[0] = vertex;
	vertex.x = -h/2.0;
	vertex.y = -w/2.0;
	vertex.z = 0.0;
	newObs.vertices[1] = vertex;
	vertex.x = -h/2.0;
	vertex.y = w/2.0;
	vertex.z = l;
	newObs.vertices[2] = vertex;
	newObs.normal.x = -1.0;
	newObs.normal.y = 0.0;
	newObs.normal.z = 0.0;
	real_obs_list.push_back(newObs);
	vertex.x = -h/2.0;
	vertex.y = w/2.0;
	vertex.z = l;
	newObs.vertices[0] = vertex;
	vertex.x = -h/2.0;
	vertex.y = w/2.0;
	vertex.z = 0.0;
	newObs.vertices[1] = vertex;
	vertex.x = -h/2.0;
	vertex.y = -w/2.0;
	vertex.z = 0.0;
	newObs.vertices[2] = vertex;
	newObs.normal.x = -1.0;
	newObs.normal.y = 0.0;
	newObs.normal.z = 0.0;
	real_obs_list.push_back(newObs);
		
	//FULL OBS LIST
		
	buffer = 0.308;
	h = h + buffer;
	w = w + buffer;
	//facing computers
	vertex.x = -h/2.0;
	vertex.y = -w/2.0;
	vertex.z = l;
	newObs.vertices[0] = vertex;
	vertex.x = -h/2.0;
	vertex.y = -w/2.0;
	vertex.z = 0.0;
	newObs.vertices[1] = vertex;
	vertex.x = h/2.0;
	vertex.y = -w/2.0;
	vertex.z = l;
	newObs.vertices[2] = vertex;
	newObs.normal.x = 0.0;
	newObs.normal.y = -1.0;
	newObs.normal.z = 0.0;
	full_obs_list.push_back(newObs);
	vertex.x = h/2.0;
	vertex.y = -w/2.0;
	vertex.z = l;
	newObs.vertices[0] = vertex;
	vertex.x = -h/2.0;
	vertex.y = -w/2.0;
	vertex.z = 0.0;
	newObs.vertices[1] = vertex;
	vertex.x = h/2.0;
	vertex.y = -w/2.0;
	vertex.z = 0.0;
	newObs.vertices[2] = vertex;
	newObs.normal.x = 0.0;
	newObs.normal.y = -1.0;
	newObs.normal.z = 0.0;
	full_obs_list.push_back(newObs);
	//facing door
	vertex.x = -h/2.0;
	vertex.y = w/2.0;
	vertex.z = l;
	newObs.vertices[0] = vertex;
	vertex.x = -h/2.0;
	vertex.y = w/2.0;
	vertex.z = 0.0;
	newObs.vertices[1] = vertex;
	vertex.x = h/2.0;
	vertex.y = w/2.0;
	vertex.z = l;
	newObs.vertices[2] = vertex;
	newObs.normal.x = 0.0;
	newObs.normal.y = 1.0;
	newObs.normal.z = 0.0;
	full_obs_list.push_back(newObs);
	vertex.x = h/2.0;
	vertex.y = w/2.0;
	vertex.z = l;
	newObs.vertices[0] = vertex;
	vertex.x = -h/2.0;
	vertex.y = w/2.0;
	vertex.z = 0.0;
	newObs.vertices[1] = vertex;
	vertex.x = h/2.0;
	vertex.y = w/2.0;
	vertex.z = 0.0;
	newObs.vertices[2] = vertex;
	newObs.normal.x = 0.0;
	newObs.normal.y = 1.0;
	newObs.normal.z = 0.0;
	full_obs_list.push_back(newObs);
	//facing right
	vertex.x = h/2.0;
	vertex.y = -w/2.0;
	vertex.z = l;
	newObs.vertices[0] = vertex;
	vertex.x = h/2.0;
	vertex.y = -w/2.0;
	vertex.z = 0.0;
	newObs.vertices
[1] = vertex;
	vertex.x = h/2.0;
	vertex.y = w/2.0;
	vertex.z = l;
	newObs.vertices[2] = vertex;
	newObs.normal.x = 1.0;
	newObs.normal.y = 0.0;
	newObs.normal.z = 0.0;
	full_obs_list.push_back(newObs);
	vertex.x = h/2.0;
	vertex.y = w/2.0;
	vertex.z = l;
	newObs.vertices[0] = vertex;
	vertex.x = h/2.0;
	vertex.y = w/2.0;
	vertex.z = 0.0;
	newObs.vertices[1] = vertex;
	vertex.x = h/2.0;
	vertex.y = -w/2.0;
	vertex.z = 0.0;
	newObs.vertices[2] = vertex;
	newObs.normal.x = 1.0;
	newObs.normal.y = 0.0;
	newObs.normal.z = 0.0;
	full_obs_list.push_back(newObs);
	//facing left
	vertex.x = -h/2.0;
	vertex.y = -w/2.0;
	vertex.z = l;
	newObs.vertices[0] = vertex;
	vertex.x = -h/2.0;
	vertex.y = -w/2.0;
	vertex.z = 0.0;
	newObs.vertices[1] = vertex;
	vertex.x = -h/2.0;
	vertex.y = w/2.0;
	vertex.z = l;
	newObs.vertices[2] = vertex;
	newObs.normal.x = -1.0;
	newObs.normal.y = 0.0;
	newObs.normal.z = 0.0;
	full_obs_list.push_back(newObs);
	vertex.x = -h/2.0;
	vertex.y = w/2.0;
	vertex.z = l;
	newObs.vertices[0] = vertex;
	vertex.x = -h/2.0;
	vertex.y = w/2.0;
	vertex.z = 0.0;
	newObs.vertices[1] = vertex;
	vertex.x = -h/2.0;
	vertex.y = -w/2.0;
	vertex.z = 0.0;
	newObs.vertices[2] = vertex;
	newObs.normal.x = -1.0;
	newObs.normal.y = 0.0;
	newObs.normal.z = 0.0;
	full_obs_list.push_back(newObs);
	*/
	
	//Hexagonal Prism
	/*d = .5;
	w = d/sqrt(3.0);
	l = 2;
	angleY = -PI/2.0;
	angleZ = 0.0;
	trans.x = sqrt(3.0)/2.0;
	trans.y = 0.0;
	trans.z = 0.0;
	
	createHexPrism(obsVerts,w,l);
	
	rotY(obsVerts,angleY);
	rotZ(obsVerts,angleZ);
	translate(obsVerts,trans);
	
	buildHexPrism(real_obs_list, obsVerts);
	
	expandPrism(obsVerts,0.2);
	
	buildHexPrism(full_obs_list, obsVerts);
	*/

}

//end Matt Beall


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
	offset = 0.3+radius;
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
