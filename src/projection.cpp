#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Vector3.h>
#include <ardrone_autonomy/Navdata.h>

// Obstacle class: sets list of 3 vertices, normal vector, and a flag variable
class obstacle
{
    public:
    std::vector<geometry_msgs::Vector3> vertices;
    geometry_msgs::Vector3 normal;
    bool flag;
    obstacle() {flag = true;}
    obstacle(std::vector<geometry_msgs::Vector3> vertex, geometry_msgs::Vector3 norm)
    {
        for(int i = 0; i<3; i++) 
        {
            vertices[i].x = vertex[i].x;
            vertices[i].y = vertex[i].y;
            vertices[i].z = vertex[i].z;
        }
        normal.x = norm.x;
        normal.y = norm.y;
        normal.z = norm.z;
        flag = true;
    }
};

geometry_msgs::Vector3 pdes_old;
geometry_msgs::Vector3 pcurr;
geometry_msgs::Vector3 pdes_new;
  
// Reads in pdes from joystick
void read_pdes(const geometry_msgs::Vector3& pdes_in)
{
    pdes_old.x = pdes_in.x;
    pdes_old.y = pdes_in.y;
    pdes_old.z = pdes_in.z;
}

// Reads in pcurr from navdata
void read_pcurr(const geometry_msgs::Vector3& pcurr_in)
{
    pcurr.x = pcurr_in.x;
    pcurr.y = pcurr_in.y;
    pcurr.z = pcurr_in.z;
}

// function to find dot product
double dot(const geometry_msgs::Vector3& A, const geometry_msgs::Vector3& B)
{
    return A.x*B.x + A.y*B.y + A.z*B.z;
}
    
// Checks if an obstacle is see-able
void check_seeable(const std::vector<obstacle>& obs_list, const geometry_msgs::Vector3& pcurr, std::vector<obstacle>& see_obs)
{
    // AB = vector from vertex to robot
    // N  = normal of obstacle
    // If dot(AB,N) > 0, object is "see-able"
    // output list of obstacles
    geometry_msgs::Vector3 AB;
    for(int i = 0; i <  obs_list.size(); i++)
    {   
        AB.x = pcurr.x - obs_list[i].vertices[0].x;
        AB.y = pcurr.y - obs_list[i].vertices[0].y;
        AB.z = pcurr.z - obs_list[i].vertices[0].z;
        if (dot(AB,obs_list[i].normal) > 0)
            see_obs.push_back(obs_list[i]);
    }  
}

void buildObstacles(std::vector<obstacle>& full_obs_list);

int main(int argc, char** argv)
{
    /* ROS INITIALIZATION STUFF */
    ros::init(argc, argv, "ARDrone_Control");
    ros::NodeHandle node;
    ros::Rate loop_rate(50);

    ros::Publisher  pdes_new_pub;
    ros::Subscriber pdes_old_sub;
    ros::Subscriber pcurr_sub;

    pdes_old_sub = node.subscribe("desired_position", 1, read_pdes);
    pcurr_sub = node.subscribe("current_position", 1, read_pcurr);
    pdes_new_pub = node.advertise<geometry_msgs::Vector3>("new_pos", 1);

    std::vector<obstacle> full_obs_list;
    buildObstacles(full_obs_list);

    while(ros::ok()) {
        std::vector<obstacle> seeable_obs;
        bool imminentCollision = false;
        // Check normals
        check_seeable(full_obs_list,pcurr,seeable_obs);
        pdes_new.x = pdes_old.x;
        pdes_new.y = pdes_old.y;
        pdes_new.z = pdes_old.z;
        if (seeable_obs.size() > 0)
        {
            // Check for collision
            if (imminentCollision)
            {
                // Project until safe, choosing minimum distance and eliminating plane each time
                // Choose pdes_new
            }
        }
        pdes_new_pub.publish(pdes_new);
    }
}

void buildObstacles(std::vector<obstacle>& full_obs_list)
{
    // Build list of obstacles here.
    double width = 10.0;
    double height = 10.0;
    double length = 20.0;
    std::vector<geometry_msgs::Vector3> vertex;
    geometry_msgs::Vector3 normal;
    obstacle newObs;
    // Wall by computers
    newObs.vertices[0].x = width/2.0;
    newObs.vertices[0].y = -length/2.0;
    newObs.vertices[0].z = 0.0;
    newObs.vertices[1].x = -width/2.0;
    newObs.vertices[1].y = -length/2.0;
    newObs.vertices[1].z = 0.0;
    newObs.vertices[2].x = -width/2.0;
    newObs.vertices[2].y = -length/2.0;
    newObs.vertices[2].z = height;
    newObs.normal.x = 0.0;
    newObs.normal.y = 1.0;
    newObs.normal.z = 0.0;
    full_obs_list.push_back(newObs);
    newObs.vertices[0].x = -width/2.0;
    newObs.vertices[0].y = -length/2.0;
    newObs.vertices[0].z = height;
    newObs.vertices[1].x = width/2.0;
    newObs.vertices[1].y = -length/2.0;
    newObs.vertices[1].z = height;
    newObs.vertices[2].x = width/2.0;
    newObs.vertices[2].y = -length/2.0;
    newObs.vertices[2].z = 0.0;
    newObs.normal.x = 0.0;
    newObs.normal.y = 1.0;
    newObs.normal.z = 0.0;
    full_obs_list.push_back(newObs);
    // Wall to right of computers
    newObs.vertices[0].x = width/2.0;
    newObs.vertices[0].y = -length/2.0;
    newObs.vertices[0].z = 0.0;
    newObs.vertices[1].x = width/2.0;
    newObs.vertices[1].y = length/2.0;
    newObs.vertices[1].z = 0.0;
    newObs.vertices[2].x = width/2.0;
    newObs.vertices[2].y = -length/2.0;
    newObs.vertices[2].z = height;
    newObs.normal.x = -1.0;
    newObs.normal.y = 0.0;
    newObs.normal.z = 0.0;
    full_obs_list.push_back(newObs);
    newObs.vertices[0].x = width/2.0;
    newObs.vertices[0].y = -length/2.0;
    newObs.vertices[0].z = height;
    newObs.vertices[1].x = width/2.0;
    newObs.vertices[1].y = length/2.0;
    newObs.vertices[1].z = height;
    newObs.vertices[2].x = width/2.0;
    newObs.vertices[2].y = length/2.0;
    newObs.vertices[2].z = 0.0;
    newObs.normal.x = -1.0;
    newObs.normal.y = 0.0;
    newObs.normal.z = 0.0;
    full_obs_list.push_back(newObs);
    // Wall to left of computers
    newObs.vertices[0].x = -width/2.0;
    newObs.vertices[0].y = -length/2.0;
    newObs.vertices[0].z = 0.0;
    newObs.vertices[1].x = -width/2.0;
    newObs.vertices[1].y = length/2.0;
    newObs.vertices[1].z = 0.0;
    newObs.vertices[2].x = -width/2.0;
    newObs.vertices[2].y = -length/2.0;
    newObs.vertices[2].z = height;
    newObs.normal.x = 1.0;
    newObs.normal.y = 0.0;
    newObs.normal.z = 0.0;
    full_obs_list.push_back(newObs);
    newObs.vertices[0].x = -width/2.0;
    newObs.vertices[0].y = -length/2.0;
    newObs.vertices[0].z = height;
    newObs.vertices[1].x = -width/2.0;
    newObs.vertices[1].y = length/2.0;
    newObs.vertices[1].z = height;
    newObs.vertices[2].x = -width/2.0;
    newObs.vertices[2].y = length/2.0;
    newObs.vertices[2].z = 0.0;
    newObs.normal.x = 1.0;
    newObs.normal.y = 0.0;
    newObs.normal.z = 0.0;
    full_obs_list.push_back(newObs);
    // Wall opposite computers
    newObs.vertices[0].x = width/2.0;
    newObs.vertices[0].y = length/2.0;
    newObs.vertices[0].z = 0.0;
    newObs.vertices[1].x = -width/2.0;
    newObs.vertices[1].y = length/2.0;
    newObs.vertices[1].z = 0.0;
    newObs.vertices[2].x = -width/2.0;
    newObs.vertices[2].y = length/2.0;
    newObs.vertices[2].z = height;
    newObs.normal.x = 0.0;
    newObs.normal.y = -1.0;
    newObs.normal.z = 0.0;
    full_obs_list.push_back(newObs);
    newObs.vertices[0].x = -width/2.0;
    newObs.vertices[0].y = length/2.0;
    newObs.vertices[0].z = height;
    newObs.vertices[1].x = width/2.0;
    newObs.vertices[1].y = length/2.0;
    newObs.vertices[1].z = height;
    newObs.vertices[2].x = width/2.0;
    newObs.vertices[2].y = length/2.0;
    newObs.vertices[2].z = 0.0;
    newObs.normal.x = 0.0;
    newObs.normal.y = -1.0;
    newObs.normal.z = 0.0;
    full_obs_list.push_back(newObs);
    // Floor
    newObs.vertices[0].x = -width/2.0;
    newObs.vertices[0].y = -length/2.0;
    newObs.vertices[0].z = 0.0;
    newObs.vertices[1].x = width/2.0;
    newObs.vertices[1].y = -length/2.0;
    newObs.vertices[1].z = 0.0;
    newObs.vertices[2].x = -width/2.0;
    newObs.vertices[2].y = length/2.0;
    newObs.vertices[2].z = 0.0;
    newObs.normal.x = 0.0;
    newObs.normal.y = 0.0;
    newObs.normal.z = 1.0;
    full_obs_list.push_back(newObs);
    newObs.vertices[0].x = width/2.0;
    newObs.vertices[0].y = -length/2.0;
    newObs.vertices[0].z = 0.0;
    newObs.vertices[1].x = width/2.0;
    newObs.vertices[1].y = length/2.0;
    newObs.vertices[1].z = 0.0;
    newObs.vertices[2].x = -width/2.0;
    newObs.vertices[2].y = length/2.0;
    newObs.vertices[2].z = 0.0;
    newObs.normal.x = 0.0;
    newObs.normal.y = 0.0;
    newObs.normal.z = 1.0;
    full_obs_list.push_back(newObs);
    // Ceiling
        newObs.vertices[0].x = -width/2.0;
    newObs.vertices[0].y = -length/2.0;
    newObs.vertices[0].z = 0.0;
    newObs.vertices[1].x = width/2.0;
    newObs.vertices[1].y = -length/2.0;
    newObs.vertices[1].z = 0.0;
    newObs.vertices[2].x = -width/2.0;
    newObs.vertices[2].y = length/2.0;
    newObs.vertices[2].z = 0.0;
    newObs.normal.x = 0.0;
    newObs.normal.y = 0.0;
    newObs.normal.z = -1.0;
    full_obs_list.push_back(newObs);
    newObs.vertices[0].x = width/2.0;
    newObs.vertices[0].y = -length/2.0;
    newObs.vertices[0].z = 0.0;
    newObs.vertices[1].x = width/2.0;
    newObs.vertices[1].y = length/2.0;
    newObs.vertices[1].z = 0.0;
    newObs.vertices[2].x = -width/2.0;
    newObs.vertices[2].y = length/2.0;
    newObs.vertices[2].z = 0.0;
    newObs.normal.x = 0.0;
    newObs.normal.y = 0.0;
    newObs.normal.z = -1.0;
    full_obs_list.push_back(newObs);
}
    
