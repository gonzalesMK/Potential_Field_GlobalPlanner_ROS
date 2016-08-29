#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <cmath>
#include <string>


/** include ros libraries**********************/
#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
/** ********************************************/

#include <boost/foreach.hpp>
//# define forEach BOOST_FOREACH

/** for global path planner interface */
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>

//#include <pcl_conversions/pcl_conversions.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

#include <set>

using namespace std;
using std::string;


#ifndef potentialField_H
#define potentialField_H

// A struct to represent cells and it cost
struct cells {
    int currentCell;
    float cost;

};

namespace potential_field {

class potentialField : public nav_core::BaseGlobalPlanner {
public:

    potentialField();                // default
    potentialField( std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    potentialField( ros::NodeHandle &nh);
    ros::NodeHandle ROSNodeHandle;

    // Override classes from interface nav_core::BaseGlobalPlanner
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

    // Potential Field's functions
    std::vector<int> potentialPlanner(int startCell, int goalCell);
    std::vector<int> findPath(int startCell, int goalCell);
    void calculateGoalPotential(int goalCell) ;
    void calculateObstaclePotential();


    // Useful functions
    void getCoordinate (float& x, float& y);
    int convertToCellIndex(float x, float y);
    void convertToCoordinate(int index, float& x, float& y);
    bool isCellInsideMap(float x, float y);
    void mapToWorld(double mx, double my, double& wx, double& wy);
    bool isStartAndGoalCellsValid(int startCell,int goalCell);
    bool isFree(int cellID);
    bool isFree(float x, float y);
    float getDistance(float x1, float y1, float x2, float y2);
    void sumPointPotential( int cellID);
    int findBestCell(int cellID);
    int getCellIndex(int i, int j);
    int getCellRowID(int index);
    int getCellColID(int index);
    void writeCostmap();
    void writePath( std::vector<int> path);
    bool isGoalAcomplished( int currentCell , int goalCell);





    float _originX;
    float _originY;
    float _resolution;
    costmap_2d::Costmap2DROS* _costmap_ros;
    double _step_size, _min_dist_from_robot;
    costmap_2d::Costmap2D* _costmap;
    //base_local_planner::WorldModel* world_model_;
    bool _initialized;
    int _width;
    int _height;
    float* _posPotMap; //position potential map;
    float* _obsPotMap; // obstacle potential map;
};

};
#endif // potentialField_H
