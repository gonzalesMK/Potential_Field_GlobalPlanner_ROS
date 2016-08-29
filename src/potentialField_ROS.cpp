#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>

#include <pluginlib/class_list_macros.h>
#include "potentialField_ROS.h"

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS( potential_field::potentialField, nav_core::BaseGlobalPlanner)

using namespace std;

int value_;
int mapSize_;
bool* OGM_;
const float RAIO = 1 ; // Raio do Potencial de um obstáculo
const float JUMP = 1 ;
const float C = 3;      // Constante da Soma do Potencial de um obstáculo
const float GOALPOTENTIALMULTIPLIER = 100;
const float WALK =  2 ; // quantas cells pular
const float GOALERROR = 0.3 ;// distancia da goal para ser considerado atiginda
const float DELTAERROR = 20 ;
static const float INFINIT_COST = INT_MAX; //!< cost of non connected nodes
float infinity_ = std::numeric_limits< float >::infinity();
float tBreak_;  // coefficient for breaking ties
ofstream MyExcelFile ("PF_result.xls", ios::trunc);

int clock_gettime(clockid_t clk_id, struct timespect *tp);

timespec diff(timespec start, timespec end)
{
  timespec temp;
    if ((end.tv_nsec-start.tv_nsec)<0) {
        temp.tv_sec = end.tv_sec-start.tv_sec-1;
        temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
    } else {
        temp.tv_sec = end.tv_sec-start.tv_sec;
        temp.tv_nsec = end.tv_nsec-start.tv_nsec;
    }
    return temp;
}

namespace potential_field
{

// Default Constructor
potentialField::potentialField()
{

}
// Using Handle
potentialField::potentialField( ros::NodeHandle &nh )
{
  ROSNodeHandle = nh;
}

// ROS default constructor
potentialField::potentialField(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  initialize(name, costmap_ros);
}

// This is a virtual function: it creates the plan
void potentialField::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    if (!_initialized)
    {

       _costmap_ros = costmap_ros;
      _costmap = _costmap_ros->getCostmap();

      ros::NodeHandle private_nh("~/" + name);

      _originX = _costmap->getOriginX();
      _originY = _costmap->getOriginY();

      _width = _costmap->getSizeInCellsX();
      _height = _costmap->getSizeInCellsY();
      _resolution = _costmap->getResolution();
      mapSize_ = _width*_height;
      tBreak_ = 1+1/(mapSize_);
      value_ =0;
       _posPotMap = new float [mapSize_];
       _obsPotMap = new float [mapSize_];

      OGM_ = new bool [mapSize_];
      for ( unsigned int iy = 0; iy < _costmap->getSizeInCellsY(); iy++)
      {
        for (unsigned int ix = 0; ix < _costmap->getSizeInCellsX(); ix++)
        {
          unsigned int cost = static_cast<int>(_costmap->getCost(ix, iy));

          if (cost < DELTAERROR )
            OGM_[iy*_width+ix]=true;
          else
            OGM_[iy*_width+ix]=false;
        }
      }

      calculateObstaclePotential();

      ROS_INFO(" Potential planner initialized successfully - Obstacle Potential Already Calculated");
      MyExcelFile << "StartID\tStartX\tStartY\tGoalID\tGoalX\tGoalY\tPlannertime(ms)\tpathLength\tnumberOfCells\t" << endl;
      _initialized = true;
    }
    else
      ROS_WARN("This planner has already been initialized... doing nothing");
  }



 bool potentialField::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
 {
     if (!_initialized)
     {
       ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
       return false;
     }

     ROS_INFO("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);


     plan.clear();

     if (goal.header.frame_id != _costmap_ros->getGlobalFrameID())
     {
       ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.", _costmap_ros->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
       return false;
     }

     tf::Stamped < tf::Pose > goal_tf;
     tf::Stamped < tf::Pose > start_tf;

     poseStampedMsgToTF(goal, goal_tf);
     poseStampedMsgToTF(start, start_tf);

     // convert the start and goal positions

     float startX = start.pose.position.x;
     float startY = start.pose.position.y;

     float goalX = goal.pose.position.x;
     float goalY = goal.pose.position.y;

     getCoordinate(startX, startY);
     getCoordinate(goalX, goalY);

     int startCell;
     int goalCell;

     if (isCellInsideMap(startX, startY) && isCellInsideMap(goalX, goalY))
     {
       startCell = convertToCellIndex(startX, startY);
       goalCell = convertToCellIndex(goalX, goalY);

       MyExcelFile << startCell <<"\t"<< start.pose.position.x <<"\t"<< start.pose.position.y <<"\t"<< goalCell <<"\t"<< goal.pose.position.x <<"\t"<< goal.pose.position.y;
     }
    else
     {
       ROS_WARN("the start or goal is out of the map");
       return false;
     }

     /////////////////////////////////////////////////////////

     // call global planner

     if (isStartAndGoalCellsValid(startCell, goalCell)){
       vector<int> bestPath;
       bestPath.clear();

       bestPath = potentialPlanner(startCell, goalCell); // !!
       writePath(bestPath);

     //if the global planner find a path ...
       if ( bestPath.size()>0)
       {
         // convert the path
         for (int i = 0; i < bestPath.size(); i++)
         {
           float x = 0.0;
           float y = 0.0;
            int index = bestPath[i];

           convertToCoordinate(index, x, y);
           geometry_msgs::PoseStamped pose = goal;

           pose.pose.position.x = x;
           pose.pose.position.y = y;
           pose.pose.position.z = 0.0;

           pose.pose.orientation.x = 0.0;
           pose.pose.orientation.y = 0.0;
           pose.pose.orientation.z = 0.0;
           pose.pose.orientation.w = 1.0;

           plan.push_back(pose);
         }

     float path_length = 0.0;

     std::vector<geometry_msgs::PoseStamped>::iterator it = plan.begin();

     geometry_msgs::PoseStamped last_pose;
     last_pose = *it;
     it++;
     for (; it!=plan.end(); ++it) {
        path_length += hypot(  (*it).pose.position.x - last_pose.pose.position.x,
                          (*it).pose.position.y - last_pose.pose.position.y );
        last_pose = *it;
     }
     cout <<"The global path length: "<< path_length<< " meters"<<endl;
     MyExcelFile << "\t" <<path_length <<"\t"<< plan.size() <<endl;
       //publish the plan
        return true;
      }
           else
       {
         ROS_WARN("The planner failed to find a path, choose other goal position");
         return false;
       }

     }

     else
     {
       ROS_WARN("Not valid start or goal");
       return false;
     }
}

 /*
  *  Convert X and Y to matrix X' and Y'
  * */
  void potentialField::getCoordinate (float& x, float& y){
     x = x - _originX;
     y = y - _originY;
 }

 /*
  *  Given (X,Y), return cell Index
  * */
  int potentialField::convertToCellIndex(float x, float y){

     float newX = x / _resolution;
     float newY = y / _resolution;
     return getCellIndex( newX, newY);
 }

 /*
  *  Given Index, return cell (X,Y)
  * */
 void potentialField::convertToCoordinate(int index, float& x, float& y){

     x = getCellColID(index) * _resolution;
     y = getCellRowID(index) * _resolution;

     x = x + _originX;
     y = y + _originY;

 }

 /*
  *  Given a point (X,Y) test if it is inside the map
  * */
 bool potentialField::isCellInsideMap(float x, float y){
     bool valid;

     valid = ! ( x >= (_width* _resolution)  || y>= (_height* _resolution) || (x< 0) || ( y< 0) );
 }

 /*
  *  Transform map --> world ( ? )
  * */
 void potentialField::mapToWorld(double mx, double my, double& wx, double& wy){
     costmap_2d::Costmap2D* costmap = _costmap_ros->getCostmap();
     wx = costmap->getOriginX() + mx * _resolution ;
     wy = costmap->getOriginY() + my * _resolution ;
 }


 /*
  *  Here is my Potential Field Calculator
  * */
std::vector<int> potentialField::potentialPlanner(int startCell, int goalCell){
    vector<int> bestPath;

    timespec time1, time2;
    /* take current time here */
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);

    calculateGoalPotential(goalCell);
    bestPath = findPath(startCell, goalCell);

    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);

    cout<<"time to generate best global path by Potential Field* = " << (diff(time1,time2).tv_sec)*1e3 + (diff(time1,time2).tv_nsec)*1e-6 << " microseconds" << endl;

    MyExcelFile <<"\t"<< (diff(time1,time2).tv_sec)*1e3 + (diff(time1,time2).tv_nsec)*1e-6 ;

   return bestPath;

 }

/*******************************************************************************/
//Function Name: calculateGoalPotential
//Inputs: cell's index
//Output: void
//Description: given some goalCell's index, it calculates the potential energy for
//  all others cells. The cost is f = simṕle distance
/*********************************************************************************/
void potentialField::calculateGoalPotential(int goalCell){
    float goalX,goalY;
    convertToCoordinate( goalCell, goalX, goalY);
    float x, y ;

     for( int i=0; i < mapSize_ ; i++){
        convertToCoordinate( i, x, y);
        _posPotMap[i] = GOALPOTENTIALMULTIPLIER * getDistance( x, y, goalX, goalY);
    }
}

/*
 * Return the best Path
 * */
std::vector<int> potentialField::findPath(int startCell, int goalCell) {
    std::vector<int> bestPath;
    int currentCell= startCell;
    bestPath.push_back(currentCell);
    while( !isGoalAcomplished(currentCell, goalCell)){
        currentCell = findBestCell( currentCell);
        if( currentCell == - 1) return bestPath;
        //ROS_INFO(" Index: %d", currentCell);
        //MyExcelFile << currentCell << endl ;
        bestPath.push_back(currentCell);
    }
    return bestPath;
}

/*******************************************************************************/
//Function Name: isStartAndGoalCellsValid
//Inputs: the start and Goal cells
//Output: true if the start and the goal cells are valid
//Description: check if the start and goal cells are valid
/*********************************************************************************/
bool potentialField::isStartAndGoalCellsValid(int startCell,int goalCell)
{
 bool isvalid=true;
 bool isFreeStartCell=isFree(startCell);
 bool isFreeGoalCell=isFree(goalCell);
    if (startCell==goalCell)
    {
    cout << "The Start and the Goal cells are the same..." << endl;
    isvalid = false;
    }
   else
   {
      if (!isFreeStartCell && !isFreeGoalCell)
      {
    cout << "The start and the goal cells are obstacle positions..." << endl;
        isvalid = false;
      }
      else
      {
    if (!isFreeStartCell)
    {
      cout << "The start is an obstacle..." << endl;
      isvalid = false;
    }
    else
    {
        if(!isFreeGoalCell)
        {
          cout << "The goal cell is an obstacle..." << endl;
          isvalid = false;
        }
        else
        {
  /*        if (findFreeNeighborCell(goalCell).size()==0)
          {
        //cout << "The goal cell is encountred by obstacles... "<< endl;
        isvalid = false;
          }
          else
          {
        if(findFreeNeighborCell(startCell).size()==0)
        {
          //cout << "The start cell is encountred by obstacles... "<< endl;
          isvalid = false;
        }
          } */
        }
    }
      }
  }
 return isvalid;
}

/*
 * Verify if the cell(Index) is free
 * */
bool potentialField::isFree(int cellID){
    return OGM_[cellID] ;
}

/*
 * Verify if the cell (x,y) is free ( it's free if it's value == 0 )
 * */
bool  potentialField::isFree(float x, float y){
    int cellID = convertToCellIndex(x, y);
    return OGM_[cellID];
}

/*
 *  Get the cell distance
 * */
float potentialField::getDistance( float x1, float y1, float x2, float y2) {
    return std::sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

/*******************************************************************************/
//Function Name: sumPointPotential
//Inputs: cell's index
//Output: void
//Description: get the cost for the neighborhood of some obstacle. The neighborhood
// are all the cells that distance R /_resolution (simple distance) from the given cell
// the cost is the function f = C / (d) - where C is a constant
/*********************************************************************************/
void potentialField::sumPointPotential( int cellID){
    float x,y;
    int index;
    float cost;
    convertToCoordinate( cellID, x, y);
    getCoordinate(x,y);
    for( register float i = - RAIO; i < RAIO; i+= JUMP * _resolution){
        for( register float j = - (RAIO - fabs(i)) ; j < (RAIO - fabs(i)) ; j += JUMP * _resolution){
            if ( isCellInsideMap(x+i,y+j) ) {
                index = convertToCellIndex(x+i ,y+j);
                if( i != 0 || j != 0 )   cost = ( C / getDistance( x+i, y+j, x, y));
                else cost = 300000;
                if( _obsPotMap[index] < cost) _obsPotMap[index] = cost ;
            }
        }
    }
}

/*******************************************************************************/
//Function Name: findBestCell
//Inputs: cell's index
//Output: cell's index
//Description: return the neighbor ( 8 movements) cell with minimun potential
/*********************************************************************************/
int potentialField::findBestCell(int cellID){
    float x,y;
    int index, best_index;
    float best_value= 999999;

    convertToCoordinate( cellID, x, y);
    getCoordinate(x,y);
    index = convertToCellIndex(x , y ) ;
    cout << " DADO: (" << x << "," << y << ")" << "COST: " << _obsPotMap[index] + _posPotMap[index] << endl ;

     for( float i = - WALK * _resolution ; i <=  WALK * _resolution ; i+= _resolution){
         for( float j = - WALK * _resolution ; j <= WALK * _resolution ; j+= _resolution){
             if( isCellInsideMap( x+i, y+j)){
                 index = convertToCellIndex( x+i, y+j);
                 if( best_value > _obsPotMap[index] + _posPotMap[index]){
                     best_value = _obsPotMap[index] + _posPotMap[index];
                     best_index = index;
                 }
             }
         }
     }
    if( best_index == cellID){
        ROS_ERROR("MINIMO LOCAL");
        return -1;
    }
    return best_index;
}


/*
 * Calculate the potential field of the obstacles
 * */
void potentialField::calculateObstaclePotential(){
    for( int i = 0 ; i < mapSize_ ; i++){
       if (!(isFree(i)))  sumPointPotential(i);

    }
}

int potentialField::getCellIndex(int i ,int j)
 {
  return i+(j*_width);
 }

int potentialField::getCellRowID(int index)
 {
    return index/_width;
 }

int potentialField::getCellColID(int index)
 {
   return index%_width;
 }



void potentialField::writeCostmap(){
    register int i;
    for ( unsigned int iy = 0; iy < _costmap->getSizeInCellsY(); iy+= 3)
    {
       MyExcelFile << endl ;
      for (unsigned int ix = 0; ix < _costmap->getSizeInCellsX(); ix+=3)
      {
        i = getCellIndex(ix , iy);

        MyExcelFile << _obsPotMap[i] + _posPotMap[i] << "\t" ;

      }
    }
}

void potentialField::writePath( std::vector<int> path){
    MyExcelFile << endl << " Path :" << endl ;

    for( std::vector<int>::iterator it = path.begin() ; it != path.end() ; ++it){
        MyExcelFile << *it << ", " ;
    }

    MyExcelFile << "ended" << endl;

}

bool potentialField::isGoalAcomplished( int currentCell , int goalCell)
{
    float xG,yG,xC,yC;
    convertToCoordinate( currentCell, xC, yC);
    convertToCoordinate( goalCell, xG, yG);
    if( getDistance(xC,yC,xG,yG) < GOALERROR ) return true;
    return false;
}

}
