// std
#include <unordered_map>

// Boost
#include <boost/shared_ptr.hpp>

// standard ros
#include "ros/ros.h"
#include "ros/console.h"
#include "eigen_conversions/eigen_msg.h"

// tf listener to tranform into correct frame
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>

// pointmachter library
#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/Timer.h"

// graph search from Ulrich
//#include <graph_search_ros/AStar.h>

// point matchter wrapper to ros
#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/transform.h"
#include "pointmatcher_ros/get_params_from_server.h"
#include "pointmatcher_ros/aliases.h"
#include "pointmatcher_ros/ros_logger.h"

// Service definitions
#include "tensorPlanner/CallGlobalMap.h"
#include "tensorPlanner/ConfirmGoal.h"
#include "tensorPlanner/ComputePlan.h"
#include "tensorPlanner/ExecutePlan.h"
#include "map_msgs/GetPointMap.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
# include <iostream>
# include <cmath>
# include <stdlib.h>
# include <stdio.h>
# include <fstream>
# include <sstream>
# include <vector>
# include <numeric>
# include <algorithm>
# include <iterator>
# include <iostream>
# include <functional>
# include <new>
# include <memory>
# include <Eigen/Core>
# include <Eigen/Eigenvalues>
# include <Eigen/Geometry>
# include <Eigen/Dense>

using namespace std;
using namespace PointMatcherSupport;
//using namespace graph_search_ros;
using namespace Eigen;

typedef Eigen::VectorXi Vi;
typedef Eigen::VectorXf Vf;
typedef Eigen::Vector3f V3f;
typedef Eigen::VectorXd Vd;

struct pathPlan{

	struct wayPointInfo{

		Matrix4f path3d;
		Matrix4f path2d;
		int flipperState;
		Vector2f directionV;

		public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		double thetaRef;
		double wRef;
		//int index;
	};

	vector<wayPointInfo> oneWayPoint;


}globalPath;

struct GridTensor
{
	int occupyId;
	double voterDist;
	
	/////////////////////////
	// Revision: SM / 10.12
	float Gcost;
	int whichList;
	int walkability;
	Eigen::MatrixXi parent;
	
	/////////////////////////
	Eigen::MatrixXf tensorPlate;
	Eigen::MatrixXf tensorStick;
	Eigen::MatrixXf tensorBall;
	Eigen::VectorXf viewDir; // first 3 elements are the viewpoint position
	GridTensor():
		occupyId(0),
		voterDist(numeric_limits<double>::max()),
		Gcost(0),
		whichList(0),
		walkability(0),
		parent(Eigen::MatrixXi::Zero(1,3)),
		tensorPlate(Eigen::MatrixXf::Zero(3,3)), // not implemented
		tensorStick(Eigen::MatrixXf::Zero(3,3)),
		tensorBall(Eigen::MatrixXf::Zero(3,3)), // not implemented
		viewDir(Eigen::VectorXf::Zero(3,1))
		{}
};

struct GridPoint
{
	const Vi coord;

	// Constructor
	GridPoint(const Vf &pt, const Vf &scale);
	GridPoint(const Vi &pt);

	static Vi pointToGrid(const Vf &pt, const Vf &scale);
	static Vf gridToPoint(const Vi &grid, const Vf &scale);
	Vf getUnscaledCoord(const Vf &scale);
};


GridPoint::GridPoint(const Vf &pt, const Vf &scale):
	coord(pointToGrid(pt, scale))
{}

GridPoint::GridPoint(const Vi &pt):
	coord(pt)
{}

Vi GridPoint::pointToGrid(const Vf &pt, const Vf &scale)
{
	Vi grid;
	grid.resize(3);
	for(int i=0; i < 3; i++)
	{
		grid(i) = int(floor(pt(i)/scale(i) + 0.5f));
	}

	return grid;
}


Vf GridPoint::gridToPoint(const Vi &grid, const Vf &scale)
{
	Vf pt;
	pt.resize(3);
	for(int i=0; i < 3; i++)
		pt(i) = float(grid(i))*scale(i);

	return pt;
}

Vf GridPoint::getUnscaledCoord(const Vf &scale)
{
	return gridToPoint(coord, scale);
}

struct hash_gridPoint
{
	size_t operator()(const GridPoint &gp) const
	{
		// TODO: is that the best hash function?
		return hash<int>()(gp.coord(0)) ^ hash<int>()(gp.coord(1)) ^ hash<int>()(gp.coord(2));
	}
};

struct eq_gridPoint
{
	bool operator()(const GridPoint &gp1, const GridPoint &gp2) const
	{
		return (gp1.coord(0) ==  gp2.coord(0)) && (gp1.coord(1) == gp2.coord(1)) && (gp1.coord(2) ==  gp2.coord(2));
	}
};



class TensorPlanner
{
	// Naming Shortcuts
	typedef std::unordered_map<GridPoint, GridTensor, hash_gridPoint, eq_gridPoint> GridMap;

	ros::NodeHandle& n;
	ros::NodeHandle& pn;
	
	ros::ServiceServer callMapSrv;
	ros::ServiceServer setGoalSrv;
	ros::ServiceServer computePlanSrv;
	ros::ServiceServer executePlanSrv;
	ros::ServiceClient computeMapClient;
	ros::Subscriber cloudSub;
	tf::TransformListener tf_listener;
	ros::Publisher cmd_vel_pub;
   	ros::Publisher flipper_cmd;
	ros::Subscriber stopExec;
	Eigen::Vector3f robotDimension;
	bool errorCalled;

	// Parameters
	const float sigma;
	const float eps;
	const float cellLength;
	const float maxOccRatio;
	const float maxOrientationAngle; // in rad
 	
	
	// flipper states
	int fs_FlatDriving;// moving along flat surface (default)
    int fs_ApproachForward;// approaching a change in orientation in forward direction
	int fs_ApproachBackward; // approaching a change in orientation in backward direction
	int fs_MaxTraction;// climbing up or down stairs
	int fs_TipOver;// moving along stairs up and then flattening OR moving along flat surface and approaching stairs downwards
	float orientTh;

	// control reference velocity
	double vRef,kx,ky,kth,kRot;
	float saliencyThLower; // below which no surface
	float saliencyCostTh; // below which tan function

	const Vf cellSizes;

	GridMap gridMap;

	//Node::NodePtr_t goalNode;


public:
	// constructor
	TensorPlanner(ros::NodeHandle& n, ros::NodeHandle& pn);
	// destructor
	~TensorPlanner();

protected:
	void gotCloud(const sensor_msgs::PointCloud2& cloudMsgIn);
	bool callGlobalMap(tensorPlanner::CallGlobalMap::Request& req, tensorPlanner::CallGlobalMap::Response& res);
	bool setGoal(tensorPlanner::ConfirmGoal::Request& req, tensorPlanner::ConfirmGoal::Response& res);
	bool computePlan(tensorPlanner::ComputePlan::Request& req, tensorPlanner::ComputePlan::Response& res);
	bool executePlan(tensorPlanner::ExecutePlan::Request& req, tensorPlanner::ExecutePlan::Response& res);
	void mergeMap(const DP & cloud, int seqId);
	void stopExecution(const std_msgs::Bool& msg);
	
	// Helper functions
	void sortTensor(PM::Matrix& eigenVa, PM::Matrix& eigenVe);
	PM::Vector serializeEigVec(const PM::Matrix eigenVe);
	void saveGridToVTK(GridMap &gridMap);
	// Revision: 04.11.2012/SM
	//Eigen::Vector3f realignNormal(const PM::Matrix& eigenVe, const GridPoint& grid_cell);
	Eigen::Vector3f realignNormal(const PM::Matrix& eigenVe, const Eigen::Vector3f& ref);
	Eigen::Matrix3f rotationMatrix(Eigen::Vector3f& u_rot, const float theta); // generate rotation matrix using quaternion information
	Eigen:: Matrix<int,6,1> generateBoundingBox(const Vi& grid_cell, V3f& robotDimension,V3f& normal);
	bool verifyBoundingBox(const Vi& grid_cell, const Vi& robot_cell, V3f& robotDimension, V3f& normal);
	float cost_heading(const Eigen::Vector3f& eigVe_child, const Eigen::Vector3f& eigVe_parent, const Eigen::Vector3f& pos_child, const Eigen::Vector3f& pos_parent, const float heading_factor);
	float cost_saliency(const float maxGlobalSal, const float saliencyCurrent, const float saliencyCostTh, const float saliencyThLower,const float saliency_factor);
	float cost_orientation(const Eigen::Vector3f& eigVe_child,const float stability_factor);
	Matrix<float,Dynamic,3> path_direction_vector(Matrix<float,Dynamic,3>& path, const float saliencyThLower, const int smoothingParameter);
	Matrix<float,Dynamic,3> two_d_path(Matrix<float,Dynamic,3>& path);// 2D way point reference frames
	Matrix<float,Dynamic,3> extract_path_normal(Matrix<float,Dynamic,3>& path);// extract path normal
	Matrix<float,Dynamic,3> path_direction_2d(Matrix<float,Dynamic,3>& path,Matrix<float,Dynamic,3> pathDirection);// 2D direction vectors
	Matrix<Matrix4f,Dynamic,1> way_point_ref3d(Matrix<float,Dynamic,3>& path);
	Matrix<int,Dynamic,1> flipper_control(Matrix<float,Dynamic,3>& path);
	Matrix<Matrix4f,Dynamic,1> way_point_ref2d(Matrix<float,Dynamic,3>& path2d);
	Vector4f rotMatrixToQuaternion (Matrix<float,3,3> rotationMatrix);
	Eigen::Matrix4f generate_transform_matrix(Vector4f& quaternion,Vector3f& pos);
	Vector2f lineControl(Vector2f& pos2dCurrent,const float headingAngle,const int closestWayPoint,const float vRef,const float kx, const float ky,const float kth);
	Matrix<float,Dynamic,2> trajectoryDefinition(Matrix<float,Dynamic,3>& path2d, Matrix<float,Dynamic,3>& vector2d);
	Vector3f robot_2d_localization(Matrix<float,4,4> robotPose,int closestWayPoint);
	Vector2f lineControl(Vector2f& pos2dCurrent,const float headingAngle, int closestWayPoint);
	Vector2f endCondition(Vector2f& pos2dCurrent,const float headingAngle);
	int closest_way_point(Vector3f& pos3d,int closestWayPoint);
	//void stopExecution(const std_msgs::bool& msg);
};

// Constructor
TensorPlanner::TensorPlanner(ros::NodeHandle& n, ros::NodeHandle& pn):
	n(n),
	pn(pn),
	tf_listener(ros::Duration(10.)),
	sigma(getParam<double>("sigma", 0.3)),
	eps(getParam<double>("knnEpsilon", 3.33)),

	cellLength(getParam<double>("cellLength", 0.11)),
	//cellLength(getParam<double>("cellLength", 0.18)),
	maxOccRatio(getParam<double>("maxOccupancyRatio", 0.04)),
	maxOrientationAngle(getParam<double>("maxOrientationAngle", 0.79)),
	cellSizes(cellLength*Vf::Ones(3))
{
	cout << "Constructor called" << endl;	
	
	// topics and services initialization
	callMapSrv = n.advertiseService("call_global_map", &TensorPlanner::callGlobalMap, this);
	setGoalSrv = n.advertiseService("set_goal", &TensorPlanner::setGoal, this);
	computePlanSrv = n.advertiseService("compute_plan", &TensorPlanner::computePlan, this);
	executePlanSrv = n.advertiseService("execute_plan", &TensorPlanner::executePlan, this);
	//cloudSub = n.subscribe("cloud_in", 2, &TensorPlanner::gotCloud, this);
	

	
	stopExec = n.subscribe("/stop_exec", 1, &TensorPlanner::stopExecution, this);
	flipper_cmd = n.advertise<std_msgs::Int32>("/posture_cmd",1);
	cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);
	computeMapClient = n.serviceClient<map_msgs::GetPointMap>("dynamic_point_map");

	// Robot size (parameter, morphing?)
	robotDimension << 0.6,0.6,0.5;// meters

// flipper states
	fs_FlatDriving = 1;// moving along flat surface (default)
	fs_ApproachForward = 2;// approaching a change in orientation in forward direction
	fs_ApproachBackward = 3; // approaching a change in orientation in backward direction
	fs_MaxTraction = 0;// climbing up or down stairs
	fs_TipOver = 4;// moving along stairs up and then flattening OR moving along flat surface and approaching stairs downwards
	orientTh = 17*(M_PI/180); // threshold orientation angle above which surface considered slope

	// control reference velocity
	 vRef = 0.1;
	 // control constants
	 kx = 0.1;
	 ky = 1;
	 kth = 1;
	 kRot = 0.1;

	// saliency thresolds
	saliencyThLower = 100; // below which no surface
	saliencyCostTh = 1500; // below which tan function
}


//-------------------------
// Services
bool TensorPlanner::callGlobalMap(tensorPlanner::CallGlobalMap::Request& req, tensorPlanner::CallGlobalMap::Response& res)
{
	cout << "Will call for a map" << endl;
	map_msgs::GetPointMap srvMsg;
	computeMapClient.call(srvMsg);
	gotCloud(srvMsg.response.map);
	return true;
	return true;
}


bool TensorPlanner::setGoal(tensorPlanner::ConfirmGoal::Request& req, tensorPlanner::ConfirmGoal::Response& res)
{
	Eigen::Vector3f gravity_vector;
	gravity_vector<<0,0,1;

	geometry_msgs::PoseStamped goalDes_global;
	geometry_msgs::PoseStamped goalDes_local;
	//tf::Stamped<tf::Pose> goalDes_local;
	//goalDes_local.setOrigin(tf::Vector3(req.goal.position.x,req.goal.position.y,req.goal.position.z));
	goalDes_local.pose= req.goal;
	goalDes_local.header.frame_id = "/base_link";
	//tf::StampedTransform goalDes_global;
	if(tf_listener.waitForTransform("/base_link", "/map",ros::Time(0), ros::Duration(2.)))
	{
		//tf_listener.lookupTransform("/map","/base_link", ros::Time(0),goalDes_global);
		tf_listener.transformPose("/map",ros::Time(0), goalDes_local,"/base_link", goalDes_global);
	}
	else
		cout<<"error in getting transform"<<endl;

	// find grid coordinates from given target point
	Eigen::Vector3f targetPosition;
	targetPosition<<goalDes_global.pose.position.x ,goalDes_global.pose.position.y,goalDes_global.pose.position.z;
	//targetPosition<<goalDes_global.getOrigin().x(),goalDes_global.getOrigin().y(),goalDes_global.getOrigin().z();//req.goal.position.x ,req.goal.position.y,req.goal.position.z;
	//Eigen::Vector3i target_cell = GridPoint::pointToGrid(targetPosition, cellSizes);
	GridPoint target_cell(targetPosition, cellSizes);
		cout<<"target desired:"<< targetPosition.transpose()<<endl;
	cout<<"target cell desired:"<<target_cell.coord.transpose()<<endl;
	// pointer to locate the grid cell
	auto target_ptr = gridMap.find(target_cell);
	if(target_ptr == gridMap.end())
	{ // is the cell in a voted zone
		res.valid_goal = req.goal;
		res.is_valid = false;
		ROS_WARN_STREAM("The map is empty in the designated zone. Map number of cells: " << gridMap.size());
		return false;
	}

	// Search for higher saliency on the z-axis
	const int nbCellZ = 4;
	const Vi coord = target_ptr->first.coord;
	auto best_ptr = target_ptr;
	V3f best_normal;
	double max_saliency = 0;
	for(int dz = -nbCellZ; dz <= nbCellZ; ++dz)
	{
		Vi offset(3,1);
		offset << coord(0), coord(1), coord(2) + dz;
		const GridPoint better_cell(offset);
		auto test_ptr = gridMap.find(better_cell);
		if(test_ptr != gridMap.end())
		{
			const Eigen::EigenSolver<PM::Matrix> solver(test_ptr->second.tensorStick);
			PM::Matrix eigenVa = solver.eigenvalues().real();
			PM::Matrix eigenVe = solver.eigenvectors().real();

			sortTensor(eigenVa, eigenVe);

			const double saliency = eigenVa(0) - eigenVa(1);
			if(max_saliency < saliency)
			{
				max_saliency = saliency;
				best_ptr = test_ptr;
				best_normal = realignNormal(eigenVe, best_ptr->second.viewDir);
			}

		}
	}

	// generate bounding box for current location and orientation
	Vi boxBound(6);	

	boxBound = generateBoundingBox(best_ptr->first.coord, robotDimension, best_normal);
	
	// check each cell in box bound if it is in obstacle after verifying if cell is in bounding box
	const int minX = boxBound(0);
	const int minY = boxBound(1);
	const int minZ = boxBound(2);
	const int maxX = boxBound(3);
	const int maxY = boxBound(4);
	const int maxZ = boxBound(5);
	
	int nbTotalCell = 0;
	int nbOccupyCell = 0;

	Eigen::Vector3i current_coord;
	for(int iRobot= minX;iRobot<=maxX;iRobot++)
	{// investigate all nodes along x on which robot rests
		for(int jRobot=minY;jRobot<= maxY;jRobot++)
		{// investigate all nodes along y on which robot rests
			for(int kRobot = minZ; kRobot <= maxZ; kRobot++)
			{
				current_coord << iRobot,jRobot,kRobot;
				GridPoint current_cell(current_coord);
				// verify if current cell in the bounding box
				bool valid_cell;
				valid_cell = verifyBoundingBox(best_ptr->first.coord, current_cell.coord,robotDimension, best_normal);
								
				if(valid_cell)
				{ // if cell is within original bounding box
					nbTotalCell++;

					auto cell_ptr = gridMap.find(current_cell);

					if (cell_ptr != gridMap.end())
					{			 
						
						const Eigen::EigenSolver<PM::Matrix> solver(cell_ptr->second.tensorStick);
						PM::Matrix eigenVa = solver.eigenvalues().real();
						PM::Matrix eigenVe = solver.eigenvectors().real();
						sortTensor(eigenVa, eigenVe);
						
						const Vf cell_normal = realignNormal(eigenVe, cell_ptr->second.viewDir);
						const float cellOri = acos(cell_normal.dot(gravity_vector));


						if(cell_ptr->second.occupyId > 0 && cellOri > maxOrientationAngle)
						{// if cell is occupied by a point from sparse cloud
							nbOccupyCell++;	
						}// if cell is occupied by a point from sparse cloud
					} // if cell is within original bounding box
				}
			}
		}
	}

	// Compute new orientation of the marker
	Eigen::Affine3d initP;
	tf::poseMsgToEigen(req.goal, initP);

	const auto rot = initP.rotation();
	const Eigen::Vector3d z_axis = rot.col(2).normalized();
	const Eigen::Vector3d normal_const = best_normal.normalized().cast<double>();

	const double ang = acos(normal_const.dot(z_axis));
	const Eigen::Vector3d rot_axis = z_axis.cross(normal_const);

	initP = Eigen::AngleAxisd(ang, rot_axis.normalized()) * initP;

	tf::poseEigenToMsg(initP, res.valid_goal);

	// Compute new position of the marker
	const Vf bestCoord = GridPoint::gridToPoint(best_ptr->first.coord, cellSizes);

	goalDes_global.pose.position.x = bestCoord(0);
	goalDes_global.pose.position.y = bestCoord(1);
	goalDes_global.pose.position.z = bestCoord(2);
	cout<<"valid goal pos:"<<bestCoord.transpose()<<endl;
	cout<<"valid goal cell:"<<best_ptr->first.coord.transpose()<<endl;
	tf_listener.transformPose("/base_link",ros::Time(0), goalDes_global,"/map", goalDes_local);

	res.valid_goal.position.x = goalDes_local.pose.position.x;
	res.valid_goal.position.y = goalDes_local.pose.position.y;
	res.valid_goal.position.z = goalDes_local.pose.position.z;
//	res.valid_goal.position.x = bestCoord(0);
//	res.valid_goal.position.y = bestCoord(1);
//	res.valid_goal.position.z = bestCoord(2);
	res.is_valid = true;

	// Check occupency ratio
	const double occRatio = (double)nbOccupyCell/(double)nbTotalCell;
	ROS_INFO_STREAM("Occupy ratio: " << occRatio);
	if(occRatio >= maxOccRatio)
	{
		res.is_valid = false;
		ROS_WARN_STREAM("Target position invalid due to large ratio of obstacle: " << occRatio << " larger than " << maxOccRatio);
	}

	// check for orientation of pose if position is safe
	const float robotOrientation = acos(best_normal.dot(gravity_vector));
	ROS_INFO_STREAM("Goal angle from gravity: " << robotOrientation << " rad (" << robotOrientation*180/M_PI << " deg)");
	if(res.is_valid)
	{
		if(robotOrientation >= maxOrientationAngle)
		{
			res.is_valid = false;
			ROS_WARN_STREAM("Target orientation invalid due large inclinasion: " << robotOrientation << " larger than " << maxOrientationAngle);
		}	
	}
	
	//cout<<"target position:"<<res.valid_goal.position.x<<","<<res.valid_goal.position.y<<","<<res.valid_goal.position.z<<endl;
	//cout<<"target cell:"<<best_ptr->first.coord.transpose()<<endl;
	return res.is_valid;
}




bool TensorPlanner::computePlan(tensorPlanner::ComputePlan::Request& req, tensorPlanner::ComputePlan::Response& res)
{
	ROS_INFO("Received request to PLAN");
	
	// clear GridTensor info
	for(auto iter=gridMap.begin(); iter != gridMap.end(); iter++)
	{
		iter->second.Gcost = 0;
		iter->second.whichList = 0;
		iter->second.walkability = 0;
		iter->second.parent = Eigen::MatrixXi::Zero(1,3);	
	}

	timer t;
	// Revision: SM/10.12
	Vector3f gravity_vector;
	gravity_vector << 0,0,1;
	
	// start and target positions/cell positions
	tf::StampedTransform transform;

//	if(tf_listener.waitForTransform("/base_link", "/map",ros::Time(0), ros::Duration(2.)))
//	{
		tf_listener.lookupTransform("/map","/base_link", ros::Time(0),transform);
//	}


	Vector3f startPos;
	startPos <<transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z();
	GridPoint startCell(startPos, cellSizes);
	auto start_ptr = gridMap.find(startCell);
	cout<<"start cell:"<<startCell.coord.transpose()<<endl;
	cout<<"start Pos:"<<startPos.transpose()<<endl;

	geometry_msgs::PoseStamped goal_global;
	geometry_msgs::PoseStamped goal_local;
	goal_local.pose = req.goal;
	goal_local.header.frame_id = "/base_link";
	if(tf_listener.waitForTransform("/base_link", "/map",ros::Time(0), ros::Duration(2.)))
	{
		tf_listener.transformPose("/map",ros::Time(0), goal_local,"/base_link", goal_global);
	}
	else
		cout<<"Error in getting global goal position while planning path"<<endl;

	Vector3f targetPos;
	targetPos << goal_global.pose.position.x,goal_global.pose.position.y,goal_global.pose.position.z;
	cout<<"targetPos:"<<targetPos.transpose()<<endl;
	GridPoint targetCell(targetPos, cellSizes);
	auto target_ptr = gridMap.find(targetCell);
	cout<<"target cell:"<<targetCell.coord.transpose()<<endl;
	
	Matrix<int,Dynamic,1> openList(gridMap.size(),1); // open list from which node with lowest F selected
	Matrix<int,Dynamic,3> openPos(gridMap.size(),3); // cell position of node in openlist
	Matrix<int,Dynamic,1> closedList(gridMap.size(),1); // closed list for nodes that have been evaluated
	Matrix<float,Dynamic,1> Fcost(gridMap.size(),1);// F cost of item on openlist
	Matrix<float,Dynamic,1> Hcost(gridMap.size(),1);// H cost of item on openlist
	Matrix<float,Dynamic,3> pathBank(gridMap.size(),3); // cell position of path determined
	//Matrix<int,Dynamic,3> parent(GridMap.size(),3); // parent of cell in final list
	RowVector3i parentVal; // parent cell values in each iteration
	parentVal << 0,0,0;
	
	// variable definitions
	int walkable =1, unwalkable = -1; // walkability
	int path = 0,found =1,nonexistent =2; // path found or non existent
	int onOpenList = 1, onClosedList = 2; // which list
	int newOpenListItemID = 0; // new item on open list
	float costSaliency = 0, costOrientation = 0,costHeading= 0,costDistance = 0, addedGcost = 0, tempGcost = 0; // cost function initialization
	
	
	
	// cost factors
	float safety_factor = 1;
	float stability_factor = 1;
	float distance_factor = 1;
	float heading_factor = 1;
	
	//smoothing parameter
	int smoothingParameter = 6;
	
	// final path
	int pathLength = 0;
	RowVector3i path_cell;
	path_cell <<0,0,0;
	
	// maximum global saliency
	float maxGlobalSal = 0; // max saliency
	for(auto iter=gridMap.begin();iter!=gridMap.end();iter++)
	{
	    const Eigen::EigenSolver<PM::Matrix> solver(iter->second.tensorStick);
	    PM::Matrix eigVa = solver.eigenvalues().real();
	    PM::Matrix eigVe = solver.eigenvectors().real();
	    sortTensor(eigVa, eigVe);
	    
	    if(maxGlobalSal < (eigVa(0)-eigVa(1)))
	      maxGlobalSal = (eigVa(0)-eigVa(1));
	}
	
	//cout<<"max global saliency:"<<maxGlobalSal<<endl;
	
	// set start cell in open list
	int numItemsOpenList = 1; // number of items in open list
	openList(0) = 0; // set it to first position in priority
	openPos.row(openList(0)) = startCell.coord.transpose();
	parentVal = startCell.coord.transpose();
	

	// TODO: add if condition to cell if cell ptr exists
	start_ptr->second.walkability = walkable;
	cout<<"Path Planning started"<<endl;
	do // until path found or non existent
	{
	  //cout<<"numItems:"<<numItemsOpenList<<endl;
	  if(numItemsOpenList !=0)
	  {// if open list is not empty
	    
		  // pop first node and put into closed list
		  parentVal = openPos.row(openList(0));
		  //cout<<"parentval:"<<parentVal<<endl;
		  
		  GridPoint ParentValue(parentVal.transpose());
		  auto parent_ptr = gridMap.find(ParentValue);
		  // put first cell from open list into closed list
		  parent_ptr->second.whichList = onClosedList; 
		  
		  // Sort List in descending order of Fcost
		  for(int i=0;i<numItemsOpenList-1;i++)
		  {
				  for(int j =0;j<numItemsOpenList -(i+1);j++)
		  {
					  if(Fcost(openList(j))<=Fcost(openList(j+1)))
			  {
						  int temp = openList(j);
						  openList(j) = openList(j+1);
						  openList(j+1) = temp;
					  }
				  }
			  }
			  
			  // we remove one cell from open list
			  numItemsOpenList--;
		  
		  // Sort list in ascending order of Fcost
		  for(int i=0;i<numItemsOpenList-1;i++)
		  {
				  for(int j =0;j<numItemsOpenList -(i+1);j++)
		  {
					  if(Fcost(openList(j))>=Fcost(openList(j+1)))
			  {
						  int temp = openList(j);
						  openList(j) = openList(j+1);
						  openList(j+1) = temp;

					  }
				  }
			  }
		  
		  // extract position, saliency and orientation information for parent
		  Vector3f parentPos = GridPoint::gridToPoint(ParentValue.coord,cellSizes);
		  const Eigen::EigenSolver<PM::Matrix> solver(parent_ptr->second.tensorStick);
		  PM::Matrix eigVaParent = solver.eigenvalues().real();
		  PM::Matrix parentVe = solver.eigenvectors().real();
		  sortTensor(eigVaParent, parentVe);
		  
		  Vector3f parentNormal = realignNormal(parentVe, parent_ptr->second.viewDir);
		  parentNormal.normalize();
		  // check cells in neighborhood of parent cell
		  for(int x = parentVal(0)-1;x<= parentVal(0) +1;x++)
		  {// along x direction
			for(int y = parentVal(1)-1;y<=parentVal(1) +1;y++)
			{// along y direction
			  for(int z = parentVal(2)-1;z<= parentVal(2) +1;z++)
			  {// along z direction
			  
					Vi child_cell(3,1);
					child_cell<<x,y,z;
					
					GridPoint child(child_cell);
					auto child_ptr = gridMap.find(child);
					//cout<<"cell:"<<child_cell.transpose()<<endl;
					if(child_ptr!=gridMap.end())
					{// if cell exists 
						  if(child_ptr->second.whichList!=onClosedList)
						  {// if cell not on closed list
							
						  // extract position, saliency and normal vector information
						  Vector3f childPos = GridPoint::gridToPoint(child_cell,cellSizes);
						  const Eigen::EigenSolver<PM::Matrix> solver(child_ptr->second.tensorStick);
						  PM::Matrix eigVaChild = solver.eigenvalues().real();
						  PM::Matrix childVe = solver.eigenvectors().real();
						  sortTensor(eigVaChild, childVe);
						  Vector3f childNormal = realignNormal(childVe, child_ptr->second.viewDir);
						  childNormal.normalize();
						
						  if((eigVaChild(0)-eigVaChild(1))<saliencyThLower)
						  {
							  child_ptr->second.walkability = unwalkable;
						  }
						  
						  if(child_ptr->second.walkability != unwalkable)
						  {  
							  float temp = childNormal.dot(gravity_vector);
							  if(temp>1)
								temp = 1;
								else if(temp<-1)
								temp = -1;
									
								float childOrient = acos(temp);
								if(childOrient>maxOrientationAngle)
								{
									  child_ptr->second.walkability = unwalkable;
								}
						  }
						  
						  if(child_ptr->second.walkability != unwalkable)
						  {
								// generate bounding box for current location and orientation
								Vi boxBound(6);	

								boxBound = generateBoundingBox(child_ptr->first.coord, robotDimension, childNormal);
								
								// check each cell in box bound if it is in obstacle after verifying if cell is in bounding box
								const int minX = boxBound(0);
								const int minY = boxBound(1);
								const int minZ = boxBound(2);
								const int maxX = boxBound(3);
								const int maxY = boxBound(4);
								const int maxZ = boxBound(5);
								
								int nbTotalCell = 0;
								int nbOccupyCell = 0;

								Eigen::Vector3i current_coord;
								for(int iRobot= minX;iRobot<=maxX;iRobot++)
								{// investigate all nodes along x on which robot rests
								for(int jRobot=minY;jRobot<= maxY;jRobot++)
								{// investigate all nodes along y on which robot rests
										for(int kRobot = minZ; kRobot <= maxZ; kRobot++)
										{
											current_coord << iRobot,jRobot,kRobot;
											GridPoint current_cell(current_coord);
											// verify if current cell in the bounding box
											bool valid_cell;
											valid_cell = verifyBoundingBox(child_ptr->first.coord, current_cell.coord,robotDimension, childNormal);
															
											if(valid_cell)
											{ // if cell is within original bounding box
												nbTotalCell++;

												auto cell_ptr = gridMap.find(current_cell);

												if (cell_ptr != gridMap.end())
												{			 
													
													const Eigen::EigenSolver<PM::Matrix> solver(cell_ptr->second.tensorStick);
													PM::Matrix eigenVa = solver.eigenvalues().real();
													PM::Matrix eigenVe = solver.eigenvectors().real();
													sortTensor(eigenVa, eigenVe);
													
													const Vf cell_normal = realignNormal(eigenVe, cell_ptr->second.viewDir);
													const float cellOri = acos(cell_normal.dot(gravity_vector));

													if(cell_ptr->second.occupyId > 0 && cellOri > maxOrientationAngle)
													{// if cell is occupied by a point from sparse cloud
														nbOccupyCell++;	
													}// if cell is occupied by a point from sparse cloud
												} // if cell is within original bounding box
											}
										}
									}
								}
								// Check occupency ratio
								const double occRatio = (double)nbOccupyCell/(double)nbTotalCell;
								if(occRatio >= maxOccRatio)
								{
								  child_ptr->second.walkability = unwalkable;
								}
								else
								  child_ptr->second.walkability = walkable;

								 //cout<< "total number of cells:"<<nbTotalCell<<",occupied cells:"<<nbOccupyCell<<endl;
							}

						  	  	//cout<<"child saliency:"<<(eigVaChild(0)-eigVaChild(1))<<endl;
								//cout<<"walkability of cell:"<<child_ptr->second.walkability<<endl;
								if(child_ptr->second.walkability == walkable)
								{// if cell is in free set
								  
								  if(child_ptr->second.whichList!=onOpenList)
								  {// if cell not on open list
									  newOpenListItemID++; // assign unique ID
									  int m = numItemsOpenList;
									  openList(m) = newOpenListItemID; // add item to bottom of list
									  openPos.row(newOpenListItemID) = child_cell.transpose();
									  
									  // Compute G cost of current cell
									  
									  // saliency cost

									  costSaliency = cost_saliency(maxGlobalSal, (eigVaChild(0)-eigVaChild(1)), saliencyCostTh, saliencyThLower,safety_factor);
									  //cout<<"costSaliency:" <<costSaliency<<endl;
									  // distance cost
									  costDistance = distance_factor*((childPos-parentPos).norm());
									  //cout<<"costDistance:" <<costDistance<<endl;
									  // orientation cost
									  costOrientation = cost_orientation(childNormal,stability_factor);
									  //cout<<"costOrientation:" <<costOrientation<<endl;
									  // heading cost
									  costHeading = cost_heading(childNormal, parentNormal,  childPos, parentPos,heading_factor);
									  //cout<<"costHeading:" <<costHeading<<endl;
									  // total added cost
									  addedGcost = costSaliency + costDistance + costOrientation + costHeading;
									  //cout<<"addedGcost:"<<addedGcost<<endl;
									  // total G cost
									  child_ptr->second.Gcost = addedGcost + parent_ptr->second.Gcost;
									  //cout<<"Gcost:" <<child_ptr->second.Gcost<<endl;
									  //Hcost
									  Hcost(openList(m)) = (childPos-targetPos).norm();
									  //cout<<"Hcost:" <<Fcost(openList(m))<<endl;
									  // Fcost
									  Fcost(openList(m)) = child_ptr->second.Gcost + Hcost(openList(m));
									  //cout<<"Fcost:" <<Fcost(openList(m))<<endl;
									  //cout<<"--------"<<endl;
									  // set parent
									  child_ptr->second.parent = parentVal;
									  
									  //add to openlist
									  child_ptr->second.whichList = onOpenList;
									  
									  numItemsOpenList++;	    
								  }// if cell not on open list
								  else
								  {// if cell already on open list
									  // saliency cost
									  costSaliency = cost_saliency(maxGlobalSal,(eigVaChild(0)-eigVaChild(1)), saliencyCostTh, saliencyThLower,safety_factor);
									  
									  // distance cost
									  costDistance = distance_factor*((childPos-parentPos).norm());
									  
									  // orientation cost
									  costOrientation = cost_orientation(childNormal,stability_factor);
									  
									  // heading cost
									  costHeading = cost_heading(childNormal, parentNormal,  childPos, parentPos,heading_factor);
									  
									  // added Gcost
									  addedGcost = costSaliency + costDistance + costOrientation + costHeading;
									  
									  tempGcost = addedGcost + parent_ptr->second.Gcost;
									  
									  if(tempGcost<child_ptr->second.Gcost)
									  {// if tempGcost lower
									  child_ptr->second.Gcost = tempGcost;
									  child_ptr->second.parent = parentVal;
									  for(int i=0;i<numItemsOpenList;i++)
									  {
										  if(openPos(i,0)==child_cell(0) && openPos(i,1)==child_cell(1) && openPos(i,2)==child_cell(2))//openPos.row(openList(i)).array() == child_cell.transpose().array())
										Fcost(openList(i)) = child_ptr->second.Gcost + Hcost(openList(i));
									  }  
									  }// if tempGcost lower
								  }// if cell already on open list	  
								}// if cell is in free set	 
							}// if cell not on closed list   
					}// if cell exists    
			}// along z direction
		  }// along y direction	      
		  }// along x direction
		  
			// Sort list in ascending order of Fcost
		  for(int i=0;i<numItemsOpenList-1;i++)
		  {
			  for(int j =0;j<numItemsOpenList -(i+1);j++)
			  {
				if(Fcost(openList(j))>=Fcost(openList(j+1)))
				{
				    int temp = openList(j);
				    openList(j) = openList(j+1);
				    openList(j+1) = temp;
				}
			  }
		    }

	  }// if open list is not empty
	  else
	  {
	    res.plan_found = false;
		cout<<"path nonexistent"<<endl;
	    path = nonexistent;break;
	  }
	  if(target_ptr->second.whichList == onOpenList)
	  {
	    res.plan_found = true; 
	    cout<<"target achieved cell:"<<target_ptr->first.coord.transpose()<<endl;
		cout<<"path found"<<endl;
	    path = found;break;  
	  }
	  
	}while(path !=found || path !=nonexistent); // until path found or non existent

	if(path == found)
	{
	  // determine length of path
	  path_cell = targetCell.coord.transpose();
	  do
	  {
	    
	    GridPoint path1(path_cell.transpose());
	    auto path_ptr = gridMap.find(path1);
	    path_cell = path_ptr->second.parent;
	    pathLength++;
	    
	  }while(path_cell(0) != startCell.coord(0) || path_cell(1) != startCell.coord(1) || path_cell(2) != startCell.coord(2));
	  //cout<<"path length:"<<pathLength<<endl;
	 
	  path_cell = targetCell.coord.transpose();
	  int currentPathPosition = pathLength;
	  do
	  {
		
	    pathBank.row(currentPathPosition) = (GridPoint::gridToPoint(path_cell,cellSizes)).transpose();
	    GridPoint path1(path_cell.transpose());
	    auto path_ptr= gridMap.find(path1);	   
	    path_cell = path_ptr->second.parent;
	    currentPathPosition--;
	    
	  }while(path_cell(0) != startCell.coord(0) || path_cell(1) != startCell.coord(1) || path_cell(2) != startCell.coord(2));
	  pathBank.row(currentPathPosition) = (GridPoint::gridToPoint(path_cell,cellSizes)).transpose();
	  
	  pathBank.conservativeResize(pathLength+1,3);
	  
	  cout << "path is:"<<pathBank<<endl;
	  
//	  // orientation of reference frame associated with each path point
//	  Matrix<Matrix3f,Dynamic,1> pathOrientation3d(pathBank.rows(),1);
//	  pathOrientation3d =path_orientation3d(pathBank);
//
//	  	  for(int i=0;i<pathOrientation3d.rows();i++)
//		cout<<"refFrame3d:"<<pathOrientation3d(i)<<endl;
//
//
//	  // quaternions to represent the reference frame orientations
//	  Matrix<double,4,Dynamic> orientQuaternion3d(4, pathOrientation3d.rows());
//	  for(int i = 0;i<pathOrientation3d.rows();i++)
//			orientQuaternion3d.col(i) = rotMatrixToQuaternion(pathOrientation3d(i));
//
//	  cout<<"quaternion3d:"<<orientQuaternion3d.transpose()<<endl;
//
//
//	  Eigen::Matrix3d test;
//
//	  test =rotationMatrixTest(orientQuaternion3d(3,0),orientQuaternion3d(0,0),orientQuaternion3d(1,0), orientQuaternion3d(2,0));
//
//	  cout<<"test:"<<test<<endl;

	  // way point reference frame 3d
	  Matrix<Matrix4f,Dynamic,1> path_3d(pathBank.rows(),1);
	  path_3d = way_point_ref3d(pathBank);

	  cout<<"wayPointRef3d done"<<endl;
	  // generate direction vectors
	  Matrix<float,Dynamic,3> directionVec3d(pathBank.rows(),3);
	  directionVec3d = path_direction_vector( pathBank, saliencyThLower, smoothingParameter);
	  cout<<"direction3d:"<<directionVec3d<<endl;

	  // generate 2D path
	  Matrix<float,Dynamic,3> path2d(pathBank.rows(),3);
	  path2d = two_d_path(pathBank);
	  cout<<"2D path is:" <<path2d<<endl;


	  // way point reference frame 2d
	  Matrix<Matrix4f,Dynamic,1> path_2d(path2d.rows(),1);
	  path_2d = way_point_ref2d(path2d);

	  cout<<"wayPointRef2d done"<<endl;
	  //2D direction vectors
	  Matrix<float,Dynamic,3> directionVec2d(pathBank.rows(),3);
	  directionVec2d = path_direction_2d(pathBank,directionVec3d);
	   cout<< "direction vectors 2d:" << directionVec2d<<endl;
	  // output3D and 2D path

	  // flipper control
	  Matrix<int,Dynamic,1> flipperSt(pathBank.rows(),1);
	  flipperSt = flipper_control( pathBank);

	  // heading reference angle
	  Matrix<float,Dynamic,2> thetaRef_wRef(path2d.rows(),2);
	  thetaRef_wRef = trajectoryDefinition(path2d, directionVec2d);

	  // clear structure
		globalPath.oneWayPoint.clear();

		// output csv files
		std::ofstream pathOutput;
		pathOutput.open("path3d.csv");
		std::ofstream pathOutput1;
		pathOutput1.open("pathVectors3d.csv");
		std::ofstream pathOutput2;
		pathOutput2.open("pathVectors2d.csv");
		std::ofstream pathOutput3;
		pathOutput3.open("path2d.csv");
		std::ofstream pathOutput6;
		pathOutput6.open("flipperState.csv");
		
		// output
		for(int i=0;i<pathBank.rows();i++)
		{
			pathPlan::wayPointInfo wayPoint;

			wayPoint.path3d = path_3d(i);
			wayPoint.path2d = path_2d(i);
			wayPoint.directionV = directionVec2d.row(i).head(2);

			wayPoint.flipperState = flipperSt(i);
			wayPoint.thetaRef = thetaRef_wRef(i,0);
			wayPoint.wRef = thetaRef_wRef(i,1);

			globalPath.oneWayPoint.push_back(wayPoint);

			// csv output
			pathOutput<<pathBank(i,0)<<","<<pathBank(i,1)<<","<<pathBank(i,2)<<endl;
			pathOutput1<<directionVec3d(i,0)<<","<<directionVec3d(i,1)<<","<<directionVec3d(i,2)<<endl;
			pathOutput2<<directionVec2d(i,0)<<","<<directionVec2d(i,1)<<","<<directionVec2d(i,2)<<endl;
			pathOutput3<<path2d(i,0)<<","<<path2d(i,1)<<","<<path2d(i,2)<<endl;
			pathOutput6<<flipperSt(i)<<endl;
		}
	}

	cout<<"Path Planning Over in "<<t.elapsed()<<" [seconds]"<<endl;
	t.restart();
	//res.plan_found = true; //A path is feasible
	
	return res.plan_found;
}



bool TensorPlanner::executePlan(tensorPlanner::ExecutePlan::Request& req, tensorPlanner::ExecutePlan::Response& res)
{
	ROS_INFO("Received request to MOVE");
	//TODO: control strategy goes here
	//TODO: publish cmd_vel
	
	int closestWayPoint = 0;
	Vector4f q;
	Vector3f pos;
	Vector2f vD_wD;
	Vector3f pos2d_heading;
	Vector2f pos2dCurrent;
	int flipperConfig=1;
	float headingAngle;
	int i = 0;
	float robotWidth =0.6;
	Matrix<float,Dynamic,3> output2d= MatrixXf::Zero(100000,3);
	Matrix<float,Dynamic,3> ref2d= MatrixXf::Zero(100000,3);
	Matrix<float,Dynamic,3> output3d = MatrixXf::Zero(100000,3);
	Matrix<float,Dynamic,2> outputRobotInput = MatrixXf::Zero(100000,2);
	double rate = 5.0;
	ros::Time oldTime = ros::Time::now();

	while((!errorCalled) && n.ok())
	{
		timer t;

		tf::StampedTransform transform;
		cout << "Iteration: "<< i<< " (errorCalled: "<<errorCalled<<")"<<endl;
		//cout<<"looking for robot pose"<<endl;
		try{
			tf_listener.lookupTransform("/map","/base_link", ros::Time(0),transform);
		}
		catch(tf::TransformException& ex)
		{
			ROS_ERROR("Received exception while looking up transform");
		}
		// convert transformation matrix to transformation matrix
		q << transform.getRotation().x(),transform.getRotation().y(),transform.getRotation().z(), transform.getRotation().w();
		pos << transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z();

		Matrix<float,4,4> robotPose;
		robotPose = generate_transform_matrix(q,pos);
		cout << "robot Pose:" <<robotPose<<endl;
		output3d.row(i) = pos.transpose();

		// find closest way point
		closestWayPoint = closest_way_point(pos,closestWayPoint);
		cout<<"closestway point:"<<closestWayPoint<<endl;

		// 2d localization
		pos2d_heading = robot_2d_localization(robotPose, closestWayPoint);

		pos2dCurrent = pos2d_heading.head(2);
		headingAngle = pos2d_heading(2);
		output2d.row(i) = pos2d_heading.transpose();
		cout<<"pos2d:"<<pos2dCurrent.transpose()<<endl;

		pathPlan::wayPointInfo wayPointRefOutput;
		wayPointRefOutput = globalPath.oneWayPoint[closestWayPoint];
		ref2d.row(i) << wayPointRefOutput.path2d.topRightCorner(2,1).transpose(),wayPointRefOutput.thetaRef;

		// check end condition
		Vector2f dFinal_orientFinal;// first element distance to dest/second is orient difference
		dFinal_orientFinal = endCondition(pos2dCurrent,headingAngle);
		cout << "dfinal:"<<dFinal_orientFinal(0)<<",orientFinal:"<<dFinal_orientFinal(1)<<endl;

		if(dFinal_orientFinal(0)>0.1 || dFinal_orientFinal(1) >M_PI/4)
		{
			vD_wD = lineControl(pos2dCurrent,headingAngle,closestWayPoint);

			if(fabs(vD_wD(1))>0.2)
			{
				vD_wD(0) = vD_wD(0)*0.2/fabs(vD_wD(1));
				vD_wD(1) = vD_wD(1)*0.2/fabs(vD_wD(1));
			}

			/*float vR = vD_wD(0) + 2*vD_wD(1)*robotWidth;
			float vL = vD_wD(0) - 2*vD_wD(1)*robotWidth;

			float a =max(fabs(vL),fabs(vR));
			if(a>0.5)
			{
					vL *= 0.5/a;
					vR *= 0.5/a;
			}

			vD_wD(0) = (vL + vR)/2;
			vD_wD(1) = (vR - vL)/(2*robotWidth);*/
			outputRobotInput.row(i) = vD_wD.transpose();
			// flipper state configuration
			pathPlan::wayPointInfo flipperWayPoint;
			flipperWayPoint = globalPath.oneWayPoint[closestWayPoint];
			flipperConfig = flipperWayPoint.flipperState;
			cout<<"flipper config:"<<flipperConfig<<endl;
		}
		else
		{
			res.plan_executed = true;
			break;
		}
		// publish velocity and angular velocity command

		cout << "vD:"<<vD_wD(0)<<",wD:"<<vD_wD(1)<<endl;
		geometry_msgs::Twist inputVel;
		inputVel.linear.x = vD_wD(0);
		inputVel.linear.y = 0;
		inputVel.linear.z = 0;
		inputVel.angular.x = 0;
		inputVel.angular.y = 0;
		inputVel.angular.z = vD_wD(1);
		
		cmd_vel_pub.publish(inputVel);
		std_msgs::Int32 flipper_msg;
		flipper_msg.data = flipperConfig;
		flipper_cmd.publish(flipper_msg);

		//cout<<"time for loop:"<<t.elapsed()<<endl;
		t.restart();
		//cout<<"lets redo the loop"<<endl;

		

		ros::Duration timePassed = (ros::Time::now() -oldTime);
		ros::Duration timeLeft = ros::Duration(1.0/rate) - timePassed;
		if(timeLeft.toSec()>0)
		{
			timeLeft.sleep();
		} else
		{	
			ros::Duration(0.001).sleep();
			ROS_WARN("function duration too long");
		}
		ros::spinOnce();
		oldTime = ros::Time::now();
		i++;
	}
	
	errorCalled = false;

	output2d.conservativeResize(i+1,3);
	output3d.conservativeResize(i+1,3);
	ref2d.conservativeResize(i+1,3);
	outputRobotInput.conservativeResize(i+1,2);
	std::ofstream pathOutput4;
	pathOutput4.open("robotAct2d.csv");
	std::ofstream pathOutput5;
	pathOutput5.open("robotAct3d.csv");
	std::ofstream pathOutput6;
	pathOutput6.open("reference2d.csv");
	std::ofstream pathOutput8;
	pathOutput8.open("robotInput.csv");


	for(int i=0;i<output2d.rows();i++)
	{
		pathOutput4<<output2d(i,0)<<","<<output2d(i,1)<<","<<output2d(i,2)<<endl;
		pathOutput5<<output3d(i,0)<<","<<output3d(i,1)<<","<<output3d(i,2)<<endl;
		pathOutput6<<ref2d(i,0)<<","<<ref2d(i,1)<<","<<ref2d(i,2)<<endl;
		pathOutput8<<outputRobotInput(i,0)<<","<<outputRobotInput(i,1)<<endl;
	}




	 //Everything when alright
	return res.plan_executed;
}

//-------------------------

// Callback for point cloud
void TensorPlanner::gotCloud(const sensor_msgs::PointCloud2& cloudMsgIn)
{
	typedef typename Nabo::NearestNeighbourSearch<float> NNS;
	const auto eye =  PM::Matrix::Identity(3,3); 
	sensor_msgs::PointCloud2 new_cloud;

	timer t;

	// TODO add frame name parameter, change for /map
	/*if (tf_listener.waitForTransform(cloudMsgIn.header.frame_id, "/odom",
				cloudMsgIn.header.stamp, ros::Duration(1.)))
	{
		pcl_ros::transformPointCloud("/odom", cloudMsgIn, new_cloud, tf_listener);
	} else
	{
		ROS_WARN_STREAM("Couldn't transform input point cloud into /map frame, dropping it.");
		return;
	}*/

	cout << "trying conversion of pt cloud" << endl;
	// Convert point cloud msg in a matrix
	DP cloud(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloudMsgIn));

	cout << "Received : " << cloud.features.cols() << " nb pts" << endl;

	// Convert point cloud
	const int pointsCount = cloud.features.cols();
	if (pointsCount== 0)
	{
		ROS_ERROR("I found no good points in the cloud");
		return;
	}

	std::shared_ptr<NNS> kdTree;
	const int maxNbPts = 100;
	PM::Matches matches(
		typename PM::Matches::Dists(maxNbPts, pointsCount),
		typename PM::Matches::Ids(maxNbPts, pointsCount)
	);

	//for (int i=0; i < cloud.descriptorLabels.size(); i++)
	//	cout << cloud.descriptorLabels[i].text << endl;
	//abort();

	// Build a kdTree with the cloud
	kdTree.reset(NNS::create(cloud.features, cloud.features.rows() - 1, NNS::KDTREE_LINEAR_HEAP));

	// Search for nearest neighbors, put results in 'matches'
	kdTree->knn(cloud.features, matches.ids, matches.dists, maxNbPts, eps, 0, sigma);

	// Prepare memory for tensors
	cloud.allocateDescriptor("eigValues", 3);
	cloud.allocateDescriptor("normals", 3);
	cloud.allocateDescriptor("minTangents", 3);
	cloud.allocateDescriptor("maxTangents", 3);

	// create shortcuts to access the new descriptors
	DP::View eigenValues = cloud.getDescriptorViewByName("eigValues");
	DP::View normals = cloud.getDescriptorViewByName("normals");
	DP::View minTangents = cloud.getDescriptorViewByName("minTangents");
	DP::View maxTangents = cloud.getDescriptorViewByName("maxTangents");

	// for all the points in the cloud
	for (int i = 0; i < pointsCount; ++i)
	{
		const auto voter = cloud.features.col(i).head(3);
		PM::Matrix tens = eye;

		// for all the nearest neigbors of that points
		for(int j = 0; j < maxNbPts; j++)
		{
			if(matches.dists(j,i) == numeric_limits<float>::infinity())
			{
				// early out: no more closest point is dist = inf
				break;
			}
			else
			{
				const int refIndex(matches.ids(j,i));
				const float r = matches.dists(j,i);//already squared!
				const auto votee = cloud.features.col(refIndex).head(3);
				const auto v = votee - voter;
				PM::Matrix vv = v*v.transpose().normalized();// Revision- SM	
				tens += exp(-r/pow(sigma,2))*(eye - vv);		
			}
		}
		
		const Eigen::EigenSolver<PM::Matrix> solver(tens);
		PM::Matrix eigenVa = solver.eigenvalues().real();
		PM::Matrix eigenVe = solver.eigenvectors().real();

		sortTensor(eigenVa, eigenVe);

		// Add computes tensor to the point cloud
		eigenValues.col(i) = eigenVa;
		normals.col(i) = eigenVe.col(0);
		minTangents.col(i) = eigenVe.col(1);
		maxTangents.col(i) = eigenVe.col(2);
	}

	cout << "Done computing sparse voting in " << t.elapsed() << " [s]" << endl;

	t.restart();
	mergeMap(cloud, cloudMsgIn.header.seq);
	cout << "Done computing dense voting in " << t.elapsed() << " [s]" << endl;
	
	// DEBUG: dump vtk file
	t.restart();
	cloud.save("DEBUG_sparseVoting.vtk");
	saveGridToVTK(gridMap);
	cout << "Done dumping VTK files in " << t.elapsed() << " [s]" << endl;
}


// Helper functions

Eigen::Vector3f TensorPlanner::realignNormal(const PM::Matrix& eigenVe, const Eigen::Vector3f& ref)
{
		Eigen::Vector3f normal_out;
		const Eigen::Vector3f normal = eigenVe.col(0);
		const double orient = ref.dot(normal);
		if(orient < 0.0)
			normal_out = -normal;
		else
			normal_out = normal;
		
	return normal_out; 
}

void TensorPlanner::sortTensor(PM::Matrix& eigenVa, PM::Matrix& eigenVe) // to sort in descending order of eigenvalues
{
	// REVISION: Author:SM/Date: 16.10
	// TODO: ensure right-handed axis
	for(unsigned int i=0;i<eigenVa.rows();i++){
	   for(unsigned int j= 0;j<i;j++){
			if(eigenVa(j)<=eigenVa(i)){
				eigenVa.row(j).swap(eigenVa.row(i));
				eigenVe.col(j).swap(eigenVe.col(i));
			}
		}
	}	

}

PM::Vector TensorPlanner::serializeEigVec(const PM::Matrix eigenVe)
{
	// serialize row major
	const int eigenVeDim = eigenVe.cols();
	PM::Vector output(eigenVeDim*eigenVeDim);
	for(int k=0; k < eigenVe.cols(); k++)
	{
		output.segment(k*eigenVeDim, eigenVeDim) = 
			eigenVe.row(k).transpose();
	}

	return output;
}

// transformation matrix
Eigen::Matrix4f TensorPlanner::generate_transform_matrix(Vector4f& quaternion,Vector3f& pos)
{

	Eigen::Matrix4f T;
	Matrix3f R;


	const float a = quaternion(3);
	const float b = quaternion(0);
	const float c = quaternion(1);
	const float d = quaternion(2);

	const float R11 = a*a +b*b -c*c - d*d;
	const float R12 = 2*b*c - 2*a*d;
	const float R13 = 2*b*d + 2*a*c;
	const float R21 = 2*b*c + 2*a*d;
	const float R22 = a*a -b*b +c*c -d*d;
	const float R23 = 2*c*d - 2*a*b;
	const float R31 = 2*b*d - 2*a*c;
	const float R32 = 2*c*d + 2*a*b;
	const float R33 = a*a -b*b -c*c +d*d;

	// rotation matrix corresponding to quaternion
	R << R11,R12,R13,
		 R21,R22,R23,
		 R31,R32,R33;

	T.topLeftCorner(3,3) << R;
	T.bottomLeftCorner(1,4)<<0,0,0,1;
	T.topRightCorner(3,1)<< pos;

	return T;
}

Eigen::Matrix3f TensorPlanner::rotationMatrix(Eigen::Vector3f& u_rot, const float theta)
{
	
	Eigen::Vector4f quaternion;
	Eigen::Matrix3f R;
	
	// generate quaternion
	quaternion << cos(theta/2), u_rot(0)*sin(theta/2),u_rot(1)*sin(theta/2),u_rot(2)*sin(theta/2);

	const float a = quaternion(0);
	const float b = quaternion(1);
	const float c = quaternion(2);
	const float d = quaternion(3);

	// beware of the signs!!!
	const float R11 = a*a +b*b -c*c - d*d;
	const float R12 = 2*b*c - 2*a*d;
	const float R13 = 2*b*d + 2*a*c;
	const float R21 = 2*b*c + 2*a*d;
	const float R22 = a*a -b*b +c*c -d*d;
	const float R23 = 2*c*d - 2*a*b;
	const float R31 = 2*b*d - 2*a*c;
	const float R32 = 2*c*d + 2*a*b;
	const float R33 = a*a -b*b -c*c +d*d;

	// rotation matrix corresponding to quaternion
	R << R11,R12,R13,
		 R21,R22,R23,
		 R31,R32,R33;

	return R;
}

// generate bounding box based on position and orientation of cell 
Eigen::Matrix<int,6,1> TensorPlanner::generateBoundingBox(const Vi& grid_cell, V3f& robotDimension, V3f& normal)
{
	// compute half lengths of robot
	const float robotLengthHalf = robotDimension(0)/2;
	const float robotBreadthHalf = robotDimension(1)/2;

	//dimensions in vector format
	Eigen::Vector3f xBox(robotLengthHalf,0,0);
	Eigen::Vector3f yBox(0,robotBreadthHalf,0);
	Eigen::Vector3f zBox(0,0,robotDimension(2));

	// Output box vector
	Eigen::Matrix<int,6,1> boxBound;

	// Rotation angles
	Eigen::Vector3f e_k(0,0,1); // unit z vector

	Eigen::Vector3f e_k_current; // current surface normal

	e_k_current =normal;

	Eigen::Vector3f u_rot;// rotation axis for quaternion
	u_rot = (e_k.cross(e_k_current)).normalized();

	const float theta = acos(e_k.dot(e_k_current));

	// Rotation matrix
	Eigen::Matrix3f R;
	R = rotationMatrix(u_rot,theta);
	
	Eigen::Vector3f xCurrent;
	Eigen::Vector3f yCurrent;
	Eigen::Vector3f zCurrent;

	xCurrent = R*xBox;
	yCurrent = R*yBox;
	zCurrent = R*zBox;

	Eigen::Vector3f pMin;
	Eigen::Vector3f pMax;

	Eigen::Vector3f p = GridPoint::gridToPoint(grid_cell, cellSizes);

	pMin = ((xCurrent.array().abs() + yCurrent.array().abs() + zCurrent.array().abs()));
	pMax = ((xCurrent.array().abs() + yCurrent.array().abs() + zCurrent.array().abs()));

	pMin.head(2) = (p.head(2) - pMin.head(2));
	pMax.head(2) = (p.head(2) + pMax.head(2));

	const float xz = xCurrent(2);
	const float yz = yCurrent(2);

	pMin(2) = grid_cell(2)*cellSizes(2) - cellSizes(2) - fabs(xz) - fabs(yz);
	pMax(2) = grid_cell(2)*cellSizes(2) + pMax(2);

	const int minX = floor(pMin(0)/cellSizes(0));
	const int minY = floor(pMin(1)/cellSizes(1));
	const int minZ = floor(pMin(2)/cellSizes(2));

	const int maxX = ceil(pMax(0)/cellSizes(0));
	const int maxY = ceil(pMax(1)/cellSizes(1));
	const int maxZ = ceil(pMax(2)/cellSizes(2));

	boxBound << minX,minY,minZ,maxX,maxY,maxZ;

	return boxBound;

}

// verify if cell part of bounding box representing the robot dimensions
bool TensorPlanner::verifyBoundingBox(const Vi& grid_cell, const Vi& robot_cell,Eigen::Vector3f& robotDimension,Eigen::Vector3f& normal)
{
		bool valid_cell; // boolean to affirm if cell is valid and inside the robot to check for obstacles
		// Rotation angles
		Eigen::Vector3f e_k(0,0,1); // unit z vector

		Eigen::Vector3f e_k_current; // current surface normal

		e_k_current =normal;

		Eigen::Vector3f u_rot;// rotation axis for quaternion
		u_rot = (e_k.cross(e_k_current)).normalized();

		const float theta = acos(e_k.dot(e_k_current));

		// Rotation matrix
		Eigen::Matrix3f R;
		R = rotationMatrix(u_rot,theta);

		// Inverse rotation matrix
		Eigen::Matrix3f invR;
		invR = R.inverse();

		// current position
		Eigen::Vector3f p_current = GridPoint::gridToPoint(robot_cell, cellSizes);
		Eigen::Vector3f p_parent  = GridPoint::gridToPoint(grid_cell, cellSizes);

		// position of cell after rotation and translation back to origin
		Eigen::Vector3f p;

		p = invR*(p_current - p_parent);

		if(p(0)<= robotDimension(0)/2 && p(0)>=-robotDimension(0)/2 && p(1)<= robotDimension(1)/2 && p(1)>=- robotDimension(1)/2 && p(2)<= robotDimension(1) && p(2)>=-cellSizes(2))
		{

			valid_cell = true;
		}
		else
			valid_cell = false;

		return valid_cell;
}

float TensorPlanner::cost_heading(const Eigen::Vector3f& eigVe_child, const Eigen::Vector3f& eigVe_parent, const Eigen::Vector3f& pos_child, const Eigen::Vector3f& pos_parent,const float heading_factor)
{
	float costHeading;
	float changeSurfaceNormal;
	Eigen::Vector3f movementVector;
	Eigen::Vector3f yBodyFixed;
	Eigen::Vector3f yHeading;
	float approachAngle;
	
	//eigVe_child.normalize();
	//eigVe_parent.normalize();
	
	float temp = eigVe_parent.dot(eigVe_child);
	if(temp>1)
	  temp = 1;
	else if(temp<-1)
	  temp = -1;
	
	// compute change in surface normal 
	changeSurfaceNormal = acos(temp);
	
	if(changeSurfaceNormal>M_PI/2)
	  changeSurfaceNormal = M_PI - changeSurfaceNormal;
	
	// movement vector
	movementVector = pos_child - pos_parent;
	movementVector.normalize();
	
	// body fixed y axis
	yBodyFixed = eigVe_parent.cross(movementVector);
	yBodyFixed.normalize();
	
	// y axis due to change in surface normal
	yHeading = eigVe_parent.cross(eigVe_child);
	yHeading.normalize();
	
	approachAngle = 1 - fabs(yBodyFixed.dot(yHeading));
	
	costHeading = heading_factor*changeSurfaceNormal*approachAngle;
	
	return costHeading;
}

float TensorPlanner::cost_saliency(const float maxGlobalSal, const float saliencyCurrent, const float saliencyCostTh, const float saliencyThLower,const float safety_factor)
{
	float costSaliency;
	
	if(saliencyCurrent>= saliencyCostTh)
	  costSaliency = safety_factor*(1- ((log(saliencyCurrent) - log(saliencyCostTh))/(log(maxGlobalSal)-log(saliencyCostTh))));
	else
	  costSaliency = safety_factor*tan((-M_PI/2)*(1 + ((saliencyCurrent - (saliencyThLower-1))/(saliencyCostTh -(saliencyThLower-1)))))+1;
	
	return costSaliency;
}

float TensorPlanner::cost_orientation(const Eigen::Vector3f& eigVe_child,const float stability_factor)
{
	float costOrientation;
	Eigen::Vector3f gravity_vector;
	gravity_vector<<0,0,1;
	float theta=0;
	
	float temp = acos(gravity_vector.dot(eigVe_child));
	if(temp>1)
	  temp = 1;
	else if(temp<-1)
	  temp = -1;
	
	theta = acos(temp);
	
	costOrientation =  pow(tan((90/(maxOrientationAngle*180/M_PI))*theta)*stability_factor,2);
	
	return costOrientation;
}

Matrix<float,Dynamic,3> TensorPlanner::extract_path_normal(Matrix<float,Dynamic,3>& path)
{    // extract surface normals along path
    Matrix<float,Dynamic,3> pathNormal(path.rows(),3);
    Vector3f pathPoint;
    Vector3i pathCell;
    
    for(int i=0;i<path.rows();i++)
    {
	  pathPoint = path.row(i).transpose();
	  pathCell = GridPoint::pointToGrid(pathPoint, cellSizes);
	  GridPoint pathInd(pathCell);
	  auto path_ptr =gridMap.find(pathInd);
	  
	  const Eigen::EigenSolver<PM::Matrix> solver(path_ptr->second.tensorStick);
	  PM::Matrix eigenVa = solver.eigenvalues().real();
	  PM::Matrix eigenVe = solver.eigenvectors().real();
	  sortTensor(eigenVa, eigenVe);
	  Vector3f pathN = realignNormal(eigenVe, path_ptr->second.viewDir);
	  pathNormal.row(i) = pathN.transpose();
    }
    return pathNormal;
}

Matrix<Matrix4f,Dynamic,1> TensorPlanner::way_point_ref3d(Matrix<float,Dynamic,3>& path)
{
    Matrix<Matrix4f,Dynamic,1> wayPointRef3d(path.rows(),1);
    Matrix<float,Dynamic,3> pathNormal(path.rows(),3);
    Matrix<float,Dynamic,3> movementVector(path.rows(),3);
    Matrix4f q;
    Vector3f x_axis;
    Vector3f y_axis;
    Vector3f z_axis;
    pathNormal = extract_path_normal(path);

    for(int i =0;i<path.rows();i++)
    {
	  if(i>0)
		movementVector.row(i) = path.row(i) - path.row(i-1);
	  else
		movementVector.row(i) = path.row(i+1) - path.row(i);
		
	x_axis = movementVector.row(i).transpose().normalized();
	  
	z_axis = pathNormal.row(i).transpose().normalized();
	  
	y_axis = z_axis.cross(x_axis);
	  
	y_axis.normalize();
	q.topLeftCorner(3,3) << x_axis,y_axis,z_axis;
	q.bottomLeftCorner(1,4)<< 0,0,0,1;
	q.topRightCorner(3,1)<<path.row(i).transpose();
		  
	wayPointRef3d(i) = q;
    }
    
    return wayPointRef3d;
}

Matrix<float,Dynamic,3> TensorPlanner::path_direction_vector(Matrix<float,Dynamic,3>& path, const float saliencyThLower, const int smoothingParameter)
{
	
  Matrix<float,Dynamic,3> pathDirection(path.rows(),3);
  Matrix<float,3,1> movementVector;
  Vector3f pVector;
  Vector3f pEnd;
  Vector3f pStart;
  Vector3f pCurrent;
  Vector3f unitV;
  Vector3f pTest;

  for(int i = 0;i<path.rows()-1;i++)
  {
	 pathDirection.row(i) = path.row(i+1) - path.row(i);
	 pathDirection.row(i).normalize();
	cout<<"hoi1"<<endl;
	
	 pStart = path.row(i);
	 
	 int isSafe=0,safetyTh=0;
	 int j = i+2;

	  while(j<i+smoothingParameter && isSafe == safetyTh && j<path.rows())
	  {
		pEnd = path.row(j);
cout<<"hoi2"<<endl;

		pVector = pEnd -pStart;
		unitV = pVector.normalized();
		float l1 = pVector.norm();
		
		isSafe = 0;
		safetyTh = 0;
		float l2 = 0;
		int step =1;
cout<<"hoi3"<<endl;

		pCurrent = pStart + step*unitV;
		  while(l2<l1)
		  {
				for(int kBox = floor(pCurrent(2)); kBox<=ceil(pCurrent(2));kBox++)
				{
				  for(int jBox = floor(pCurrent(1)); jBox<=ceil(pCurrent(1));jBox++)
				  {
					for(int iBox = floor(pCurrent(0)); iBox<= ceil(pCurrent(0));iBox++)
					{

						  safetyTh++;
						  Vi box_cell(3,1);
						  box_cell<<iBox,jBox,kBox;
						  
						  GridPoint box(box_cell);
						  auto box_ptr = gridMap.find(box);
cout<<"hoi4"<<endl;
						  if(box_ptr!=gridMap.end())
						  {	
							if(box_ptr->second.walkability ==1) // walkable
							 	isSafe++;
						  }	  
					}
				  }
				}
				step++;
				pCurrent = pStart + step*unitV;
				pTest = pCurrent - pStart;
				l2 = pTest.norm();
cout<<"hoi5"<<endl;

		  };
		  
		  if(isSafe == safetyTh)
		  {
			pVector.normalize();

			pathDirection.row(i) = pathDirection.row(i) + pVector.transpose();
		  }

		  j++;
	cout<<"hoi6"<<endl;
	
	    }

	    if(pathDirection.row(i).norm()!=0)
			pathDirection.row(i).normalize(); 
	}
	pathDirection.row(pathDirection.rows()-1)<<0,0,0;
cout<<"hoi6"<<endl;

	return pathDirection;
}

Matrix<float,Dynamic,3> TensorPlanner::path_direction_2d(Matrix<float,Dynamic,3>& path,Matrix<float,Dynamic,3> pathDirection)
{
	
    Matrix<float,Dynamic,3> pathDirection2d(path.rows(),3);
    Vector3f movementVectorInit;
    Matrix<float,Dynamic,3> pathNormal(path.rows(),3);
    Vector3f surfaceNormal;
    //movementVectorInit = (path.row(1) - path.row(0)).transpose().normalized();
    pathNormal = extract_path_normal(path);
    Vector3f planarDirection;
    Vector3f movementInit2d;
    movementInit2d << 1,0,0;
    Matrix3f Rot;
	
    for(int i =0;i<path.rows();i++)
    {
	  if(i==path.rows()-1)
		pathDirection2d.row(i) <<0,0,0;
	  else
	  {
		surfaceNormal = (pathNormal.row(i)).transpose();
		surfaceNormal.normalize();
		
		
		planarDirection = pathDirection.row(i).transpose() - surfaceNormal*(pathDirection.row(i).transpose().dot(surfaceNormal));
		planarDirection.normalize();	
		
		if(i==0)
				movementVectorInit = planarDirection;
		
		float temp = movementVectorInit.dot(movementInit2d);
		if(temp>1)
		    temp = 1;
		else if(temp<-1)
		    temp = -1;
		
		float theta = acos(temp);
		
		// to decide which direction to rotate, the cross product is taken
		Vector3f temp1;
		temp1 = movementVectorInit.cross(movementInit2d);

		if(temp1.dot(surfaceNormal)>0)
		    theta = -theta;
		
		Rot<< cos(theta), sin(theta),0,-sin(theta),cos(theta),0,0,0,1;

		planarDirection = Rot*planarDirection;
		
		RowVector2f temp3 = planarDirection.head(2).normalized().transpose();
		pathDirection2d.row(i)<<temp3,0 ;		
	  }
	  
    }

    return pathDirection2d;
    
}

Matrix<Matrix4f,Dynamic,1> TensorPlanner::way_point_ref2d(Matrix<float,Dynamic,3>& path2d)
{
	int pathLength = path2d.rows();
	Matrix<Matrix4f,Dynamic,1> wayPointRef2d(pathLength,1);
	Matrix4f q;

	Vector3f movementVector;
	Vector3f x_axis;
	Vector3f y_axis;
	Vector3f z_axis;
	z_axis <<0,0,1;
	Vector3f path;


	for(int i = 0;i<pathLength;i++)
	{
		path = path2d.row(i).transpose();

		// determine movement vector
		if(i>0)
			movementVector= path2d.row(i) - path2d.row(i-1);
		else
			movementVector= path2d.row(i+1) - path2d.row(i);

		// define reference frame
		x_axis = movementVector.normalized().transpose();
		y_axis = z_axis.cross(x_axis);
		y_axis.normalize();

		q.topLeftCorner(3,3) << x_axis,y_axis,z_axis;
		q.bottomLeftCorner(1,4)<< 0,0,0,1;
		q.topRightCorner(3,1)<<path;

		wayPointRef2d(i) = q;

	}

	return wayPointRef2d;
}

Matrix<float,Dynamic,3> TensorPlanner::two_d_path(Matrix<float,Dynamic,3>& path)
{
	Matrix<float,Dynamic,3> path2d(path.rows(),3);
    Matrix<float,Dynamic,3> pathNormal(path.rows(),3);
    Matrix<float,Dynamic,3> movementVector(path.rows(),3);
    Vector3f x_axis;
    Vector3f y_axis;
    Vector3f z_axis;
    z_axis <<0,0,1;
    float theta=0;
    
    for(int i  = 0; i< movementVector.rows();i++)
    {
	  if(i>0)
		movementVector.row(i) = path.row(i) - path.row(i-1);
	  else
		movementVector.row(i) = path.row(i+1) - path.row(i);
	  
    }

    // extract surface normals along path
    pathNormal = extract_path_normal(path);
    path2d.row(0) << 0,0,0;

	for (int nPath = 0;nPath<path.rows()-1;nPath++)
    {
	  
		Vector3f surfaceNormal;
		Vector3f planarMove1;
		Vector3f planarMove2;
		Vector3f move1;
		Vector3f move2;

		
		move2 = (movementVector.row(nPath+1)).transpose();
		move1 = (movementVector.row(nPath)).transpose();

		// surface normal at the way point
		surfaceNormal = (pathNormal.row(nPath) + pathNormal.row(nPath+1)).transpose();
	
		surfaceNormal.normalize();

		// projection of the surface normal in plane perpendicular to surface normal
		planarMove1 = move1 - surfaceNormal*(move1.dot(surfaceNormal));
		planarMove2 = move2 - surfaceNormal*(move2.dot(surfaceNormal));

		// define the robot body fixed reference frame
		Vector3f z_rot = surfaceNormal;
		Vector3f x_rot = planarMove1.normalized();
		Vector3f y_rot = z_rot.cross(x_rot).normalized();
		
		Matrix3f Rot;
		Rot<< x_rot,y_rot,z_rot;
		planarMove2 = ((planarMove2.transpose()*Rot));
		double a = planarMove2(0);
		double b = planarMove2(1);
		double turn = atan2(b,a);
		theta = theta + turn;
		path2d.row(nPath+1) << path2d(nPath,0) + move2.norm()*cos(theta), path2d(nPath,1) + move2.norm()*sin(theta),0;
		
    }
    
    return path2d;
}

Matrix<int,Dynamic,1> TensorPlanner::flipper_control(Matrix<float,Dynamic,3>& path)
{
	Matrix<int,Dynamic,1> fs_current(path.rows(),1);
    VectorXf robotOrient(path.rows(),1);
    Matrix<float,Dynamic,3> pathNormal(path.rows(),3);
    Matrix<int,Dynamic,1> isEdge(path.rows(),1);
    Vector3f gravity_vector;
    Vector3f movementVector;
    gravity_vector<<0,0,1;
    int Yes = 1,No = 0;
    Vector3f bodyFixedY; // robot frame y axis
    Vector3f slopeY; // y axis from change in slope
    int approachFS = 8; // approach waypoints to edge that are set to transition state
    int followFS = 3; // after edge waypoints that are set to transition state

    // extract surface normals along path
    pathNormal = extract_path_normal(path);
    
    // set steady state flipper configurations
    for(int i=0;i<path.rows();i++)
    {
    	float temp = pathNormal.row(i).transpose().dot(gravity_vector);
	  if(temp>1)
		temp = 1;
	  else if(temp<-1)
		temp = -1;
	  
	  robotOrient(i) = acos(temp);
	cout<<"robot orientation:"<<robotOrient(i)<<endl;	  
	if(robotOrient(i)>orientTh)
		fs_current(i) = fs_MaxTraction;
	  else
		fs_current(i) = fs_FlatDriving;
    }

    // edge detection
    for(int i =0;i<path.rows()-1;i++)
    {
		if(fs_current(i)==fs_FlatDriving && fs_current(i+1)==fs_MaxTraction)
		{
		    isEdge(i) = Yes;
		}
		else if(fs_current(i)==fs_MaxTraction && fs_current(i+1)==fs_FlatDriving)
		{
			isEdge(i) = Yes;
		}
		else
		{
			isEdge(i) = No;
		}
    }

    // determine transition flipperstates
    for(int i=1;i<path.rows();i++)
    {
    	  if(isEdge(i)==Yes)
    	  {
    		  Vector3f slopeY;

    		  // the movement vector approaching the edge
    		  movementVector = (path.row(i) - path.row(i-1)).transpose();
		  
    		  movementVector.normalize();
    		  // the body fixed y axis given by cross product between surface normal and movement vector( k X i)
    		  bodyFixedY = pathNormal.row(i-1).transpose().cross(movementVector);
    		  slopeY = pathNormal.row(i+1).transpose().cross(pathNormal.row(i-1).transpose());

    		  if((bodyFixedY.dot(slopeY))>=0)
    		  {
    			  for(int slopeUp = i- approachFS;slopeUp<=i+ followFS;slopeUp++)
    			  {
    				  if(slopeUp>=0 && slopeUp<fs_current.rows())
    					  fs_current(slopeUp) = fs_ApproachForward;
    			  }
    		  }
    		  else if((bodyFixedY.dot(slopeY))<0)
    		  {
    			for(int slopeTipOver = i- approachFS;slopeTipOver<=i+ followFS;slopeTipOver++)
    			{
				if(slopeTipOver>=0 && slopeTipOver<fs_current.rows())
    				fs_current(slopeTipOver) = fs_TipOver;
			}
    		  }
    	  }
    }
    return fs_current;
}

Matrix<float,Dynamic,2> TensorPlanner::trajectoryDefinition(Matrix<float,Dynamic,3>& path2d, Matrix<float,Dynamic,3>& vector2d)
{
	int pathLength = path2d.rows();
	Matrix<float,Dynamic,2> thetaRef_wRef(pathLength,2);// first column is reference heading angle. second column is reference angular velocity
	float tRef=0;
	VectorXf thetaRef(pathLength);
	VectorXf wRef(pathLength);

	// reference heading angle
	for(int i =0;i<pathLength;i++)
	{
		float y = vector2d(i,1);
		float x = vector2d(i,0);
		thetaRef(i) =(atan2(y,x));
		float temp1 = 2*M_PI;
		float temp2 = thetaRef(i);
		thetaRef(i) = temp2 - floor(temp2/temp1)*(temp1);
	}

	// reference angular velocity
	for(int i =0;i<pathLength;i++)
	{
		if(i==0)
		{
			float dist;
			dist = (path2d.row(i+1).head(2)-path2d.row(i).head(2)).norm();
			tRef = dist/vRef;
			float dtheta;
			dtheta = thetaRef(i+1) - thetaRef(i);
			if(fabs(dtheta)>M_PI)
				dtheta = 2*M_PI - fabs(dtheta);
			wRef(i) = dtheta/tRef;
		}
		else
		{
			float dist;
			dist = (path2d.row(i).head(2)-path2d.row(i-1).head(2)).norm();
			tRef = dist/vRef;
			float dtheta;
			dtheta = thetaRef(i) - thetaRef(i-1);
			if(fabs(dtheta)>M_PI)
				dtheta = 2*M_PI - fabs(dtheta);

			wRef(i) = dtheta/tRef;
		}
	}

	thetaRef_wRef << thetaRef,wRef;
	return thetaRef_wRef;

}

Vector2f TensorPlanner::endCondition(Vector2f& pos2dCurrent,const float headingAngle)
{
	int pathLength = globalPath.oneWayPoint.size();
	pathPlan::wayPointInfo wayPoint;
	wayPoint = globalPath.oneWayPoint[pathLength-1];
	float thetaRef = wayPoint.thetaRef;
	Vector2f dFinal_orientFinal;

	// distance to target
	float dFinal;
	dFinal = (wayPoint.path2d.topRightCorner(2,1) - pos2dCurrent).norm();

	// orientation difference with target pose
	float orientFinal;
	orientFinal =fabs(headingAngle - thetaRef);

	dFinal_orientFinal << dFinal, orientFinal;

	return dFinal_orientFinal;
}

int TensorPlanner::closest_way_point(Vector3f& pos, int closestWayPoint)
{
	int pathLength = globalPath.oneWayPoint.size();
	pathPlan::wayPointInfo wayPoint;
	// find closest way point
	float cur_dist;
	float mymin = (pos-globalPath.oneWayPoint[closestWayPoint].path3d.topRightCorner(3,1)).norm();
	int mywp = closestWayPoint;
	cout << "Previous closest: " << closestWayPoint << ": " << mymin << endl;
	for(int i=closestWayPoint+1; i<pathLength; i++)
	{
		cur_dist = (pos-globalPath.oneWayPoint[i].path3d.topRightCorner(3,1)).norm();
		cout << globalPath.oneWayPoint[i].path3d.topRightCorner(3,1).transpose() << " " << pos.transpose() << " " << cur_dist << endl;
		if (cur_dist<mymin)
		{
			mymin = cur_dist;
			mywp = i;
		}
	}
	
	return mywp;
}

Vector3f TensorPlanner::robot_2d_localization(Matrix<float,4,4> robotPose, int closestWayPoint)
{
		pathPlan::wayPointInfo wayPoint;
		Vector3f pos2d_theta2d;
		Vector2f pos2d;
		float theta2d;

//		// find closest way point
//		VectorXf dist(pathLength);
//
//		for(int i=0;i<pathLength;i++)
//		{
//			wayPoint = globalPath.oneWayPoint[i];
//			float temp1 = (wayPoint.path3d.topRightCorner(3,1) - robotPose.topRightCorner(3,1)).norm();
//			dist(i) = abs(temp1);
//		}
//
//		for(int i =closestWayPoint;i<dist.rows();i++)
//		{
//			if(dist(i)<dist(closestWayPoint))
//			{
//				closestWayPoint = i;
//			}
//		}

		//Representation of robot pose in closest way point reference frame
		wayPoint = globalPath.oneWayPoint[closestWayPoint];
		Matrix4f wayPointRF =wayPoint.path3d ; // reference way point reference frame
		Matrix4f qRobot_wayPointRF; // robot in the way point reference frame
		Matrix4f Rot;
		Rot.topLeftCorner(3,3) = wayPointRF.topLeftCorner(3,3).transpose();
		Rot.topRightCorner(3,1) = -wayPointRF.topLeftCorner(3,3).transpose()*wayPointRF.topRightCorner(3,1);
		Rot.bottomLeftCorner(1,4) << 0,0,0,1;

		qRobot_wayPointRF = Rot*robotPose;

		// reorient surface normal to align with local z axis
		Vector3f z_wayPoint;
		z_wayPoint<< 0,0,1;

		Vector3f zRobot_wayPointRF = qRobot_wayPointRF.block(0,2,3,1); // z axis from the 4X4 matrix
		zRobot_wayPointRF. normalize();

		// determine axis of rotation
		Vector3f uRot_wayPointRF = zRobot_wayPointRF.cross(z_wayPoint);
		uRot_wayPointRF.normalize(); // beware of 0 if they're aligned

		float thetaRot_wayPointRF = (acos(z_wayPoint.dot(zRobot_wayPointRF))); // beware of numerical >1

		// rotation matrix to align the robot reference frame and way point reference frame
		Matrix3f Rot_wayPointRF;
		Rot_wayPointRF = rotationMatrix(uRot_wayPointRF,thetaRot_wayPointRF);

		//Alignment transformation matrix
		Matrix4f T_wayPointRF;
		T_wayPointRF.topLeftCorner(3,3) = Rot_wayPointRF;
		T_wayPointRF.bottomLeftCorner(1,4) << 0,0,0,1;
		T_wayPointRF.topRightCorner(3,1) << 0,0,0;

		// Realigned robot reference frame
		Matrix4f qReorient_wayPointRF = T_wayPointRF*qRobot_wayPointRF;
		/////////
		// CHECK IF NORMALIZATION NECESSARY OF X,Y COMPONENTS ONLY

		// ignore the z components of the reoriented robot reference frame
		qReorient_wayPointRF.row(2) << 0,0,1,0;
		qReorient_wayPointRF.row(3) << 0,0,0,1;
		
		float theta_temp = atan2(qReorient_wayPointRF(1,0),qReorient_wayPointRF(0,0));
		Vector3f u_temp;
		u_temp<< 0,0,1;

		Matrix3f Rot_temp;
		Rot_temp = rotationMatrix(u_temp,theta_temp);
	
		Matrix4f q_temp;
		q_temp.topLeftCorner(3,3) = Rot_temp;
		q_temp.bottomLeftCorner(1,4)<<0,0,0,1;
		q_temp.topRightCorner(3,1)<< qReorient_wayPointRF(0,3),qReorient_wayPointRF(1,3),0;

		Matrix4f waypointRF2d;
		waypointRF2d = wayPoint.path2d;
		Matrix4f q2d = waypointRF2d*q_temp;
 
		
		pos2d = q2d.topRightCorner(2,1);

		float yOrient =q2d(1,0);
		float xOrient = q2d(0,0);
		theta2d = atan2(yOrient,xOrient);

		pos2d_theta2d << pos2d,theta2d;

		return pos2d_theta2d;
}
Vector2f TensorPlanner::lineControl(Vector2f& pos2dCurrent,const float headingAngle,int closestWayPoint)
{
	pathPlan::wayPointInfo wayPoint;
	wayPoint = globalPath.oneWayPoint[closestWayPoint];

	Vector2f path2d;
	path2d = wayPoint.path2d.topRightCorner(2,1);
	double thetaRef = wayPoint.thetaRef;
	double wRef = wayPoint.wRef;
	Vector2f vD_wD;
	Vector3f errorWF;
	Vector3f errorBF;
	float vD;
	float wD;


	errorWF.head(2) = path2d - pos2dCurrent;
	float etheta = headingAngle - thetaRef;
	float temp1 = 2*M_PI;
	float temp2 = etheta;
	etheta = temp2 - floor(temp2/temp1)*(temp1);

	if(etheta>M_PI)
		etheta = etheta - 2*M_PI;

	errorWF(2) = etheta;

	Matrix3f Rot;

	Rot << cos(headingAngle),sin(headingAngle),0,-sin(headingAngle),cos(headingAngle),0,0,0,1;

	errorBF = Rot*errorWF;
	float eBx = errorBF(0);
	float eBy = errorBF(1);
	float eBth = errorBF(2);

	if(closestWayPoint==0 && fabs(eBth)>M_PI/4)
	{
		cout<<"control 1"<<endl;
		vD =0;
		wD = -kRot*eBth;
		}
	else
	{
		cout<<"control 2"<<endl;
		vD = fabs(vRef*cos(eBth) +kx*eBx);

		if(fabs(eBth)>0.001)
			wD = wRef- kth*eBth + (ky*vD*sin(eBth)/eBth)*eBy;
		else
			wD = wRef - kth*eBth + ky*vD*eBy;
	}
	//	}

	vD_wD << vD,wD;
	cout<<"vD wD from controller:"<<vD_wD(0)<<","<<vD_wD(1)<<endl;
	return vD_wD;
}


void TensorPlanner::saveGridToVTK(GridMap &gridMap)
{
	DP::Labels featureLabels;
	featureLabels.push_back(DP::Label("x", 1));
	featureLabels.push_back(DP::Label("y", 1));
	featureLabels.push_back(DP::Label("z", 1));
	DP::Labels descriptorLabels;
	descriptorLabels.push_back(DP::Label("inliers", 1));
	descriptorLabels.push_back(DP::Label("eigValues", 3));
	descriptorLabels.push_back(DP::Label("normals", 3));
	descriptorLabels.push_back(DP::Label("observationDirections", 3));

	DP cloud(featureLabels, descriptorLabels, gridMap.size());
	DP::View inliers = cloud.getDescriptorViewByName("inliers");
	DP::View eigValues= cloud.getDescriptorViewByName("eigValues");
	DP::View normals = cloud.getDescriptorViewByName("normals");
	DP::View obsDirection = cloud.getDescriptorViewByName("observationDirections");
	
	int i=0;
	for(auto iter=gridMap.begin(); iter != gridMap.end(); iter++, i++)
	{
		// add coordinates of the grid
		const Vf pt = GridPoint::gridToPoint(iter->first.coord, cellSizes);
		cloud.features.col(i) = pt;
// 		
		// add normal vectors
		// TODO: consider having a function for normal extraction
		const Eigen::EigenSolver<PM::Matrix> solver(iter->second.tensorStick);
		PM::Matrix eigenVa = solver.eigenvalues().real();
		PM::Matrix eigenVe = solver.eigenvectors().real();

		sortTensor(eigenVa, eigenVe);

		// Add computes tensor to the point cloud
		// TODO: move that to a function
		eigValues.col(i) = eigenVa;
		const Eigen::Vector3f normal = eigenVe.col(0);
		const Eigen::Vector3f obsDir = iter->second.viewDir;
		const double orient = obsDir.dot(normal);
		if(orient < 0.0)
			normals.col(i) = -normal;
		else
			normals.col(i) = normal;

		// Add observation direction
		obsDirection.col(i) = obsDir;

		// add voter flags
		inliers(0, i) = iter->second.occupyId;
	}

	cloud.save("DEBUG_denseVoting.vtk");
}

//-------------------------
// Push sparse tensor in a dense map
void TensorPlanner::mergeMap(const DP & cloud, int seqId)
{
	// avoid a seqId of zero
	seqId++;
	cout << "seqId: " << seqId << endl;

	// TODO: adjust that for faster computation time
	const Vi sigmaSpan = GridPoint::pointToGrid(2.25f*sigma*Vf::Ones(3), cellSizes);
	cout << "sigma span: " << 2*sigmaSpan.transpose() << endl;

	DP::ConstView voter_normals = cloud.getDescriptorViewByName("normals");
	DP::ConstView voter_lambdas = cloud.getDescriptorViewByName("eigValues");
	
	//DP::ConstView vp_x = cloud.getDescriptorViewByName("vp_x");
	//DP::ConstView vp_y = cloud.getDescriptorViewByName("vp_y");
	//DP::ConstView vp_z = cloud.getDescriptorViewByName("vp_z");	
	
	DP::ConstView obsDirection = cloud.getDescriptorViewByName("observationDirections");
	
	for(int i=0; i<cloud.features.cols(); i++)
	//for(int i=0; i<1; i++)
	{
		cout<<"\r dense voting progress:"<<((double)i/(double)cloud.features.cols())*100;cout.flush();
		GridPoint voter(cloud.features.col(i), cellSizes);
		auto voter_ptr = gridMap.find(voter);

		bool voteNeeded = false;
		if(voter_ptr != gridMap.end())
		{
			
			if(voter_ptr->second.occupyId < seqId)
			{
				voteNeeded = true;
			}
		}
		else
		{
			
			voteNeeded = true;
		}

		
		if(voteNeeded)
		{
			// the position of the voter
			const Vf voter_pt = cloud.features.col(i).topRows(3);

			// extract the primary lamda and associated eigenvector corresponding to normal						
			const float lamda_stick = voter_lambdas(0,i);
			const Eigen::Vector3f v_n = voter_normals.col(i);

			// viewpoint vector information of the voter,i.e vector connecting voter to view point
			Eigen::Vector3f obs_voter;

			//////////////////////////////////////////////////////////////////////////////////////
			//Eigen::Vector3f correctedObservationDirection;
			//correctedObservationDirection<<obsDirection(0,i),obsDirection(1,i),obsDirection(2,i)+0.1;
			/////////////////////////////////////////////////////////////////////////////////////
			obs_voter<<obsDirection.col(i);// - voter_pt;// vp_x.col(i) - voter_pt.row(0), vp_y.col(i) - voter_pt.row(1), vp_z.col(i) - voter_pt.row(2);
			obs_voter.normalize();

			for(int x = -sigmaSpan(0); x <= sigmaSpan(0); x++)
			{
				for(int y = -sigmaSpan(1); y <= sigmaSpan(1); y++)
				{
					for(int z = -sigmaSpan(2); z <= sigmaSpan(2); z++)
					{
						Vi offset(3,1);
						offset << x, y, z;

						const int radius = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
						if(radius > sigmaSpan(0)) // assume square cells
						{
							//cout << "rejected" << endl;
						}
						else
						{

							GridPoint votee(voter.coord + offset);

							auto votee_ptr = gridMap.find(votee);
							
							// Check if votee cell exists 
							if(votee_ptr == gridMap.end())
							{
								// Create a new cell
								GridTensor tens;
								gridMap[votee] = tens;
								votee_ptr = gridMap.find(votee);
							}
							// else  votee cell does exist

							
							// if votee cell exists then compute stick vote
							//Revision: Author-SM/ date:23.10.2012					
							// obtain positions of votee
											
							const Vf votee_pt = GridPoint::gridToPoint(votee.coord, cellSizes);
						
							
							// vector joining voter to votee			
							const Eigen::Vector3f v = votee_pt - voter_pt;
							const float r = v.norm();

							if(r!=0)
							{
								const Eigen::Vector3f e_v = v/r;
								const Eigen::Vector3f v_t = v_n.cross(e_v.cross(v_n));
								const float theta = asin(e_v.dot(v_n));	
								// Decoupled weighting profile for angle and distance
								// Angle based weighting
								const int n = 4;
								const float w_angle = pow(cos(theta),n);

								// Distance based weighting
								// Note: No need to check if points are within 3*sigma
								const float z = r/sigma;
								const float w_dist = pow(z,2)*(pow((z-3),4))/16;
		
								// stick vote
								const Eigen::Vector3f v_c = v_n*cos(2*theta) - v_t*sin(2*theta);
								const double w = w_angle*w_dist;

								votee_ptr->second.tensorStick += lamda_stick*w*v_c*v_c.transpose();
								
								// transfer the viewpoint information to the dense grid cell if voter is closest to the votee cell
									if(r < votee_ptr->second.voterDist)
										{
											votee_ptr->second.viewDir = obs_voter;
											votee_ptr->second.voterDist = r;
										}
								
							} 
						}
					}
				}
			}
			
			
			// Mark voter cell as occupied	
			auto iter = gridMap.find(voter);
			iter->second.occupyId = seqId;
		}
	}

	cout << "Nb point in the grid: " << gridMap.size() << endl;
}

// Destructor
TensorPlanner::~TensorPlanner()
{
	
}

void TensorPlanner::stopExecution(const std_msgs::Bool& msg)
{	
	ROS_WARN_STREAM("Receiving request to abort: " << msg.data);
     	errorCalled = msg.data;
}

// Main function supporting the node
int main(int argc, char **argv)
{
	ros::init(argc, argv, "tensor_planner");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	TensorPlanner planner(n, pn);
	ros::spin();
	
	
	/***********
	ros:Rate myrate(10);
	
	while(1)
	{
		myfunc();
		
		myrate.sleep();
	}
		
	
	************/
	
	return 0;
}

