// standard
#include <iostream>
#include <string>
#include <stdio.h>

// ROS 
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pointmatcher/PointMatcher.h>
#include <pointmatcher_ros/point_cloud.h>
// services and messages
#include "tensorPlanner/CallGlobalMap.h"
#include "tensorPlanner/ConfirmGoalStamped.h"
#include "tensorPlanner/ComputePlan.h"
#include "tensorPlanner/ExecutePlan.h"
#include "tensorPlanner/Serialization.h"
#include <map_msgs/GetPointMap.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Path.h>

// Various functions
#include "tensorUtils.h"
// TensorCell and TensorMap
#include "TensorMap.h"
// Search node and algorithm
#include "GenericSearch.h"
#include "DStarLite.h"
// Path execution
#include "PathExecution.h"
// Parameters
#include "trp_params.h"
// Timing
#include "MyTimer.h"


using namespace std;

//! Motion planning and execution from point cloud using tensor voting
class TensorRePlanner {
public:
	//! Constructor. ROS::init() is assumed to have been called before.
	TensorRePlanner();

	//! Destructor (nothing done).
	virtual ~TensorRePlanner();

protected:
	//! Tensor map
	TensorMap tensor_map;

	//! Path planner
	DStarPathPlanner planner;

	//! Path execution
	PathExecution path_execution;

	//! Current Goal
	geometry_msgs::Pose goal;

	//! Current Start
	geometry_msgs::Pose start;

	//! Input point cloud (before filtering and sparse voting)
	PointCloud input_point_cloud;

	//! Flag stating plan should use start and not look for robot position
	bool use_start;

	//! public NodeHandle
	ros::NodeHandle n;

	//! private NodeHandle
	ros::NodeHandle n_;

	// services proposed
	//! Getting map from mapper
	ros::ServiceServer callMapSrv;
	//! Setting goal to plan path to 
	ros::ServiceServer setGoalSrv;
	//! Computing plan toward the goal
	ros::ServiceServer computePlanSrv;
	//! Start executing plan toward the goal
	ros::ServiceServer executePlanSrv;
	//! Store/load state
	ros::ServiceServer serializationSrv;

	// publications
	//! velocity command
	ros::Publisher cmd_vel_pub;
	//! flipper command
	ros::Publisher flipper_cmd_pub;
	//! path for initial planning
	ros::Publisher path_pub;
	//! path for successive replanning
	ros::Publisher repath_pub;

	// subscriptions
	//! tf
	tf::TransformListener tf_listener;
	//! stop execution
	ros::Subscriber stopExecutionSub;
	//! getting map
	ros::ServiceClient getPointMapClient;

	// callbacks
	//! Callback for getting map from mapper
	bool callMapSrvCB(tensorPlanner::CallGlobalMap::Request& req,
			tensorPlanner::CallGlobalMap::Response& res);
	//! Callback for setting goal to plan path to 
	bool setGoalSrvCB(tensorPlanner::ConfirmGoalStamped::Request& req,
			tensorPlanner::ConfirmGoalStamped::Response& res);
	//! Callback for computing plan toward the goal
	bool computePlanSrvCB(tensorPlanner::ComputePlan::Request& req,
			tensorPlanner::ComputePlan::Response& res);
	//! Callback to start executing plan toward the goal
	bool executePlanSrvCB(tensorPlanner::ExecutePlan::Request& req,
			tensorPlanner::ExecutePlan::Response& res);
	//! Callback to store or load the state
	bool serializationSrvCB(tensorPlanner::Serialization::Request& req,
			tensorPlanner::Serialization::Response& res);
	//! Callback to stop execution
	void stopExecutionCB(const std_msgs::Bool& msg);

	//! Get current robot pose in map coordinate frame
	geometry_msgs::Pose getRobotPose() const;

	//! Publish a path in ROS (for quick visualization)
	void publishPath(const vector<PathElement>& path, ros::Publisher& pub);

	//! Serialize planning problem: input point cloud + start and goal positions
	void serializeProblem(const string& directory) const;

	//! Deserialize planning problem (and set use_start flag)
	void deSerializeProblem(const string& directory);
};



// Constructor
TensorRePlanner::TensorRePlanner():
	tensor_map(),
	planner(tensor_map),
	use_start(false),
	n_("~"),
	tf_listener(ros::Duration(20.))
{
	//ROS_INFO("Start of constructor.");
	// parameters
	
	// publications and clients
	cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	flipper_cmd_pub = n.advertise<std_msgs::Int32>("/posture_cmd", 1);
	path_pub = n.advertise<nav_msgs::Path>("/planned_path", 1);
	repath_pub = n.advertise<nav_msgs::Path>("/replanned_path", 1);
	getPointMapClient = n.serviceClient<map_msgs::GetPointMap>("dynamic_point_map");
	


	// subscriptions and services (last so that the rest is initialized)
	stopExecutionSub = n.subscribe("/stop_exec", 1, &TensorRePlanner::stopExecutionCB, this);
	callMapSrv = n.advertiseService("call_global_map", &TensorRePlanner::callMapSrvCB, this);
	setGoalSrv = n.advertiseService("set_goal", &TensorRePlanner::setGoalSrvCB, this);
	computePlanSrv = n.advertiseService("compute_plan", &TensorRePlanner::computePlanSrvCB, this);
	executePlanSrv = n.advertiseService("execute_plan", &TensorRePlanner::executePlanSrvCB, this);
	serializationSrv = n.advertiseService("serialization", &TensorRePlanner::serializationSrvCB, this);
	//ROS_INFO("End of constructor.");
}


// Destructor (nothing done)
TensorRePlanner::~TensorRePlanner() {
	// Nothing to do?
}



/*
 * callback section
 */
// Callback for getting map from mapper
bool TensorRePlanner::callMapSrvCB(tensorPlanner::CallGlobalMap::Request& req,
		tensorPlanner::CallGlobalMap::Response& res) {
	ROS_INFO_STREAM("callMapSrvCB");
	map_msgs::GetPointMap srvMsg;
	getPointMapClient.call(srvMsg);
	sensor_msgs::PointCloud2 tmp_cloud = srvMsg.response.map;
	if (srvMsg.response.map.header.frame_id!="/map") {
		if (!tf_listener.waitForTransform(tmp_cloud.header.frame_id, "/map",
					tmp_cloud.header.stamp, ros::Duration(2.))) {
			ROS_WARN("Cannot transform pointcloud into global frame; ignoring.");
			return false;
		}
		pcl_ros::transformPointCloud("/map", tmp_cloud, tmp_cloud,
				tf_listener);
	}
	input_point_cloud = PointMatcher_ros::\
			rosMsgToPointMatcherCloud<float>(tmp_cloud);
	tensor_map.import(input_point_cloud);
	ROS_INFO_STREAM("callMapSrvCB: done");
	return true;
}


// Callback for setting goal to plan path to 
bool TensorRePlanner::setGoalSrvCB(tensorPlanner::ConfirmGoalStamped::Request& req,
		tensorPlanner::ConfirmGoalStamped::Response& res) {
	ROS_INFO_STREAM("setGoalSrvCB");
	// getting goal in global reference frame
	geometry_msgs::PoseStamped goal_global;
	if (!tf_listener.waitForTransform(req.goal.header.frame_id, "/map",
				req.goal.header.stamp, ros::Duration(2.))) {
		ROS_WARN_STREAM("Cannot transform goal into global frame; ignoring.");
		res.is_valid = false;
		return false;
	}
	tf_listener.transformPose("/map", req.goal, goal_global);
	// validate and register goal to planner
	geometry_msgs::PoseStamped new_goal;
	new_goal.header = goal_global.header;
	res.is_valid = planner.validateGoal(goal_global.pose, new_goal.pose);
	// change updated goal back into original frame
	tf_listener.transformPose(req.goal.header.frame_id, new_goal, res.valid_goal);
	if (res.is_valid) {
		goal = new_goal.pose;
	}
	ROS_INFO_STREAM("setGoalSrvCB: "<<(res.is_valid?"valid":"invalid"));
	return res.is_valid;
}


// Callback for computing plan toward the goal
bool TensorRePlanner::computePlanSrvCB(tensorPlanner::ComputePlan::Request& req,
		tensorPlanner::ComputePlan::Response& res) {
	ROS_INFO_STREAM("computePlanSrvCB");
	res.plan_found = false;
	// setting start for planner
	geometry_msgs::Pose start_pose;
	if (use_start) {
		start_pose = start;	// it was loaded
		use_start = false;	// next time I can use /tf
	} else {
		start_pose = getRobotPose(); // get start from /tf
	}
	planner.setStart(start_pose);
	path_execution.reInit(start_pose);

	// doing path planning
	MyTimer init_plan_timer;
	init_plan_timer.start();
	bool success=planner.computeInitialPath();
	init_plan_timer.stop();
	init_plan_timer.print("Initial planning: ", "plan_timing.csv");
	// TODO save timer values in a file
	if (success) {
		ROS_INFO_STREAM("Planning successful.");
		vector<const TensorCell*> path;
		bool got_path = planner.fillPath(path);
		if (!got_path) {
			ROS_ERROR_STREAM("Couldn't get path after successful planning?!");
			return false;
		}
		res.plan_found = true;
		path_execution.decoratePath(path);
		publishPath(path_execution.decorated_path, path_pub);
		/*int t = 0;
		for (auto &it: path) {
			ROS_INFO_STREAM(t++ << ": (" << it->position(0) << ", " << it->position(1)\
					<< ", " << it->position(2) << ") [" << it->stick_sal << "]");
		}*/
		return true;
	} else {
		ROS_WARN_STREAM("Couldn't find path.");
		return false;
	}
}


// Callback to start executing plan toward the goal
bool TensorRePlanner::executePlanSrvCB(tensorPlanner::ExecutePlan::Request& req,
		tensorPlanner::ExecutePlan::Response& res) {
	ROS_INFO_STREAM("executePlanSrvCB");
	bool end_of_path=false;
	FILE* executed = fopen("executed_path.csv", "w");

	ros::Rate loop_rate(5); // FIXME find correct control frequency
	
	// initialize current slope and climbing state
	path_execution.reInit(getRobotPose());

	MyTimer replanning_timer;
	// TODO save_timer into a file

	while ((!end_of_path)&&ros::ok()) {
		geometry_msgs::Pose robot_pose = getRobotPose();
		Vector3f rounded_pos = tensor_map.indexToPosition(
				tensor_map.positionToIndex(poseToVector(robot_pose)));
		fprintf(executed, "%f,%f,%f,%f,%f,%f", robot_pose.position.x,
				robot_pose.position.y, robot_pose.position.z, rounded_pos(0),
				rounded_pos(1), rounded_pos(2));
		Vector2f g_p(goal.position.x, goal.position.y);
		Vector2f s_p(robot_pose.position.x, robot_pose.position.y);
		if ((g_p-s_p).norm()<0.2) {
			fprintf(executed, ",\"Goal reached\"\n");
			ROS_INFO("Goal reached");
			break;
		}
		planner.setStart(robot_pose);

		ROS_INFO("Replanning");
		replanning_timer.start();
		bool success=planner.rePlan();
		replanning_timer.stop();
		if (!success) {
			fprintf(executed, ",\"Retrying\"");
			ROS_WARN_STREAM("Couldn't find path. Retrying once.");
			planner = DStarPathPlanner(tensor_map);
			robot_pose = getRobotPose();
			planner.setStart(robot_pose);
			planner.setGoal(goal);
			replanning_timer.start();
			success=planner.rePlan();
			replanning_timer.stop();
			if (!success) {
				fprintf(executed, ",\"Failed\"\n");
				fclose(executed);
				ROS_INFO_STREAM("executePlanSrvCB: fail");
				replanning_timer.print("Replanning: ", "plan_timing.csv");
				return false;
			}
		}
		//ROS_INFO_STREAM("Replanning successful:");
		vector<const TensorCell*> path;
		bool got_path = planner.fillPath(path);
		if (!got_path) {
			ROS_ERROR_STREAM("Couldn't get path after successful replanning?!");
			fprintf(executed, ",\"fillPath failure\"\n");
			fclose(executed);
			ROS_INFO_STREAM("executePlanSrvCB: fail");
			replanning_timer.print("Replanning: ", "plan_timing.csv");
			return false;
		}
		if (!path.size()) {
			ROS_INFO_STREAM("Reached the goal.");
			fprintf(executed, ",\"Reached\"");
		}
		path_execution.decoratePath(path);
		publishPath(path_execution.decorated_path, repath_pub);
		path_execution.serialize("./tmp_path.csv");

		geometry_msgs::Twist cmd_vel_msg; // default to stop
		std_msgs::Int32 posture_msg;
		posture_msg.data = 1; // default to drive
		end_of_path = path_execution.executePath(robot_pose, cmd_vel_msg,
			&(posture_msg.data));
		ROS_INFO_STREAM("Plan ok: executing (v="<<cmd_vel_msg.linear.x<<", w="<<cmd_vel_msg.angular.z<<"), posture="<<posture_msg.data);
		// publish commands (or default if end_of_path)
		cmd_vel_pub.publish(cmd_vel_msg);
		flipper_cmd_pub.publish(posture_msg);

		// FIXME do I need to spinOnce?
		//ros::spinOnce()
		fprintf(executed, "\n");
		loop_rate.sleep();
		ros::spinOnce();
	}
	ROS_INFO_STREAM("executePlanSrvCB: done");
	replanning_timer.print("Replanning: ", "plan_timing.csv");
	fclose(executed);
	return true;
}


// Callback to store or load the state
bool TensorRePlanner::serializationSrvCB(tensorPlanner::Serialization::Request& req,
		tensorPlanner::Serialization::Response& res) {
	ROS_INFO_STREAM("serializationSrvCB: "<<static_cast<int>(req.store)<<" "<<
			static_cast<int>(req.step)<<" "<<req.directory);
	if (req.store) {
		switch (req.step){
		case tensorPlanner::Serialization::Request::SPARSE_MAP:
			tensor_map.serializeSparse(req.directory+"/sparse.csv");
			break;
		case tensorPlanner::Serialization::Request::DENSE_MAP:
			tensor_map.serialize(req.directory);
			break;
		case tensorPlanner::Serialization::Request::PLANNER:
			planner.serialize(req.directory);
			break;
		case tensorPlanner::Serialization::Request::PATH:
			path_execution.serialize(req.directory+"/path.csv");
			break;
		case tensorPlanner::Serialization::Request::PROBLEM:
			serializeProblem(req.directory);
			break;
		default:
			ROS_WARN_STREAM("Unknown process step to serialize: "<<req.step);
		}
	} else {
		switch (req.step){
		case tensorPlanner::Serialization::Request::SPARSE_MAP:
			tensor_map.deSerializeSparse(req.directory+"/sparse.csv");
			break;
		case tensorPlanner::Serialization::Request::DENSE_MAP:
			tensor_map.deSerialize(req.directory);
			break;
		case tensorPlanner::Serialization::Request::PLANNER:
			planner.deSerialize(req.directory);
			break;
		case tensorPlanner::Serialization::Request::PATH:
			path_execution.deSerialize(req.directory+"/path.csv");
			break;
		case tensorPlanner::Serialization::Request::PROBLEM:
			deSerializeProblem(req.directory);
			break;
		default:
			ROS_WARN_STREAM("Unknown process step to deserialize: "<<req.step);
		}
	}
	ROS_INFO_STREAM("serializationSrvCB: done");
	return true;
}


// Callback to stop execution
void TensorRePlanner::stopExecutionCB(const std_msgs::Bool& msg) {
	ROS_INFO_STREAM("stopExecutionCB: "<<msg);
	// TODO
	return;
}


// Get current robot pose in map coordinate frame
geometry_msgs::Pose TensorRePlanner::getRobotPose() const {
	// First get the position of the robot in the map frame
	geometry_msgs::PoseStamped robot_pose;
	robot_pose.header.frame_id = "/base_link";
	robot_pose.pose.position.x = 0;
	robot_pose.pose.position.y = 0;
	// Using the projection on the ground
	robot_pose.pose.position.z = -0.0705; // gather parameters FIXME
	robot_pose.pose.orientation.x = 0;
	robot_pose.pose.orientation.y = 0;
	robot_pose.pose.orientation.z = 0;
	robot_pose.pose.orientation.w = 1;
	string error_string;
	int err = tf_listener.getLatestCommonTime("/base_link", "/map",
			robot_pose.header.stamp, &error_string);
	if (err!=tf::NO_ERROR) {
		ROS_WARN_STREAM("Cannot find position of robot.");
		throw err;
	}
	geometry_msgs::PoseStamped new_pose;
	tf_listener.transformPose("/map", robot_pose, new_pose);
	return new_pose.pose;
}


// Publish a path in ROS (for quick visualization)
void TensorRePlanner::publishPath(const vector<PathElement>& path,
		ros::Publisher& pub) {
	nav_msgs::Path path_msg;
	path_msg.header.frame_id = "/map";
	path_msg.header.stamp = ros::Time::now();
	Vector3f old_direction(1, 0, 0);
	for (auto &it: path) {
		geometry_msgs::PoseStamped pose_msg;
		pose_msg.header = path_msg.header;
		if (it.direction.norm()<0.001) {
			pose_msg.pose = vectorsToPose(it.position, it.normal,
					old_direction);
		} else {
			pose_msg.pose = vectorsToPose(it.position, it.normal,
					it.direction);
			old_direction = it.direction;
		}
		path_msg.poses.push_back(pose_msg);
	}
	pub.publish(path_msg);
}


// Serialize planning problem
void TensorRePlanner::serializeProblem(const string& directory) const {
	serializePointCloud(input_point_cloud, directory+"/input_point_cloud.csv");
	/*
	 * serialization of start and goal poses in csv
	 * 
	 * format:
	 * x,y,z,qx,qy,qz,qw for start
	 * x,y,z,qx,qy,qz,qw for goal
	 */
	FILE* start_goal_file = fopen((directory+"/start_goal.csv").c_str(), "w");
	// should check error FIXME
	geometry_msgs::Pose pose;
	if (use_start) {
		pose = start;
	} else {
		pose = getRobotPose();
	}
	const auto p1 = pose.position;
	const auto q1 = pose.orientation;
	fprintf(start_goal_file, "%f,%f,%f,%f,%f,%f,%f\n",
			p1.x, p1.y, p1.z, q1.x, q1.y, q1.z, q1.w);
	const auto p2 = goal.position;
	const auto q2 = goal.orientation;
	fprintf(start_goal_file, "%f,%f,%f,%f,%f,%f,%f\n",
			p2.x, p2.y, p2.z, q2.x, q2.y, q2.z, q2.w);
	fclose(start_goal_file);
}


// Deserialize planning problem (and set use_start flag)
void TensorRePlanner::deSerializeProblem(const string& directory) {
	deSerializePointCloud(input_point_cloud,
			directory+"/input_point_cloud.csv");
	/*
	 * deserialization of start and goal poses in csv
	 * 
	 * format:
	 * x,y,z,qx,qy,qz,qw for start
	 * x,y,z,qx,qy,qz,qw for goal
	 */
	FILE* start_goal_file = fopen((directory+"/start_goal.csv").c_str(), "r");
	// should check error FIXME
	float x, y, z, qx, qy, qz, qw;
	if (fscanf(start_goal_file, "%f,%f,%f,%f,%f,%f,%f\n",
			&x, &y, &z, &qx, &qy, &qz, &qw)!=7) {
		ROS_ERROR_STREAM("Couldn't load start and goal positions");
		fclose(start_goal_file);
		return;
	}
	start.position.x = x;
	start.position.y = y;
	start.position.z = z;
	start.orientation.x = qx;
	start.orientation.y = qy;
	start.orientation.z = qz;
	start.orientation.w = qw;
	if (fscanf(start_goal_file, "%f,%f,%f,%f,%f,%f,%f\n",
			&x, &y, &z, &qx, &qy, &qz, &qw)!=7) {
		ROS_ERROR_STREAM("Couldn't load start and goal positions");
		fclose(start_goal_file);
		return;
	}
	goal.position.x = x;
	goal.position.y = y;
	goal.position.z = z;
	goal.orientation.x = qx;
	goal.orientation.y = qy;
	goal.orientation.z = qz;
	goal.orientation.w = qw;
	fclose(start_goal_file);
	use_start = true;
	auto new_goal = goal;
	tensor_map.import(input_point_cloud);
	if (!planner.validateGoal(goal, new_goal)) {
		ROS_ERROR_STREAM("Goal is not valid.");
	}
}



/*
 * main
 */
int main(int argc, char **argv) {
	ros::init(argc, argv, "tensor_replanner");
	cout << "Initializing tensor_replanner" << endl;
	TensorRePlanner trp;
	TRPParams params;
	load_params("test.yaml", params);
	cost_functions_params = &params.cost_functions;
	tensor_map_params = &params.tensor_map;
	path_execution_params = &params.path_execution;
	cout << "Spinning" << endl;
	ros::spin();
	return 0;
}
