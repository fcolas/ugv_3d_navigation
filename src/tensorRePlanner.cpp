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
// action interface similar to move_base
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/server/simple_action_server.h>

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

typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> ActionServer;


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
	geometry_msgs::PoseStamped aligned_goal;

	//! Current Start
	geometry_msgs::Pose start;

	//! Input point cloud (before filtering and sparse voting)
	PointCloud input_point_cloud;

	//! Flag stating plan should use start and not look for robot position
	bool use_start;

	//! Flag stating cancel has been called
	bool cancel_called;

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

	// action server
	//! action server
	ActionServer as_;
	//! feedback message
	move_base_msgs::MoveBaseFeedback feedback_;
	//! result message (empty?)
	move_base_msgs::MoveBaseResult result_;
	//! subsriber for simple_goal
	ros::Subscriber goal_sub;
	//! goal publisher to forward simple_goal
	ros::Publisher action_goal_pub;


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
	//! Callback for the action server
	void executeCB(const move_base_msgs::MoveBaseGoalConstPtr& goal);
	//! Callback for simple goal
	void simpleGoalCB(const geometry_msgs::PoseStamped& goal);

	// Actual implementation
	//! Get map
	bool callMap();
	//! Set goal
	bool setGoal(const geometry_msgs::PoseStamped& goal_pose);
	//! Compute plan
	bool computePlan();
	//! Execute plan
	bool executePlan();
	//! Align to goal
	bool finalAlignment();
	//! Get current robot pose in map coordinate frame
	geometry_msgs::PoseStamped getRobotPoseStamped() const;
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
	cancel_called(false),
	n_("~"),
	tf_listener(ros::Duration(20.)),
	as_(n, "trp_as", boost::bind(&TensorRePlanner::executeCB, this, _1), false)
{
	//ROS_INFO("Start of constructor.");
	// parameters
	
	// publications and clients
	cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	flipper_cmd_pub = n.advertise<std_msgs::Int32>("/posture_cmd", 1);
	path_pub = n.advertise<nav_msgs::Path>("/planned_path", 1);
	repath_pub = n.advertise<nav_msgs::Path>("/replanned_path", 1);
	getPointMapClient = n.serviceClient<map_msgs::GetPointMap>("dynamic_point_map");
	action_goal_pub = n.advertise<move_base_msgs::MoveBaseActionGoal>("/trp_as/goal", 1);

	
	// starting action server
	as_.start();

	// subscriptions and services (last so that the rest is initialized)
	stopExecutionSub = n.subscribe("simple_cancel", 1, &TensorRePlanner::stopExecutionCB, this);
	callMapSrv = n.advertiseService("call_global_map", &TensorRePlanner::callMapSrvCB, this);
	setGoalSrv = n.advertiseService("set_goal", &TensorRePlanner::setGoalSrvCB, this);
	computePlanSrv = n.advertiseService("compute_plan", &TensorRePlanner::computePlanSrvCB, this);
	executePlanSrv = n.advertiseService("execute_plan", &TensorRePlanner::executePlanSrvCB, this);
	serializationSrv = n.advertiseService("serialization", &TensorRePlanner::serializationSrvCB, this);
	goal_sub = n.subscribe("simple_goal", 1, &TensorRePlanner::simpleGoalCB, this);

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
	res.dummy = callMap();
	ROS_INFO_STREAM("callMapSrvCB: done");
	return res.dummy;
}


// Callback for setting goal to plan path to 
bool TensorRePlanner::setGoalSrvCB(tensorPlanner::ConfirmGoalStamped::Request& req,
		tensorPlanner::ConfirmGoalStamped::Response& res) {
	ROS_INFO_STREAM("setGoalSrvCB");
	res.is_valid = setGoal(req.goal);
	if (res.is_valid) {
		res.valid_goal = aligned_goal;
	}
	ROS_INFO_STREAM("setGoalSrvCB: "<<(res.is_valid?"valid":"invalid"));
	return res.is_valid;
}


// Callback for computing plan toward the goal
bool TensorRePlanner::computePlanSrvCB(tensorPlanner::ComputePlan::Request& req,
		tensorPlanner::ComputePlan::Response& res) {
	ROS_INFO_STREAM("computePlanSrvCB");
	res.plan_found = computePlan();
	ROS_INFO_STREAM("computePlanSrvCB: "<<(res.plan_found?"valid":"invalid"));
	return res.plan_found;
}


// Callback to start executing plan toward the goal
bool TensorRePlanner::executePlanSrvCB(tensorPlanner::ExecutePlan::Request& req,
		tensorPlanner::ExecutePlan::Response& res) {
	ROS_INFO_STREAM("executePlanSrvCB");
	res.plan_executed = executePlan();
	ROS_INFO_STREAM("executePlanSrvCB: done");
	return res.plan_executed;
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
	cancel_called = true;
	return;
}


// Callback for the action server
void TensorRePlanner::executeCB(const move_base_msgs::MoveBaseGoalConstPtr& goal) {
	ROS_INFO_STREAM("[trp_as] received goal: "<<goal->target_pose);
	cancel_called = false;

	// initialize feedback
	feedback_.base_position = getRobotPoseStamped();

	// getting map
	ROS_INFO_STREAM("[trp_as] requesting map...");
	if (!callMap()) {
		ROS_WARN_STREAM("[trp_as] cannot get map: aborting.");
		as_.setAborted(result_, "Couldn't get map.");
		return;
	} else {
		ROS_INFO_STREAM("[trp_as] got map.");
	}
	
	// setting goal
	ROS_INFO_STREAM("[trp_as] validating goal...");
	if (!setGoal(goal->target_pose)) {
		ROS_WARN_STREAM("[trp_as] goal invalid: aborting.");
		as_.setAborted(result_, "Goal invalid.");
		return;
	} else {
		ROS_INFO_STREAM("[trp_as] goal valid.");
	}
	
	// computing plan
	ROS_INFO_STREAM("[trp_as] computing initial path...");
	if (!computePlan()) {
		ROS_WARN_STREAM("[trp_as] couldn't find initial path.");
		as_.setAborted(result_, "Couldn't find initial path.");
		return;
	} else {
		ROS_INFO_STREAM("[trp_as] found initial path.");
	}

	// executing plan
	ROS_INFO_STREAM("[trp_as] executing path...");
	if (!executePlan()) {
		if (as_.isPreemptRequested()) {
			ROS_INFO_STREAM("[trp_as] preempting.");
			as_.setPreempted();
		} else if (cancel_called) {
			ROS_INFO_STREAM("[trp_as] cancelling.");
			as_.setAborted(result_, "Cancel called.");
			cancel_called = false;
		} else {
			ROS_INFO_STREAM("[trp_as] failure.");
			as_.setAborted(result_, "Failed to execute the path.");
		}
	} else if (!path_execution.cur_slope) {
		if (!finalAlignment()) {
			if (as_.isPreemptRequested()) {
				ROS_INFO_STREAM("[trp_as] preempting.");
				as_.setPreempted();
			} else if (cancel_called) {
				ROS_INFO_STREAM("[trp_as] cancelling.");
				as_.setAborted(result_, "Cancel called.");
				cancel_called = false;
			} else {
				ROS_INFO_STREAM("[trp_as] failure.");
				as_.setAborted(result_, "Failed to align to the goal.");
			}
		} else {
			ROS_INFO_STREAM("[trp_as] success.");
			as_.setSucceeded(result_);
		}
	} else {
		ROS_INFO_STREAM("[trp_as] success.");
		as_.setSucceeded(result_);
	}
}


// Callback for simple goal (forward to action goal)
void TensorRePlanner::simpleGoalCB(const geometry_msgs::PoseStamped& goal) {
	ROS_INFO_STREAM("[trp_sg] received simple goal: " << goal << " (forwarding)");
	move_base_msgs::MoveBaseActionGoal action_goal;
	action_goal.header.stamp = ros::Time::now();
	action_goal.goal.target_pose = goal;
	action_goal_pub.publish(action_goal);
}


// Get map
bool TensorRePlanner::callMap() {
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
	return true;
}


// Set goal
bool TensorRePlanner::setGoal(const geometry_msgs::PoseStamped& goal_pose) {
	// getting goal in global reference frame
	geometry_msgs::PoseStamped goal_global;
	if (!tf_listener.waitForTransform(goal_pose.header.frame_id, "/map",
				goal_pose.header.stamp, ros::Duration(2.))) {
		ROS_WARN_STREAM("Cannot transform goal into global frame; ignoring.");
		return false;
	}
	tf_listener.transformPose("/map", goal_pose, goal_global);
	// validate and register goal to planner
	geometry_msgs::PoseStamped new_goal;
	new_goal.header = goal_global.header;
	bool res = planner.validateGoal(goal_global.pose, new_goal.pose);
	if (res) {
		// change updated goal back into original frame
		tf_listener.transformPose(goal_pose.header.frame_id, new_goal, aligned_goal);
		goal = new_goal.pose;
	}
	return res;
}


// Compute plan
bool TensorRePlanner::computePlan() {
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
	if (success) {
		ROS_INFO_STREAM("Planning successful.");
		vector<const TensorCell*> path;
		bool got_path = planner.fillPath(path);
		if (!got_path) {
			ROS_ERROR_STREAM("Couldn't get path after successful planning?!");
			return false;
		}
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


// Execute plan
bool TensorRePlanner::executePlan() {
	bool end_of_path=false;
	FILE* executed = fopen("executed_path.csv", "w");

	ros::Rate loop_rate(5); // FIXME find correct control frequency
	
	// initialize current slope and climbing state
	path_execution.reInit(getRobotPose());

	MyTimer replanning_timer;

	// flag to say if pose is ok for this loop
	bool loop_ok = true;

	geometry_msgs::Pose robot_pose;

	while ((!end_of_path)&&ros::ok()&&(!as_.isPreemptRequested())&&(!cancel_called)) {
		loop_rate.sleep();
		ros::spinOnce();
		// getting position of the robot
		try {
			feedback_.base_position = getRobotPoseStamped();
			robot_pose = feedback_.base_position.pose;
			loop_ok = true;
		} catch (int) {
			loop_ok = false;
		}
		if (!loop_ok) {
			continue;
		}
		Vector3f rounded_pos = tensor_map.indexToPosition(
				tensor_map.positionToIndex(poseToVector(robot_pose)));
		fprintf(executed, "%f,%f,%f,%f,%f,%f", robot_pose.position.x,
				robot_pose.position.y, robot_pose.position.z, rounded_pos(0),
				rounded_pos(1), rounded_pos(2));
		Vector2f g_p(goal.position.x, goal.position.y);
		Vector2f s_p(robot_pose.position.x, robot_pose.position.y);
		// checking for close proximity with the goal
		if ((g_p-s_p).norm()<path_execution_params->execution_params.max_distance) {
			fprintf(executed, ",\"Goal reached\"\n");
			ROS_INFO("Goal reached");
			end_of_path = true;
			break;
		}
		planner.setStart(robot_pose);

		// updating global plan
		ROS_INFO("Replanning");
		replanning_timer.start();
		bool success=planner.rePlan();
		replanning_timer.stop();
		if (!success) {
			fprintf(executed, ",\"Retrying\"");
			ROS_WARN_STREAM("Couldn't find path. Retrying once.");
			planner = DStarPathPlanner(tensor_map);
			try {
				robot_pose = getRobotPose();
				loop_ok = true;
			} catch (int) {
				loop_ok = false;
			}
			if (!loop_ok) {
				continue;
			}
			planner.setStart(robot_pose);
			planner.setGoal(goal);
			replanning_timer.start();
			success=planner.rePlan();
			replanning_timer.stop();
			if (!success) {
				fprintf(executed, ",\"Failed\"\n");
				ROS_INFO_STREAM("executePlanSrvCB: fail");
				replanning_timer.print("Replanning: ", "plan_timing.csv");
				break;
			}
		}
		//ROS_INFO_STREAM("Replanning successful:");
		vector<const TensorCell*> path;
		bool got_path = planner.fillPath(path);
		if (!got_path) {
			ROS_ERROR_STREAM("Couldn't get path after successful replanning?!");
			fprintf(executed, ",\"fillPath failure\"\n");
			ROS_INFO_STREAM("executePlanSrvCB: fail");
			replanning_timer.print("Replanning: ", "plan_timing.csv");
			break;
		}
		if (!path.size()) {
			ROS_INFO_STREAM("Reached the goal.");
			fprintf(executed, ",\"Reached\"");
			end_of_path = true;
			break;
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

		fprintf(executed, "\n");
	}
	// can be here for several reasons:
	// - !ros::ok
	// - preemption/cancel_called
	// - success (end_of_path=true)
	// - failure
	if (as_.isPreemptRequested()) {
		fprintf(executed, ",\"preempt requested\"\n");
	}
	if (!end_of_path) {
		ROS_INFO_STREAM("stopping execution.");
		// stop
		geometry_msgs::Twist cmd_vel_msg;
		cmd_vel_pub.publish(cmd_vel_msg);
	}

	replanning_timer.print("Replanning: ", "plan_timing.csv");
	fclose(executed);

	return end_of_path;
}


// Align to goal
bool TensorRePlanner::finalAlignment() {
	bool aligned=false;

	ros::Rate loop_rate(5); // FIXME find correct control frequency
	
	geometry_msgs::Pose robot_pose;

	enum {TurnToPosition, MoveToPosition, TurnToOrientation} state = TurnToPosition;

	const float v_max = path_execution_params->execution_params.v_max_flat;
	const float w_max = path_execution_params->execution_params.w_max_flat;

	// TODO: put that as parameters
	const float pre_orient_limit = 2.5*M_PI/180.; //FIXME
	const float position_limit = 0.05; //FIXME
	const float last_orient_limit = 1.*M_PI/180.; //FIXME
	const float P_rot = 1. ; //FIXME
	const float P_trans = 1.; // FIXME

	while ((!aligned)&&ros::ok()&&(!as_.isPreemptRequested())&&(!cancel_called)) {
		loop_rate.sleep();
		ros::spinOnce();
		// getting position of the goal with respect to robot
		geometry_msgs::PoseStamped goal = aligned_goal;
		string error_string;
		int err = tf_listener.getLatestCommonTime("/base_link", "/map",
				goal.header.stamp, &error_string);
		if (err!=tf::NO_ERROR) {
			ROS_WARN_STREAM("Cannot find position of goal with respect to robot.");
			continue;
		}
		geometry_msgs::PoseStamped new_pose;
		if (tf_listener.waitForTransform("/base_link", goal.header.frame_id,
					goal.header.stamp, ros::Duration(1))) {
			tf_listener.transformPose("/base_link", goal, new_pose);
		} else {
			ROS_WARN_STREAM("Cannot transform the goal pose with respect to robot.");
			continue;
		}
		const float x = new_pose.pose.position.x;
		const float y = new_pose.pose.position.y;
		const float d = sqrt(x*x + y*y);
		const float q0 = new_pose.pose.orientation.w;
		const float q1 = new_pose.pose.orientation.x;
		const float q2 = new_pose.pose.orientation.y;
		const float q3 = new_pose.pose.orientation.z;
		const float yaw = atan2(2*q1*q2+2*q0*q3, q1*q1+q0*q0-q3*q3-q2*q2);
		float angle = atan2(y, x);

		geometry_msgs::Twist cmd_vel_msg;
		ROS_INFO_STREAM("[trp_align] (x, y, yaw, d, angle)=("<<x<<", "<<y<<", "<<yaw<<", "<<d<<", "<<angle<<")");
		switch (state) {
		case TurnToPosition:
			if ((x<0)&&(cos(yaw-angle)<0)) {
				// don't turn back to turn again
				angle = atan2(y, -x);
			}
			if (fabs(angle)<pre_orient_limit) { // looking at the goal
				state = MoveToPosition;
				ROS_INFO_STREAM("[trp_align] TurnToPosition -> MoveToPosition");
			} else { // turning to head to goal
				// P controller
				cmd_vel_msg.angular.z = std::min(w_max, std::max(-w_max, P_rot*angle)); 
				cmd_vel_pub.publish(cmd_vel_msg);
				ROS_INFO_STREAM("[trp_align] TurnToPosition: w="<<cmd_vel_msg.angular.z<<" (angle="<<angle<<")");
			}
			break;
		case MoveToPosition:
			if (d<position_limit) { // close enough
				state = TurnToOrientation;
				ROS_INFO_STREAM("[trp_align] MoveToPosition -> TurnToOrientation");
			} else { // moving closer
				// P controller
				if (x>0) {
					cmd_vel_msg.linear.x = std::min(v_max, P_trans*d);
				} else {
					cmd_vel_msg.linear.x = -std::min(v_max, P_trans*d);
				}
				cmd_vel_pub.publish(cmd_vel_msg);
				ROS_INFO_STREAM("[trp_align] MoveToPosition: v="<<cmd_vel_msg.linear.x<<" (d="<<d<<", x="<<x<<")");
			}
			break;
		case TurnToOrientation:
			if (cos(yaw)>cos(last_orient_limit)) { // good enough
				aligned = true;
				ROS_INFO_STREAM("[trp_align] TurnToOrientation -> Done!");
			} else { // turning to align to orientation
				// P controller
				cmd_vel_msg.angular.z = std::min(w_max, std::max(-w_max, P_rot*yaw));
				cmd_vel_pub.publish(cmd_vel_msg);
				ROS_INFO_STREAM("[trp_align] TurnToOrientation: w="<<cmd_vel_msg.angular.z<<" (yaw="<<yaw<<")");
			}
			break;
		}
	}
	// can be here for several reasons:
	// - !ros::ok
	// - preemption/cancel_called
	// - success (end_of_path=true)
	// - failure
	if (!aligned) {
		ROS_INFO_STREAM("stopping alignment.");
		// stop
		geometry_msgs::Twist cmd_vel_msg;
		cmd_vel_pub.publish(cmd_vel_msg);
	}
	return aligned;
}

// Get current robot pose in map coordinate frame
geometry_msgs::PoseStamped TensorRePlanner::getRobotPoseStamped() const {
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
	if (tf_listener.waitForTransform("/map", robot_pose.header.frame_id, robot_pose.header.stamp, ros::Duration(1))) {
		tf_listener.transformPose("/map", robot_pose, new_pose);
	} else {
		ROS_WARN_STREAM("Cannot transform the pose of the support point.");
		throw 1;
	}
	return new_pose;
}
// Get current robot pose in map coordinate frame
geometry_msgs::Pose TensorRePlanner::getRobotPose() const {
	return getRobotPoseStamped().pose;
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
	string config_filename;
	ros::NodeHandle nh("~");
	if (!nh.getParam("config", config_filename)) {
		config_filename = "test.yaml";
	}
	cout << "Loading config file: " << config_filename << endl;
	load_params(config_filename, params);
	cost_functions_params = &params.cost_functions;
	tensor_map_params = &params.tensor_map;
	path_execution_params = &params.path_execution;
	cout << "Spinning" << endl;
	ros::spin();
	return 0;
}
