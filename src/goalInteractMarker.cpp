#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <geometry_msgs/Pose.h>

#include <math.h>

// Service definitions
#include "ugv_3d_navigation/CallGlobalMap.h"
#include "ugv_3d_navigation/ConfirmGoalStamped.h"
#include "ugv_3d_navigation/ComputePlan.h"
#include "ugv_3d_navigation/ExecutePlan.h"
#include "ugv_3d_navigation/Serialization.h"

using namespace visualization_msgs;
using namespace interactive_markers;

boost::shared_ptr<InteractiveMarkerServer> server;

MenuHandler menu_handler;

MenuHandler::EntryHandle h_map;
MenuHandler::EntryHandle h_goal;
MenuHandler::EntryHandle h_plan;
MenuHandler::EntryHandle h_exe;
MenuHandler::EntryHandle h_save_sparse;
MenuHandler::EntryHandle h_load_sparse;
MenuHandler::EntryHandle h_save_dense;
MenuHandler::EntryHandle h_load_dense;
MenuHandler::EntryHandle h_save_planner;
MenuHandler::EntryHandle h_load_planner;
MenuHandler::EntryHandle h_save_path;
MenuHandler::EntryHandle h_load_path;
MenuHandler::EntryHandle h_save_startgoal;
MenuHandler::EntryHandle h_load_startgoal;

ros::ServiceClient callMapClient;
ros::ServiceClient goalClient;
ros::ServiceClient planClient;
ros::ServiceClient exeClient;
ros::ServiceClient serializationClient;
tf::TransformListener* ptf_listener;


// Helper functions
bool isPoseEqual(const geometry_msgs::Pose p1, const geometry_msgs::Pose p2)
{
	return p1.position.x == p2.position.x;
}

// Callbacks
void validateMenu(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	menu_handler.setVisible(h_plan, false);
	menu_handler.setVisible(h_exe, false);
	menu_handler.reApply( *server);
  server->applyChanges();
}


void callMapCallback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{

 	ugv_3d_navigation::CallGlobalMap srvMsg;
	callMapClient.call(srvMsg);
	
}

void setGoalCallback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{

 	ugv_3d_navigation::ConfirmGoalStamped srvMsg;
	srvMsg.request.goal.pose = feedback->pose;
	srvMsg.request.goal.header.stamp = ros::Time::now();
	// see makeMenuMarker
	srvMsg.request.goal.header.frame_id = "/map";
	goalClient.call(srvMsg);
	if(srvMsg.response.is_valid)
	{
		server->setPose( feedback->marker_name, srvMsg.response.valid_goal.pose );

		// if valid goal found
		menu_handler.setVisible(h_plan, true);
		menu_handler.reApply( *server);
		server->applyChanges();
	}
}


void planCallback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	ROS_INFO_STREAM("Now planning ");
	//Call service
	ugv_3d_navigation::ComputePlan srvMsg;
	srvMsg.request.goal = feedback->pose;
	//srvMsg.request.dummy = true;
	planClient.call(srvMsg);
	
	ROS_INFO_STREAM("Response: " << srvMsg.response.plan_found);
	if(srvMsg.response.plan_found)
	{
		menu_handler.setVisible(h_exe, true);
		menu_handler.reApply( *server);
		server->applyChanges();
	}
}


void executeCallback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	ROS_INFO_STREAM("Executing plan");
	//Call service
	ugv_3d_navigation::ExecutePlan srvMsg;
	srvMsg.request.dummy = true;
	exeClient.call(srvMsg);
	
	if(srvMsg.response.plan_executed)
	{
		//be happy!
	}
}

void saveSparseCallBack(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
	ROS_INFO_STREAM("Saving sparse map");
	// Call service
	ugv_3d_navigation::Serialization srvMsg;
	srvMsg.request.step = ugv_3d_navigation::Serialization::Request::SPARSE_MAP;
	srvMsg.request.store = true;
	srvMsg.request.directory = ".";
	serializationClient.call(srvMsg);
	ROS_INFO_STREAM("Success: "<<srvMsg.response.done);
}

void loadSparseCallBack(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
	ROS_INFO_STREAM("Loading sparse map");
	// Call service
	ugv_3d_navigation::Serialization srvMsg;
	srvMsg.request.step = ugv_3d_navigation::Serialization::Request::SPARSE_MAP;
	srvMsg.request.store = false;
	srvMsg.request.directory = ".";
	serializationClient.call(srvMsg);
	ROS_INFO_STREAM("Success: "<<srvMsg.response.done);
}

void saveDenseCallBack(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
	ROS_INFO_STREAM("Saving dense map");
	// Call service
	ugv_3d_navigation::Serialization srvMsg;
	srvMsg.request.step = ugv_3d_navigation::Serialization::Request::DENSE_MAP;
	srvMsg.request.store = true;
	srvMsg.request.directory = ".";
	serializationClient.call(srvMsg);
	ROS_INFO_STREAM("Success: "<<srvMsg.response.done);
}

void loadDenseCallBack(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
	ROS_INFO_STREAM("Loading dense map");
	// Call service
	ugv_3d_navigation::Serialization srvMsg;
	srvMsg.request.step = ugv_3d_navigation::Serialization::Request::DENSE_MAP;
	srvMsg.request.store = false;
	srvMsg.request.directory = ".";
	serializationClient.call(srvMsg);
	ROS_INFO_STREAM("Success: "<<srvMsg.response.done);
}

void savePlannerCallBack(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
	ROS_INFO_STREAM("Saving planner state");
	// Call service
	ugv_3d_navigation::Serialization srvMsg;
	srvMsg.request.step = ugv_3d_navigation::Serialization::Request::PLANNER;
	srvMsg.request.store = true;
	srvMsg.request.directory = ".";
	serializationClient.call(srvMsg);
	ROS_INFO_STREAM("Success: "<<srvMsg.response.done);
}

void loadPlannerCallBack(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
	ROS_INFO_STREAM("Loading planner state");
	// Call service
	ugv_3d_navigation::Serialization srvMsg;
	srvMsg.request.step = ugv_3d_navigation::Serialization::Request::PLANNER;
	srvMsg.request.store = false;
	srvMsg.request.directory = ".";
	serializationClient.call(srvMsg);
	ROS_INFO_STREAM("Success: "<<srvMsg.response.done);
}

void savePathCallBack(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
	ROS_INFO_STREAM("Saving path");
	// Call service
	ugv_3d_navigation::Serialization srvMsg;
	srvMsg.request.step = ugv_3d_navigation::Serialization::Request::PATH;
	srvMsg.request.store = true;
	srvMsg.request.directory = ".";
	serializationClient.call(srvMsg);
	ROS_INFO_STREAM("Success: "<<srvMsg.response.done);
}

void loadPathCallBack(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
	ROS_INFO_STREAM("Loading path");
	// Call service
	ugv_3d_navigation::Serialization srvMsg;
	srvMsg.request.step = ugv_3d_navigation::Serialization::Request::PATH;
	srvMsg.request.store = false;
	srvMsg.request.directory = ".";
	serializationClient.call(srvMsg);
	ROS_INFO_STREAM("Success: "<<srvMsg.response.done);
}

void saveStartGoalCallBack(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
	ROS_INFO_STREAM("Saving start and goal positions and input point cloud");
	// Call service
	ugv_3d_navigation::Serialization srvMsg;
	srvMsg.request.step = ugv_3d_navigation::Serialization::Request::PROBLEM;
	srvMsg.request.store = true;
	srvMsg.request.directory = ".";
	serializationClient.call(srvMsg);
	ROS_INFO_STREAM("Success: "<<srvMsg.response.done);
}

void loadStartGoalCallBack(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
	ROS_INFO_STREAM("Loading start and goal positions and input point cloud");
	// Call service
	ugv_3d_navigation::Serialization srvMsg;
	srvMsg.request.step = ugv_3d_navigation::Serialization::Request::PROBLEM;
	srvMsg.request.store = false;
	srvMsg.request.directory = ".";
	serializationClient.call(srvMsg);
	ROS_INFO_STREAM("Success: "<<srvMsg.response.done);
}
	

void addRotAndTransCtrl(InteractiveMarker &int_marker, const double w, const double x, const double y, const double z, const std::string name)
{
	InteractiveMarkerControl control;
	control.orientation.w = w;
	control.orientation.x = x;
	control.orientation.y = y;
	control.orientation.z = z;
	//control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
	control.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE;
	control.name = "rotate_" + name;

  int_marker.controls.push_back( control );
	
	// Create translation ctrl
	// keep the same quaternion
	//control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
	//control.name = "translate_"+name;
  //int_marker.controls.push_back( control );
}

void makeMenuMarker( std::string name )
{
	InteractiveMarker int_marker;

	// Header
	// TODO: adapt that for real system
	int_marker.header.frame_id = "/map";
	// Initial pose and shape
//	int_marker.pose.position.y = 0.0;
//	int_marker.pose.position.x = 0.0;
//	int_marker.pose.position.z = 0.0;

//	geometry_msgs::PoseStamped marker_local;
//	geometry_msgs::PoseStamped marker_global;
	tf::StampedTransform transform;
	if(ptf_listener->waitForTransform( "/base_link","/map",ros::Time(0), ros::Duration(2.)))
		ptf_listener->lookupTransform("/map","/base_link", ros::Time(0),transform);

	int_marker.pose.position.x = transform.getOrigin().x();
	int_marker.pose.position.y = transform.getOrigin().y();
	int_marker.pose.position.z = transform.getOrigin().z();

	int_marker.scale = 1;

 	// Information
	int_marker.name = name;
	int_marker.description = "Right click to interact with the planner";
	
	// Create 6 DoF control axis
	addRotAndTransCtrl(int_marker, 1, 1, 0, 0, "x");
	addRotAndTransCtrl(int_marker, 1, 0, 1, 0, "z");
	addRotAndTransCtrl(int_marker, 1, 0, 0, 1, "y");

	// Create a gray box to support a menu
	Marker grayBox;
  grayBox.type = Marker::CUBE;
  grayBox.scale.x = 0.60;
  grayBox.scale.y = 0.60;
  grayBox.scale.z = 0.5;
  grayBox.color.r = 0.5;
  grayBox.color.g = 0.5;
  grayBox.color.b = 0.5;
  grayBox.color.a = 0.75;

	InteractiveMarkerControl control;
	control = InteractiveMarkerControl();
  control.markers.push_back(grayBox);
  control.interaction_mode = InteractiveMarkerControl::BUTTON;
	int_marker.controls.push_back(control);

  server->insert( int_marker );
	server->setCallback(int_marker.name, &validateMenu, visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE);
}




void initMenu()
{
  h_map = menu_handler.insert( "Request Global Map", &callMapCallback);
  h_goal = menu_handler.insert( "Set goal", &setGoalCallback);
	h_plan = menu_handler.insert( "Plan to goal", &planCallback);
	menu_handler.setVisible(h_plan, false);
	h_exe = menu_handler.insert( "Go to goal", &executeCallback);
	menu_handler.setVisible(h_exe, false);
	h_save_sparse = menu_handler.insert("Save sparse map", &saveSparseCallBack);
	h_load_sparse = menu_handler.insert("Load sparse map", &loadSparseCallBack);
	h_save_dense = menu_handler.insert("Save dense map", &saveDenseCallBack);
	h_load_dense = menu_handler.insert("Load dense map", &loadDenseCallBack);
	h_save_planner = menu_handler.insert("Save planner state", &savePlannerCallBack);
	h_load_planner = menu_handler.insert("Load planner state", &loadPlannerCallBack);
	h_save_path = menu_handler.insert("Save path", &savePathCallBack);
	h_load_path = menu_handler.insert("Load path", &loadPathCallBack);
	h_save_startgoal = menu_handler.insert("Save problem", &saveStartGoalCallBack);
	h_load_startgoal = menu_handler.insert("Load problem", &loadStartGoalCallBack);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "goal");
	ros::NodeHandle n;
	ptf_listener = new(tf::TransformListener);

	callMapClient = n.serviceClient<ugv_3d_navigation::CallGlobalMap>("call_global_map");
	goalClient = n.serviceClient<ugv_3d_navigation::ConfirmGoalStamped>("set_goal");
	planClient = n.serviceClient<ugv_3d_navigation::ComputePlan>("compute_plan");
	exeClient = n.serviceClient<ugv_3d_navigation::ExecutePlan>("execute_plan");
	serializationClient = n.serviceClient<ugv_3d_navigation::Serialization>("serialization");
  server.reset( new InteractiveMarkerServer("goal","",false) );

  initMenu();

  makeMenuMarker( "planner_menu" );

  menu_handler.apply( *server, "planner_menu" );
  server->applyChanges();

  ros::spin();

  server.reset();
}
