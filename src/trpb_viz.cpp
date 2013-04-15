// standard
#include <iostream>
#include <string>
#include <stdio.h>

// ROS 
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
//#include <pointmatcher/PointMatcher.h>
#include <pointmatcher_ros/point_cloud.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/JointState.h>

// Various functions
#include "tensorUtils.h"


using namespace std;

//! Vizualization of the computations of trp_batch
class TRPBViz {
public:
	//! Constructor. ROS::init() is assumed to have been called before.
	TRPBViz(const string& directory, const string& filename);

	//! Destructor
	virtual ~TRPBViz();

protected:
	//! public NodeHandle
	ros::NodeHandle n;

	//! private NodeHandle
	ros::NodeHandle n_;

	//! /tf for robot position
	tf::StampedTransform tf_start;
	//! /tf for base_link
	tf::StampedTransform tf_base_link;

	//! Publisher of input point cloud
	ros::Publisher input_point_cloud_pub;

	//! Publisher of sparse map
	ros::Publisher sparse_map_pub;

	//! /tf broadcaster
	tf::TransformBroadcaster tf_broadcaster;
	
	//! Fake joint state publisher (for robot model)
	ros::Publisher joint_state_pub;

	//! Joint states to be published
	sensor_msgs::JointState joint_states;

	//! Publisher of goal marker
	ros::Publisher goal_pub;

	//! Publisher of planned path
	ros::Publisher path_pub;

	//! Publisher of reference path
	ros::Publisher ref_pub;

	//! Timer for the loop
	ros::Timer timer;

	//! Timer callback
	void timerCallback(const ros::TimerEvent& e);

	//! Load full content of a given directory
	bool loadDirectory(const string& directory);

	//! Load start and goal positions
	bool loadStartGoal(const string& filename, geometry_msgs::Pose& start,
			geometry_msgs::Pose& goal);

	//! Load path
	bool loadPath(const string& filename, nav_msgs::Path& path);
};


// Constructor. ROS::init() is assumed to have been called before.
TRPBViz::TRPBViz(const string& directory, const string& filename):
	n_("~") {
	input_point_cloud_pub = n.advertise<sensor_msgs::PointCloud2>
			("/input_point_cloud", 1, true);
	sparse_map_pub = n.advertise<sensor_msgs::PointCloud2>
			("/sparse_map", 1, true);
	goal_pub = n.advertise<visualization_msgs::Marker>("/goal", 1, true);
	path_pub = n.advertise<nav_msgs::Path>("/path", 1, true);
	ref_pub = n.advertise<nav_msgs::Path>("/ref_path", 1, true);
	joint_state_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
	joint_states.header.frame_id = "/base_link";
	joint_states.name = {"left_track_j", "right_track_j",
			"front_left_flipper_j", "front_right_flipper_j",
			"rear_left_flipper_j", "rear_right_flipper_j", "laser_j"};
	joint_states.position = {0, 0, M_PI, M_PI, M_PI_2, M_PI_2, 0};
	joint_states.header.stamp = ros::Time::now();
	joint_state_pub.publish(joint_states);
	
	tf_base_link.setOrigin({0, 0, 0.0705});
	tf_base_link.setRotation({0, 0, 0, 1});
	tf_base_link.frame_id_ = "/base_footprint";
	tf_base_link.child_frame_id_ = "/base_link";
	tf_base_link.stamp_ = ros::Time::now();
	tf_broadcaster.sendTransform(tf_base_link);

	if (loadDirectory(directory)) {
		nav_msgs::Path path;
		bool p = loadPath(filename, path);
		if (p) {
			path_pub.publish(path);
		}
		ROS_INFO("Publishing.");
		timer = n.createTimer(ros::Rate(1), &TRPBViz::timerCallback, this);
	} else {
		ROS_ERROR("Nothing to publish.");
	}
}


// Destructor
TRPBViz::~TRPBViz() {
	// nothing to do?
}


// Timer callback
void TRPBViz::timerCallback(const ros::TimerEvent& e) {
	tf_start.stamp_ = ros::Time::now();
	tf_broadcaster.sendTransform(tf_start);
	tf_base_link.stamp_ = ros::Time::now();
	tf_broadcaster.sendTransform(tf_base_link);
	joint_states.header.stamp = ros::Time::now();
	joint_state_pub.publish(joint_states);
}


// Load full content of a given directory
bool TRPBViz::loadDirectory(const string& directory) {
	PointCloud tmp_cloud;
	// Input point cloud
	sensor_msgs::PointCloud2 input_point_cloud;
	bool ipc = deSerializePointCloud(tmp_cloud,
			directory+"/input_point_cloud.csv");
	if (ipc) {
		input_point_cloud = PointMatcher_ros::\
				pointMatcherCloudToRosMsg<float>(tmp_cloud, "/map",
				ros::Time::now());
		input_point_cloud_pub.publish(input_point_cloud);
	}
	// Sparse map point cloud (if any)
	sensor_msgs::PointCloud2 sparse_map;
	bool sm = deSerializePointCloud(tmp_cloud,
			directory+"/sparse_map.csv");
	if (sm) {
		sparse_map = PointMatcher_ros::\
				pointMatcherCloudToRosMsg<float>(tmp_cloud, "/map",
				ros::Time::now());
		sparse_map_pub.publish(sparse_map);
	}
	// Robot position (start)
	geometry_msgs::Pose start;
	// Goal position
	geometry_msgs::Pose goal;
	bool sg = loadStartGoal(directory+"/start_goal.csv", start, goal);
	if (sg) {
		// FIXME correct z by 0.0705
		tf::poseMsgToTF(start, tf_start);
		tf_start.frame_id_ = "/map";
		tf_start.child_frame_id_ = "/base_footprint";
		tf_start.stamp_ = ros::Time::now();
		tf_broadcaster.sendTransform(tf_start);
		visualization_msgs::Marker marker;
		marker.header.frame_id = "/map";
		marker.header.stamp = ros::Time();
		marker.ns = "Goal";
		marker.id = 0;
		marker.type = visualization_msgs::Marker::CUBE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose = goal;
		marker.scale.x = 0.60;
		marker.scale.y = 0.60;
		marker.scale.z = 0.5;
		marker.color.r = 0.5;
		marker.color.g = 0.5;
		marker.color.b = 0.5;
		marker.color.a = .75;
		marker.lifetime = ros::Duration();
		goal_pub.publish(marker);
	} else {
		tf_start.setOrigin({0, 0, 0});
		tf_start.setRotation({0, 0, 0, 1});
		tf_start.frame_id_ = "/map";
		tf_start.child_frame_id_ = "/base_footprint";
	}
	// Path
	nav_msgs::Path path;
	bool p = loadPath(directory+"/path.csv", path);
	if (p) {
		ref_pub.publish(path);
	}
	return (ipc||sm||sg||p);
}


// Load start and goal positions
bool TRPBViz::loadStartGoal(const string& filename, geometry_msgs::Pose& start,
		geometry_msgs::Pose& goal) {
	FILE* start_goal_file = fopen(filename.c_str(), "r");
	if (!start_goal_file) {
		return false;
	}
	float x, y, z, qx, qy, qz, qw;
	if (fscanf(start_goal_file, "%f,%f,%f,%f,%f,%f,%f\n",
			&x, &y, &z, &qx, &qy, &qz, &qw)!=7) {
		fclose(start_goal_file);
		return false;
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
		fclose(start_goal_file);
		return false;
	}
	goal.position.x = x;
	goal.position.y = y;
	goal.position.z = z;
	goal.orientation.x = qx;
	goal.orientation.y = qy;
	goal.orientation.z = qz;
	goal.orientation.w = qw;
	fclose(start_goal_file);
	return true;
}


// Load path
bool TRPBViz::loadPath(const string& filename, nav_msgs::Path& path) {
	path.header.frame_id = "/map";
	path.header.stamp = ros::Time::now();
	path.poses.clear();
	/*
	 * deserialization of path in csv
	 * 
	 * format:
	 * i,j,k,x,y,z,stick_sal,nx,ny,nz
	 */
	FILE* path_file = fopen(filename.c_str(), "r");
	if (!path_file) {
		return false;
	}
	float x, y, z, nx, ny, nz, dx, dy, dz;
	int posture, edge;
	int n;
	int line = 1;
	geometry_msgs::PoseStamped tmp_pose;
	tmp_pose.header = path.header;
	while (!feof(path_file)) {
		n = fscanf(path_file, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d\n",
				&x, &y, &z, &nx, &ny, &nz, &dx, &dy, &dz, &posture, &edge);
		if (n!=11) {
			cout << "Error reading line "<<line<<" in "<<filename<<
				": only "<<n<<" tokens found."<<endl;
		} else {
			tmp_pose.pose.position.x = x;
			tmp_pose.pose.position.y = y;
			tmp_pose.pose.position.z = z;
			// could get the orientation from the direction and the normal
			path.poses.push_back(tmp_pose);
		}
	}
	fclose(path_file);
	return true;
}


/*
 * main
 */
int main(int argc, char **argv) {
	ros::init(argc, argv, "trpb_viz");
	string directory, filename;
	if (argc>=2) {
		directory = argv[1];
	} else {
		directory = "test";
	}
	if (argc>=3) {
		filename = argv[2];
	} else {
		filename = "test/path.csv";
	}
	TRPBViz trp_viz(directory, filename);
	ros::spin();
	return 0;
}
