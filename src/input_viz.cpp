// standard
#include <iostream>
#include <string>
#include <stdio.h>

// ROS 
#include <ros/ros.h>
#include <pointmatcher_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

// Various functions
#include "tensorUtils.h"


using namespace std;

//! Publication of the input point cloud
class InputViz {
public:
	//! Constructor. ROS::init() is assumed to have been called before.
	InputViz(const string& filename="input_point_cloud.csv");

	//! Destructor
	virtual ~InputViz();

protected:
	//! public NodeHandle
	ros::NodeHandle n;

	//! private NodeHandle
	ros::NodeHandle n_;

	//! Publisher of input point cloud
	ros::Publisher input_point_cloud_pub;
};


// Constructor. ROS::init() is assumed to have been called before.
InputViz::InputViz(const string& filename):
	n_("~") {
	input_point_cloud_pub = n.advertise<sensor_msgs::PointCloud2>
			("/input_point_cloud", 1, true);
	
	PointCloud tmp_cloud;
	sensor_msgs::PointCloud2 input_point_cloud;
	bool ipc = deSerializePointCloud(tmp_cloud, filename);
	if (ipc) {
		input_point_cloud = PointMatcher_ros::\
				pointMatcherCloudToRosMsg<float>(tmp_cloud, "/map",
				ros::Time::now());
		input_point_cloud_pub.publish(input_point_cloud);
		ROS_INFO("Publishing.");
	} else {
		ROS_ERROR_STREAM("Couldn't load "<<filename);
	}
}


// Destructor
InputViz::~InputViz() {
	// nothing to do?
}


/*
 * main
 */
int main(int argc, char **argv) {
	ros::init(argc, argv, "input_viz");
	string filename;
	if (argc>=2) {
		InputViz iv(argv[1]);
	} else {
		InputViz iv;
	}
	ros::spin();
	return 0;
}
