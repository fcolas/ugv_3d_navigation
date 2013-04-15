// standard
#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>

// ROS 
#include <ros/ros.h>
#include <pointmatcher_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <nabo/nabo.h>

// Various functions
#include "tensorUtils.h"

using namespace std;
using namespace Nabo;

PointCloud random_filter(const PointCloud& cloud, size_t nb_max) {
	const size_t nb_points = cloud.features.cols();
	const bool is_desc(cloud.descriptors.rows()*cloud.descriptors.cols());
	PointCloud result(cloud.featureLabels, cloud.descriptorLabels, nb_max);
	size_t i_new=0;
	//cout << RAND_MAX << " " << SIZE_MAX << endl;
	for (size_t i_old=0; i_old<nb_points; ++i_old) {
		if (i_new==nb_max) {
			break;
		}
		// adapting probability so that number of points is reached
		if ((nb_points-i_old)*rand()<RAND_MAX*(nb_max-i_new)) {
			if (is_desc) {
				result.descriptors.col(i_new) = cloud.descriptors.col(i_old);
			}
			result.features.col(i_new) = cloud.features.col(i_old);
			++i_new;
		}
	}
	assert(i_new==nb_max);
	return result;
}

PointCloud distance_filter(const PointCloud& cloud, float min_dist) {
	shared_ptr<NNSearchF> kdTree(NNSearchF::create(cloud.features,
			cloud.features.rows()-1, NNSearchF::KDTREE_TREE_HEAP));

	const int nb_points = cloud.features.cols();
	NNSearchF::IndexMatrix indices(100, nb_points);
	NNSearchF::Matrix dists2(100, nb_points);
	const float min2 = pow(min_dist, 2);
	// finding out points to remove
	vector<bool> is_not_removed(nb_points, true);
	kdTree->knn(cloud.features, indices, dists2, 100, 0, NNSearchF::SORT_RESULTS, min_dist);
	for (int i=0; i<nb_points;++i) {
		if (is_not_removed[i]) {
			for (int j=0; j<100; ++j) {
				if (dists2(j, i)>min2) {
					break;
				} else {
					is_not_removed[indices(j, i)] = false;
				}
			}
		}
	}
	// recreating point cloud
	const bool is_desc(cloud.descriptors.rows()*cloud.descriptors.cols());
	const int nb_new = count(is_not_removed.cbegin(), is_not_removed.cend(), true);
	PointCloud result(cloud.featureLabels, cloud.descriptorLabels, nb_new);
	int i_new = 0;
	for (int i_old=0; i_old<nb_points; ++i_old) {
		if (is_not_removed[i_old]) {
			if (is_desc) {
				result.descriptors.col(i_new) = cloud.descriptors.col(i_old);
			}
			result.features.col(i_new) = cloud.features.col(i_old);
			++i_new;
		}
	}
	cout <<"Filtering: "<<nb_points<<"->"<<nb_new<<endl;
	return result;
}


//! Vizualization of the computations of trp_batch
class DistFiltEval {
public:
	//! Constructor. ROS::init() is assumed to have been called before.
	DistFiltEval();

	//! Destructor
	virtual ~DistFiltEval();

	//! Load input from a given file
	bool loadInput(const string& filename, float min_dist);

protected:
	//! public NodeHandle
	ros::NodeHandle n;

	//! private NodeHandle
	ros::NodeHandle n_;

	//! Publisher of input point cloud
	ros::Publisher input_point_cloud_pub;

	//! Publisher of dist filtered map
	ros::Publisher dist_filter_pub;
	
	//! Publisher of random filtered map
	ros::Publisher random_filter_pub;
	
};


// Constructor. ROS::init() is assumed to have been called before.
DistFiltEval::DistFiltEval():
	n_("~") {
	input_point_cloud_pub = n.advertise<sensor_msgs::PointCloud2>
			("/input_point_cloud", 1, true);
	dist_filter_pub = n.advertise<sensor_msgs::PointCloud2>
			("/dist_filtered", 1, true);
	random_filter_pub = n.advertise<sensor_msgs::PointCloud2>
			("/random_filtered", 1, true);

}


// Destructor
DistFiltEval::~DistFiltEval() {
	// nothing to do?
}

// Load full content of a given directory
bool DistFiltEval::loadInput(const string& filename, float min_dist) {
	PointCloud tmp_cloud, new_cloud;
	// Input point cloud
	sensor_msgs::PointCloud2 point_cloud;

	bool ipc = deSerializePointCloud(tmp_cloud, filename);
	if (!ipc) {
		ROS_ERROR_STREAM("Couldn't load "<<filename);
		return false;
	}
	
	point_cloud = PointMatcher_ros::\
			pointMatcherCloudToRosMsg<float>(tmp_cloud, "/map",
			ros::Time::now());
	input_point_cloud_pub.publish(point_cloud);
	ROS_INFO_STREAM("Loaded: " << filename << " with " << tmp_cloud.features.cols() <<
			"pts");

	new_cloud = distance_filter(tmp_cloud, min_dist);
	point_cloud = PointMatcher_ros::\
			pointMatcherCloudToRosMsg<float>(new_cloud, "/map",
			ros::Time::now());
	dist_filter_pub.publish(point_cloud);
	ROS_INFO_STREAM("Filtered down to "<<new_cloud.features.cols() << "pts");

	new_cloud = random_filter(tmp_cloud, new_cloud.features.cols());
	point_cloud = PointMatcher_ros::\
			pointMatcherCloudToRosMsg<float>(new_cloud, "/map",
			ros::Time::now());
	random_filter_pub.publish(point_cloud);
	ROS_INFO_STREAM("Filtered down to "<<new_cloud.features.cols() << "pts");

	return true;
}



/*
 * main
 */
int main(int argc, char **argv) {
	ros::init(argc, argv, "dist_filter_eval");
	string filename = "test/input_point_cloud.csv";
	float min_dist=0.05;
	if (argc>=2) {
		filename = argv[1];
	}
	if (argc>=3) {
		min_dist = atof(argv[2]);
	}
	DistFiltEval dfe;
	dfe.loadInput(filename, min_dist);
	ros::spin();
	return 0;
}
