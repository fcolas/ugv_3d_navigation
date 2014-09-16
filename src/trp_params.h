#ifndef TRP_PARAMS_H
#define TRP_PARAMS_H

#include "yaml-cpp-pm/yaml.h"
#include <Eigen/Eigen>
#include <string>
#include <cmath>

using namespace std;

/*
 * Parameter objects
 */
//! Parameters of the distance filter
struct DistanceFilterParams {
	//! Constructor for default values
	DistanceFilterParams():
		min_distance(0.03),
		max_knn(100) {}
	float min_distance;
	int max_knn;
};


//! Parameters for sparse voting
struct SparseVotingParams {
	//! Constructor for default values
	SparseVotingParams():
		sigma(0.3),
		max_dist(0.6),
		max_knn(100) {}
	float sigma;
	float max_dist;
	int max_knn;
};


//! Parameters for dense voting
struct DenseVotingParams {
	//! Constructor for default values
	DenseVotingParams():
		sigma(0.3),
		max_dist(0.6),
		max_knn(2000),
		obs_dir_offset(0.3),
		absolute_saliency_threshold(100) {}
	float sigma;
	float max_dist;
	int max_knn;
	float obs_dir_offset;
	float absolute_saliency_threshold;
};


//! Parameters for traversability
struct TraversabilityParams {
	//! Constructor for default values
	TraversabilityParams():
		length(0.7),
		width(0.6),
		height(0.5),
		diameter(sqrt(pow(length, 2)+pow(width, 2))),
		inflation(0.1),
		ground_buffer(0.09),
		min_saliency(100),
		max_slope(M_PI_4),
		max_points_in_free_cell(0),
		min_free_cell_ratio(0.95),
		max_points_in_bounding_box(100),
		min_support_points(100) {}
	float length;
	float width;
	float height;
	float diameter;
	float inflation;
	float ground_buffer;
	float min_saliency;
	float max_slope;
	int max_points_in_free_cell;
	float min_free_cell_ratio;
	int max_points_in_bounding_box;
	int min_support_points;
};


//! Parameters for tensor map
struct TensorMapParams {
	//! Constructor for default values
	TensorMapParams():
		cell_dimensions({0.09, 0.09, 0.09}),
		origin({0., 0., 0.}) {}
	Eigen::Vector3f cell_dimensions;
	Eigen::Vector3f origin;
	DistanceFilterParams distance_filter;
	SparseVotingParams sparse_voting;
	DenseVotingParams dense_voting;
	TraversabilityParams traversability;
};


//! Parameters for cost functions
struct CostFunctionsParams {
	//! Constructor for default values
	CostFunctionsParams():
		saliency_factor(1.),
		orientation_factor(1.),
		distance_factor(1.),
		heading_factor(5.),
		max_slope(M_PI_4),
		saliency_threshold(1000),
		maximal_saliency(10000) {}
	float saliency_factor;
	float orientation_factor;
	float distance_factor;
	float heading_factor;
	float max_slope;
	float saliency_threshold;
	float maximal_saliency;
};


//! Parameters for path decoration with flipper information
struct FlippersParams {
	//! Constructor for default values
	FlippersParams():
		slope_threshold(17*M_PI/180),
		flat_threshold(5*M_PI/180),
		change_threshold(10*M_PI/180),
		approach_up_before(7),
		approach_up_after(3),
		approach_down_before(7),
		approach_down_after(3),
		convex_up_before(0),
		convex_up_after(3),
		convex_down_before(0),
		convex_down_after(3) {}
	float slope_threshold;
	float flat_threshold;
	float change_threshold;
	int approach_up_before;
	int approach_up_after;
	int approach_down_before;
	int approach_down_after;
	int convex_up_before;
	int convex_up_after;
	int convex_down_before;
	int convex_down_after;
}; 


//! Parameters for path following
struct PathFollowingParams {
	//! Constructor for default values
	PathFollowingParams():
		v_max_flat(0.1),
		w_max_flat(0.3),
		v_max_slope(0.05),
		w_max_slope(0.075),
		max_distance(0.2),
		ignore_radius(0.25) {}
	float v_max_flat;
	float w_max_flat;
	float v_max_slope;
	float w_max_slope;
	float max_distance;
	float ignore_radius;
};


//! Parameters for path execution
struct LocalPlannerParams {
	//! Constructor for default values
	LocalPlannerParams() {} // unneeded 
		FlippersParams flippers_params;
		PathFollowingParams path_following_params;
};


//! Parameters for tensor_replanner
struct TRPParams {
	//! Constructor for default values
	TRPParams() {} // unneeded 
	TensorMapParams tensor_map;
	CostFunctionsParams cost_functions;
	LocalPlannerParams local_planner;
};


/*
 * Global interface
 */
//! Load trp configuration from a yaml file
void load_params(const string& filename, TRPParams& p);

//! Save trp configuration in a yaml file
void save_params(const string& filename, const TRPParams& p);


/*
 * Specific interface
 */
//! yaml-cpp interface for DistanceFilterParams
void operator>> (const YAML_PM::Node& node, DistanceFilterParams& p);
YAML_PM::Emitter& operator<< (YAML_PM::Emitter& out, const DistanceFilterParams& p);

//! yaml-cpp interface for SparseVotingParams
void operator>> (const YAML_PM::Node& node, SparseVotingParams& p);
YAML_PM::Emitter& operator<< (YAML_PM::Emitter& out, const SparseVotingParams& p);

//! yaml-cpp interface for DenseVotingParams
void operator>> (const YAML_PM::Node& node, DenseVotingParams& p);
YAML_PM::Emitter& operator<< (YAML_PM::Emitter& out, const DenseVotingParams& p);

//! yaml-cpp interface for TraversabilityParams
void operator>> (const YAML_PM::Node& node, TraversabilityParams& p);
YAML_PM::Emitter& operator<< (YAML_PM::Emitter& out, const TraversabilityParams& p);

//! yaml-cpp interface for TensorMapParams
void operator>> (const YAML_PM::Node& node, TensorMapParams& p);
YAML_PM::Emitter& operator<< (YAML_PM::Emitter& out, const TensorMapParams& p);

//! yaml-cpp interface for CostFunctionsParams
void operator>> (const YAML_PM::Node& node, CostFunctionsParams& p);
YAML_PM::Emitter& operator<< (YAML_PM::Emitter& out, const CostFunctionsParams& p);

//! yaml-cpp interface for FlippersParams
void operator>> (const YAML_PM::Node& node, FlippersParams& p);
YAML_PM::Emitter& operator<< (YAML_PM::Emitter& out, const FlippersParams& p);

//! yaml-cpp interface for PathFollowingParams
void operator>> (const YAML_PM::Node& node, PathFollowingParams& p);
YAML_PM::Emitter& operator<< (YAML_PM::Emitter& out, const PathFollowingParams& p);

//! yaml-cpp interface for LocalPlannerParams
void operator>> (const YAML_PM::Node& node, LocalPlannerParams& p);
YAML_PM::Emitter& operator<< (YAML_PM::Emitter& out, const LocalPlannerParams& p);

//! yaml-cpp interface for TRPParams
void operator>> (const YAML_PM::Node& node, TRPParams& p);
YAML_PM::Emitter& operator<< (YAML_PM::Emitter& out, const TRPParams& p);


#endif
