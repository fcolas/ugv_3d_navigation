#include "trp_params.h"
#include <fstream>

using namespace std;

/*
 * Global interface
 */
// Load trp configuration from a yaml file
void load_params(const string& filename, TRPParams& p) {
	ifstream fin(filename);
	YAML_PM::Parser parser(fin);
	YAML_PM::Node doc;
	parser.GetNextDocument(doc);
	doc >> p;
	fin.close();
}

// Save trp configuration in a yaml file
void save_params(const string& filename, const TRPParams& p) {
	YAML_PM::Emitter out;
	ofstream fout(filename);
	out << p;
	fout << out.c_str();
	fout.close();
}


/*
 * Specific interface
 */
// yaml-cpp interface for DistanceFilterParams
// extraction
void operator>> (const YAML_PM::Node& node, DistanceFilterParams& p) {
	try{
		node["min_distance"] >> p.min_distance;
	} catch(YAML_PM::KeyNotFound& e) {}
	try{
		node["max_knn"] >> p.max_knn;
	} catch(YAML_PM::KeyNotFound& e) {}
}
// emission
YAML_PM::Emitter& operator<< (YAML_PM::Emitter& out, const DistanceFilterParams& p) {
	out << YAML_PM::BeginMap;
	out << YAML_PM::Key << "min_distance" << YAML_PM::Value << p.min_distance;
	out << YAML_PM::Key << "max_knn" << YAML_PM::Value << p.max_knn;
	out << YAML_PM::EndMap;
	return out;
}


// yaml-cpp interface for SparseVotingParams
// extraction
void operator>> (const YAML_PM::Node& node, SparseVotingParams& p) {
	try{
		node["sigma"] >> p.sigma;
	} catch(YAML_PM::KeyNotFound& e) {}
	try{
		node["max_dist"] >> p.max_dist;
	} catch(YAML_PM::KeyNotFound& e) {}
	try{
		node["max_knn"] >> p.max_knn;
	} catch(YAML_PM::KeyNotFound& e) {}
}
// emission
YAML_PM::Emitter& operator<< (YAML_PM::Emitter& out, const SparseVotingParams& p) {
	out << YAML_PM::BeginMap;
	out << YAML_PM::Key << "sigma" << YAML_PM::Value << p.sigma;
	out << YAML_PM::Key << "max_dist" << YAML_PM::Value << p.max_dist;
	out << YAML_PM::Key << "max_knn" << YAML_PM::Value << p.max_knn;
	out << YAML_PM::EndMap;
	return out;
}


// yaml-cpp interface for DenseVotingParams
// extraction
void operator>> (const YAML_PM::Node& node, DenseVotingParams& p) {
	try{
		node["sigma"] >> p.sigma;
	} catch(YAML_PM::KeyNotFound& e) {}
	try{
		node["max_dist"] >> p.max_dist;
	} catch(YAML_PM::KeyNotFound& e) {}
	try{
		node["max_knn"] >> p.max_knn;
	} catch(YAML_PM::KeyNotFound& e) {}
	try{
		node["obs_dir_offset"] >> p.obs_dir_offset;
	} catch(YAML_PM::KeyNotFound& e) {}
	try{
		node["absolute_saliency_threshold"] >> p.absolute_saliency_threshold;
	} catch(YAML_PM::KeyNotFound& e) {}
}
// emission
YAML_PM::Emitter& operator<< (YAML_PM::Emitter& out, const DenseVotingParams& p) {
	out << YAML_PM::BeginMap;
	out << YAML_PM::Key << "sigma" << YAML_PM::Value << p.sigma;
	out << YAML_PM::Key << "max_dist" << YAML_PM::Value << p.max_dist;
	out << YAML_PM::Key << "max_knn" << YAML_PM::Value << p.max_knn;
	out << YAML_PM::Key << "obs_dir_offset" << YAML_PM::Value << p.obs_dir_offset;
	out << YAML_PM::Key << "absolute_saliency_threshold" << YAML_PM::Value << p.absolute_saliency_threshold;
	out << YAML_PM::EndMap;
	return out;
}


// yaml-cpp interface for TraversabilityParams
// extraction
void operator>> (const YAML_PM::Node& node, TraversabilityParams& p) {
	try{
		node["length"] >> p.length;
	} catch(YAML_PM::KeyNotFound& e) {}
	try{
		node["width"] >> p.width;
	} catch(YAML_PM::KeyNotFound& e) {}
	try{
		node["height"] >> p.height;
	} catch(YAML_PM::KeyNotFound& e) {}
	try{
		node["diameter"] >> p.diameter;
	} catch(YAML_PM::KeyNotFound& e) {}
	try{
		node["inflation"] >> p.inflation;
	} catch(YAML_PM::KeyNotFound& e) {}
	try{
		node["ground_buffer"] >> p.ground_buffer;
	} catch(YAML_PM::KeyNotFound& e) {}
	try{
		node["min_saliency"] >> p.min_saliency;
	} catch(YAML_PM::KeyNotFound& e) {}
	try{
		node["max_slope"] >> p.max_slope;
	} catch(YAML_PM::KeyNotFound& e) {}
	try{
		node["max_points_in_free_cell"] >> p.max_points_in_free_cell;
	} catch(YAML_PM::KeyNotFound& e) {}
	try{
		node["min_free_cell_ratio"] >> p.min_free_cell_ratio;
	} catch(YAML_PM::KeyNotFound& e) {}
	try{
		node["max_points_in_bounding_box"] >> p.max_points_in_bounding_box;
	} catch(YAML_PM::KeyNotFound& e) {}
	try{
		node["min_support_points"] >> p.min_support_points;
	} catch(YAML_PM::KeyNotFound& e) {}
}
// emission
YAML_PM::Emitter& operator<< (YAML_PM::Emitter& out, const TraversabilityParams& p) {
	out << YAML_PM::BeginMap;
	out << YAML_PM::Key << "length" << YAML_PM::Value << p.length;
	out << YAML_PM::Key << "width" << YAML_PM::Value << p.width;
	out << YAML_PM::Key << "height" << YAML_PM::Value << p.height;
	out << YAML_PM::Key << "diameter" << YAML_PM::Value << p.diameter;
	out << YAML_PM::Key << "inflation" << YAML_PM::Value << p.inflation;
	out << YAML_PM::Key << "ground_buffer" << YAML_PM::Value << p.ground_buffer;
	out << YAML_PM::Key << "min_saliency" << YAML_PM::Value << p.min_saliency;
	out << YAML_PM::Key << "max_slope" << YAML_PM::Value << p.max_slope;
	out << YAML_PM::Key << "max_points_in_free_cell" << YAML_PM::Value << p.max_points_in_free_cell;
	out << YAML_PM::Key << "min_free_cell_ratio" << YAML_PM::Value << p.min_free_cell_ratio;
	out << YAML_PM::Key << "max_points_in_bounding_box" << YAML_PM::Value << p.max_points_in_bounding_box;
	out << YAML_PM::Key << "min_support_points" << YAML_PM::Value << p.min_support_points;
	out << YAML_PM::EndMap;
	return out;
}


// yaml-cpp interface for TensorMapParams
// extraction
void operator>> (const YAML_PM::Node& node, TensorMapParams& p) {
	try{
		node["cell_dimensions"][0] >> p.cell_dimensions(0);
		node["cell_dimensions"][1] >> p.cell_dimensions(1);
		node["cell_dimensions"][2] >> p.cell_dimensions(2);
	} catch(YAML_PM::KeyNotFound& e) {}
	try{
		node["origin"][0] >> p.origin(0);
		node["origin"][1] >> p.origin(1);
		node["origin"][2] >> p.origin(2);
	} catch(YAML_PM::KeyNotFound& e) {}
	try{
		node["distance_filter"] >> p.distance_filter;
	} catch(YAML_PM::KeyNotFound& e) {}
	try{
		node["sparse_voting"] >> p.sparse_voting;
	} catch(YAML_PM::KeyNotFound& e) {}
	try{
		node["dense_voting"] >> p.dense_voting;
	} catch(YAML_PM::KeyNotFound& e) {}
	try{
		node["traversability"] >> p.traversability;
	} catch(YAML_PM::KeyNotFound& e) {}
}
// emission
YAML_PM::Emitter& operator<< (YAML_PM::Emitter& out, const TensorMapParams& p) {
	out << YAML_PM::BeginMap;
	out << YAML_PM::Key << "cell_dimensions" << YAML_PM::Value << YAML_PM::BeginSeq;
	out << p.cell_dimensions[0] << p.cell_dimensions[1] << p.cell_dimensions[2] << YAML_PM::EndSeq;
	out << YAML_PM::Key << "origin" << YAML_PM::Value << YAML_PM::BeginSeq;
	out << p.origin[0] << p.origin[1] << p.origin[2] << YAML_PM::EndSeq;
	out << YAML_PM::Key << "distance_filter" << YAML_PM::Value << p.distance_filter;
	out << YAML_PM::Key << "sparse_voting" << YAML_PM::Value << p.sparse_voting;
	out << YAML_PM::Key << "dense_voting" << YAML_PM::Value << p.dense_voting;
	out << YAML_PM::Key << "traversability" << YAML_PM::Value << p.traversability;
	out << YAML_PM::EndMap;
	return out;
}


// yaml-cpp interface for CostFunctionsParams
// extraction
void operator>> (const YAML_PM::Node& node, CostFunctionsParams& p) {
	try{
		node["saliency_factor"] >> p.saliency_factor;
	} catch(YAML_PM::KeyNotFound& e) {}
	try{
		node["orientation_factor"] >> p.orientation_factor;
	} catch(YAML_PM::KeyNotFound& e) {}
	try{
		node["distance_factor"] >> p.distance_factor;
	} catch(YAML_PM::KeyNotFound& e) {}
	try{
		node["heading_factor"] >> p.heading_factor;
	} catch(YAML_PM::KeyNotFound& e) {}
	try{
		node["max_slope"] >> p.max_slope;
	} catch(YAML_PM::KeyNotFound& e) {}
	try{
		node["saliency_threshold"] >> p.saliency_threshold;
	} catch(YAML_PM::KeyNotFound& e) {}
	try{
		node["maximal_saliency"] >> p.maximal_saliency;
	} catch(YAML_PM::KeyNotFound& e) {}
}
// emission
YAML_PM::Emitter& operator<< (YAML_PM::Emitter& out, const CostFunctionsParams& p) {
	out << YAML_PM::BeginMap;
	out << YAML_PM::Key << "saliency_factor" << YAML_PM::Value << p.saliency_factor;
	out << YAML_PM::Key << "orientation_factor" << YAML_PM::Value << p.orientation_factor;
	out << YAML_PM::Key << "distance_factor" << YAML_PM::Value << p.distance_factor;
	out << YAML_PM::Key << "heading_factor" << YAML_PM::Value << p.heading_factor;
	out << YAML_PM::Key << "max_slope" << YAML_PM::Value << p.max_slope;
	out << YAML_PM::Key << "saliency_threshold" << YAML_PM::Value << p.saliency_threshold;
	out << YAML_PM::Key << "maximal_saliency" << YAML_PM::Value << p.maximal_saliency;
	out << YAML_PM::EndMap;
	return out;
}


// yaml-cpp interface for FlippersParams
// extraction
void operator>> (const YAML_PM::Node& node, FlippersParams& p) {
	try{
		node["slope_threshold"] >> p.slope_threshold;
	} catch(YAML_PM::KeyNotFound& e) {}
	try{
		node["flat_threshold"] >> p.flat_threshold;
	} catch(YAML_PM::KeyNotFound& e) {}
	try{
		node["change_threshold"] >> p.change_threshold;
	} catch(YAML_PM::KeyNotFound& e) {}
	try{
		node["approach_up_before"] >> p.approach_up_before;
	} catch(YAML_PM::KeyNotFound& e) {}
	try{
		node["approach_up_after"] >> p.approach_up_after;
	} catch(YAML_PM::KeyNotFound& e) {}
	try{
		node["approach_down_before"] >> p.approach_down_before;
	} catch(YAML_PM::KeyNotFound& e) {}
	try{
		node["approach_down_after"] >> p.approach_down_after;
	} catch(YAML_PM::KeyNotFound& e) {}
	try{
		node["convex_up_before"] >> p.convex_up_before;
	} catch(YAML_PM::KeyNotFound& e) {}
	try{
		node["convex_up_after"] >> p.convex_up_after;
	} catch(YAML_PM::KeyNotFound& e) {}
	try{
		node["convex_down_before"] >> p.convex_down_before;
	} catch(YAML_PM::KeyNotFound& e) {}
	try{
		node["convex_down_after"] >> p.convex_down_after;
	} catch(YAML_PM::KeyNotFound& e) {}
}
// emission
YAML_PM::Emitter& operator<< (YAML_PM::Emitter& out, const FlippersParams& p) {
	out << YAML_PM::BeginMap;
	out << YAML_PM::Key << "slope_threshold" << YAML_PM::Value << p.slope_threshold;
	out << YAML_PM::Key << "flat_threshold" << YAML_PM::Value << p.flat_threshold;
	out << YAML_PM::Key << "change_threshold" << YAML_PM::Value << p.change_threshold;
	out << YAML_PM::Key << "approach_up_before" << YAML_PM::Value << p.approach_up_before;
	out << YAML_PM::Key << "approach_up_after" << YAML_PM::Value << p.approach_up_after;
	out << YAML_PM::Key << "approach_down_before" << YAML_PM::Value << p.approach_down_before;
	out << YAML_PM::Key << "approach_down_after" << YAML_PM::Value << p.approach_down_after;
	out << YAML_PM::Key << "convex_up_before" << YAML_PM::Value << p.convex_up_before;
	out << YAML_PM::Key << "convex_up_after" << YAML_PM::Value << p.convex_up_after;
	out << YAML_PM::Key << "convex_down_before" << YAML_PM::Value << p.convex_down_before;
	out << YAML_PM::Key << "convex_down_after" << YAML_PM::Value << p.convex_down_after;
	out << YAML_PM::EndMap;
	return out;
}

// yaml-cpp interface for PathFollowingParams
// extraction
void operator>> (const YAML_PM::Node& node, PathFollowingParams& p) {
	try{
		node["v_max_flat"] >> p.v_max_flat;
	} catch(YAML_PM::KeyNotFound& e) {}
	try{
		node["w_max_flat"] >> p.w_max_flat;
	} catch(YAML_PM::KeyNotFound& e) {}
	try{
		node["v_max_slope"] >> p.v_max_slope;
	} catch(YAML_PM::KeyNotFound& e) {}
	try{
		node["w_max_slope"] >> p.w_max_slope;
	} catch(YAML_PM::KeyNotFound& e) {}
	try{
		node["max_distance"] >> p.max_distance;
	} catch(YAML_PM::KeyNotFound& e) {}
	try{
		node["ignore_radius"] >> p.ignore_radius;
	} catch(YAML_PM::KeyNotFound& e) {}
}
// emission
YAML_PM::Emitter& operator<< (YAML_PM::Emitter& out, const PathFollowingParams& p) {
	out << YAML_PM::BeginMap;
	out << YAML_PM::Key << "v_max_flat" << YAML_PM::Value << p.v_max_flat;
	out << YAML_PM::Key << "w_max_flat" << YAML_PM::Value << p.w_max_flat;
	out << YAML_PM::Key << "v_max_slope" << YAML_PM::Value << p.v_max_slope;
	out << YAML_PM::Key << "w_max_slope" << YAML_PM::Value << p.w_max_slope;
	out << YAML_PM::Key << "max_distance" << YAML_PM::Value << p.max_distance;
	out << YAML_PM::Key << "ignore_radius" << YAML_PM::Value << p.ignore_radius;
	out << YAML_PM::EndMap;
	return out;
}

// yaml-cpp interface for LocalPlannerParams
// extraction
void operator>> (const YAML_PM::Node& node, LocalPlannerParams& p) {
	try{
		node["flippers_params"] >> p.flippers_params;
	} catch(YAML_PM::KeyNotFound& e) {}
	try{
		node["path_following_params"] >> p.path_following_params;
	} catch(YAML_PM::KeyNotFound& e) {}
}
// emission
YAML_PM::Emitter& operator<< (YAML_PM::Emitter& out, const LocalPlannerParams& p) {
	out << YAML_PM::BeginMap;
	out << YAML_PM::Key << "flippers_params" << YAML_PM::Value << p.flippers_params;
	out << YAML_PM::Key << "path_following_params" << YAML_PM::Value << p.path_following_params;
	out << YAML_PM::EndMap;
	return out;
}

// yaml-cpp interface for TRPParams
// extraction
void operator>> (const YAML_PM::Node& node, TRPParams& p) {
	try{
		node["tensor_map"] >> p.tensor_map;
	} catch(YAML_PM::KeyNotFound& e) {}
	try{
		node["cost_functions"] >> p.cost_functions;
	} catch(YAML_PM::KeyNotFound& e) {}
	try{
		node["local_planner"] >> p.local_planner;
	} catch(YAML_PM::KeyNotFound& e) {}
}
// emission
YAML_PM::Emitter& operator<< (YAML_PM::Emitter& out, const TRPParams& p) {
	out << YAML_PM::BeginMap;
	out << YAML_PM::Key << "tensor_map" << YAML_PM::Value << p.tensor_map;
	out << YAML_PM::Key << "cost_functions" << YAML_PM::Value << p.cost_functions;
	out << YAML_PM::Key << "local_planner" << YAML_PM::Value << p.local_planner;
	out << YAML_PM::EndMap;
	return out;
}
