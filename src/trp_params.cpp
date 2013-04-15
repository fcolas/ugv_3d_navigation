#include "trp_params.h"
#include <fstream>

using namespace std;

/*
 * Global interface
 */
// Load trp configuration from a yaml file
void load_params(const string& filename, TRPParams& p) {
	ifstream fin(filename);
	YAML::Parser parser(fin);
	YAML::Node doc;
	parser.GetNextDocument(doc);
	doc >> p;
	fin.close();
}

// Save trp configuration in a yaml file
void save_params(const string& filename, const TRPParams& p) {
	YAML::Emitter out;
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
void operator>> (const YAML::Node& node, DistanceFilterParams& p) {
	try{
		node["min_distance"] >> p.min_distance;
	} catch(YAML::KeyNotFound& e) {}
	try{
		node["max_knn"] >> p.max_knn;
	} catch(YAML::KeyNotFound& e) {}
}
// emission
YAML::Emitter& operator<< (YAML::Emitter& out, const DistanceFilterParams& p) {
	out << YAML::BeginMap;
	out << YAML::Key << "min_distance" << YAML::Value << p.min_distance;
	out << YAML::Key << "max_knn" << YAML::Value << p.max_knn;
	out << YAML::EndMap;
	return out;
}


// yaml-cpp interface for SparseVotingParams
// extraction
void operator>> (const YAML::Node& node, SparseVotingParams& p) {
	try{
		node["sigma"] >> p.sigma;
	} catch(YAML::KeyNotFound& e) {}
	try{
		node["max_dist"] >> p.max_dist;
	} catch(YAML::KeyNotFound& e) {}
	try{
		node["max_knn"] >> p.max_knn;
	} catch(YAML::KeyNotFound& e) {}
}
// emission
YAML::Emitter& operator<< (YAML::Emitter& out, const SparseVotingParams& p) {
	out << YAML::BeginMap;
	out << YAML::Key << "sigma" << YAML::Value << p.sigma;
	out << YAML::Key << "max_dist" << YAML::Value << p.max_dist;
	out << YAML::Key << "max_knn" << YAML::Value << p.max_knn;
	out << YAML::EndMap;
	return out;
}


// yaml-cpp interface for DenseVotingParams
// extraction
void operator>> (const YAML::Node& node, DenseVotingParams& p) {
	try{
		node["sigma"] >> p.sigma;
	} catch(YAML::KeyNotFound& e) {}
	try{
		node["max_dist"] >> p.max_dist;
	} catch(YAML::KeyNotFound& e) {}
	try{
		node["max_knn"] >> p.max_knn;
	} catch(YAML::KeyNotFound& e) {}
	try{
		node["obs_dir_offset"] >> p.obs_dir_offset;
	} catch(YAML::KeyNotFound& e) {}
	try{
		node["absolute_saliency_threshold"] >> p.absolute_saliency_threshold;
	} catch(YAML::KeyNotFound& e) {}
}
// emission
YAML::Emitter& operator<< (YAML::Emitter& out, const DenseVotingParams& p) {
	out << YAML::BeginMap;
	out << YAML::Key << "sigma" << YAML::Value << p.sigma;
	out << YAML::Key << "max_dist" << YAML::Value << p.max_dist;
	out << YAML::Key << "max_knn" << YAML::Value << p.max_knn;
	out << YAML::Key << "obs_dir_offset" << YAML::Value << p.obs_dir_offset;
	out << YAML::Key << "absolute_saliency_threshold" << YAML::Value << p.absolute_saliency_threshold;
	out << YAML::EndMap;
	return out;
}


// yaml-cpp interface for TraversabilityParams
// extraction
void operator>> (const YAML::Node& node, TraversabilityParams& p) {
	try{
		node["length"] >> p.length;
	} catch(YAML::KeyNotFound& e) {}
	try{
		node["width"] >> p.width;
	} catch(YAML::KeyNotFound& e) {}
	try{
		node["height"] >> p.height;
	} catch(YAML::KeyNotFound& e) {}
	try{
		node["diameter"] >> p.diameter;
	} catch(YAML::KeyNotFound& e) {}
	try{
		node["inflation"] >> p.inflation;
	} catch(YAML::KeyNotFound& e) {}
	try{
		node["ground_buffer"] >> p.ground_buffer;
	} catch(YAML::KeyNotFound& e) {}
	try{
		node["min_saliency"] >> p.min_saliency;
	} catch(YAML::KeyNotFound& e) {}
	try{
		node["max_slope"] >> p.max_slope;
	} catch(YAML::KeyNotFound& e) {}
	try{
		node["max_points_in_free_cell"] >> p.max_points_in_free_cell;
	} catch(YAML::KeyNotFound& e) {}
	try{
		node["min_free_cell_ratio"] >> p.min_free_cell_ratio;
	} catch(YAML::KeyNotFound& e) {}
	try{
		node["max_points_in_bounding_box"] >> p.max_points_in_bounding_box;
	} catch(YAML::KeyNotFound& e) {}
	try{
		node["min_support_points"] >> p.min_support_points;
	} catch(YAML::KeyNotFound& e) {}
}
// emission
YAML::Emitter& operator<< (YAML::Emitter& out, const TraversabilityParams& p) {
	out << YAML::BeginMap;
	out << YAML::Key << "length" << YAML::Value << p.length;
	out << YAML::Key << "width" << YAML::Value << p.width;
	out << YAML::Key << "height" << YAML::Value << p.height;
	out << YAML::Key << "diameter" << YAML::Value << p.diameter;
	out << YAML::Key << "inflation" << YAML::Value << p.inflation;
	out << YAML::Key << "ground_buffer" << YAML::Value << p.ground_buffer;
	out << YAML::Key << "min_saliency" << YAML::Value << p.min_saliency;
	out << YAML::Key << "max_slope" << YAML::Value << p.max_slope;
	out << YAML::Key << "max_points_in_free_cell" << YAML::Value << p.max_points_in_free_cell;
	out << YAML::Key << "min_free_cell_ratio" << YAML::Value << p.min_free_cell_ratio;
	out << YAML::Key << "max_points_in_bounding_box" << YAML::Value << p.max_points_in_bounding_box;
	out << YAML::Key << "min_support_points" << YAML::Value << p.min_support_points;
	out << YAML::EndMap;
	return out;
}


// yaml-cpp interface for TensorMapParams
// extraction
void operator>> (const YAML::Node& node, TensorMapParams& p) {
	try{
		node["cell_dimensions"][0] >> p.cell_dimensions(0);
		node["cell_dimensions"][1] >> p.cell_dimensions(1);
		node["cell_dimensions"][2] >> p.cell_dimensions(2);
	} catch(YAML::KeyNotFound& e) {}
	try{
		node["origin"][0] >> p.origin(0);
		node["origin"][1] >> p.origin(1);
		node["origin"][2] >> p.origin(2);
	} catch(YAML::KeyNotFound& e) {}
	try{
		node["distance_filter"] >> p.distance_filter;
	} catch(YAML::KeyNotFound& e) {}
	try{
		node["sparse_voting"] >> p.sparse_voting;
	} catch(YAML::KeyNotFound& e) {}
	try{
		node["dense_voting"] >> p.dense_voting;
	} catch(YAML::KeyNotFound& e) {}
	try{
		node["traversability"] >> p.traversability;
	} catch(YAML::KeyNotFound& e) {}
}
// emission
YAML::Emitter& operator<< (YAML::Emitter& out, const TensorMapParams& p) {
	out << YAML::BeginMap;
	out << YAML::Key << "cell_dimensions" << YAML::Value << YAML::BeginSeq;
	out << p.cell_dimensions[0] << p.cell_dimensions[1] << p.cell_dimensions[2] << YAML::EndSeq;
	out << YAML::Key << "origin" << YAML::Value << YAML::BeginSeq;
	out << p.origin[0] << p.origin[1] << p.origin[2] << YAML::EndSeq;
	out << YAML::Key << "distance_filter" << YAML::Value << p.distance_filter;
	out << YAML::Key << "sparse_voting" << YAML::Value << p.sparse_voting;
	out << YAML::Key << "dense_voting" << YAML::Value << p.dense_voting;
	out << YAML::Key << "traversability" << YAML::Value << p.traversability;
	out << YAML::EndMap;
	return out;
}


// yaml-cpp interface for CostFunctionsParams
// extraction
void operator>> (const YAML::Node& node, CostFunctionsParams& p) {
	try{
		node["saliency_factor"] >> p.saliency_factor;
	} catch(YAML::KeyNotFound& e) {}
	try{
		node["orientation_factor"] >> p.orientation_factor;
	} catch(YAML::KeyNotFound& e) {}
	try{
		node["distance_factor"] >> p.distance_factor;
	} catch(YAML::KeyNotFound& e) {}
	try{
		node["heading_factor"] >> p.heading_factor;
	} catch(YAML::KeyNotFound& e) {}
	try{
		node["max_slope"] >> p.max_slope;
	} catch(YAML::KeyNotFound& e) {}
	try{
		node["saliency_threshold"] >> p.saliency_threshold;
	} catch(YAML::KeyNotFound& e) {}
	try{
		node["maximal_saliency"] >> p.maximal_saliency;
	} catch(YAML::KeyNotFound& e) {}
}
// emission
YAML::Emitter& operator<< (YAML::Emitter& out, const CostFunctionsParams& p) {
	out << YAML::BeginMap;
	out << YAML::Key << "saliency_factor" << YAML::Value << p.saliency_factor;
	out << YAML::Key << "orientation_factor" << YAML::Value << p.orientation_factor;
	out << YAML::Key << "distance_factor" << YAML::Value << p.distance_factor;
	out << YAML::Key << "heading_factor" << YAML::Value << p.heading_factor;
	out << YAML::Key << "max_slope" << YAML::Value << p.max_slope;
	out << YAML::Key << "saliency_threshold" << YAML::Value << p.saliency_threshold;
	out << YAML::Key << "maximal_saliency" << YAML::Value << p.maximal_saliency;
	out << YAML::EndMap;
	return out;
}


// yaml-cpp interface for FlippersParams
// extraction
void operator>> (const YAML::Node& node, FlippersParams& p) {
	try{
		node["slope_threshold"] >> p.slope_threshold;
	} catch(YAML::KeyNotFound& e) {}
	try{
		node["flat_threshold"] >> p.flat_threshold;
	} catch(YAML::KeyNotFound& e) {}
	try{
		node["approach_up_before"] >> p.approach_up_before;
	} catch(YAML::KeyNotFound& e) {}
	try{
		node["approach_up_after"] >> p.approach_up_after;
	} catch(YAML::KeyNotFound& e) {}
	try{
		node["approach_down_before"] >> p.approach_down_before;
	} catch(YAML::KeyNotFound& e) {}
	try{
		node["approach_down_after"] >> p.approach_down_after;
	} catch(YAML::KeyNotFound& e) {}
	try{
		node["convex_up_before"] >> p.convex_up_before;
	} catch(YAML::KeyNotFound& e) {}
	try{
		node["convex_up_after"] >> p.convex_up_after;
	} catch(YAML::KeyNotFound& e) {}
	try{
		node["convex_down_before"] >> p.convex_down_before;
	} catch(YAML::KeyNotFound& e) {}
	try{
		node["convex_down_after"] >> p.convex_down_after;
	} catch(YAML::KeyNotFound& e) {}
}
// emission
YAML::Emitter& operator<< (YAML::Emitter& out, const FlippersParams& p) {
	out << YAML::BeginMap;
	out << YAML::Key << "slope_threshold" << YAML::Value << p.slope_threshold;
	out << YAML::Key << "flat_threshold" << YAML::Value << p.flat_threshold;
	out << YAML::Key << "approach_up_before" << YAML::Value << p.approach_up_before;
	out << YAML::Key << "approach_up_after" << YAML::Value << p.approach_up_after;
	out << YAML::Key << "approach_down_before" << YAML::Value << p.approach_down_before;
	out << YAML::Key << "approach_down_after" << YAML::Value << p.approach_down_after;
	out << YAML::Key << "convex_up_before" << YAML::Value << p.convex_up_before;
	out << YAML::Key << "convex_up_after" << YAML::Value << p.convex_up_after;
	out << YAML::Key << "convex_down_before" << YAML::Value << p.convex_down_before;
	out << YAML::Key << "convex_down_after" << YAML::Value << p.convex_down_after;
	out << YAML::EndMap;
	return out;
}

// yaml-cpp interface for ExecutionParams
// extraction
void operator>> (const YAML::Node& node, ExecutionParams& p) {
	try{
		node["v_max_flat"] >> p.v_max_flat;
	} catch(YAML::KeyNotFound& e) {}
	try{
		node["w_max_flat"] >> p.w_max_flat;
	} catch(YAML::KeyNotFound& e) {}
	try{
		node["v_max_slope"] >> p.v_max_slope;
	} catch(YAML::KeyNotFound& e) {}
	try{
		node["w_max_slope"] >> p.w_max_slope;
	} catch(YAML::KeyNotFound& e) {}
	try{
		node["max_distance"] >> p.max_distance;
	} catch(YAML::KeyNotFound& e) {}
	try{
		node["ignore_radius"] >> p.ignore_radius;
	} catch(YAML::KeyNotFound& e) {}
}
// emission
YAML::Emitter& operator<< (YAML::Emitter& out, const ExecutionParams& p) {
	out << YAML::BeginMap;
	out << YAML::Key << "v_max_flat" << YAML::Value << p.v_max_flat;
	out << YAML::Key << "w_max_flat" << YAML::Value << p.w_max_flat;
	out << YAML::Key << "v_max_slope" << YAML::Value << p.v_max_slope;
	out << YAML::Key << "w_max_slope" << YAML::Value << p.w_max_slope;
	out << YAML::Key << "max_distance" << YAML::Value << p.max_distance;
	out << YAML::Key << "ignore_radius" << YAML::Value << p.ignore_radius;
	out << YAML::EndMap;
	return out;
}

// yaml-cpp interface for PathExecutionParams
// extraction
void operator>> (const YAML::Node& node, PathExecutionParams& p) {
	try{
		node["flippers_params"] >> p.flippers_params;
	} catch(YAML::KeyNotFound& e) {}
	try{
		node["execution_params"] >> p.execution_params;
	} catch(YAML::KeyNotFound& e) {}
}
// emission
YAML::Emitter& operator<< (YAML::Emitter& out, const PathExecutionParams& p) {
	out << YAML::BeginMap;
	out << YAML::Key << "flippers_params" << YAML::Value << p.flippers_params;
	out << YAML::Key << "execution_params" << YAML::Value << p.execution_params;
	out << YAML::EndMap;
	return out;
}

// yaml-cpp interface for TRPParams
// extraction
void operator>> (const YAML::Node& node, TRPParams& p) {
	try{
		node["tensor_map"] >> p.tensor_map;
	} catch(YAML::KeyNotFound& e) {}
	try{
		node["cost_functions"] >> p.cost_functions;
	} catch(YAML::KeyNotFound& e) {}
	try{
		node["path_execution"] >> p.path_execution;
	} catch(YAML::KeyNotFound& e) {}
}
// emission
YAML::Emitter& operator<< (YAML::Emitter& out, const TRPParams& p) {
	out << YAML::BeginMap;
	out << YAML::Key << "tensor_map" << YAML::Value << p.tensor_map;
	out << YAML::Key << "cost_functions" << YAML::Value << p.cost_functions;
	out << YAML::Key << "path_execution" << YAML::Value << p.path_execution;
	out << YAML::EndMap;
	return out;
}
