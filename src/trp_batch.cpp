// standard
#include <iostream>
#include <string>
#include <stdio.h>
#include <boost/filesystem.hpp>

// ROS 
#include <pointmatcher/PointMatcher.h>
#include <geometry_msgs/Pose.h>

// Various functions
#include "tensorUtils.h"
// TensorCell and TensorMap
#include "TensorMap.h"
// Search node and algorithm
#include "DStarLite.h"
// Path execution
#include "PathExecution.h"
// Parameters
#include "trp_params.h"
// Timer
#include "MyTimer.h"

using namespace std;

//! Motion planning and execution from point cloud using tensor voting
class TensorRePlannerBatch {
public:
	//! Constructor. ROS::init() is assumed to have been called before.
	TensorRePlannerBatch();

	//! Destructor (nothing done).
	virtual ~TensorRePlannerBatch();

	//! Do planning
	bool plan(const string& outdir);

	//! Load planning problem
	bool loadProblem(const string& directory);

	//! Reset planner to starting state (keeping problem but losing sparse and dense maps)
	void fullResetPlanner();

	//! Reset path planner
	void resetPathPlanner();

	//! Expand all search nodes from start
	void expand();

//protected:
	//! Flag to check that a problem has been loaded
	bool loaded;

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
};



// Constructor
TensorRePlannerBatch::TensorRePlannerBatch():
	loaded(false),
	tensor_map(),
	planner(tensor_map)
{
	// nothing to do?
}


// Destructor (nothing done)
TensorRePlannerBatch::~TensorRePlannerBatch() {
	// Nothing to do?
}


// Do planning (loading a problem from a directory and saving path
bool TensorRePlannerBatch::plan(const string& outdir) {
	if (!loaded) {
		cout << "Problem not loaded." << endl;
		return false;
	}

	auto new_goal = goal;
	cout << "\t\t";
	tensor_map.import(input_point_cloud);
	if (!planner.validateGoal(goal, new_goal)) {
		cout << "Goal is not valid." << endl;
		return false;
	}
	planner.setStart(start);

	cout << "Initialization of the planner:" << endl;

	dense_voting_timer.print("\t- Dense voting: ");
	dense_voting_timer.reset();
	cell_access_timer.print("\t- Cell access: ");
	cell_access_timer.reset();
	traversability_timer.print("\t- Traversability: ");
	traversability_timer.reset();

	MyTimer plan_timer;
	cout << "\t\t";
	plan_timer.start();
	bool success=planner.computeInitialPath();
	plan_timer.stop();
	plan_timer.print("\t- Planning: ");
	distance_filter_timer.print("\t- Distance filter: ");
	sparse_voting_timer.print("\t- Sparse voting: ");
	dense_voting_timer.print("\t- Dense voting: ");
	cell_access_timer.print("\t- Cell access: ");
	traversability_timer.print("\t- Traversability: ");
	search_loop_timer.print("\t- Search loop: ");

	if (success) {
		cout << "Planning successful." << endl;
		vector<const TensorCell*> path;
		bool got_path = planner.fillPath(path);
		if (!got_path) {
			//cout << "Couldn't get path after successful planning?!" << endl;
			return false;
		}
		path_execution.decoratePath(path);
		//planner.serialize(outdir);
		if (outdir!="") {
			path_execution.serialize(outdir+"/path.csv");
		}
		return true;
	} else {
		cout << "Couldn't find path." << endl;
		return false;
	}
}

void TensorRePlannerBatch::expand() {
	if (!loaded) {
		cout << "Problem not loaded." << endl;
		return;
	}
	tensor_map.import(input_point_cloud);
	MyTimer plan_timer;
	cout << "\t\t";
	plan_timer.start();
	planner.expandFrom(poseToVector(start));
	plan_timer.stop();
	plan_timer.print("\t- Planning: ");
	distance_filter_timer.print("\t- Distance filter: ");
	sparse_voting_timer.print("\t- Sparse voting: ");
	dense_voting_timer.print("\t- Dense voting: ");
	cell_access_timer.print("\t- Cell access: ");
	traversability_timer.print("\t- Traversability: ");
	search_loop_timer.print("\t- Search loop: ");
}

// Load planning problem
bool TensorRePlannerBatch::loadProblem(const string& directory) {
	deSerializePointCloud(input_point_cloud,
			directory+"/input_point_cloud.csv");
	float min_x, max_x, min_y, max_y, min_z, max_z;
	int nx, ny, nz;
	min_x = input_point_cloud.features.row(0).minCoeff();
	max_x = input_point_cloud.features.row(0).maxCoeff();
	min_y = input_point_cloud.features.row(1).minCoeff();
	max_y = input_point_cloud.features.row(1).maxCoeff();
	min_z = input_point_cloud.features.row(2).minCoeff();
	max_z = input_point_cloud.features.row(2).maxCoeff();
	nx = static_cast<int>(ceil((max_x-min_x)/0.09));
	ny = static_cast<int>(ceil((max_y-min_y)/0.09));
	nz = static_cast<int>(ceil((max_z-min_z)/0.09));
	cout << "Bounding box:" << endl;
	cout <<"("<<min_x<<", "<<min_y<<", "<<min_z<<") -> (";
	cout <<max_x<<", "<<max_y<<", "<<max_z<<"): ";
	cout <<nx<<"*"<<ny<<"*"<<nz<<" = "<<nx*ny*nz << endl;
	/*
	 * deserialization of start and goal poses in csv
	 * 
	 * format:
	 * x,y,z,qx,qy,qz,qw for start
	 * x,y,z,qx,qy,qz,qw for goal
	 */
	FILE* start_goal_file = fopen((directory+"/start_goal.csv").c_str(), "r");
	if (!start_goal_file) {
		return false;
	}
	float x, y, z, qx, qy, qz, qw;
	if (fscanf(start_goal_file, "%f,%f,%f,%f,%f,%f,%f\n",
			&x, &y, &z, &qx, &qy, &qz, &qw)!=7) {
		cout << "Couldn't load start and goal positions" << endl;
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
		cout << "Couldn't load start and goal positions" << endl;
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
	loaded = true;
	return true;
}


// Reset planner
void TensorRePlannerBatch::fullResetPlanner() {
	tensor_map = TensorMap();
	planner = DStarPathPlanner(tensor_map);
	path_execution = PathExecution();
	distance_filter_timer.reset();
	sparse_voting_timer.reset();
	dense_voting_timer.reset();
	traversability_timer.reset();
	search_loop_timer.reset();
	cell_access_timer.reset();
}

// Reset path planner
void TensorRePlannerBatch::resetPathPlanner() {
	planner = DStarPathPlanner(tensor_map);
	distance_filter_timer.reset();
	sparse_voting_timer.reset();
	dense_voting_timer.reset();
	traversability_timer.reset();
	search_loop_timer.reset();
	cell_access_timer.reset();
}


// other main
int main_lazy_dense_eval(int argc, char **argv) {
	TensorRePlannerBatch trpb;
	TRPParams params;
	cost_functions_params = &params.cost_functions;
	tensor_map_params = &params.tensor_map;
	path_execution_params = &params.path_execution;

	string directory;
	if (argc>=2) {
		directory = argv[1];
	} else {
		directory = "test";
	}
	if (argc>=3) {
		load_params(argv[2], params);
	}
	if (!trpb.loadProblem(directory)) {
		cout << "Fail to load problem" << endl;
	}
	
	cout << "Cold planning:" << endl;
	trpb.plan("");

	distance_filter_timer.reset();
	sparse_voting_timer.reset();
	dense_voting_timer.reset();
	traversability_timer.reset();
	search_loop_timer.reset();
	cell_access_timer.reset();

	cout << "Planning again:" << endl;
	trpb.plan("");
	return 0;
}

// other main
int main_saliency_params(int argc, char **argv) {
	TensorRePlannerBatch trpb;
	TRPParams params;
	cost_functions_params = &params.cost_functions;
	tensor_map_params = &params.tensor_map;
	path_execution_params = &params.path_execution;

	params.cost_functions.saliency_factor = 100;
	params.cost_functions.orientation_factor = 0;
	params.cost_functions.distance_factor = 1;
	params.cost_functions.heading_factor = 0;

	string directory, outdir;
	if (argc>=2) {
		directory = argv[1];
	} else {
		directory = "test";
	}
	if (argc>=3) {
		outdir = argv[2];
	} else {
		outdir = directory;
	}
	if (argc>=4) {
		load_params(argv[3], params);
	}
	if (!trpb.loadProblem(directory)) {
		cout << "Fail to load problem" << endl;
	}
	
	// saliency threshold tests
	for(float min_sal: {1., 2.,  5., 10., 25., 50., 100.})
	for(float sal_th: {1., 2., 5., 10., 25., 50., 100.}) 
	for(float max_sal: {50., 100., 200., 1000.}) {
		trpb.resetPathPlanner();
		params.tensor_map.traversability.min_saliency = min_sal;
		params.cost_functions.saliency_threshold = sal_th;
		params.cost_functions.maximal_saliency = max_sal;
		cout << "min_saliency="<<min_sal;
		cout << " saliency_threshold="<<sal_th;
		cout << " maximal_saliency="<<max_sal<<":"<<endl;
		char newdir[1025];
		sprintf(newdir, "%s_min%03.0f_th%03.0f_max%04.0f", outdir.c_str(),
				min_sal, sal_th, max_sal);
		boost::filesystem::path dir(newdir);
		if(!boost::filesystem::exists(dir)) {
			boost::filesystem::create_directories(dir);
		}
		save_params(string(newdir)+"/config.yaml", params);
		trpb.plan(newdir);
	}

	return 0;
}


// other main
int main_cost_weights(int argc, char **argv) {
	TensorRePlannerBatch trpb;
	TRPParams params;
	cost_functions_params = &params.cost_functions;
	tensor_map_params = &params.tensor_map;
	path_execution_params = &params.path_execution;

	string directory, outdir;
	if (argc>=2) {
		directory = argv[1];
	} else {
		directory = "test";
	}
	if (argc>=3) {
		outdir = argv[2];
	} else {
		outdir = directory;
	}
	if (argc>=4) {
		load_params(argv[3], params);
	}
	if (!trpb.loadProblem(directory)) {
		cout << "Fail to load problem" << endl;
	}
	
	float sal_f, ori_f, dist_f, heading_f;
	// validation @ 1
	sal_f = ori_f = dist_f = heading_f = 1;
	{
		trpb.resetPathPlanner();
		params.cost_functions.saliency_factor = sal_f;
		params.cost_functions.orientation_factor = ori_f;
		params.cost_functions.distance_factor = dist_f;
		params.cost_functions.heading_factor = heading_f;
		cout << "saliency_factor="<<sal_f;
		cout << " orientation_factor="<<ori_f;
		cout << " distance_factor="<<dist_f;
		cout << " heading_factor="<<heading_f<<":"<<endl;
		char newdir[1025];
		sprintf(newdir, "%s_s%04.1f_o%04.1f_d%04.1f_h%04.1f", outdir.c_str(),
				sal_f, ori_f, dist_f, heading_f);
		boost::filesystem::path dir(newdir);
		if(!boost::filesystem::exists(dir)) {
			boost::filesystem::create_directories(dir);
		}
		save_params(string(newdir)+"/config.yaml", params);
		trpb.plan(newdir);
	}
	// relative weight tests
	for(float sal_f: {0., 0.5,  1., 2., 5., 10., 50.})
	for(float ori_f: {0., 0.5, 1., 2., 5., 10., 50.})
	for(float dist_f: {1., 1.5, 2., 5., 10., 50.})
	for(float heading_f: {0., 0.5, 1., 2., 5., 10., 50.}) {
		trpb.resetPathPlanner();
		params.cost_functions.saliency_factor = sal_f;
		params.cost_functions.orientation_factor = ori_f;
		params.cost_functions.distance_factor = dist_f;
		params.cost_functions.heading_factor = heading_f;
		cout << "saliency_factor="<<sal_f;
		cout << " orientation_factor="<<ori_f;
		cout << " distance_factor="<<dist_f;
		cout << " heading_factor="<<heading_f<<":"<<endl;
		char newdir[1025];
		sprintf(newdir, "%s_s%04.1f_o%04.1f_d%04.1f_h%04.1f", outdir.c_str(),
				sal_f, ori_f, dist_f, heading_f);
		boost::filesystem::path dir(newdir);
		if(!boost::filesystem::exists(dir)) {
			boost::filesystem::create_directories(dir);
		}
		save_params(string(newdir)+"/config.yaml", params);
		trpb.plan(newdir);
	}

	return 0;
}


// other main
int main_cost_weights_figure(int argc, char **argv) {
	TensorRePlannerBatch trpb;
	TRPParams params;
	cost_functions_params = &params.cost_functions;
	tensor_map_params = &params.tensor_map;
	path_execution_params = &params.path_execution;

	string directory, outdir;
	if (argc>=2) {
		directory = argv[1];
	} else {
		directory = "test";
	}
	if (argc>=3) {
		outdir = argv[2];
	} else {
		outdir = directory;
	}
	if (argc>=4) {
		load_params(argv[3], params);
	}
	if (!trpb.loadProblem(directory)) {
		cout << "Fail to load problem" << endl;
	}
	
	for(float sal_f: {0., 1.})
	for(float ori_f: {0., 1.})
	for(float dist_f: {0., 1.})
	for(float heading_f: {0., 1.}) {
		if ((sal_f+ori_f+dist_f+heading_f)!=1.) {
			continue;
		}
		trpb.resetPathPlanner();
		params.cost_functions.saliency_factor = sal_f;
		params.cost_functions.orientation_factor = ori_f;
		params.cost_functions.distance_factor = dist_f;
		params.cost_functions.heading_factor = heading_f;
		cout << "saliency_factor="<<sal_f;
		cout << " orientation_factor="<<ori_f;
		cout << " distance_factor="<<dist_f;
		cout << " heading_factor="<<heading_f<<":"<<endl;
		char newdir[1025];
		sprintf(newdir, "%s_s%04.1f_o%04.1f_d%04.1f_h%04.1f", outdir.c_str(),
				sal_f, ori_f, dist_f, heading_f);
		boost::filesystem::path dir(newdir);
		if(!boost::filesystem::exists(dir)) {
			boost::filesystem::create_directories(dir);
		}
		save_params(string(newdir)+"/config.yaml", params);
		trpb.plan(newdir);
	}

	return 0;
}


/*
 * main
 */
int main_param_swipe(int argc, char **argv) {
	TensorRePlannerBatch trpb;
	TRPParams params;
	cost_functions_params = &params.cost_functions;
	tensor_map_params = &params.tensor_map;
	path_execution_params = &params.path_execution;
	string directory, outdir;
	if (argc>=2) {
		directory = argv[1];
	} else {
		directory = "test";
	}
	if (argc>=3) {
		outdir = argv[2];
	} else {
		outdir = directory;
	}
	if (argc>=4) {
		load_params(argv[3], params);
	}

	MyTimer loading_timer;
	loading_timer.start();
	bool ok=trpb.loadProblem(directory);
	loading_timer.stop();
	cout << "Loading: ";
	loading_timer.print();
	if (!ok) {
		cout << "Couldn't load problem: aborting." << endl;
		return 1;
	}

	for (float df_min_dist: {0.001, 0.01, 0.015, 0.03, 0.045, 0.06, 0.075, 0.09}) {
		params.tensor_map.distance_filter.min_distance = df_min_dist;
		for (float sigma: {0.05, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.75, 1.}) {
			params.tensor_map.sparse_voting.sigma = sigma;
			params.tensor_map.dense_voting.sigma = sigma;
			for (float sigma_factor: {2., 2.5, 3.}) {
				float max_dist = sigma_factor*sigma;
				params.tensor_map.sparse_voting.max_dist = max_dist;
				params.tensor_map.dense_voting.max_dist = max_dist;
				params.tensor_map.sparse_voting.max_knn = min(400, int(100*pow(max_dist/0.6, 1.5)*pow(0.05/df_min_dist, 1.5)));
				params.tensor_map.dense_voting.max_knn = min(20000, int(1000*pow(max_dist/0.6, 1.5)*pow(0.05/df_min_dist, 1.5)));
				params.tensor_map.traversability.max_points_in_bounding_box =
						min(500, params.tensor_map.sparse_voting.max_knn);
				params.tensor_map.traversability.min_support_points =
						params.tensor_map.traversability.max_points_in_bounding_box;
				for (float min_sal: {2, 5, 10, 25, 50}) {
					params.tensor_map.traversability.min_saliency = min_sal;
					params.cost_functions.saliency_threshold = 2*min_sal;
					params.cost_functions.maximal_saliency = 5*pow(min_sal, 2);

					trpb.fullResetPlanner();
					cout << "min_dist="<<df_min_dist;
					cout << " sigma="<<sigma;
					cout << " sigma_factor="<<sigma_factor;
					cout << " min_sal="<<min_sal<<":"<<endl;
					char newdir[1025];
					sprintf(newdir, "%s_df%.3f_s%.2f_sf%.1f_ms%.0f", outdir.c_str(),
							df_min_dist, sigma, sigma_factor, min_sal);
					boost::filesystem::path dir(newdir);
					if(!boost::filesystem::exists(dir)) {
						boost::filesystem::create_directories(dir);
					}
					save_params(string(newdir)+"/config.yaml", params);
					trpb.plan(newdir);
				}
			}
		}
	}
	return 0;
}

// other other main
int main_expand(int argc, char** argv) {
	TensorRePlannerBatch trpb;
	TRPParams params;
	cost_functions_params = &params.cost_functions;
	tensor_map_params = &params.tensor_map;
	path_execution_params = &params.path_execution;

	string directory, outdir;
	if (argc>=2) {
		directory = argv[1];
	} else {
		directory = "test";
	}
	cout << "Problem to be loaded: " << directory << endl;
	if (argc>=3) {
		outdir = argv[2];
	} else {
		outdir = directory;
	}
	cout << "Saving to: " << outdir << endl;
	if (argc>=4) {
		cout << "Loading parameters from: " << argv[3] << endl;
		load_params(argv[3], params);
	} else {
		cout << "Using default parameters." << endl;
	}
	if (!trpb.loadProblem(directory)) {
		cout << "Fail to load problem" << endl;
	}
	
	boost::filesystem::path dir(outdir);
	if(!boost::filesystem::exists(dir)) {
		boost::filesystem::create_directories(dir);
	}
	save_params(string(outdir)+"/config.yaml", params);
	
	cout << "Expansion:" << endl;
	trpb.expand();
	cout << "Serialization..." << endl;
	trpb.planner.serialize(outdir);
	cout << "Done." << endl;
	return 0;

}

int main_default_params(int argc, char **argv) {
	cout << "Creating and dumping default parameters" << endl;
	TRPParams params;
	string filename = "default.yaml";
	if (argc>=2) {
		filename = argv[1];
	}
	save_params(filename, params);
	return 0;
}



/*
 * main
 */
int main(int argc, char **argv) {
	//return main_default_params(argc, argv);
	//return main_param_swipe(argc, argv);
	//return main_lazy_dense_eval(argc, argv);
	//return main_cost_weights(argc, argv);
	//return main_cost_weights_figure(argc, argv);
	//return main_saliency_params(argc, argv);
	return main_expand(argc, argv);
}
