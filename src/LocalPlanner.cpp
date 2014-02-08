#include "LocalPlanner.h"
#include <algorithm>
#include "tensorUtils.h"
// conversion ROS<->Eigen
#include <eigen_conversions/eigen_msg.h>
#include <cmath>
#include <limits>
#include <boost/range/adaptor/reversed.hpp>

// Parameters
LocalPlannerParams* local_planner_params;

// Constructor
LocalPlanner::LocalPlanner::LocalPlanner() {
	// nothing to do	
}

// Destructor
LocalPlanner::~LocalPlanner() {
	// nothing to do
}

// Re-init at start of a new path
void LocalPlanner::reInit() {
	past_poses.clear();
	past_cells.clear();
}

//! Record current pose for the path
void LocalPlanner::setCurrentPose(const geometry_msgs::Pose& pose,
		const TensorCell& cell) {
	// Append current cell to past_cells
	if (past_cells.empty())	{
		const Vector3f& n = cell.normal;
		const Vector3f d = quaternionToDirection(pose.orientation);
		past_cells.push_back(PathElement());
		PathElement& path_element = past_cells.back();
		path_element.position = cell.position;
		path_element.normal = n;
		path_element.angle_to_vertical = cell.angle_to_vertical;
		path_element.direction = d;
		path_element.projected_direction = (n.cross(d)).cross(n);
		path_element.inclination_change = 0;
	} else {
		// check that the robot changed cell
		if (past_cells.back().position != cell.position) {
			// check that it is not looping
			// TODO evaluate that it's really needed
			int i=0;
			for (auto &path_cell: past_cells) {
				if (path_cell.position == cell.position) {
					// remove all elements starting from first loop
					past_cells.resize(i);
					break;
				}
				++i;
			}
			PathElement& previous = past_cells.back();
			const Vector3f& n = cell.normal;
			const Vector3f d =
					(cell.position - previous.position).normalized();
			const Vector3f p_d = (n.cross(d)).cross(n);
			past_cells.push_back(PathElement());
			PathElement& path_element = past_cells.back();
			path_element.position = cell.position;
			path_element.normal = n;
			path_element.angle_to_vertical = cell.angle_to_vertical;
			path_element.direction = d;
			path_element.projected_direction = p_d;
			path_element.inclination_change = n.dot(previous.projected_direction);
		}
	}
	// Append current pose to past_poses
	if (past_poses.empty()) {
		past_poses.push_back(pose);
	} else {
		// check that we've moved enough to need a new pose
		const geometry_msgs::Pose& past = past_poses.back();
		if ((linearDistance(past, pose)>0.01) ||
				(angularDistance(past, pose)>0.05)) {
			past_poses.push_back(pose);
		}
	}
}


// Build a path with posture and direction
void LocalPlanner::decoratePath(vector<const TensorCell*>& path) {

	// parameters
	FlippersParams fp = local_planner_params->flippers_params;
	const float slope_threshold = fp.slope_threshold;
	const float flat_threshold = fp.flat_threshold;
	const float sin_change_threshold = sin(fp.change_threshold);
	
	decorated_path.clear();
	// fill in past and current cells 
	cout << "[LP::decoratePath] starting to fill path with past cells." << endl;
	for (auto &cell: past_cells) {
		decorated_path.push_back(DecoratedPathElement(cell));
	}
	current_index = decorated_path.size();
	cout << "[LP::decoratePath] current_index:" << current_index << endl;
	// fill in future cells
	cout << "[LP::decoratePath] starting to fill path with current path." << endl;
	for (auto &cell: path) {
		// there is always a previous if setCurrentPose has been called
		Vector3f& prev_pos = decorated_path.back().position;
		Vector3f& prev_p_d = decorated_path.back().projected_direction;
		decorated_path.push_back(DecoratedPathElement(*cell));
		const Vector3f& n = cell->normal;
		const Vector3f d = (cell->position - prev_pos).normalized();
		const Vector3f p_d = (n.cross(d)).cross(n);
		decorated_path.back().direction = d;
		decorated_path.back().projected_direction = p_d;
		decorated_path.back().inclination_change = n.dot(prev_p_d);
	}
	cout << "[LP::decoratePath] Done filling path." << endl;
	/*
	 * Actual decoration
	 * First: static postures are defined wrt to the inclination of the surface
	 * in two passes
	 * Second: edges are defined (i) at the interface between static postures
	 * and (ii) significant changes in inclination
	 * Third: transition postures are set around edges
	 */
	DecoratedPathElement* previous = NULL;
	bool was_flat = false;
	// two passes for proper static postures
	cout << "[LP::decoratePath] First static pass." << endl;
	for (auto &current: decorated_path) {
		if (!previous) {
			previous = &current;
			continue;
		}
		// first pass for posture
		if ((current.angle_to_vertical<flat_threshold) || 
				(was_flat&&(current.angle_to_vertical<slope_threshold))) {
			current.posture = 1;
			was_flat = true;
		} else {
			current.posture = 0;
			was_flat = false;
		}
		previous = &current;
	}
	previous = NULL;
	cout << "[LP::decoratePath] Second static pass." << endl;
	for (auto &current: boost::adaptors::reverse(decorated_path)) {
		if (!previous) {
			previous = &current;
			continue;
		}
		if ((current.angle_to_vertical<flat_threshold) || 
				(was_flat&&(current.angle_to_vertical<slope_threshold))) {
			current.posture = 1;
			was_flat = true;
		} else {
			current.posture = 0;
			was_flat = false;
		}
		previous = &current;
	}
	// setting edge
	previous = NULL;
	cout << "[LP::decoratePath] Setting edge." << endl;
	for (auto &current: decorated_path) {
		if (!previous) {
			previous = &current;
			continue;
		}
		bool incl = (fabs(current.inclination_change)>=sin_change_threshold);
		bool posture = (current.posture!=previous->posture);
		current.edge = (incl||posture);
	}
	// decorate with transition posture
	const unsigned int path_length = decorated_path.size();
	cout << "[LP::decoratePath] Transition pass." << endl;
	for (unsigned int i=0; i<path_length; i++) {
		DecoratedPathElement* current = &(decorated_path[i]);
		if (current->edge) {
			int posture, before, after;
			// finding limits before and after
			if (current->inclination_change>0) { // approach
				posture = 2;
				if (current->projected_direction[2]>=0) { // going up
					before = fp.approach_up_before;
					after = fp.approach_up_after;
				} else { // going down
					before = fp.approach_down_before;
					after = fp.approach_down_after;
				}
			} else { // convex
				posture = 4;
				if (current->projected_direction[2]>=0) { // going up
					before = fp.convex_up_before;
					after = fp.convex_up_after;
				} else { // going down
					before = fp.convex_down_before;
					after = fp.convex_down_after;
				}
			}
			// applying posture
			int ii=i; // cast into int
			int pl = path_length; // cast into int
			int start = max(0, ii-before);
			int end = min(pl, ii+after+1);
			// fill transition
			for (int j=start; j<end; ++j) {
				decorated_path[j].posture = posture;
			}
		}
	}
	cout << "[LP::decoratePath] Decoration done." << endl;
}


// Execute path
bool LocalPlanner::getCommands(geometry_msgs::Twist& cmd_vel, int* posture) {
	PathFollowingParams pf = local_planner_params->path_following_params;
	const float v_max_easy = pf.v_max_flat;
	const float w_max_easy = pf.w_max_flat;
	const float v_max_diff = pf.v_max_slope;
	const float w_max_diff = pf.w_max_slope;
	const float dist_threshold = pf.max_distance;
	const float min_distance = pf.ignore_radius;

	const unsigned int path_length = decorated_path.size();
	if (current_index == path_length) {
		// we're there enough
		return true;
	}

	// posture from decoration
	*posture = decorated_path[current_index].posture;

	// velocity command
	cmd_vel.linear.y = 0;
	cmd_vel.linear.z = 0;
	cmd_vel.angular.x = 0;
	cmd_vel.angular.y = 0;
	float v_max, w_max;
	if (*posture!=1) {
		v_max = v_max_diff;
		w_max = w_max_diff;
	} else {
		v_max = v_max_easy;
		w_max = w_max_easy;
	}
	// transformation to get relative coordinates
	const geometry_msgs::Pose pose = past_poses.back();
	Affine3d robot_to_map;
	tf::poseMsgToEigen(pose, robot_to_map);
	Affine3f map_to_robot = robot_to_map.inverse().cast<float>();
	// select look-ahead positions
	// ignore goals that are too close
	int id_min = current_index;
	for(;id_min<static_cast<int>(path_length)-1; ++id_min) {
		Vector3f rel_pos = map_to_robot*decorated_path[id_min].position;
		if ((pow(rel_pos(0), 2)+pow(rel_pos(1), 2))>=pow(min_distance, 2)) {
			break;
		}
	}
	//const int id_min = min(1, path_length-1);
	const int id_max = min(current_index+10, path_length-1); // looking up to ~1m FIXME param
	if (id_min>id_max) {  //shouldn't happen
		id_min = id_max;
	}
	const int nb_max = id_max-id_min+1;
	// Computing local coordinates
	MatrixXf local_points(3, nb_max);
	int i;
	for (i=0; i<nb_max; ++i) {
		local_points.col(i) = map_to_robot*decorated_path[i+id_min].position;
	}
	// Finding longest suitable trajectory
	float R, res, max_res;
	for (i=nb_max; i>0;--i) {
		local_points.conservativeResize(3, i);
		getLastPointRadius(local_points, &R, &res, &max_res);
		if (max_res<pow(dist_threshold, 2)) {
			break;
		}
	}
	// Computing commands
	if (R == numeric_limits<float>::infinity()) {
		cmd_vel.linear.x = v_max;
		cmd_vel.angular.z = 0;
	} else if (fabs(R)<(v_max/w_max)) {
		cmd_vel.angular.z = (R>=0)?w_max:-w_max;
		cmd_vel.linear.x = fabs(R*w_max);
	} else {
		cmd_vel.linear.x = v_max;
		cmd_vel.angular.z = v_max/R;
	}
	return false;	
}

// Serialize decorated path
void LocalPlanner::serialize(const string& directory) const {
	/*
	 * serialization of path in csv
	 * 
	 * format:
	 * x,y,z,nx,ny,nz,angle,dx,dy,dz,pdx,pdy,pdz,incl,posture,edge
	 */
	FILE* path_file = fopen((directory+"/decorated_path.csv").c_str(), "w");
	// should check error FIXME
	for (auto &it: decorated_path) { // C++11 loop
		fprintf(path_file, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d\n",
				it.position(0), it.position(1), it.position(2),
				it.normal(0), it.normal(1), it.normal(2),
				it.angle_to_vertical,
				it.direction(0), it.direction(1), it.direction(2),
				it.projected_direction(0), it.projected_direction(1),
				it.projected_direction(2), it.inclination_change,
				it.posture, static_cast<int>(it.edge));
	}
	fclose(path_file);
	/*
	 * serialization of context in csv
	 *
	 * format:
	 * current_index
	 *
	 * px,py,pz,qx,qy,qz,qw
	 *
	 * x,y,z,nx,ny,nz,angle,dx,dy,dz,pdx,pdy,pdz,incl
	 */
	FILE* lp_file = fopen((directory+"/local_planner.csv").c_str(), "w");
	// should check error FIXME
	fprintf(lp_file, "%d\n\n", current_index);
	for (auto &it: past_poses) { // C++11 loop
		fprintf(lp_file, "%f,%f,%f,%f,%f,%f,%f\n",
				it.position.x, it.position.y, it.position.z,
				it.orientation.x, it.orientation.y, it.orientation.z,
				it.orientation.w);
	}
	fprintf(lp_file, "\n");
	for (auto &it: past_cells) { // C++11 loop
		fprintf(lp_file, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
				it.position(0), it.position(1), it.position(2),
				it.normal(0), it.normal(1), it.normal(2),
				it.angle_to_vertical,
				it.direction(0), it.direction(1), it.direction(2),
				it.projected_direction(0), it.projected_direction(1),
				it.projected_direction(2), it.inclination_change);
	}
	fclose(lp_file);
}

// Deserialize decorated path
void LocalPlanner::deSerialize(const string& directory) {
	decorated_path.clear();
	past_poses.clear();
	past_cells.clear();
	/*
	 * deserialization of path in csv
	 * 
	 * format:
	 * x,y,z,nx,ny,nz,angle,dx,dy,dz,pdx,pdy,pdz,incl,posture,edge
	 */
	FILE* path_file = fopen((directory+"/decorated_path.csv").c_str(), "r");
	// should check error FIXME
	float x, y, z, nx, ny, nz, angle, dx, dy, dz, pdx, pdy, pdz, incl;
	int posture, edge;
	int n;
	int line = 1;
	while (!feof(path_file)) {
		n = fscanf(path_file,
				"%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d\n",
				&x, &y, &z, &nx, &ny, &nz, &angle, &dx, &dy, &dz, &pdx, &pdy,
				&pdz, &incl, &posture, &edge);
		if (n!=16) {
			cout << "Error reading line "<<line<<" in "<<(directory+"/decorated_path.csv")<<
				": only "<<n<<" tokens found."<<endl;
		} else {
			decorated_path.push_back(DecoratedPathElement());
			DecoratedPathElement& decorated_path_element = decorated_path.back();
			decorated_path_element.position = Vector3f(x, y, z);
			decorated_path_element.normal = Vector3f(nx, ny, nz);
			decorated_path_element.angle_to_vertical = angle;
			decorated_path_element.direction = Vector3f(dx, dy, dz);
			decorated_path_element.projected_direction = Vector3f(pdx, pdy, pdz);
			decorated_path_element.inclination_change = incl;
			decorated_path_element.posture = posture;
			decorated_path_element.edge = static_cast<bool>(edge);
		}
	}
	fclose(path_file);
	/*
	 * deserialization of context in csv
	 *
	 * format:
	 * current_index
	 *
	 * px,py,pz,qx,qy,qz,qw
	 *
	 * x,y,z,nx,ny,nz,angle,dx,dy,dz,pdx,pdy,pdz,incl
	 */
	FILE* lp_file = fopen((directory+"/local_planner.csv").c_str(), "r");
	// should check error FIXME
	float px, py, pz, qx, qy, qz, qw;
	n = fscanf(lp_file, "%d\n\n", &current_index);
	if (n!=1) {
		cout << "Error reading line "<<line<<" in "<<directory+"/local_planner.csv"<<
			": only "<<n<<" tokens found."<<endl;
	}
	while (n) {
		n = fscanf(lp_file, "%f,%f,%f,%f,%f,%f,%f\n",
				&px, &py, &pz, &qx, &qy, &qz, &qw);
		if (n&&(n!=7)) {
			cout << "Error reading line "<<line<<" in "<<directory+"/local_planner.csv"<<
				": only "<<n<<" tokens found."<<endl;
		} else {
			past_poses.push_back(geometry_msgs::Pose());
			geometry_msgs::Pose& pose = past_poses.back();
			pose.position.x = px;
			pose.position.y = py;
			pose.position.z = pz;
			pose.orientation.x = qx;
			pose.orientation.y = qy;
			pose.orientation.z = qz;
			pose.orientation.w = qw;
		}
	}
	// TODO check if I need to read a line there of if it has been consumed
	// before
	while (!feof(lp_file)) {
		n = fscanf(lp_file,
				"%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
				&x, &y, &z, &nx, &ny, &nz, &angle, &dx, &dy, &dz, &pdx, &pdy,
				&pdz, &incl);
		if (n!=14) {
			cout << "Error reading line "<<line<<" in "<<directory+"/local_planner.csv"<<
				": only "<<n<<" tokens found."<<endl;
		} else {
			past_cells.push_back(PathElement());
			PathElement& path_element = past_cells.back();
			path_element.position = Vector3f(x, y, z);
			path_element.normal = Vector3f(nx, ny, nz);
			path_element.angle_to_vertical = angle;
			path_element.direction = Vector3f(dx, dy, dz);
			path_element.projected_direction = Vector3f(pdx, pdy, pdz);
			path_element.inclination_change = incl;
		}
	}
	fclose(lp_file);
}

