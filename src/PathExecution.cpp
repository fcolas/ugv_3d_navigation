#include "PathExecution.h"
#include <algorithm>
#include "tensorUtils.h"
// conversion ROS<->Eigen
#include <eigen_conversions/eigen_msg.h>
#include <cmath>
#include <limits>

// Parameters
PathExecutionParams* path_execution_params;

// Constructor
PathExecution::PathExecution():
	after_edge(false),
	cur_climbing_state(FLAT),
	cur_slope(false)
{
	// nothing to do
}


// Destructor
PathExecution::~PathExecution() {
	// nothing to do
}


//! Re-init at start of execution
void PathExecution::reInit(const geometry_msgs::Pose& robot_pose) {
	const float slope_threshold = 20*M_PI/180.;
	after_edge = false;
	Vector3f direction = quaternionToDirection(robot_pose.orientation);
	if (fabs(direction(2))>sin(slope_threshold)) {
		cur_slope = true;
		cur_climbing_state = (direction(2)>0)?UP:DOWN;
	} else {
		cur_slope = false;
		cur_climbing_state = FLAT;
	}
}


// Build a path with posture and direction
void PathExecution::decoratePath(vector<const TensorCell*>& path) {
	FlippersParams fp = path_execution_params->flippers_params;
	const float slope_threshold = fp.slope_threshold;
	const float flat_threshold = fp.flat_threshold;
	const unsigned int path_length = path.size();
	decorated_path.clear();
	decorated_path.reserve(path_length);
	PathElement* previous = NULL;
	// first pass for initial fill
	// initialize from memory
	bool was_slope = cur_slope;
	ClimbingState last_climbing_state = cur_climbing_state;

	for (auto &cell: path) {
		decorated_path.push_back(PathElement());
		PathElement& path_element = decorated_path.back();
		path_element.position = cell->position;
		path_element.normal = cell->normal;
		path_element.angle_to_vertical = cell->angle_to_vertical;
		path_element.climbing_state = FLAT; // initialization
		path_element.edge = false; // initialization
		if (previous) {
			path_element.direction = path_element.position - previous->position;
		}
		// posture
		if ((cell->angle_to_vertical>slope_threshold)||
				((cell->angle_to_vertical>flat_threshold)&&was_slope)) {
			path_element.posture = 0; // flat FIXME
			if ((was_slope)&&(last_climbing_state!=FLAT)) {
				// get climbing state from before
				path_element.climbing_state = last_climbing_state;
			} else if ((cell->angle_to_vertical>slope_threshold)&&previous) {
				// enough slope to get it accurately + I know where I come from
				Vector3f y = path_element.normal.cross(path_element.direction);
				Vector3f x = y.cross(path_element.normal);
				path_element.climbing_state = (x(2)>0)?UP:DOWN;
				// XXX debug
				cout << path_element.direction.transpose() << " ->((" << x.transpose() << "); (" <<
						y.transpose() << "); (" << path_element.normal.transpose() << ")):" <<
						path_element.climbing_state << endl;
				//if (path_element.climbing_state==DOWN) {
				//	serialize("tmptmppath.csv");
				//	assert(false);
				//}
				//assert(path_element.climbing_state!=DOWN);
			}
			// it might be that at first cell, last_climbing_state==cur_climbing_state==FLAT,previous==NULL,angle_to_vertical>slope_threshold
			was_slope = true;
		} else {
			path_element.posture = 1; // drive
			was_slope = false;
		}

		last_climbing_state = path_element.climbing_state;
		previous = &path_element;
	}
	const int init_posture = decorated_path[0].posture;
	//const ClimbingState init_climbing_state = decorated_path[0].climbing_state;
	// extend flat posture to the other direction
	for (unsigned int i=path_length; i>0; --i) {
		PathElement* pe = &(decorated_path[i-1]);
		if ((pe->angle_to_vertical>slope_threshold)||
				((pe->angle_to_vertical>flat_threshold)&&was_slope)) {
			pe->posture = 0;
			if (was_slope&&(i<path_length)) {
				pe->climbing_state = decorated_path[i].climbing_state;
			} // else don't change climbing_state
			was_slope = true;
		} else {
			was_slope = false;
		}
	}
	// FIXME deciding on edge can be done after and by comparison with
	// last_climbing_state
	if ((init_posture!=decorated_path[0].posture)||(cur_climbing_state!=decorated_path[0].climbing_state)||
			(after_edge&&(last_edge_cell == decorated_path[0].position))) {
		decorated_path[0].edge = true;
	}
	// resetting edge correctly
	for (unsigned int i=1; i<path_length; ++i) {
		PathElement* pe = &(decorated_path[i]);
		pe->edge = (pe->climbing_state!=decorated_path[i-1].climbing_state);
	}

	// setting transition states
	for (unsigned int i=0; i<path_length; ++i) {
		if (decorated_path[i].edge) {
			// find orientation of the edge
			ClimbingState prev_cs;
			ClimbingState cur_cs = decorated_path[i].climbing_state;
			if (i) {
				prev_cs = decorated_path[i-1].climbing_state;
			} else {
				prev_cs = cur_climbing_state;
			}
			int posture;
			int before;
			int after;
			// refactor the following tests
			if ((!i)&&(after_edge&&(last_edge_cell == decorated_path[0].position))) {
				posture = last_posture;
				if (posture==2) {
					if (cur_cs==UP) {
						before = fp.approach_up_before;
						after = fp.approach_up_after;
					} else {
						before = fp.approach_down_before;
						after = fp.approach_down_after;
					}
				} else {
					if (prev_cs==DOWN) {
						before = fp.convex_down_before;
						after = fp.convex_down_after;
					} else {
						before = fp.convex_up_before;
						after = fp.convex_up_after;
					}
				}
			} else {		
				if (cur_cs==UP) { // approach upwards
					posture = 2;
					before = fp.approach_up_before;
					after = fp.approach_up_after;
				} else if (prev_cs==DOWN) { // approach downwards
					posture = 2;
					before = fp.approach_down_before;
					after = fp.approach_down_after;
				} else if (prev_cs==UP) { // convex upwards
					posture = 4;
					before = fp.convex_up_before;
					after = fp.convex_up_after;
				} else { // convex downwards
					posture = 4;
					before = fp.convex_down_before;
					after = fp.convex_down_after;
				}
			}
			if ((after+before)<0) {
				cout << "Warning empty range for posture " << posture << endl;
				continue;
			}
			int ii=i;
			int pl = path_length;
			int start = max(0, ii-before);
			int end = min(pl, ii+after+1);
			// fill if start is shifted after
			for (int j=ii; j<start; ++j) {
				if (prev_cs!=FLAT) {
					decorated_path[j].posture = 0;
				} else {
					decorated_path[j].posture = 1;
				}
			}
			// fill if start is shifted before
			for (int j=end; j<ii; ++j) {
				if (cur_cs!=FLAT) {
					decorated_path[j].posture = 0;
				} else {
					decorated_path[j].posture = 1;
				}
			}
			// fill transition
			for (int j=start; j<end; ++j) {
				decorated_path[j].posture = posture;
			}
		}
	}
}

// Execute path
// Just computes command to reach waypoint following the closest one
bool PathExecution::executePath(const geometry_msgs::Pose& pose,
		geometry_msgs::Twist& cmd_vel, int* posture) {
	// gather parameters somewhere FIXME
	ExecutionParams ep = path_execution_params->execution_params;
	FlippersParams fp = path_execution_params->flippers_params;
	const float v_max_easy = ep.v_max_flat;
	const float w_max_easy = ep.w_max_flat;
	const float v_max_diff = ep.v_max_slope;
	const float w_max_diff = ep.w_max_slope;
	const float dist_threshold = ep.max_distance;
	const float min_distance = ep.ignore_radius;

	const int path_length = decorated_path.size();

	// pose
	const Vector3f position = poseToVector(pose);
	//const Vector3f direction = quaternionToDirection(pose.orientation);

	if ((!path_length)||\
			((decorated_path[path_length-1].position-position).norm() <=\
			 	dist_threshold)) {
		// should say something
		// but in any case, we've reached the "end"
		return true;
	}
	
	// projection into robot reference frame
	Affine3d robot_to_map;
	tf::poseMsgToEigen(pose, robot_to_map);
	Affine3f map_to_robot = robot_to_map.inverse().cast<float>();

	// storing edge for later and setting posture
	*posture = decorated_path[0].posture;
	if (decorated_path[0].edge) {
		last_edge_cell = decorated_path[0].position;
		last_edge_position = position;
		last_posture = *posture;
		after_edge = true;
	} else if (after_edge) {
		// FIXME the ending condition depends on posture: do it in a better way
		int nb;
		if (last_posture == 2) {
			if (cur_climbing_state==DOWN) {
				nb = fp.approach_down_after;
			} else {
				nb = fp.approach_up_after;
			}
		} else if (last_posture == 4) {
			if (cur_climbing_state==UP) {
				nb = fp.convex_up_after;
			} else {
				nb = fp.convex_down_after;
			}
		} else {
			assert(false);
		}
		// TODO use parameter for cell size
		if ((position - last_edge_position).array().abs().maxCoeff()<nb*0.11) {
			*posture = last_posture;
		} else {
			after_edge = false;
		}
	}
	// storing climbing state
	cur_climbing_state = decorated_path[0].climbing_state;
	cur_slope = (cur_climbing_state!=FLAT);
	// XXX debug
	cout << "CS: " << cur_climbing_state << "; slope: " << cur_slope << "; last_posture: "<<
			last_posture << "; after edge: " << after_edge << "; last_edge_p: " <<  last_edge_position.transpose() << "; position: " << position.transpose() << "; (last_cell: " <<last_edge_cell.transpose() << "; " << decorated_path[0].position.transpose() << ")"<<endl;

	// getting velocity command
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

	int id_min;
	for(id_min=0;id_min<path_length-1; ++id_min) {
		Vector3f rel_pos = map_to_robot*decorated_path[id_min].position;
		if ((pow(rel_pos(0), 2)+pow(rel_pos(1), 2))>=pow(min_distance, 2)) {
			break;
		}
	}
	//const int id_min = min(1, path_length-1);
	const int id_max = min(10, path_length-1); // looking up to ~1m FIXME param
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
void PathExecution::serialize(const string& filename) const {
	/*
	 * serialization of path in csv
	 * 
	 * format:
	 * x,y,z,nx,ny,nz,dx,dy,dz,posture,edge
	 */
	FILE* path_file = fopen(filename.c_str(), "w");
	// should check error FIXME
	for (auto &it: decorated_path) { // C++11 loop
		fprintf(path_file, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d\n",
				it.position(0), it.position(1), it.position(2),
				it.normal(0), it.normal(1), it.normal(2),
				it.direction(0), it.direction(1), it.direction(2),
				it.posture, static_cast<int>(it.edge));
	}
	fclose(path_file);
}


//! Deserialize decorated path
void PathExecution::deSerialize(const string& filename) {
	decorated_path.clear();
	/*
	 * deserialization of path in csv
	 * 
	 * format:
	 * i,j,k,x,y,z,stick_sal,nx,ny,nz
	 */
	FILE* path_file = fopen(filename.c_str(), "r");
	// should check error FIXME
	float x, y, z, nx, ny, nz, dx, dy, dz;
	int posture, edge;
	int n;
	int line = 1;
	while (!feof(path_file)) {
		n = fscanf(path_file, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d\n",
				&x, &y, &z, &nx, &ny, &nz, &dx, &dy, &dz, &posture, &edge);
		if (n!=11) {
			cout << "Error reading line "<<line<<" in "<<filename<<
				": only "<<n<<" tokens found."<<endl;
		} else {
			decorated_path.push_back(PathElement());
			PathElement& path_element = decorated_path.back();
			path_element.position = Vector3f(x, y, z);
			path_element.normal = Vector3f(nx, ny, nz);
			path_element.direction = Vector3f(dx, dy, dz);
			path_element.posture = posture;
			path_element.edge = static_cast<bool>(edge);
		}
	}
	fclose(path_file);
}

