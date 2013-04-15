#ifndef PATH_EXECUTION_H
#define PATH_EXECUTION_H

#include "TensorMap.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Eigen>

// Parameters
#include "trp_params.h"

using namespace std;
using namespace Eigen;

//! Type for climbing state
enum ClimbingState {FLAT=0, UP, DOWN};

//! Parameters
extern PathExecutionParams* path_execution_params;

//! Single step in path
struct PathElement {
	//! 3D position in map coordinate frame
	Vector3f position;
	//! Normal (is it useful?)
	Vector3f normal;
	//! Angle to vertical
	float angle_to_vertical;
	//! Direction to next node
	Vector3f direction;
	//! Whether the robot is climbing up, or down
	ClimbingState climbing_state;

	//! Flipper posture
	int posture;
	//! Edge between here and next
	bool edge;
	
};

//! Path execution on the NIFTi robot
class PathExecution {
public:
	//! Decorated path
	vector<PathElement> decorated_path;

	//! Constructor
	PathExecution();

	//! Destructor
	virtual ~PathExecution();

	//! Re-init at start of execution
	void reInit(const geometry_msgs::Pose& robot_pose);

	//! Build a path with posture and direction
	void decoratePath(vector<const TensorCell*>& path);

	//! Execute path
	bool executePath(const geometry_msgs::Pose& pose,
			geometry_msgs::Twist& cmd_vel, int* posture);

	//! Serialize decorated path
	void serialize(const string& filename) const;
	//! Deserialize decorated path
	void deSerialize(const string& filename);
protected:
	//! There was an edge before
	bool after_edge;

	//! Position of last edge
	Vector3f last_edge_position;

	//! Cell position of the last edge
	Vector3f last_edge_cell;

	//! Posture of last edge
	int last_posture;

	//! Current climbing state
	ClimbingState cur_climbing_state;

	//! Current slope
	bool cur_slope;
};


#endif
