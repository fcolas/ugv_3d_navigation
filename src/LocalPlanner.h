#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

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
extern LocalPlannerParams* local_planner_params;

//! Single step in path
struct PathElement {
	//! 3D position in map coordinate frame
	Vector3f position;
	//! Normal (is it useful?)
	Vector3f normal;
	//! Angle to vertical
	float angle_to_vertical;
	//! Direction from previous node
	// has to be normalized
	Vector3f direction;
	//! Projected direction on the support plane
	Vector3f projected_direction;
	//! Change in inclination (sin of angle)
	float inclination_change;
	
	//! Empty constructor
	PathElement() {}
	//! Constructor to copy a TensorCell
	PathElement(const TensorCell& cell):
		position(cell.position),
		normal(cell.normal),
		angle_to_vertical(cell.angle_to_vertical) {}
};

//! Single step in path
struct DecoratedPathElement: PathElement {
	//! Flipper posture
	int posture;
	//! Edge between here and next
	bool edge;

	//! Empty constructor
	DecoratedPathElement() {}
	//! Constructor to copy undecorated path element
	DecoratedPathElement(const PathElement& pe):
		PathElement(pe),
		posture(1),
		edge(false)	{}
	//! Constructor to copy a TensorCell
	DecoratedPathElement(const TensorCell& cell):
		PathElement(cell),
		posture(1),
		edge(false)	{}
};

//! Path execution on the NIFTi robot
class LocalPlanner {
public:
	//! Decorated path
	vector<DecoratedPathElement> decorated_path;

	//! Constructor
	LocalPlanner();

	//! Destructor
	virtual ~LocalPlanner();

	//! Re-init at start of new path
	void reInit();

	//! Record current pose for the path
	void setCurrentPose(const geometry_msgs::Pose& pose,
			const TensorCell& cell);

	//! Build a path with posture and direction
	void decoratePath(vector<const TensorCell*>& path);

	//! Execute path
	bool getCommands(geometry_msgs::Twist& cmd_vel, int* posture);

	//! Serialize decorated path
	void serialize(const string& directory) const;
	//! Deserialize decorated path
	void deSerialize(const string& directory);

protected:
	//! past path (poses)
	vector<geometry_msgs::Pose> past_poses;

	//! past path (cells)
	vector<PathElement> past_cells;

	//! current index in decorated path
	unsigned int current_index;
};


#endif
