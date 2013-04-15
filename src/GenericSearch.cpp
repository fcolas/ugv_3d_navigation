#include "GenericSearch.h"
#include <cmath>

CostFunctionsParams* cost_functions_params;

/*
 * SearchNode
 */
// Destructor
SearchNode::~SearchNode() {
	// Nothing to do
}

// Compute intrinsic cost of the cell (called by constructor only)
float SearchNode::getCost() const {
	const float saliency_factor = cost_functions_params->saliency_factor; 
	const float orientation_factor = cost_functions_params->orientation_factor;
	const float sal_th = cost_functions_params->saliency_threshold;
	const float max_sal = cost_functions_params->maximal_saliency;
	const float max_orientation_angle = cost_functions_params->max_slope;
	
	//costStickSaliency
	float cost_stick_saliency;
	if (cell.stick_sal>=max_sal) {
		cost_stick_saliency = 0.;
	} else if (cell.stick_sal>=sal_th) {
		cost_stick_saliency = 1-((log(cell.stick_sal)-log(sal_th))/\
				(log(max_sal)-log(sal_th)));
	} else {
		cost_stick_saliency = 1 + tan(-M_PI_2*(1+cell.stick_sal/sal_th));
	}
	cost_stick_saliency *= saliency_factor;

	//costOrientation
	const float tmp = tan(cell.angle_to_vertical/max_orientation_angle*M_PI_2);
	const float cost_orientation = orientation_factor * tmp*tmp;
	
	return cost_stick_saliency + cost_orientation;
}


// Cost to another node
float SearchNode::getCostTo(const SearchNode& node) const {
	const float distance_factor = cost_functions_params->distance_factor;
	const float heading_factor = cost_functions_params->heading_factor;
	//costDistance
	const float cost_distance = distance_factor *
			(cell.position-node.cell.position).norm();

	//cost Heading against slope
	const Vector3f direction = (cell.position-node.cell.position).normalized();
	const Vector3f normal = cell.normal;
	const Vector3f y = normal.cross(Vector3f(0, 0, 1)); // sin(angle_to_vertical)
	const float cost_heading = heading_factor * fabs(y.dot(direction));
	//costChangeHeading TODO require previous
	return cost_distance + cost_heading;
}


/*
 * PathPlanner
 */
// Destructor
AbstractPathPlanner::~AbstractPathPlanner() {
	// Nothing to do
}


// Check, align and potentially set goal
bool AbstractPathPlanner::validateGoal(const geometry_msgs::Pose& goal,
		geometry_msgs::Pose& new_goal) {
	CellKey indices(dense_map->positionToIndex(poseToVector(goal)));
	new_goal = goal;
	float best_saliency = 0;
	geometry_msgs::Pose best_goal(goal);
	// checking several cells around
	for (int i=0; i<11; ++i) {
		// checking center -> 1 up -> 1 down -> 2 up -> 2 down...
		if (i&1) {
			indices.k += i;
			//cout << "+"<< (i+1)/2 <<": ";
		} else {
			//cout << "-"<< i/2 <<": ";
			indices.k -= i;
		}

		new_goal.position =
			vectorToPosition(dense_map->indexToPosition(indices));
		TensorCell& cell((*dense_map)[indices]);
		if (dense_map->isTraversable(new_goal, false)) {
			//cout << best_saliency << " " << cell.stick_sal << endl;
			if (cell.stick_sal>best_saliency) {
				best_saliency = cell.stick_sal;
				alignPose(new_goal, cell.normal, best_goal);
			}
		}
	}
	// commiting the best
	if (best_saliency>0) {
		new_goal = best_goal;
		setGoal(best_goal);
		return true;
	} else {
		new_goal = goal;
		return false;
	}
}

