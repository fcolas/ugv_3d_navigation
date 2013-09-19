#ifndef D_STAR_LITE_H
#define D_STAR_LITE

#include "GenericSearch.h"
#include "MyPriorityQueue.h"
#include "MyTimer.h"
#include <vector>
// for pairs
#include <utility>

using namespace std;

//! Timing
extern MyTimer search_loop_timer;

//! Specialized SearchNode for D* lite
struct DStarSearchNode: public SearchNode {
	//! Overloaded constructor
	DStarSearchNode(TensorCell& cell);

	//! Destructor
	virtual ~DStarSearchNode();

	//! Current cost to go to goal
	float g;
	//! Estimate of cost to goal based on 1st order neighbors
	float rhs;
	//! Heuristic estimate of cost to start
	float h;
	//! Key to sort SearchNodes
	pair<float, float> key;
	//! Fast check to know if in openlist
	bool is_open;
	//! Child node (parent in search but child in path)
	DStarSearchNode* child;
	//! Existing neighbors
	vector<DStarSearchNode*> neighbors;

	//! Compute key 
	void calcKey(float k_m=0);

	//! Priority comparison
	// Inverted as lower priority is higher key
	inline bool operator<(const DStarSearchNode& other) const {
		return key>other.key;
	}
};	


//! Comparison between nodes
struct DStarSNComp {
	// Inverted: lower priority is higher key
	inline bool operator()(const DStarSearchNode* lhs,
			const DStarSearchNode* rhs) const {
		return (*lhs) < (*rhs);
	}
};


//! D* lite path planning
class DStarPathPlanner: public PathPlanner<DStarSearchNode> {
public:
	//! Constructor
	DStarPathPlanner(TensorMap& dense_map):
		PathPlanner<DStarSearchNode>(dense_map),
		k_m(0)
	{}

	//! Update node
	void updateNode(DStarSearchNode& node);

	//! Update node knowing which neighbor changed
	void updateNodeFrom(DStarSearchNode& node, DStarSearchNode& child);

	//! Get the node at a given index
	// Specialized to update neighbor list
	DStarSearchNode& getNodeAt(const CellKey& key);
	DStarSearchNode& getNodeAt(const Vector3f& position)
			{return getNodeAt(dense_map->positionToIndex(position));}

	//! Compute the initial path
	bool computeInitialPath();

	//! Update the path after motion or a change in cell cost
	bool rePlan();

	//! Expand nodes from a given position
	void expandFrom(const Vector3f& position);

	//! Get the path after planning has been done
	bool fillPath(vector<const TensorCell*>& path);

	//! Set the start or update it if needed
	void setStart(const geometry_msgs::Pose& start);

	//! Set the goal
	void setGoal(const geometry_msgs::Pose& goal);

	//! Serialize all planner content (including map)
	void serialize(const string& directory) const;

	//! Deserialize all planner content (including map)
	void deSerialize(const string& directory);

	//! Full visualization
	void appendMarker(visualization_msgs::MarkerArray& array);

protected:
	//! Check open_list consistency
	bool check_open_list();

	//! Priority queue
	MyPriorityQueue<DStarSearchNode*, DStarSNComp> open_list;

	//! Heuristic
	float heuristic(const DStarSearchNode& node) const;

	//! k_m heuristic modifier
	float k_m;
};

#endif
