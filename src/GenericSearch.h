#ifndef GENERIC_SEARCH_H
#define GENERIC_SEARCH_H

#include "TensorMap.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <memory>
#include "tensorUtils.h"
#include <string>
#include "trp_params.h"
#include <cmath>

extern CostFunctionsParams* cost_functions_params;

//! Generic search node
struct SearchNode {

	//! Generic constructor
	SearchNode(TensorCell& cell):
		cell(cell) {
		cost = getCost();
	}

	//! Destructor
	virtual ~SearchNode();

	//! Cell with geometric information
	TensorCell& cell;

	//! Intrinsic cost of the cell (computed by constructor)
	float cost;

	//! Compute intrinsic cost of the cell (called by constructor only)
	float getCost() const;

	//! Cost to another node
	float getCostTo(const SearchNode& node) const;
};


//! Generic interface for path planner
class AbstractPathPlanner {
public:
	//! Constructor
	AbstractPathPlanner(TensorMap& dense_map):
		dense_map(&dense_map) {
	}

	//! Destructor
	virtual ~AbstractPathPlanner();

	//! Check, align and potentially set goal
	bool validateGoal(const geometry_msgs::Pose& goal, geometry_msgs::Pose& new_goal);

	//! Set the starting pose
	virtual void setStart(const geometry_msgs::Pose& start)=0;

	//! Set a valid goal (should have been validated before)
	virtual void setGoal(const geometry_msgs::Pose& goal)=0;

	//! Compute the initial path
	virtual bool computeInitialPath()=0;

	//! Update the path after motion or a change in cell cost
	virtual bool rePlan()=0;

	//! Get the path after planning has been done
	virtual bool fillPath(vector<const TensorCell*>& path)=0;

	//! Serialize all planner content (including map)
	virtual void serialize(const string& directory) const=0;

	//! Deserialize all planner content (including map)
	virtual void deSerialize(const string& directory)=0;

	//TODO interface for cost update
protected:
	//! Map of the environment
	TensorMap* dense_map;
};



//! Generic handling of search node
template<class SN>
class PathPlanner: public AbstractPathPlanner {
public:
	//! Constructor
	PathPlanner(TensorMap& dense_map):
		AbstractPathPlanner(dense_map),
		goal_node(NULL),
		start_node(NULL)
	{}

	//! Fast access to existing nodes
	typedef unordered_map<CellKey, SN, CellKeyHash> SearchNodeMap;

	//! Get the list of all neighbors of a given SearchNode
	// It creates SearchNode and their TensorCell if needed
	inline vector<SN*> getNeighborsFrom(const SN& node)
			{return getNeighborsFrom(node.cell.position);}
	inline vector<SN*> getNeighborsFrom(const Vector3f& position)
			{return getNeighborsFrom(dense_map->positionToIndex(position));}
	vector<SN*> getNeighborsFrom(const CellKey& key) {
		const int i = key.i;
		const int j = key.j;
		const int k = key.k;
		vector<SN*> list;
		for (int ii=i-1; ii<=i+1; ++ii) {
			for (int jj=j-1; jj<=j+1; ++jj) {
				for (int kk=k-1; kk<=k+1; ++kk) {
					if ((ii==i)&&(jj==j)&&(kk==k)) {
						continue;
					}
					list.push_back(&getNodeAt(CellKey(ii, jj, kk)));
				}
			}
		}
		// non maximal suppression
		//const SN* this_cell =  &getNodeAt(CellKey(i, j, k));
		vector<SN*> new_list;
		const auto cell_dims = tensor_map_params->cell_dimensions.array();
		for (auto node_ptr: list) {
			if ((node_ptr==start_node)||(node_ptr==goal_node)) {
				new_list.push_back(node_ptr);
			} else {
				const auto p = node_ptr->cell.position;
				const auto n = node_ptr->cell.normal;
				const auto sal = node_ptr->cell.stick_sal;
				// getting next cells along normal vector
				const auto n_dir = ((((3+sqrt(3))/4)*((n.array()/cell_dims).matrix().normalized())).array()*cell_dims).matrix();
				const SN* cell1 = &getNodeAt(p+n_dir);
				const SN* cell2 = &getNodeAt(p-n_dir);
				//assert((cell1!=node_ptr)&&(cell2!=node_ptr)&&(cell1!=cell2));
				// checking for local maximum (inside list or not)
				if (sal>cell1->cell.stick_sal) {
					if (sal>cell2->cell.stick_sal) {
						new_list.push_back(node_ptr);
					}/* else {
						if ((cell2!=this_cell)&&(find(list.begin(), list.end(), cell2)==list.end())) {
							new_list.push_back(node_ptr);
						}
					}
				} else {
					if ((cell1!=this_cell)&&(find(list.begin(), list.end(), cell1)==list.end())) {
						if (sal>cell2->cell.stick_sal) {
							new_list.push_back(node_ptr);
						} else {
							if ((cell2!=this_cell)&&(find(list.begin(), list.end(), cell2)==list.end())) {
								new_list.push_back(node_ptr);
							}
						}
					}*/
				}
			}
		}
		return new_list;
	}


	//! Get the list of all existing neighbors of a given SearchNode
	// It does not create SearchNode nor TensorCell
	inline vector<SN*> getExistingNeighborsFrom(const SN& node) 
			{return getExistingNeighborsFrom(node.cell.position);}
	inline vector<SN*> getExistingNeighborsFrom(const Vector3f& position) 
			{return getExistingNeighborsFrom(dense_map->positionToIndex(position));}
	vector<SN*> getExistingNeighborsFrom(const CellKey& key) {
		const int i = key.i;
		const int j = key.j;
		const int k = key.k;
		vector<SN*> list;
		for (int ii=i-1; ii<=i+1; ++ii) {
			for (int jj=j-1; jj<=j+1; ++jj) {
				for (int kk=k-1; kk<=k+1; ++kk) {
					if ((ii==i)&&(jj==j)&&(kk==k)) {
						continue;
					}
					try {
						list.push_back(&search_node_map.at(CellKey(ii, jj, kk)));
					} catch (out_of_range& oor) {
						// nothing, actually
					}
				}
			}
		}
		return list;
	}


	//! Get the node at a given index
	// It creates SearchNode and TensorCell if needed
	virtual SN& getNodeAt(const CellKey& key) {
		try {
			return search_node_map.at(key);
		} catch (out_of_range& oor) {
			search_node_map.insert({key, SN((*dense_map)[key])});
			return search_node_map.at(key);
		}
	}
	inline SN& getNodeAt(const Vector3f& position)
			{return getNodeAt(dense_map->positionToIndex(position));}


	//! Set a valid goal (should have been validated before)
	void setGoal(const geometry_msgs::Pose& goal) {
		goal_node = &getNodeAt(dense_map->positionToIndex(poseToVector(goal)));
	}
	

	//! Set the starting pose
	void setStart(const geometry_msgs::Pose& start) {
		start_node = &getNodeAt(dense_map->positionToIndex(poseToVector(start)));
		start_node->cell.traversability = OK;
	}

	//! Adds a marker to visualize
	virtual void appendMarker(visualization_msgs::MarkerArray& array) {
		visualization_msgs::Marker marker;
		fillMarker(marker);
		array.markers.push_back(marker);
	}
	//! Fill a marker message to visualize
	virtual void fillMarker(visualization_msgs::Marker& marker) {
		marker.header.frame_id = "/map";
		marker.header.stamp = ros::Time();
		marker.ns = "trp";
		marker.id = 0;
		marker.type = visualization_msgs::Marker::CUBE_LIST;
		marker.action = visualization_msgs::Marker::ADD;
		marker.scale.x = 0.6*tensor_map_params->cell_dimensions[0];
		marker.scale.y = 0.6*tensor_map_params->cell_dimensions[1];
		marker.scale.z = 0.6*tensor_map_params->cell_dimensions[2];
		marker.color.a = 0.3;
		marker.color.r = 0.0;
		marker.color.g = 0.7;
		marker.color.b = 0.7;	
  		for (const auto &it: search_node_map) {
			auto pos = dense_map->indexToPosition(it.first);
			geometry_msgs::Point point;
			point.x = pos[0];
			point.y = pos[1];
			point.z = pos[2];
			marker.points.push_back(point);
		}
	}

protected:
	//! Existing search nodes
	SearchNodeMap search_node_map;

	//! SearchNode corresponding to the goal
	SN* goal_node;

	//! SearchNode corresponding to the start
	SN* start_node;
};

#endif
