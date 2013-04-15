#include "DStarLite.h"
// for min
#include <algorithm>
#include <limits>
// file I/O using formatted data
#include <cstdio>

#define infinity (numeric_limits<float>::infinity())

// Timing
MyTimer search_loop_timer;


/*
 * Specialized SearchNode for D* lite
 */
// Overloaded constructor
DStarSearchNode::DStarSearchNode(TensorCell& cell):
	SearchNode(cell),
	g(infinity),
	rhs(infinity),
	h(infinity),
	key(infinity, infinity),
	is_open(false),
	child(NULL)
{
}


// Destructor
DStarSearchNode::~DStarSearchNode()
{}


// Compute key
void DStarSearchNode::calcKey(float k_m) {
	key = {min(g, rhs)+h+k_m, min(g, rhs)};
}



/*
 * D* lite path planning
 */
// Heuristic
float DStarPathPlanner::heuristic(const DStarSearchNode& node) const {
	return (node.cell.position-start_node->cell.position).norm();
}


// Get the node at a given index
// Specialized to update neighbor list
DStarSearchNode& DStarPathPlanner::getNodeAt(const CellKey& key) {
	try {
		return search_node_map.at(key);
	} catch (out_of_range& oor) {
		auto res = search_node_map.insert({key,
						DStarSearchNode((*dense_map)[key])});
		DStarSearchNode* node = &(res.first->second);
		// setting heuristic value
		if (start_node) {
			node->h = heuristic(*node);
		} else {
			node->h = 0;	// no heuristic for expansion
		}
		// fast neighbor access for updateNode function
		for (auto &it: getExistingNeighborsFrom(*node)) {
			node->neighbors.push_back(it);
			it->neighbors.push_back(node);
		}
		return *node;
	}
}


// Update node
void DStarPathPlanner::updateNode(DStarSearchNode& node) {
	if ((&node)!=goal_node) {
		float min_rhs = infinity;
		DStarSearchNode* best_child=NULL;
		float new_rhs;
		int nb_valid = 0;
		for (auto &it: node.neighbors) {
			if (dense_map->isTraversableFrom(it->cell.position,
					node.cell.position)) {
				nb_valid += 1;
				new_rhs = it->g + node.cost + node.getCostTo(*it);
				if (new_rhs<min_rhs) {
					min_rhs = new_rhs;
					best_child = it;
				}
			}
		}
		node.rhs = min_rhs;
		node.child = best_child;
		if (best_child==NULL) {
			cout << "No closest neighbor among: " << node.neighbors.size() << 
					"(valid: " << nb_valid << ")" << endl;
			if (nb_valid) {
				for (auto &it: node.neighbors) {
					if (dense_map->isTraversableFrom(it->cell.position,
							node.cell.position)) {
						cout << "\t" << it->g << " " << node.cost << " " <<
								node.getCostTo(*it) << endl;
					}
				}
			}
		}
		// FIXME this shouldn't be necessary, right?
		if (node.h == infinity) {
			cout << "uN: h==infinity: " << &node << "; goal: " << goal_node << 
				"; start: " << start_node << endl;
			node.h = heuristic(node);
		}
		node.calcKey();
	}
	// update position in the queue
	// first we remove it if it's there
	// then we put it back if it should be there
	if (node.is_open) {
		open_list.remove(&node);
		node.is_open = false;
	}
	if (node.rhs!=node.g) {
		node.is_open = true;
		open_list.push(&node);
	}
}


// Update node knowing which neighbor changed
void DStarPathPlanner::updateNodeFrom(DStarSearchNode& node,
		DStarSearchNode& from) {
	if ((&node)==goal_node) {
		return;
	}
	const float tmp_rhs = from.g + node.cost + node.getCostTo(from);
	if (tmp_rhs>=node.rhs) {
		return;
	}
	assert(isfinite(tmp_rhs));
	node.rhs = tmp_rhs;
	node.child = &from;
	// FIXME test if I shouldn't compute h each time instead
	if (node.h == infinity) {
		node.h = heuristic(node);
	}
	node.calcKey();
	// update position in the queue
	// first we remove it if it's there
	// then we put it back if it should be there
	// There might be a faster way FIXME
	if (node.is_open) {
		open_list.remove(&node);
		node.is_open = false;
	}
	if (node.rhs!=node.g) {
		node.is_open = true;
		open_list.push(&node);
	}
}


// Compute the initial path
// A modified A* could be faster, I guess FIXME
bool DStarPathPlanner::computeInitialPath() {
	return rePlan();
}


// Update the path after motion or a change in cell cost
bool DStarPathPlanner::rePlan() {
	if ((!start_node)||(!goal_node)) {
		// not properly initialized
		cout << "Not properly initialized." << endl;
		return false;
	}
	if (start_node==goal_node) {
		// we're where we want to go
		cout << "Already there." << endl;
		return true;
	}
	if ((!start_node->is_open)&&(start_node->g<infinity)) {
		cout << "The path is already known." << endl;
		return true;
	}
	int counter = 0;
	while (true) {
		search_loop_timer.start();
		if (open_list.empty()) {
			cout << "No path found: open_list empty." << endl;
			search_loop_timer.stop();
			break; // not found
		}
		if (((*(open_list.front()))<(*start_node))&&(!start_node->is_open)) {
			cout << "Path found (" << open_list.size() << " nodes still open)." << endl;
			search_loop_timer.stop();
			break; // found
		}
		DStarSearchNode* node = open_list.pop();
		node->is_open = false;
		/*if (!check_open_list()) {
			cout << "after pop " << counter<< endl;
			cout << node->cell.position.transpose() << endl;
		}*/
		auto old_key = node->key;
		// FIXME test if I shouldn't compute h each time instead
		if (node->h == infinity) {
			cout << "h==infinity: " << node << "; goal: " << goal_node << 
				"; start: " << start_node << endl;
			node->h = heuristic(*node);
		}
		node->calcKey();
		if (old_key<node->key) {
			// robot has moved or something
			node->is_open = true;
			open_list.push(node);
		} else if (node->g>node->rhs) {
			// overconsistent
			node->g = node->rhs;
			for (auto &it: getNeighborsFrom(*node)) {
				if (dense_map->isTraversableFrom(it->cell.position,
							node->cell.position)) {
					updateNodeFrom(*it, *node);
				}
			}
		} else {
			// underconsistent
			node->g = infinity;
			for (auto &it: getNeighborsFrom(*node)) {
				if (dense_map->isTraversableFrom(it->cell.position,
							node->cell.position)) {
					updateNodeFrom(*it, *node);
				}
			}
			updateNode(*node);
		}
		search_loop_timer.stop();
		/*if (!check_open_list()) {
			return false;
		}*/
		++counter;
	}
	return start_node->g < infinity;
}

// Expand nodes from a given position
void DStarPathPlanner::expandFrom(const Vector3f& position) {
	// reset planner state
	k_m = 0;
	open_list.clear();
	for(auto &it: search_node_map) {
		it.second.g = infinity;
		it.second.rhs = infinity;
		it.second.is_open = false;
		it.second.child = NULL;
		it.second.calcKey(k_m);
	}
	// open position to expand (and set it as start for the heuristic)
	auto seed_node = &getNodeAt(position);
	seed_node->g = infinity;
	seed_node->rhs = 0;
	seed_node->h = 0;
	seed_node->calcKey(k_m);
	seed_node->child = NULL;
	seed_node->is_open = true;
	open_list.push(seed_node);
	// search loop
	int counter = 0;
	/*// counting elements for debug
	int nb_all, nb_closed, nb_open, nb_rhs_fin, nb_g_fin;
	int onb_all, onb_closed, onb_open, onb_rhs_fin, onb_g_fin;
	onb_all=onb_closed=onb_open=onb_rhs_fin=onb_g_fin=0;*/
	while (true) {
		/*// counting elements 
		nb_all=nb_closed=nb_open=nb_rhs_fin=nb_g_fin=0;
		nb_all=search_node_map.size();
		nb_open=open_list.size();
		for (auto &it: search_node_map) {
			if (isfinite(it.second.rhs)) {
				++nb_rhs_fin;
			}
			if (isfinite(it.second.g)) {
				++nb_g_fin;
			}
			if (isfinite(it.second.g)&&(it.second.g==it.second.rhs)) {
				++nb_closed;
			}
		}
		cout << counter << ": ";
		cout << nb_all << " (" << nb_all-onb_all << "), ";
		cout << nb_closed << " (" << nb_closed-onb_closed << "), ";
		cout << nb_open << " (" << nb_open-onb_open << "), ";
		cout << nb_rhs_fin << " (" << nb_rhs_fin-onb_rhs_fin << "), ";
		cout << nb_g_fin << " (" << nb_g_fin-onb_g_fin << "), ";
		cout << endl;
		onb_all = nb_all;
		onb_closed = nb_closed;
		onb_open = nb_open;
		onb_rhs_fin = nb_rhs_fin;
		onb_g_fin = nb_g_fin;
		// end counting*/
		search_loop_timer.start();
		if (open_list.empty()) {
			search_loop_timer.stop();
			break; // done
		}
		DStarSearchNode* node = open_list.pop();
		assert(isfinite(node->rhs));
		node->is_open = false;
		/*if (!check_open_list()) {
			cout << "after pop " << counter<< endl;
			cout << node->cell.position.transpose() << endl;
		}*/
		auto old_key = node->key;
		node->calcKey();
		/*// debug
		auto cell_key = dense_map->positionToIndex(node->cell.position);
		cout << "(" << cell_key.i << ", " << cell_key.j << ", " << cell_key.k << "): ";
		cout << " (" << node->g << ", " << node->rhs << "): ";
		// end debug*/
		if (old_key<node->key) {
			// robot has moved or something
			//cout << "Oh, wait!";// << endl;
			node->is_open = true;
			open_list.push(node);
		} else if (node->g>node->rhs) {
			// overconsistent
			//cout << "Closing";// << endl;
			node->g = node->rhs;
			for (auto &it: getNeighborsFrom(*node)) {
				if (dense_map->isTraversableFrom(it->cell.position,
							node->cell.position)) {
					updateNodeFrom(*it, *node);
				}
			}
		} else {
			// underconsistent
			//cout << "Oh, really?!";// << endl;
			node->g = infinity;
			for (auto &it: getNeighborsFrom(*node)) {
				if (dense_map->isTraversableFrom(it->cell.position,
							node->cell.position)) {
					updateNodeFrom(*it, *node);
				}
			}
			updateNode(*node);
		}
		//cout << " (" << node->g << ", " << node->rhs << ")" << endl;
		search_loop_timer.stop();
		++counter;
	}
	//cout << counter << endl;
}

// Set the start or update it if needed
void DStarPathPlanner::setStart(const geometry_msgs::Pose& start) {
	auto last = start_node;
	auto new_node = &getNodeAt(dense_map->positionToIndex(poseToVector(start)));
	// don't update if unnecessary
	if (last == new_node) {
		return;
	}
	PathPlanner<DStarSearchNode>::setStart(start);
	start_node->h = 0;
	// if it's an update, increase k_m instead or recomputing all keys/g
	if (last) {//&&(!((!start_node->is_open)&&(start_node->g<infinity)))) {
		k_m += heuristic(*last);
	}
	start_node->calcKey(k_m);
	if (goal_node) {
		goal_node->h = heuristic(*goal_node);
		goal_node->calcKey(k_m);
	}
}


// Set the goal
void DStarPathPlanner::setGoal(const geometry_msgs::Pose& goal) {
	// if there is already a goal, reset everything
	open_list.clear();
	for(auto &it: search_node_map) {
		it.second.g = infinity;
		it.second.rhs = infinity;
		it.second.is_open = false;
		it.second.child = NULL;
		it.second.calcKey(k_m);
	}
	// normal setup
	PathPlanner<DStarSearchNode>::setGoal(goal);
	goal_node->g = infinity;
	goal_node->rhs = 0;
//	if (start_node) { // it's now done in the constructor
//		goal_node->h = heuristic(*goal_node);
//	}
	goal_node->calcKey(k_m);
	goal_node->child = NULL;
	goal_node->is_open = true;
	open_list.push(goal_node);
}


// Get the path after planning has been done
bool DStarPathPlanner::fillPath(vector<const TensorCell*>& path) {
	path.clear();
	if (start_node==goal_node) {
		// we're already where we want to go
		return true;
	}
	if (start_node->g == infinity) {
		return false;
	}
	DStarSearchNode* node = start_node;
	while ((node)&&(node!=goal_node)) {
		node = node->child;
		path.push_back(&(node->cell));
	}
	return static_cast<bool>(node);
}

//! Check open_list consistency
bool DStarPathPlanner::check_open_list() {
	bool ok = true;
	for (auto& it: search_node_map) {
		if (it.second.is_open) {
			if (find(open_list.begin(), open_list.end(), &(it.second))==open_list.end()) {
				cout << "(" <<it.first.i<<", "<<it.first.j<<", "<<it.first.k<<
						") should be in open_list but is not!" << endl;
				cout << dense_map->indexToPosition(it.first).transpose();
				CellKey ck = dense_map->positionToIndex(dense_map->indexToPosition(it.first));
				cout <<" (" << ck.i << ", "<<ck.j << ", " << ck.k << ")" << endl;
				ok = false;
			}
		}
	}
	for (auto it: open_list) {
		if (!it->is_open) {
			cout << it->cell.position.transpose() << "is in open_list but is not open!"
					<< endl;
			CellKey ck = dense_map->positionToIndex(it->cell.position);
			cout << "(" << ck.i << ", " << ck.j << ", " << ck.k << ") -> " <<
					dense_map->indexToPosition(ck).transpose() << endl;
			ok = false;
		}
	}
	return ok;
}


// Serialize all planner content (including map)
void DStarPathPlanner::serialize(const string& directory) const {
	// dense map
	dense_map->serialize(directory);
	/*
	 * search_node_map in csv
	 * 
	 * format:
	 * i,j,k,cost,g,rhs,h,key[0],key[1],is_open,child_i,child_j,child_k
	 * cell and neighbor can be found back at deserialization
	 */
	FILE* map_file = fopen((directory+"/search_map.csv").c_str(), "w");
	// should check errors FIXME
	for (auto &it: search_node_map) { // C++11 loop
		CellKey child(infinity, infinity, infinity);
		if (it.second.child) {
			child = dense_map->positionToIndex(it.second.child->cell.position);
		}
		fprintf(map_file, "%d,%d,%d,%f,%f,%f,%f,%f,%f,%d,%d,%d,%d\n", it.first.i,
				it.first.j, it.first.k, it.second.cost,	it.second.g,
				it.second.rhs, it.second.h, it.second.key.first,
				it.second.key.second, static_cast<int>(it.second.is_open),
				child.i, child.j, child.k);
	}
	fclose(map_file);
	/*
	 * open_list in csv
	 * 
	 * format:
	 * i,j,k
	 */
	FILE* open_file = fopen((directory+"/open_list.csv").c_str(), "w");
	// should check errors FIXME
	for (auto &it: open_list) { // C++11 loop
		CellKey index = dense_map->positionToIndex(it->cell.position);
		fprintf(open_file, "%d,%d,%d\n", index.i, index.j, index.k);
	}
	fclose(open_file);
	/*
	 * rest of planner state:
	 * start, goal, k_m
	 */
	FILE* planner_file = fopen((directory+"/planner_state.csv").c_str(), "w");
	// should check errors FIXME
	CellKey start(infinity, infinity, infinity);
	if (start_node) {
		start = dense_map->positionToIndex(start_node->cell.position);
	}
	CellKey goal(infinity, infinity, infinity);
	if (goal_node) {
		goal = dense_map->positionToIndex(goal_node->cell.position);
	}
	fprintf(planner_file, "%d,%d,%d\n%d,%d,%d\n%f\n", start.i, start.j, start.k,
			goal.i, goal.j, goal.k, k_m);
	fclose(planner_file);
}


// Deserialize all planner content (including map)
void DStarPathPlanner::deSerialize(const string& directory) {
	// dense map
	dense_map->deSerialize(directory);
	/*
	 * search_node_map in csv
	 * 
	 * format:
	 * i,j,k,cost,g,rhs,h,key[0],key[1],is_open,child_i,child_j,child_k
	 * cell and neighbor can be found back at deserialization
	 */
	FILE* map_file = fopen((directory+"/search_map.csv").c_str(), "r");
	// should check errors FIXME
	int n, line;
	int i, j, k;
	float cost, g, rhs, h, key0, key1;
	int is_open, ii, jj, kk;
	line = 1;
	while (!feof(map_file)) {
		n = fscanf(map_file, "%d,%d,%d,%f,%f,%f,%f,%f,%f,%d,%d,%d,%d\n",
				&i, &j, &k, &cost, &g, &rhs, &h, &key0, &key1, &is_open, &ii,
				&jj, &kk);
		if (n!=13) {
			cout << "Error reading line "<<line<<" in "<<directory<<
				"/search_map.csv: only "<<n<<" tokens found."<<endl;
		} else {
			CellKey key({i, j, k});
			DStarSearchNode *node = &getNodeAt(key); // might be there (child pointer)
			node->cost = cost; // unnecessary
			node->g = g;
			node->rhs = rhs;
			node->h = h;
			node->key = {key0, key1}; // could call calcKey() instead
			node->is_open = static_cast<bool>(is_open);
			if ((ii!=infinity)&&(jj!=infinity)&&(kk!=infinity)) {
				node->child = &getNodeAt(CellKey(ii, jj, kk));
			}
		}
		++line;
	}
	fclose(map_file);
	/*
	 * open_list in csv
	 * 
	 * format:
	 * i,j,k
	 */
	FILE* open_file = fopen((directory+"/open_list.csv").c_str(), "r");
	// should check errors FIXME
	line = 1;
	while (!feof(open_file)) {
		n = fscanf(open_file, "%d,%d,%d\n", &i, &j, &k);
		if (n!=3) {
			cout << "Error reading line "<<line<<" in "<<directory<<
				"/open_list.csv: only "<<n<<" tokens found."<<endl;
		} else {
			open_list.push_back(&getNodeAt(CellKey(i, j, k)));
		}
		++line;
	}
	fclose(open_file);
	/*
	 * rest of planner state:
	 * start, goal, k_m
	 */
	FILE* planner_file = fopen((directory+"/planner_state.csv").c_str(), "r");
	// should check errors FIXME
	n = fscanf(open_file, "%d,%d,%d\n%d,%d,%d\n%f\n", &i, &j, &k, &ii, &jj,
			&kk, &k_m); // k_m is the member
	if (n!=7) {
		cout << "Error reading "<<directory<<
			"/planner_state.csv: only "<<n<<" tokens found."<<endl;
	} else {
		if ((i!=infinity)&&(j!=infinity)&&(k!=infinity)) {
			start_node = &getNodeAt(CellKey(i, j, k));
		}
		if ((ii!=infinity)&&(jj!=infinity)&&(kk!=infinity)) {
			goal_node = &getNodeAt(CellKey(ii, jj, kk));
		}
	}
	fclose(planner_file);
}


