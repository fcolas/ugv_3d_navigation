#ifndef TENSOR_MAP_H
#define TENSOR_MAP_H

// standard
#include <string>
#include <geometry_msgs/Pose.h>
// Include all Eigen
#include <Eigen/Eigen>
// ROS point clouds
#include <sensor_msgs/PointCloud2.h>
// libpointmatcher point clouds
#include <pointmatcher/PointMatcher.h>
// cache structure
#include <boost/functional/hash.hpp>
#include <unordered_map>
// nearest neighbor search for both sparse and dense voting
#include <nabo/nabo.h>
// for shared pointers
#include <memory>

// Parameters
#include "trp_params.h"
// Timer
#include "MyTimer.h"

using namespace std;
using namespace Eigen;
using namespace Nabo;

//! Point cloud type used
typedef PointMatcher<float>::DataPoints PointCloud;

//! Parameters
extern TensorMapParams* tensor_map_params;

//! Dense voting timer
extern MyTimer dense_voting_timer;
//! Traversability timer
extern MyTimer traversability_timer;
//! Sparse voting timer
extern MyTimer sparse_voting_timer;
//! Distance filter timer
extern MyTimer distance_filter_timer;
//! Cell access timer
extern MyTimer cell_access_timer;

/*
 * Key for cells in the cache structure
 */
//! Key of a cell
struct CellKey {
	int i, j, k;
	//! Constructor
	CellKey(int i, int j, int k):
		i(i), j(j), k(k) {}
	~CellKey() {}
	//! Equality predicate (replaces CellKeyPred)
	bool operator==(const CellKey& rhs) const {
		return ((i==rhs.i)&&(j==rhs.j)&&(k==rhs.k));
	}
};

//! Hash function object for the key
struct CellKeyHash {
	size_t operator()(const CellKey& k) const {
		size_t seed = 0;
		boost::hash_combine<int>(seed, k.i);
		boost::hash_combine<int>(seed, k.j);
		boost::hash_combine<int>(seed, k.k);
		return seed;
	}
};

//! Equality predicate for the key (replaced by operator==)
struct CellKeyPred {
	bool operator()(const CellKey& k1, const CellKey& k2) const {
		return ((k1.i==k2.i)&&(k1.j==k2.j)&&(k1.k==k2.k));
	}
};


/*
 * Cells in the cache structure
 */
//! Traversability
enum TraversabilityState {UNKNOWN=0, OK, NOT_SALIENCY, NOT_ANGLE, NOT_OBSTACLE, NOT_IN_GROUND, NOT_IN_AIR};


//! Content of a cell
struct TensorCell {
	// methods
	//! Constructor
	TensorCell(const Vector3f& position, float stick_sal,
			const Vector3f& normal, int nb_points, int traversability, float plane_prob);

	//! Destructor
	virtual ~TensorCell();

	// members
	//! Full 3D position of the cell
	Vector3f position;
	//! Stick saliency
	float stick_sal;
	//! Normal vector
	Vector3f normal;
	//! number of points in the cell
	int nb_points;
	//! Angle to vertical axis (derived from normal)
	float angle_to_vertical;
	//! Quick traversability check
	TraversabilityState traversability;
	//! Probability of a plane
	float plane_probability;
};


//! cache for dense map
//typedef unordered_map<CellKey, TensorCell, CellKeyHash, CellKeyPred> CachedTensorMap;
typedef unordered_map<CellKey, TensorCell, CellKeyHash> CachedTensorMap;


//! Full map with both dense and sparse voting
class TensorMap {
public:
	//! Constructor
	TensorMap();

	//! Destructor
	virtual ~TensorMap();

	//! Get map from point cloud
	void import(const PointCloud& point_cloud);

	//! Insert new cell
	void insertDenseCell(int i, int j, int k, const Vector3f& position,
			float stick_sal, const Vector3f& normal, int nb_points,
			int traversability, float plane_prob);

	//! Accessing cell; can do dense voting if cell not there yet
	TensorCell& operator[](const CellKey& key);

	//! Return center position of cell
	Vector3f indexToPosition(const CellKey& key) const;

	//! Return index of cell containing position
	CellKey positionToIndex(const Vector3f& position) const;
	
	//! Get traversability of a pose
	bool isTraversable(const geometry_msgs::Pose& pose, bool v=false);

	//! Get traversability of a cell coming from another cell
	bool isTraversableFrom(const Vector3f& position,
			const Vector3f& from_position, bool v=false);
	bool isTraversableFrom(const CellKey& key, const CellKey& orig,
			bool v=false) {
		return isTraversableFrom(indexToPosition(key), indexToPosition(orig));
	}
	//! Get faster traversability
	bool isEasilyTraversable(const CellKey& key);

	//! Serialize sparse map into file
	void serializeSparse(const string& filename) const;
	//! Deserialize sparse map from file
	void deSerializeSparse(const string& filename);

	//! Serialize sparse, dense and the rest (?)
	void serialize(const string& directory) const;
	//! Deserialize sparse, dense and the rest (?)
	void deSerialize(const string& directory);

protected:
	//! Point cloud with sparse voting
	PointCloud sparse_map;
	
	//! Cache of dense voting
	CachedTensorMap dense_map;

	//! kd-tree for fast neighbor search for both sparse and dense voting
	shared_ptr<NNSearchF> kdTree;

	//! Compute dense voting
	TensorCell& computeDenseVoting(const CellKey& key);
};

#endif
