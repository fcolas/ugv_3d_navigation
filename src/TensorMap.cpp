#include "TensorMap.h"
#include "tensorUtils.h"

#include <iostream>
#include <cmath>
// getting max float
#include <limits>
// file I/O using formatted data
#include <cstdio>
#include <geometry_msgs/Quaternion.h>
// C++11 time measuring
#include <chrono>

using namespace std;
using namespace std::chrono;

// Parameters
TensorMapParams* tensor_map_params;

// Dense voting timer
MyTimer dense_voting_timer;
// Traversability timer
MyTimer traversability_timer;
// Sparse voting timer
MyTimer sparse_voting_timer;
// Distance filter timer
MyTimer distance_filter_timer;
// Cell access timer
MyTimer cell_access_timer;


/*
 * TensorCell
 */
// Constructor
TensorCell::TensorCell(const Vector3f& position, float stick_sal,
		const Vector3f& normal, int nb_points, int t12y, float plane_prob):
	position(position),
	stick_sal(stick_sal),
	normal(normal),
	nb_points(nb_points),
	angle_to_vertical(acos(normal.z())),
	plane_probability(plane_prob)
{
	switch (t12y) {
	case OK:
		traversability = OK;
		break;
	case NOT_SALIENCY:
		traversability = NOT_SALIENCY;
	case NOT_ANGLE:
		traversability = NOT_ANGLE;
	case NOT_OBSTACLE:
		traversability = NOT_OBSTACLE;
	case NOT_IN_GROUND:
		traversability = NOT_IN_GROUND;
	case NOT_IN_AIR:
		traversability = NOT_IN_AIR;
		break;
	default:
		traversability = UNKNOWN;
	}
}


// Destructor
TensorCell::~TensorCell(){
	// nothing to do
}



/*
 * TensorMap
 */
// Constructor
TensorMap::TensorMap() {
	// nothing to do
}


// Destructor
TensorMap::~TensorMap(){
	// nothing to do
}

PointCloud distance_filter(const PointCloud& cloud) {
	shared_ptr<NNSearchF> kdTree(NNSearchF::create(cloud.features,
			cloud.features.rows()-1, NNSearchF::KDTREE_TREE_HEAP));
	const DistanceFilterParams& params = tensor_map_params->distance_filter;
	const int max_points = params.max_knn;
	const float min_dist = params.min_distance;

	const int nb_points = cloud.features.cols();
	NNSearchF::IndexMatrix indices(max_points, nb_points);
	NNSearchF::Matrix dists2(max_points, nb_points);
	const float min2 = pow(min_dist, 2);
	// finding out points to remove
	vector<bool> is_not_removed(nb_points, true);
	kdTree->knn(cloud.features, indices, dists2, max_points, 0, NNSearchF::SORT_RESULTS, min_dist);
	for (int i=0; i<nb_points;++i) {
		if (is_not_removed[i]) {
			for (int j=0; j<max_points; ++j) {
				if (dists2(j, i)>min2) {
					break;
				} else {
					is_not_removed[indices(j, i)] = false;
				}
			}
		}
	}
	// recreating point cloud
	const bool is_desc(cloud.descriptors.rows()*cloud.descriptors.cols());
	const int nb_new = count(is_not_removed.cbegin(), is_not_removed.cend(), true);
	PointCloud result(cloud.featureLabels, cloud.descriptorLabels, nb_new);
	int i_new = 0;
	for (int i_old=0; i_old<nb_points; ++i_old) {
		if (is_not_removed[i_old]) {
			if (is_desc) {
				result.descriptors.col(i_new) = cloud.descriptors.col(i_old);
			}
			result.features.col(i_new) = cloud.features.col(i_old);
			++i_new;
		}
	}
	cout <<"Filtering: "<<nb_points<<"->"<<nb_new<<endl;
	return result;
}


// Get map from point cloud
void TensorMap::import(const PointCloud& point_cloud){
	// filtering
	distance_filter_timer.start();
	sparse_map = distance_filter(point_cloud);
	distance_filter_timer.stop();

	/* sparse voting */
	sparse_voting_timer.start();
	// nearest neighbor search
	const SparseVotingParams& params = tensor_map_params->sparse_voting;
	const int maxNbPts = params.max_knn; // max number of neighbors
	const float sigma = params.sigma; // sigma parameter for tensor voting
	const float max_dist = params.max_dist; // maximum distance for knn
	const float sigma2 = pow(sigma, 2);
	kdTree.reset(NNSearchF::create(sparse_map.features,
			sparse_map.features.rows()-1, NNSearchF::KDTREE_TREE_HEAP));
	// matches and search
	NNSearchF::IndexMatrix indices(maxNbPts, sparse_map.features.cols());
	NNSearchF::Matrix dists2(maxNbPts, sparse_map.features.cols());
	kdTree->knn(sparse_map.features, indices, dists2, maxNbPts, 0, NNSearchF::SORT_RESULTS,
			max_dist);

	// new rows for tensors
	PointCloud::Labels labels;
	labels.push_back({"eigen_values", 3});
	labels.push_back({"eigen_vectors", 9});
	sparse_map.allocateDescriptors(labels);
	auto eigen_values = sparse_map.getDescriptorViewByName("eigen_values");
	auto eigen_vectors = sparse_map.getDescriptorViewByName("eigen_vectors");

	const auto eye3 = Matrix3f::Identity();
	// actual voting
	for (int i=0; i<sparse_map.features.cols(); ++i) {
		//cout << "\rSparse voting from neighbors: " << i << "/" << 
		//		sparse_map.features.cols()<<":"; cout.flush();
		const Vector3f voter = sparse_map.features.col(i).head(3);
		// compute tensor for voter
		Matrix3f tensor = Matrix3f::Zero();
		for (int j=0; j<maxNbPts; ++j) {
			if (dists2(j, i) == numeric_limits<float>::infinity()) {
				break;
			}
			const Vector3f votee = sparse_map.features.col(indices(j, i)).head(3);
			const Matrix3f vv = (votee-voter)*((votee-voter).transpose());
			const float n_vv = fabs(((votee-voter).transpose())*(votee-voter));
			tensor += exp(-dists2(j, i)/sigma2)*(eye3 - vv/n_vv);
		}
		// decompose tensor
		const EigenSolver<Matrix3f> solver(tensor);
		Vector3f eigenVals = solver.eigenvalues().real();
		Matrix3f eigenVectors = solver.eigenvectors().real();
		sortTensor(eigenVals, eigenVectors);
		MatrixXf e_v(eigenVectors);
		e_v.resize(9, 1);
		// store into point cloud
		eigen_values.col(i) = eigenVals;
		eigen_vectors.col(i) = e_v;
	}
	//cout << endl;
	sparse_voting_timer.stop();
}


// Compute dense voting
TensorCell& TensorMap::computeDenseVoting(const CellKey& key) {
	// timing
	dense_voting_timer.start();

	// getting parameters
	const DenseVotingParams& params = tensor_map_params->dense_voting;
	const int maxNbPts = params.max_knn; // max number of neighbors
	const float sigma = params.sigma; // tensor voting parameter
	const float max_dist = params.max_dist; // maximum radius
	const Vector3f& cell_dims = tensor_map_params->cell_dimensions;
	//const float cell2 = cell_dims.squaredNorm()/4;
	const float obs_dir_offset = params.obs_dir_offset;
	const float absolute_saliency_threshold = params.absolute_saliency_threshold;

	// position of the point in space
	const Vector3f position(indexToPosition(key));
	// data in the point cloud
	auto eigen_vectors = sparse_map.getDescriptorViewByName("eigen_vectors");
	auto eigen_values = sparse_map.getDescriptorViewByName("eigen_values");
	auto obsDirection = sparse_map.getDescriptorViewByName\
				("observationDirections");
	// tensor to vote into
	Matrix3f tensor = Matrix3f::Identity();
	// getting observation direction from closest point to realign normal
	float min_dist2 = numeric_limits<float>::infinity();
	int closest_index = -1;
	// nearest neighbor search of points for voters

	VectorXi indices(maxNbPts);
	VectorXf dists2(maxNbPts);
	kdTree->knn(position, indices, dists2, maxNbPts, 0, NNSearchF::SORT_RESULTS,
			max_dist);
	// going through all neighboring points of the center of the cell
	int nb_points = 0;
	int i;
	for (i = 0; i<maxNbPts; ++i) {
		if (dists2(i) == numeric_limits<float>::infinity()) { // no match anymore
			break;
		}
		if (dists2(i) == 0.) { // null vote
			++nb_points;
			continue;
		}
		int index = indices(i);
		// closest point for the point of view
		if (dists2(i)<min_dist2) {
			min_dist2 = dists2(i);
			closest_index = index;
		}
		// counting the number of points in the cell
		const Vector3f voter = sparse_map.features.col(index).head(3);
		if (((voter-position).array().abs() <= (cell_dims/2).array()).all()) {
			++nb_points;
		}
		/*
		 * Full tensor voting
		 */
		const Vector3f v = position - voter;
		Vector3f v_hat;
		if (v.isZero()) {
			v_hat = v;
		} else {
			v_hat = v.normalized();
		}
		// saliencies
		float saliencies[3];
		saliencies[0] = eigen_values(0, index) - eigen_values(1, index); 
		saliencies[1] = eigen_values(1, index) - eigen_values(2, index); 
		saliencies[2] = eigen_values(2, index);
		MatrixXf e_vectors(eigen_vectors.col(index));
		e_vectors.resize(3, 3);
		// normal spaces
		Matrix3f normal_spaces[3];
		normal_spaces[0] = e_vectors.col(0) * (e_vectors.col(0).transpose());
		normal_spaces[1] = normal_spaces[0] + e_vectors.col(1) * (e_vectors.col(1).transpose());
		normal_spaces[2] = normal_spaces[1] + e_vectors.col(2) * (e_vectors.col(2).transpose());
		// accumulator for all votes of this voter
		Matrix3f A = Matrix3f::Zero();
		// loop for all 3 votes
		for (int d=0; d<3; ++d) {
			// normal vector
			const Vector3f v_n = normal_spaces[d] * v;
			Vector3f v_n_hat;
			if (v_n.isZero()) {
				v_n_hat = v_n;
			} else {
				v_n_hat = v_n.normalized();
			}
			// tangent vector
			const Vector3f v_t = v - v_n;
			Vector3f v_t_hat;
			if (v_t.isZero()) {
				v_t_hat = v_t;
			} else {
				v_t_hat = v_t.normalized();
			}
			// normal for votee
			const float cos_theta = v_hat.dot(v_t_hat);
			const float sin_theta = v_hat.dot(v_n_hat);
			const float cos_2theta = 2*pow(cos_theta, 2) - 1;
			const float sin_2theta = 2*cos_theta*sin_theta;
			const Vector3f v_c_hat = cos_2theta * v_n_hat - sin_2theta * v_t_hat;
			// weight functions
			// using n=4 as in King FIXME
			const float w_angle = pow(cos_theta, 2*4);
			const float z = v.norm()/sigma;
			const float w_dist = pow(z,2)*(pow((z-3),4))/16;
			// vote for this dimension
			const Matrix3f A_d = w_dist*w_angle*v_c_hat*v_c_hat.transpose() +
					w_dist*(normal_spaces[d] - v_n_hat*v_n_hat.transpose());
			A += saliencies[d] * A_d;
		}
		// standard voting FIXME
		//tensor += A;
		// normalized voting FIXME
		tensor += A / (eigen_values(0, index) + absolute_saliency_threshold);
		//
		/* // old vote
		// actual vote
		Vector3f e_v = position - voter;
		const float r = sqrt(dists2(i));
		e_v /= r;
		const float stick_sal = eigen_values(0, index) - eigen_values(1, index);
		MatrixXf e_vectors(eigen_vectors.col(index));
		e_vectors.resize(3, 3);
		const Vector3f v_n = e_vectors.col(0);
		const Vector3f v_t = v_n.cross(e_v.cross(v_n));
		const float cos_theta = e_v.dot(v_t);
		const float theta = atan2(e_v.dot(v_n), cos_theta);
		// Decoupled weighting profile for angle and distance
		// Angle based weighting
		const float w_angle = pow(cos_theta,4);
		// Distance based weighting
		const float z = r/sigma;
		const float w_dist = pow(z,2)*(pow((z-3),2))/16;
		// stick vote
		// could compute sin and cos based on cos_theta and sin_theta FIXME
		const Vector3f v_c = v_n*cos(2*theta) - v_t*sin(2*theta);
		const float w = w_angle*w_dist;
		tensor += stick_sal*w*v_c*v_c.transpose();
		//tensor += stick_sal*w*v_c*v_c.transpose()/(eigen_values(0, index) + absolute_saliency_threshold);
		*/
	}
	// analyse resulting tensor
	const EigenSolver<Matrix3f> solver(tensor);
	Vector3f eigenVals = solver.eigenvalues().real();
	Matrix3f eigenVectors = solver.eigenvectors().real();
	sortTensor(eigenVals, eigenVectors);
	float stick_sal = eigenVals(0)-eigenVals(1);
	float s = eigenVals(0)-eigenVals(1);
	float p = eigenVals(1)-eigenVals(2);
	float b = eigenVals(2);
	float Z = eigenVals(0);

	//cout << "Saliency distribution: (" << s/Z << ", " << p/Z << ", " << b/Z << "); abs_sal=" << Z << "; p_plane=" << s/(Z+absolute_saliency_threshold) << " (" << absolute_saliency_threshold << ")" << endl;
	Vector3f normal;
	if (closest_index == -1) {
		// no point at all
		normal = {0., 0., 0.};
		stick_sal = 0.;
	} else {
		// flip normal according to closest point
		const Vector3f obsDir = obsDirection.col(closest_index)+
				Vector3f(0, 0, obs_dir_offset);
		const Vector3f v_n = eigenVectors.col(0);
		if (obsDir.dot(v_n)<0.) {
			normal = -v_n;
		} else {
			normal = v_n;
		}
	}

	dense_map.insert({key, TensorCell(position, stick_sal, normal, nb_points,
				UNKNOWN, s/(Z+absolute_saliency_threshold))});
	// stopping timer
	dense_voting_timer.stop();

	return dense_map.at(key);
}

// Insert new cell
void TensorMap::insertDenseCell(int i, int j, int k, const Vector3f& position,
		float stick_sal, const Vector3f& normal, int nb_points,
		int traversability, float plane_prob) {
	dense_map.insert({CellKey(i, j, k),
			TensorCell(position, stick_sal, normal, nb_points, traversability, plane_prob)});
}


// Accessing cell; will do dense voting if cell not there yet
TensorCell& TensorMap::operator[](const CellKey& key) {
	cell_access_timer.start();
	try {
		dense_map.at(key);
		cell_access_timer.stop();
		//cout << "Hit: "<<key.i<<", "<<key.j<<", "<<key.k<<endl;
		return dense_map.at(key);
	} catch (out_of_range& oor) {
		//cout << "Miss: "<<key.i<<", "<<key.j<<", "<<key.k<<endl;
		computeDenseVoting(key);
		cell_access_timer.stop();
		return dense_map.at(key);
	}
}


// Return center position of cell
Vector3f TensorMap::indexToPosition(const CellKey& key) const {
	const Vector3f& cell_dims = tensor_map_params->cell_dimensions;
	const Vector3f& origin = tensor_map_params->origin;
	return origin + Vector3f(key.i*cell_dims(0), key.j*cell_dims(1),
			key.k*cell_dims(2));
}


inline int myround(float f) { //not symmetrical round
	return static_cast<int>(floor(f+0.5));
}
// Return index of cell containing position
CellKey TensorMap::positionToIndex(const Vector3f& position) const {
	const Vector3f& cell_dims = tensor_map_params->cell_dimensions;
	const Vector3f& origin = tensor_map_params->origin;
	const Vector3f delta = position - origin;
	int i = myround(delta(0)/cell_dims(0));
	int j = myround(delta(1)/cell_dims(1));
	int k = myround(delta(2)/cell_dims(2));
	return CellKey(i, j, k);
}


// Get traversability of a pose
bool TensorMap::isTraversable(const geometry_msgs::Pose& pose, bool v) {
	traversability_timer.start();
	const Vector3f position(poseToVector(pose));
	const CellKey& key(positionToIndex(position));
	const TensorCell& cell((*this)[key]);

	// check it is easily traversable
	if (isEasilyTraversable(key)) {
		traversability_timer.stop();
		return true;
	}
	// if not, check in details

	// getting parameters
	const TraversabilityParams& params = tensor_map_params->traversability;
	const float min_stick_sal = params.min_saliency;
	const float max_orientation_angle = params.max_slope;
	const int stick_sal_threshold = params.max_points_in_free_cell;
	const float min_free_cell_ratio = params.min_free_cell_ratio;
	const int nb_max_points = params.max_points_in_bounding_box;
	const int nb_min_support = params.min_support_points;
	const float length = params.length;
	const float width = params.width;
	const float height = params.height;
	const float l_2 = length/2. + params.inflation;
	const float w_2 = width/2. + params.inflation;
	const float ground_buffer = params.ground_buffer;
	const float buf2 = ground_buffer/2;
	const float h_2 = height/2;
	const float h0_2 = (height + buf2)/2;
	const float h1_2 = (height - buf2)/2;

	// check for enough saliency
	if (cell.stick_sal<min_stick_sal) {
		if (v) {
			cout << "Saliency too low: "<<cell.stick_sal<<"/"<<
					min_stick_sal<<endl;
		}
		traversability_timer.stop();
		return false;
	}
	// check orientation is correct
	if (cell.angle_to_vertical>max_orientation_angle) {
		if (v) {
			cout << "Angle too steep: "<<cell.angle_to_vertical<<"/"<<
					max_orientation_angle<<endl;
		}
		traversability_timer.stop();
		return false;
	}
	/*
	 * check absence of obstacle
	 */
	// align orientation to surface
	Matrix3f alignedAxes;
	alignQuaternion(pose.orientation, cell.normal, alignedAxes);
	Vector3f x = alignedAxes.col(0);
	Vector3f y = alignedAxes.col(1);
	Vector3f z = alignedAxes.col(2);
	// generate bounding box
	Vector3f max_corner = l_2*x.array().abs()+\
						  w_2*y.array().abs()+\
						  h0_2*z.array().abs();
	Vector3f center = position + h_2*z;
	CellKey min_key = positionToIndex(center - max_corner);
	CellKey max_key = positionToIndex(center + max_corner);
	// shifted up to separate support layer FIXME
	center += (buf2/2)*z;
	// look through bounding box for possible obstacles
	float nb_robot_cells = 0.;
	float nb_ok_robot_cells = 0.;
	int nb_points_in_cells = 0;
	int nb_support_points = 0;
	for (int i=min_key.i; i<max_key.i+1; ++i)
	for (int j=min_key.j; j<max_key.j+1; ++j)
	for (int k=min_key.k; k<max_key.k+1; ++k) {
		// check cell is in the robot
		TensorCell c = (*this)[CellKey(i, j, k)];
		Vector3f rel_pos = c.position - center;
		Vector3f rel_pos_aligned = alignedAxes.transpose()*rel_pos;
		float rel_x, rel_y, rel_z;
		rel_x = rel_pos_aligned(0);
		rel_y = rel_pos_aligned(1);
		rel_z = rel_pos_aligned(2);
		if ((fabs(rel_x)>l_2)||(fabs(rel_y)>w_2)) {
			continue;
		}
		if (fabs(rel_z)>h1_2) {
			if (fabs(rel_z+h_2)<buf2) {
				nb_support_points += c.nb_points;
			}
		} else {
			// check no obstacle
			nb_robot_cells += 1;
			if (c.nb_points<=stick_sal_threshold) {
				nb_ok_robot_cells += 1;
			} else if (c.angle_to_vertical <= max_orientation_angle) {
				nb_ok_robot_cells += 1;
				nb_points_in_cells += c.nb_points;
			}
		}
	}
	// exit if not enough free cells
	if ((nb_ok_robot_cells/nb_robot_cells)<min_free_cell_ratio)
	{
		if (v) {
			cout << "not enough free cells: "<<
					nb_ok_robot_cells<<"/"<<
					min_free_cell_ratio*nb_robot_cells<<" ("<<
					nb_robot_cells<<")"<<endl;
		}
		traversability_timer.stop();
		return false;
	}
	if (nb_points_in_cells>nb_max_points) {
		if (v) {
			cout << "too many (ground) points in cells: "<<
					nb_points_in_cells<<"/"<<nb_max_points<<endl;
		}
		traversability_timer.stop();
		return false;
	}
	if (nb_support_points<nb_min_support) {
		if (v) {
			cout << "too few support points in cells: "<<
					nb_support_points<<"/"<<nb_min_support<<endl;
		}
		traversability_timer.stop();
		return false;
	}
	// cell seems valid
	if (v) {
		cout << "valid" << endl;
	}
	traversability_timer.stop();
	return true;

}

// Get traversability of a cell coming from another cell
bool TensorMap::isTraversableFrom(const Vector3f& position,
		const Vector3f& from_position, bool v) {
	// computing 3D pose
	geometry_msgs::Pose pose;
	// position
	pose.position.x = position(0);
	pose.position.y = position(1);
	pose.position.z = position(2);
	// orientation from unity vectors
	const Vector3f x = (position-from_position).normalized();
	const Vector3f y = Vector3f(0, 0, 1).cross(x);
	const Vector3f z = x.cross(y);
	Matrix3f rotation_matrix;
	rotation_matrix.col(0) = x;
	rotation_matrix.col(1) = y;
	rotation_matrix.col(2) = z;
	matrixToQuaternion(rotation_matrix, pose.orientation);
	return isTraversable(pose, v);
}


// Get faster traversability
bool TensorMap::isEasilyTraversable(const CellKey& key) {
	TensorCell& cell((*this)[key]);
	// check cache
	if (cell.traversability!=UNKNOWN) {
		return (cell.traversability==OK);
	}
	// compute it
	const bool v = false;

	// getting parameters
	const TraversabilityParams& params = tensor_map_params->traversability;
	const float min_stick_sal = params.min_saliency;
	const float max_orientation_angle = params.max_slope;
	const int stick_sal_threshold = params.max_points_in_free_cell;
	const float min_free_cell_ratio = params.min_free_cell_ratio;
	const int nb_max_points = params.max_points_in_bounding_box;
	const int nb_min_support = params.min_support_points;
	const float diameter = params.diameter;
	const float d_2 = diameter/2. + params.inflation;
	const float r2 = d_2*d_2;
	const float height = params.height;
	const float ground_buffer = params.ground_buffer;
	const float buf2 = ground_buffer/2;
	const float h_2 = height/2;
	const float h0_2 = (height + buf2)/2;
	const float h1_2 = (height - buf2)/2;

	// check for enough saliency
	if (cell.stick_sal<min_stick_sal) {
		if (v) {
			cout << "Saliency too low: "<<cell.stick_sal<<"/"<<
					min_stick_sal<<endl;
		}
		cell.traversability = NOT_SALIENCY;		
		return false;
	}
	// check orientation is correct
	if (cell.angle_to_vertical>max_orientation_angle) {
		if (v) {
			cout << "Angle too steep: "<<cell.angle_to_vertical<<"/"<<
					max_orientation_angle<<endl;
		}
		cell.traversability = NOT_ANGLE;		
		return false;
	}
	/*
	 * check absence of obstacle
	 */
	// align orientation to surface
	Matrix3f alignedAxes;
	geometry_msgs::Quaternion quat;
	quat.x = quat.y = quat.z = 0;
	quat.w = 1;
	alignQuaternion(quat, cell.normal, alignedAxes);
	Vector3f x = alignedAxes.col(0);
	Vector3f y = alignedAxes.col(1);
	Vector3f z = alignedAxes.col(2);
	// generate bounding box
	Vector3f max_corner = d_2*x.array().abs()+\
						  d_2*y.array().abs()+\
						  h0_2*z.array().abs();
	Vector3f center = cell.position + h_2*z;
	CellKey min_key = positionToIndex(center - max_corner);
	CellKey max_key = positionToIndex(center + max_corner);
	// shifted up to separate support layer FIXME
	center += buf2/2*z;
	// look through bounding box for possible obstacles
	float nb_robot_cells = 0.;
	float nb_ok_robot_cells = 0.;
	int nb_points_in_cells = 0;
	int nb_support_points = 0;
	for (int i=min_key.i; i<max_key.i+1; ++i)
	for (int j=min_key.j; j<max_key.j+1; ++j)
	for (int k=min_key.k; k<max_key.k+1; ++k) {
		// check cell is in the robot
		TensorCell c = (*this)[CellKey(i, j, k)];
		Vector3f rel_pos = c.position - center;
		Vector3f rel_pos_aligned = alignedAxes.transpose()*rel_pos;
		float rel_x, rel_y, rel_z;
		rel_x = rel_pos_aligned(0);
		rel_y = rel_pos_aligned(1);
		rel_z = rel_pos_aligned(2);
		if (pow(rel_x, 2)+pow(rel_y, 2)>r2) {
			continue;
		}
		if (fabs(rel_z)>h1_2) {
			if (fabs(rel_z+h_2)<buf2) {
				nb_support_points += c.nb_points;
			}
		} else {
			// check no obstacle
			nb_robot_cells += 1;
			if (c.nb_points<=stick_sal_threshold) {
				nb_ok_robot_cells += 1;
			} else if (c.angle_to_vertical <= max_orientation_angle) {
				nb_ok_robot_cells += 1;
				nb_points_in_cells += c.nb_points;
			}
		}
	}
	// exit if not enough free cells
	if ((nb_ok_robot_cells/nb_robot_cells)<min_free_cell_ratio)
	{
		if (v) {
			cout << "not enough free cells: "<<
					nb_ok_robot_cells<<"/"<<
					min_free_cell_ratio*nb_robot_cells<<" ("<<
					nb_robot_cells<<")"<<endl;
		}
		cell.traversability = NOT_OBSTACLE;		
		return false;
	}
	if (nb_points_in_cells>nb_max_points) {
		if (v) {
			cout << "too many (ground) points in cells: "<<
					nb_points_in_cells<<"/"<<nb_max_points<<endl;
		}
		cell.traversability = NOT_IN_GROUND;		
		return false;
	}
	if (nb_support_points<nb_min_support) {
		if (v) {
			cout << "too few support points in cells: "<<
					nb_support_points<<"/"<<nb_min_support<<endl;
		}
		cell.traversability = NOT_IN_AIR;		
		return false;
	}
	// cell seems valid
	if (v) {
		cout << "valid" << endl;
	}
	cell.traversability = OK;		
	return true;
}


// Serialize sparse map into file
void TensorMap::serializeSparse(const string& filename) const {
	serializePointCloud(sparse_map, filename);
}


// Deserialize sparse map from file
void TensorMap::deSerializeSparse(const string& filename){
	deSerializePointCloud(sparse_map, filename);
	// reinitialize kdtree
	kdTree.reset(NNSearchF::create(sparse_map.features,
			sparse_map.features.rows()-1, NNSearchF::KDTREE_TREE_HEAP));
}


// Serialize sparse, dense and the rest (?)
void TensorMap::serialize(const string& directory) const{
	serializeSparse(directory+"/sparse.csv");
	/*
	 * dense serialization in csv
	 * 
	 * format:
	 * i,j,k,x,y,z,stick_sal,nx,ny,nz,nb_points,traversability
	 */
	FILE* dense_file = fopen((directory+"/dense.csv").c_str(), "w");
	// should check error FIXME
	for (auto &it: dense_map) { // C++11 loop
		fprintf(dense_file, "%d,%d,%d,%f,%f,%f,%f,%f,%f,%f,%d,%d,%f\n", it.first.i,
				it.first.j, it.first.k, it.second.position(0),
				it.second.position(1), it.second.position(2),
				it.second.stick_sal, it.second.normal(0),
				it.second.normal(1), it.second.normal(2),
				it.second.nb_points, it.second.traversability, it.second.plane_probability);
	}
	fclose(dense_file);
}


// Deserialize sparse, dense and the rest (?)
void TensorMap::deSerialize(const string& directory){
	deSerializeSparse(directory+"/sparse.csv");
	/*
	 * dense deserialization in csv
	 * 
	 * format:
	 * i,j,k,x,y,z,stick_sal,nx,ny,nz,nb_points,traversability
	 */
	FILE* dense_file = fopen((directory+"/dense.csv").c_str(), "r");
	// should check error FIXME
	int i, j, k;
	float stick_sal;
	float x, y, z, nx, ny, nz;
	int nb_points, traversability;
	float plane_prob;
	int n;
	int line = 1;
	while (!feof(dense_file)) {
		n = fscanf(dense_file, "%d,%d,%d,%f,%f,%f,%f,%f,%f,%f,%d,%d,%f\n",
				&i, &j, &k, &x, &y, &z, &stick_sal, &nx, &ny, &nz, &nb_points,
				&traversability, &plane_prob);
		if (n!=13) {
			cout << "Error reading line "<<line<<" in "<<directory<<
				"/dense.csv: only "<<n<<" tokens found."<<endl;
		} else {
			insertDenseCell(i, j, k, Vector3f(x, y, z), stick_sal,
					Vector3f(nx, ny, nz), nb_points, traversability, plane_prob);
		}
	}
	fclose(dense_file);
}


