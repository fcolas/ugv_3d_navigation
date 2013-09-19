#include "tensorUtils.h"
#include <cmath>
#include <limits>
// file I/O using formatted data
#include <cstdio>
// dynamic file I/O 
#include <fstream>
#include <sstream>

// Convert ROS pose to Vector3f
Vector3f poseToVector(const geometry_msgs::Pose& pose) {
	const float px = static_cast<float>(pose.position.x);
	const float py = static_cast<float>(pose.position.y);
	const float pz = static_cast<float>(pose.position.z);
	return Vector3f({px, py, pz});
}


// Convert ROS pose to Eigen::Quaternionf
Quaternionf poseToQuaternion(const geometry_msgs::Pose& pose) {
	const float w = static_cast<float>(pose.orientation.w);
	const float x = static_cast<float>(pose.orientation.x);
	const float y = static_cast<float>(pose.orientation.y);
	const float z = static_cast<float>(pose.orientation.z);
	return Quaternionf(w, x, y, z);
}

// Linear distance between poses
float linearDistance(const geometry_msgs::Pose& p1,
		const geometry_msgs::Pose& p2) {
	return (poseToVector(p1)-poseToVector(p2)).norm();
}


// Angular distance between poses
float angularDistance(const geometry_msgs::Pose& p1,
		const geometry_msgs::Pose& p2) {
	return poseToQuaternion(p1).angularDistance(poseToQuaternion(p2));
}


// Convert Vector3f to ROS Positionf
geometry_msgs::Point vectorToPosition(const Vector3f& vector) {
	geometry_msgs::Point p;
	p.x = vector(0);
	p.y = vector(1);
	p.z = vector(2);
	return p;
}


// Convert ROS quaternion into direction vector
Vector3f quaternionToDirection(const geometry_msgs::Quaternion& quaternion) {
	const float w = quaternion.w;
	const float x = quaternion.x;
	const float y = quaternion.y;
	const float z = quaternion.z;
	return Vector3f(w*w+x*x-y*y-z*z, 2*x*y - 2*w*z, 2*x*z + 2*w*y);
}


// Compute radius (least squares fitting)
void getLeastSquaresRadius(const MatrixXf& points, float* R, float* res,
		float* max_res) {
	// points are assumed to be in robot frame
	float y2 = 0;
	float yd = 0;
	for (auto i=0; i<points.cols(); ++i) {
		const float xi = points(0, i);
		const float yi = points(1, i);
		y2 += 2*pow(yi, 2);
		yd += yi*(pow(xi, 2)+pow(yi, 2));
	}
	float worst_res = 0;
	float res_acc = 0;
	if (y2<1e-6) {
		*R = std::numeric_limits<float>::infinity();
	} else {
		*R = yd/y2;
	}
	float cur_res;
	for (auto i=0; i<points.cols(); ++i) {
		const float xi = points(0, i);
		const float yi = points(1, i);
		if (xi<0) {
			cur_res = pow(xi, 2) + pow(yi, 2);
		} else if (y2<1e-6) {
			cur_res = yi*yi;
		} else {
			cur_res = fabs(pow(yi-*R, 2)+xi*xi -pow(*R, 2));
		}
		res_acc += cur_res;
		if (worst_res<cur_res) {
			worst_res = cur_res;
		}
	}
	*res = res_acc/points.cols();
	*max_res = worst_res;
}


// Compute radius (last point)
void getLastPointRadius(const MatrixXf& points, float* R, float* res,
		float* max_res) {
	// compute radius
	const float x = points(0, points.cols()-1);
	const float y = points(1, points.cols()-1);
	const float vdt = sqrt(pow(x, 2) + pow(y, 2));
	const float wdt = 2.*atan2(y, x);
	if (fabs(wdt)<1e-6) {
		*R = std::numeric_limits<float>::infinity();
	} else {
		*R = vdt/wdt;
	}
	// compute residuals
	float worst_res = 0;
	float res_acc = 0;
	float cur_res;
	for (auto i=0; i<points.cols(); ++i) {
		const float xi = points(0, i);
		const float yi = points(1, i);
		if (xi<0) {
			cur_res = pow(xi, 2) + pow(yi, 2);
		} else if (wdt<1e-6) {
			cur_res = yi*yi;
		} else {
			cur_res = fabs(pow(yi-*R, 2)+xi*xi -pow(*R, 2));
		}
		res_acc += cur_res;
		if (worst_res<cur_res) {
			worst_res = cur_res;
		}
	}
	*res = res_acc/points.cols();
	*max_res = worst_res;
}


// Convert position, normal, direction to full ROS pose
geometry_msgs::Pose vectorsToPose(const Vector3f& position,
		const Vector3f& normal, const Vector3f& direction) {
	geometry_msgs::Pose p;
	p.position = vectorToPosition(position);
	Matrix3f axes;
	axes.col(0) = direction.normalized();
	axes.col(2) = normal.normalized();
	axes.col(1) = axes.col(2).cross(axes.col(0));
	matrixToQuaternion(axes, p.orientation);
	return p;
}


// Sort eigen vectors and values in descending order
void sortTensor(Vector3f& eigenVa, Matrix3f& eigenVe) {
	// "bubble sort" for 3 elements
	if (eigenVa(2)>eigenVa(1)) {
		eigenVe.col(1).swap(eigenVe.col(2));
		eigenVa.row(1).swap(eigenVa.row(2));
	}
	if (eigenVa(1)>eigenVa(0)) {
		eigenVe.col(0).swap(eigenVe.col(1));
		eigenVa.row(0).swap(eigenVa.row(1));
	}
	if (eigenVa(2)>eigenVa(1)) {
		eigenVe.col(1).swap(eigenVe.col(2));
		eigenVa.row(1).swap(eigenVa.row(2));
	}
}


// Convert quaternion into rotation matrix
void quaternionToMatrix(const geometry_msgs::Quaternion& quaternion,
		Matrix3f& rotation_matrix) {
/*  Using Eigen is probably better
	const float w = quaternion.w;
	const float x = quaternion.x;
	const float y = quaternion.y;
	const float z = quaternion.z;
	const float ww = w*w;
	const float wx = w*x;
	const float wy = w*y;
	const float wz = w*z;
	const float xx = x*x;
	const float xy = x*y;
	const float xz = x*z;
	const float yy = y*y;
	const float yz = y*z;
	const float zz = z*z;
	rotation_matrix << ww+xx-yy-zz,	2*xy+2*wz,	2*xz-2*wy,\
						2*xy-2*wz,	ww-xx+yy-zz,	2*yz+2*wx,\
						2*xz+2*wy,	2*yz-2*wx,	ww-xx-yy+zz;*/
	const float w = static_cast<float>(quaternion.w);
	const float x = static_cast<float>(quaternion.x);
	const float y = static_cast<float>(quaternion.y);
	const float z = static_cast<float>(quaternion.z);
	Quaternion<float> eigen_quat(w, x, y, z);
	rotation_matrix = eigen_quat.toRotationMatrix();
}


// Convert rotation matrix into quaternion
void matrixToQuaternion(const Matrix3f& rotation_matrix,
		geometry_msgs::Quaternion& quaternion) {
	Quaternion<float> eigen_quat(rotation_matrix);
	quaternion.w = eigen_quat.w();
	quaternion.x = eigen_quat.x();
	quaternion.y = eigen_quat.y();
	quaternion.z = eigen_quat.z();
}


// Align a rotation quaternion to a normal vector; yield the rotation matrix
void alignQuaternion(const geometry_msgs::Quaternion& quaternion,
		const Vector3f& normal, Matrix3f& alignedAxes){
	Matrix3f pose_rotation;
	quaternionToMatrix(quaternion, pose_rotation);
	Vector3f z_orig = pose_rotation.col(2);
	Vector3f cross = z_orig.cross(normal);
	float w = z_orig.dot(normal);
	float theta = atan2(cross.norm(), w);
	cross *= sin(theta/2)/cross.norm();
	Quaternion<float> delta_q({static_cast<float>(cos(theta/2)), cross(0), cross(1), cross(2)});
	alignedAxes = delta_q.toRotationMatrix()*pose_rotation;
}


// Align a pose to a normal vector
void alignPose(const geometry_msgs::Pose& pose, const Vector3f& normal,
		geometry_msgs::Pose& aligned_pose) {
	Matrix3f alignedAxes;
	alignQuaternion(pose.orientation, normal, alignedAxes);
	aligned_pose.position = pose.position;
	matrixToQuaternion(alignedAxes, aligned_pose.orientation);
}


// Serialize point cloud
bool serializePointCloud(const PointCloud& point_cloud,
		const string& filename) {
	/*
	 * sparse serialization in csv
	 * 
	 * format:
	 * '#' separates features from descriptors
	 * label1,dim1,label2,dim2...
	 * v1.1,...v1.dim1,v2.1,...v2.dim2,...
	 */
	const int nb_points = point_cloud.features.cols();
	const int dim_feat = point_cloud.features.rows();
	const int nb_feat = point_cloud.featureLabels.size();
	const int dim_desc = point_cloud.descriptors.rows(); 
	const int nb_desc = point_cloud.descriptorLabels.size();
	
	FILE* point_cloud_file = fopen(filename.c_str(), "w");
	if (!point_cloud_file) {
		return false;
	}
	
	if (nb_desc) {
		// header
		for (auto &it: point_cloud.featureLabels) {
			fprintf(point_cloud_file, "%s,%d,", it.text.c_str(), static_cast<int>(it.span));
		}
		fprintf(point_cloud_file, "#,");
		for (int j=0;j<nb_desc-1; ++j) {
			fprintf(point_cloud_file, "%s,%d,", point_cloud.descriptorLabels[j].text.c_str(),
					static_cast<int>(point_cloud.descriptorLabels[j].span));
		}
		fprintf(point_cloud_file, "%s,%d\n", point_cloud.descriptorLabels[nb_desc-1].text.c_str(),
				static_cast<int>(point_cloud.descriptorLabels[nb_desc-1].span));
		// points
		for (int i=0; i<nb_points; ++i) {
			for (int j=0; j<dim_feat; ++j) {
				fprintf(point_cloud_file, "%f,", point_cloud.features(j, i));
			}
			for (int j=0; j<dim_desc-1; ++j) {
				fprintf(point_cloud_file, "%f,", point_cloud.descriptors(j, i));
			}
			fprintf(point_cloud_file, "%f\n", point_cloud.descriptors(dim_desc-1, i));
		}
	} else {
		// header
		for (int j=0;j<nb_feat-1; ++j) {
			fprintf(point_cloud_file, "%s,%d,", point_cloud.featureLabels[j].text.c_str(),
					static_cast<int>(point_cloud.featureLabels[j].span));
		}
		fprintf(point_cloud_file, "%s,%d\n", point_cloud.featureLabels[nb_feat-1].text.c_str(),
				static_cast<int>(point_cloud.featureLabels[nb_feat-1].span));
		// points
		for (int i=0; i<nb_points; ++i) {
			for (int j=0; j<dim_feat-1; ++j) {
				fprintf(point_cloud_file, "%f,", point_cloud.features(j, i));
			}
			fprintf(point_cloud_file, "%f\n", point_cloud.features(dim_feat-1, i));
		}
	}
	fclose(point_cloud_file);
	return true;
}


// Deserialize point cloud
bool deSerializePointCloud(PointCloud& point_cloud, const string& filename) {
	ifstream point_cloud_file;
	string line;
	stringstream line_stream;
	string token, name;
	istringstream token_s;
	int dim;
	int nb_feat=0;
	int nb_desc=0;
	int dim_feat=0;
	int dim_desc=0;
	PointCloud::Labels feature_labels;
	PointCloud::Labels descriptor_labels;
	
	point_cloud_file.open(filename.c_str());
	if (!point_cloud_file) {
		return false;
	}
	
	// parsing header line
	getline(point_cloud_file, line);
	line_stream.str(line);
	// feature labels
	while (getline(line_stream, token, ',')) {
		if (token=="#") {
			break;
		}
		name = token;
		getline(line_stream, token, ',');
		token_s.clear(); token_s.str(token);
		token_s >> dim;
		feature_labels.push_back({name, dim});
		nb_feat += 1;
		dim_feat += dim;
	}
	// descriptor labels
	if (token=="#") {
		while (getline(line_stream, token, ',')) {
			name = token;
			getline(line_stream, token, ',');
			token_s.clear(); token_s.str(token);
			token_s >> dim;
			descriptor_labels.push_back({name, dim});
			nb_desc += 1;
			dim_desc += dim;
		}
	}
	size_t capacity = 1<<16;
	size_t point_idx = 0;
	point_cloud = PointCloud(feature_labels, descriptor_labels, capacity);
	if (!nb_desc) {
		point_cloud.descriptors.conservativeResize(0, 0);
	}
	while (getline(point_cloud_file, line)) {
		line_stream.clear(); line_stream.str(line);
		// features
		VectorXf feature(dim_feat);
		for (int i=0; i<dim_feat; ++i) {
			getline(line_stream, token, ',');
			token_s.clear();token_s.str(token);
			token_s >> feature(i);
		}
		// descriptors
		VectorXf descriptor(dim_desc);
		if (nb_desc) {
			for (int i=0; i<dim_desc; ++i) {
				getline(line_stream, token, ',');
				token_s.clear();token_s.str(token);
				token_s >> descriptor(i);
			}
		}
		if (point_idx>=capacity) { // adaptive memory allocation
			capacity *= 2;
			point_cloud.features.conservativeResize(dim_feat, capacity);
			if (nb_desc) {
				point_cloud.descriptors.conservativeResize(dim_desc, capacity);
			}
		}
		point_cloud.features.col(point_idx) = feature;
		if (nb_desc) {
			point_cloud.descriptors.col(point_idx) = descriptor;
		}
		++point_idx;
	}
	point_cloud.features.conservativeResize(dim_feat, point_idx);
	if (nb_desc) {
		point_cloud.descriptors.conservativeResize(dim_desc, point_idx);
	}
	point_cloud_file.close();
	return true;
}


