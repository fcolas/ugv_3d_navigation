#ifndef TENSOR_UTILS_H
#define TENSOR_UTILS_H

#include <Eigen/Eigen>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <pointmatcher/PointMatcher.h>
#include <string>

using namespace Eigen;
using namespace std;

// point cloud we use here
typedef PointMatcher<float>::DataPoints PointCloud;


// FIXME homogeneize interfaces (either return value of void)

//! Convert ROS pose to Vector3f
Vector3f poseToVector(const geometry_msgs::Pose& pose);

//! Convert Vector3f to ROS Point
geometry_msgs::Point vectorToPosition(const Vector3f& vector);

//! Convert ROS quaternion into direction vector
Vector3f quaternionToDirection(const geometry_msgs::Quaternion& quaternion);

//! Compute radius (least squares fitting)
void getLeastSquaresRadius(const MatrixXf& points, float* R, float* res,
		float* max_res);

//! Compute radius (last point)
void getLastPointRadius(const MatrixXf& points, float* R, float* res,
		float* max_res);

//! Convert position, normal, direction to full ROS pose
geometry_msgs::Pose vectorsToPose(const Vector3f& position,
		const Vector3f& normal, const Vector3f& direction);

//! Sort eigen vectors and values in descending order
void sortTensor(Vector3f& eigenVa, Matrix3f& eigenVe);

//! Convert quaternion into rotation matrix
void quaternionToMatrix(const geometry_msgs::Quaternion& quaternion,
		Matrix3f& rotation_matrix);

//! Convert rotation matrix into quaternion
void matrixToQuaternion(const Matrix3f& rotation_matrix,
		geometry_msgs::Quaternion& quaternion);

//! Align a rotation quaternion to a normal vector; yield the rotation matrix
void alignQuaternion(const geometry_msgs::Quaternion& quaternion,
		const Vector3f& normal, Matrix3f& alignedAxes);

//! Align a pose to a normal vector
void alignPose(const geometry_msgs::Pose& pose, const Vector3f& normal,
		geometry_msgs::Pose& aligned_pose);

//! Serialize point cloud
bool serializePointCloud(const PointCloud& point_cloud, const string& filename);

//! Deserialize point cloud
bool deSerializePointCloud(PointCloud& point_cloud, const string& filename);

#endif
