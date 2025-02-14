#ifndef TYPES_H
#define TYPES_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <octomap/octomap.h>


namespace coloradar {

// Point Constraints
template<typename T>
concept Pcl3dPointType = std::is_base_of_v<pcl::PointXYZ, T>;

//template<typename T>
//concept Pcl4dPointType = std::is_base_of_v<pcl::PointXYZI, T>;
template<typename T>
concept Pcl4dPointType = requires(T point) {
    { point.x } -> std::convertible_to<float>;
    { point.y } -> std::convertible_to<float>;
    { point.z } -> std::convertible_to<float>;
    { point.intensity } -> std::convertible_to<float>;
};

template<typename T>
concept PclPointType = Pcl3dPointType<T> || Pcl4dPointType<T>;

template<typename T>
concept OctomapPointType = std::is_base_of_v<octomap::point3d, T>;

template<typename T>
concept PointType = PclPointType<T> || OctomapPointType<T>;


// Cloud Constraints
template<typename T>
concept PclCloudType = requires { typename T::PointType; } && std::is_base_of_v<pcl::PointCloud<typename T::PointType>, T> && PclPointType<typename T::PointType>;

template<typename T>
concept OctomapCloudType = std::is_base_of_v<octomap::Pointcloud, T>;

template<typename T>
concept CloudType = PclCloudType<T> || OctomapCloudType<T>;


// Pose Constraints
template<typename T>
concept PclPoseType = std::is_base_of_v<Eigen::Affine3f, T>;

template<typename T>
concept OctoPoseType = std::is_base_of_v<octomath::Pose6D, T>;

template<typename T>
concept PoseType = PclPoseType<T> || OctoPoseType<T>;


// Pose Traits
template<PoseType PoseT>
struct PoseTraits;

template<>
struct PoseTraits<octomath::Pose6D> {
    using TranslationType = octomath::Vector3;
    using RotationType = octomath::Quaternion;
};
template<>
struct PoseTraits<Eigen::Affine3f> {
    using TranslationType = Eigen::Vector3f;
    using RotationType = Eigen::Quaternionf;
};

}

#endif
