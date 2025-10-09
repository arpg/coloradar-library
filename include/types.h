/**
 * @file types.h
 * @brief Defines C++ concepts and traits for geometric data types.
 *
 * This file establishes a set of type constraints (concepts) for point clouds, points, and poses
 * from different libraries (PCL, OctoMap). It also provides traits for accessing
 * properties of these types in a generic way, ensuring interoperability within the library.
 *
 * @date 2025-10-08
 * @author Anna Zavei-Boroda
 */

 
 #ifndef TYPES_H
 #define TYPES_H
 
 #include "definitions.h"
 
 
 namespace coloradar {
 
 // Point Constraints
 /**
  * @brief Concept for PCL point types with at least X, Y, and Z coordinates.
  * @tparam T The type to check.
  */
 template<typename T>
 concept Pcl3dPointType = std::is_base_of_v<pcl::PointXYZ, T>;
 
 /**
  * @brief Concept for PCL-like point types with X, Y, Z, and intensity fields.
  * @tparam T The type to check.
  */
 template<typename T>
 concept Pcl4dPointType = requires(T point) {
     { point.x } -> std::convertible_to<float>;
     { point.y } -> std::convertible_to<float>;
     { point.z } -> std::convertible_to<float>;
     { point.intensity } -> std::convertible_to<float>;
 };
 
 /**
  * @brief Concept for any PCL-compatible point type (3D or 4D).
  * @tparam T The type to check.
  */
 template<typename T>
 concept PclPointType = Pcl3dPointType<T> || Pcl4dPointType<T>;
 
 /**
  * @brief Concept for OctoMap point types.
  * @tparam T The type to check.
  */
 template<typename T>
 concept OctomapPointType = std::is_base_of_v<octomap::point3d, T>;
 
 /**
  * @brief Concept for any supported point type from PCL or OctoMap.
  * @tparam T The type to check.
  */
 template<typename T>
 concept PointType = PclPointType<T> || OctomapPointType<T>;
 
 
 // Cloud Constraints
 /**
  * @brief Concept for PCL point cloud types.
  * @tparam T The type to check.
  */
 template<typename T>
 concept PclCloudType = requires { typename T::PointType; } && std::is_base_of_v<pcl::PointCloud<typename T::PointType>, T> && PclPointType<typename T::PointType>;
 
 /**
  * @brief Concept for OctoMap point cloud types.
  * @tparam T The type to check.
  */
 template<typename T>
 concept OctomapCloudType = std::is_base_of_v<octomap::Pointcloud, T>;
 
 /**
  * @brief Concept for any supported point cloud type from PCL or OctoMap.
  * @tparam T The type to check.
  */
 template<typename T>
 concept CloudType = PclCloudType<T> || OctomapCloudType<T>;
 
 
 // Pose Constraints
 /**
  * @brief Concept for PCL-compatible pose types (Eigen-based).
  * @tparam T The type to check.
  */
 template<typename T>
 concept PclPoseType = std::is_base_of_v<Eigen::Affine3f, T>;
 
 /**
  * @brief Concept for OctoMap pose types.
  * @tparam T The type to check.
  */
 template<typename T>
 concept OctoPoseType = std::is_base_of_v<octomath::Pose6D, T>;
 
 /**
  * @brief Concept for any supported pose type from PCL or OctoMap.
  * @tparam T The type to check.
  */
 template<typename T>
 concept PoseType = PclPoseType<T> || OctoPoseType<T>;
 
 
 // Pose Traits
 /**
  * @brief Provides associated types for a given pose type.
  * This struct template is specialized for concrete pose types to allow generic
  * access to their corresponding translation and rotation types.
  * @tparam PoseT The pose type.
  */
 template<PoseType PoseT>
 struct PoseTraits;
 
 /**
  * @brief Specialization of PoseTraits for octomath::Pose6D.
  */
 template<>
 struct PoseTraits<octomath::Pose6D> {
     using TranslationType = octomath::Vector3;
     using RotationType = octomath::Quaternion;
 };
 /**
  * @brief Specialization of PoseTraits for Eigen::Affine3f.
  */
 template<>
 struct PoseTraits<Eigen::Affine3f> {
     using TranslationType = Eigen::Vector3f;
     using RotationType = Eigen::Quaternionf;
 };
 
 }
 
 #endif
