/**
 * @file internal_utils.h
 * @brief Utility functions used by the main modules of the library.
 *
 * This file contains helper functions for string manipulation, file and directory handling,
 * point and pose conversions, Eigen transformations, point cloud reading and filtering, 
 * and YAML key parsing. These are intended for internal library use.
 *
 * @date 2025-10-07
 * @author Anna Zavei-Boroda
 */


#ifndef INTERNAL_H
#define INTERNAL_H


#include "types.h"


namespace coloradar::internal {
// STRING MANIPULATION
    /**
    * @brief Converts a string to lowercase.
    * @param s The input string.
    * @return A lowercase copy of the input string.
    */
    std::string toLower(std::string s);

    /**
    * @brief Extracts the first integer found in a string.
    * @param input The input string.
    * @return The first integer found, or std::nullopt if none exists.
    */    
    std::optional<int> extractFirstInt(const std::string& input);
    

// FILE AND DIRECTORY HANDLING
    /**
    * @brief Checks whether a filesystem path exists.
    * @param path The path to check.
    * @throws std::filesystem::filesystem_error if the path does not exist.
    */  
    void checkPathExists(const std::filesystem::path& path);

    /**
    * @brief Ensures that a directory exists.
    * Creates the directory specified by `dirPath` along with any necessary parent directories if they do not already exist. 
    * Does nothing if the directory already exists.
    *
    * @param dirPath The directory path to create.
    * @throws std::filesystem::filesystem_error if the directory cannot be created.
    */
    void createDirectoryIfNotExists(const std::filesystem::path& dirPath);
    
    /**
    * @brief Reads files from a directory and returns them as an ordered array of paths.
    * This function scans the specified directory for files that match a given prefix and extension.
    * It expects file names to follow the pattern: `<filePrefix><index><fileExtension>`, where `<index>` is an integer starting from 0.
    * Files are returned in a vector sorted by the index.
    * If `arraySize` is provided (>= 0), exactly that number of files is expected.
    * If `arraySize` is -1, all matching files in the directory are used. 
    * The function throws a `std::runtime_error` if:
    *   - The number of matching files does not equal the expected `arraySize`.
    *   - Any file index in the expected range is missing.
    * Malformed file names (non-integer indices) are ignored.
    *
    * @param directoryPath Path to the directory to scan.
    * @param filePrefix Optional prefix that file names must start with. Default is empty string (matches all).
    * @param fileExtension Optional file extension to filter by (with or without leading dot). Default is empty string (matches all).
    * @param arraySize Expected number of files in the array. Set to -1 to use all matching files.
    * @return A vector of `std::filesystem::path` objects, ordered by file index.
    * @throws std::runtime_error if the number of files found does not match `arraySize`, or if any expected indexed file is missing.
    */
    std::vector<std::filesystem::path> readArrayDirectory(
        const std::filesystem::path& directoryPath,
        const std::string& filePrefix = "",
        const std::string& fileExtension = "",
        const int& arraySize = -1
    );


// YAML PARSING
    /**
    * @brief Parses a YAML node as a boolean. Returns the node value as a bool. If the node is missing, returns the provided default value.
    * @param nodeValue The YAML node to parse.
    * @param defaultValue Value to return if the node is missing.
    * @return The boolean value from the YAML node or defaultValue.
    * @throws std::runtime_error if the node exists but is not a valid bool.
    */
    bool parseBoolYamlKey(const YAML::Node &nodeValue, bool defaultValue);

    /**
    * @brief Parses a YAML node as an integer. Returns the node value as an int. If the node is missing, returns the provided default value.
    * @param nodeValue The YAML node to parse.
    * @param defaultValue Value to return if the node is missing.
    * @return The integer value from the YAML node or defaultValue.
    * @throws std::runtime_error if the node exists but is not a valid int.
    */
    int parseIntYamlKey(const YAML::Node &nodeValue, int defaultValue);
    
    /**
    * @brief Parses a YAML node as a float. Returns the node value as a float. If the node is missing, returns the provided default value.
    * @param nodeValue The YAML node to parse.
    * @param defaultValue Value to return if the node is missing.
    * @return The float value from the YAML node or defaultValue.
    * @throws std::runtime_error if the node exists but is not a valid float.
    */
    float parseFloatYamlKey(const YAML::Node &nodeValue, float defaultValue);


// GEOMETRIC OBJECT MANIPULATION
    /**
    * @brief Constructs a 4D point object of a PCL type PointT.
    * Creates a point with X, Y, Z coordinates and an intensity value `i`.
    * Intended for PCL point types that store intensity or extra fields (4D points).
    *
    * @tparam PointT The PCL 4D point type (e.g., pcl::PointXYZI).
    * @param x X-coordinate of the point.
    * @param y Y-coordinate of the point.
    * @param z Z-coordinate of the point.
    * @param i Intensity value of the point.
    * @return A PointT object with the specified coordinates and intensity.
    */
    template<coloradar::Pcl4dPointType PointT> 
    PointT makePoint(const float& x, const float& y, const float& z, const float& i);
 
    /**
    * @brief Constructs a 3D point object of type PointT.
    * Creates a point with X, Y, Z coordinates. Ignores the intensity value.
    * Intended for generic point types that do not store intensity.
    *
    * @tparam PointT The point type (e.g., pcl::PointXYZ or custom 3D point types).
    * @param x X-coordinate of the point.
    * @param y Y-coordinate of the point.
    * @param z Z-coordinate of the point.
    * @param i Ignored; included for API consistency with 4D point constructor.
    * @return A PointT object with the specified coordinates.
    */
    template<coloradar::PointType PointT> 
    PointT makePoint(const float& x, const float& y, const float& z, const float& i);
 
    /**
    * @brief Constructs a pose object of type PoseT (PCL-style) from translation and rotation components.
    * @tparam PoseT The PCL pose type (e.g., Eigen::Isometry3f).
    * @param translation The translation vector.
    * @param rotation The rotation component (quaternion).
    * @return A PoseT object representing the pose with the specified translation and rotation.
    */
    template<coloradar::PclPoseType PoseT>
    PoseT makePose(const typename coloradar::PoseTraits<PoseT>::TranslationType& translation, const typename coloradar::PoseTraits<PoseT>::RotationType& rotation);
 
    /**
    * @brief Constructs a pose object of type PoseT (Octomap-style) from translation and rotation components.
    * @tparam PoseT The Octomap-compatible pose type.
    * @param translation The translation vector.
    * @param rotation The rotation component (quaternion).
    * @return A PoseT object representing the pose with the specified translation and rotation.
    */
    template<coloradar::OctoPoseType PoseT>
    PoseT makePose(const typename coloradar::PoseTraits<PoseT>::TranslationType& translation, const typename coloradar::PoseTraits<PoseT>::RotationType& rotation);

    /**
    * @brief Extracts an Eigen translation vector from a PCL-style pose object.
    * @tparam PoseT The PCL pose type (e.g., Eigen::Affine3f).
    * @param pose The input pose.
    * @return Eigen::Vector3f containing the translation (x, y, z).
    */
    template<coloradar::PclPoseType PoseT>
    Eigen::Vector3f toEigenTrans(const PoseT& pose);

    /**
    * @brief Extracts an Eigen translation vector from a Octomap-style pose object.
    * @tparam PoseT The Octomap-compatible pose type.
    * @param pose The input pose.
    * @return Eigen::Vector3f containing the translation (x, y, z).
    */
    template<coloradar::OctoPoseType PoseT>
    Eigen::Vector3f toEigenTrans(const PoseT& pose);

    /**
    * @brief Converts an Eigen translation vector to the specified translation type.
    * @tparam TransT The output translation type.
    * @param r The input Eigen::Vector3f translation.
    * @return Translation object of type TransT.
    */
    template<typename TransT>
    TransT fromEigenTrans(const Eigen::Vector3f& r);

    /**
    * @brief An extension of the template with a specific implementation for octomath::Vector3.
    * @param r The input Eigen::Vector3f translation.
    * @return octomath::Vector3 representing the translation.
    */
    template<>
    octomath::Vector3 fromEigenTrans(const Eigen::Vector3f& r);
    
    /**
    * @brief Extracts an Eigen quaternion object from a PCL-style pose object.
    * @tparam PoseT The PCL pose type (e.g., Eigen::Affine3f).
    * @param pose The input pose.
    * @return Eigen::Quaternionf representing the rotation.
    */
    template<coloradar::PclPoseType PoseT>
    Eigen::Quaternionf toEigenQuat(const PoseT& pose);
    
    /**
    * @brief Extracts an Eigen quaternion object from a Octomap-style pose object.
    * @tparam PoseT The Octomap-compatible pose type.
    * @param pose The input pose.
    * @return Eigen::Quaternionf representing the rotation.
    */
    template<coloradar::OctoPoseType PoseT>
    Eigen::Quaternionf toEigenQuat(const PoseT& pose);

    /**
    * @brief Converts an Eigen quaternion to the specified translation type.
    * @tparam RotationT The output rotation type.
    * @param r The input Eigen::Quaternionf rotation.
    * @return Rotation object of type RotationT.
    */
    template<typename RotationT>
    RotationT fromEigenQuat(const Eigen::Quaternionf& r);
    
    /**
    * @brief An extension of the template with a specific implementation for octomath::Quaternion.
    * @param r The input Eigen::Quaternionf rotation.
    * @return octomath::Quaternion representing the rotation.
    */
    template<>
    octomath::Quaternion fromEigenQuat(const Eigen::Quaternionf& r);
 
    /**
    * @brief Converts a PCL-style pose object to an Eigen::Affine3f pose.
    * @tparam PoseT The PCL pose type (e.g., Eigen::Affine3f).
    * @param pose The input pose.
    * @return Eigen::Affine3f representing the pose.
    */
    template<coloradar::PclPoseType PoseT>
    Eigen::Affine3f toEigenPose(const PoseT& pose);

    /**
    * @brief Converts an Octomap-style pose object to an Eigen::Affine3f pose.
    * @tparam PoseT The Octomap-compatible pose type.
    * @param pose The input pose.
    * @return Eigen::Affine3f representing the pose.
    */
    template<coloradar::OctoPoseType PoseT>
    Eigen::Affine3f toEigenPose(const PoseT& pose);
    
    /**
    * @brief Converts an Eigen::Affine3f pose to the specified PCL-style pose object.
    * @tparam PoseT The PCL pose type (e.g., Eigen::Affine3f).
    * @param pose The input Eigen::Affine3f.
    * @return PoseT object representing the pose.
    */
    template<coloradar::PclPoseType PoseT>
    PoseT fromEigenPose(const Eigen::Affine3f& pose);
    
    /**
    * @brief Converts an Eigen::Affine3f pose to the specified Octomap-style pose object.
    * @tparam PoseT The Octomap-compatible pose type.
    * @param pose The input Eigen::Affine3f.
    * @return PoseT object representing the pose.
    */
    template<coloradar::OctoPoseType PoseT>
    PoseT fromEigenPose(const Eigen::Affine3f& pose);
 

// POINT CLOUD HANDLING
    /**
    * @brief Reads a binary point cloud file into a point cloud object.
    * Loads a point cloud from a binary file, where each point is stored as four consecutive `float` values in the order: X, Y, Z, intensity (I).
    * The function automatically determines the number of points based on the file size.
    *
    * @tparam PointT The point type (e.g., pcl::PointXYZI) to store in the cloud.
    * @tparam CloudT The cloud container type (e.g., pcl::PointCloud<PointT>).
    * @param binPath Path to the binary lidar point cloud file.
    * @return A shared pointer to a CloudT object containing the loaded points.
    * @throws std::filesystem::filesystem_error if the file path does not exist.
    * @throws std::runtime_error if the file cannot be opened, if reading fails, or if the resulting point cloud is empty.
    */
    template<coloradar::PointType PointT, coloradar::CloudType CloudT> std::shared_ptr<CloudT> 
    readLidarPointCloud(const std::filesystem::path& binPath);

    /**
    * @brief Filters a point cloud in place based on horizontal and vertical field of view (FOV) and range.
    * Removes points outside the specified horizontal FOV, vertical FOV, or maximum range.
    *
    * @tparam PointT The point type (e.g., pcl::PointXYZI).
    * @tparam CloudT The cloud container type (e.g., pcl::PointCloud<PointT>).
    * @param cloud Shared pointer to the input point cloud, which will be modified in place.
    * @param horizontalFov Horizontal field of view in degrees (0 < horizontalFov <= 360).
    * @param verticalFov Vertical field of view in degrees (0 < verticalFov <= 180).
    * @param range Maximum distance from the origin to include points (range > 0).
    * @throws std::runtime_error if any of the FOV or range parameters are invalid.
    */
    template<typename PointT, typename CloudT>
    void filterFov(std::shared_ptr<CloudT>& cloud, const float& horizontalFov, const float& verticalFov, const float& range);

}

#include "internal_utils/internal_utils.hpp"


#endif
