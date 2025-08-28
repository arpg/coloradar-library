#ifndef INTERNAL_H
#define INTERNAL_H


#include "types.h"


namespace coloradar::internal {

    std::string toLower(std::string s);
    
    void checkPathExists(const std::filesystem::path& path);
    void createDirectoryIfNotExists(const std::filesystem::path& dirPath);
    std::vector<std::filesystem::path> readArrayDirectory(
        const std::filesystem::path& directoryPath,
        const std::string& filePrefix = "",
        const std::string& fileExtension = "",
        const int& arraySize = -1
    );

    template<coloradar::Pcl4dPointType PointT> PointT makePoint(const float& x, const float& y, const float& z, const float& i);
    template<coloradar::PointType PointT> PointT makePoint(const float& x, const float& y, const float& z, const float& i);

    template<coloradar::PclPoseType PoseT>
    PoseT makePose(const typename coloradar::PoseTraits<PoseT>::TranslationType& translation, const typename coloradar::PoseTraits<PoseT>::RotationType& rotation);
    template<coloradar::OctoPoseType PoseT>
    PoseT makePose(const typename coloradar::PoseTraits<PoseT>::TranslationType& translation, const typename coloradar::PoseTraits<PoseT>::RotationType& rotation);

    template<coloradar::PclPoseType PoseT> Eigen::Vector3f toEigenTrans(const PoseT& pose);
    template<coloradar::PclPoseType PoseT> Eigen::Quaternionf toEigenQuat(const PoseT& pose);
    template<coloradar::OctoPoseType PoseT> Eigen::Vector3f toEigenTrans(const PoseT& pose);
    template<coloradar::OctoPoseType PoseT> Eigen::Quaternionf toEigenQuat(const PoseT& pose);

    template<typename TransT> TransT fromEigenTrans(const Eigen::Vector3f& r);
    template<typename RotationT> RotationT fromEigenQuat(const Eigen::Quaternionf& r);
    template<> octomath::Vector3 fromEigenTrans(const Eigen::Vector3f& r);
    template<> octomath::Quaternion fromEigenQuat(const Eigen::Quaternionf& r);

    template<coloradar::PclPoseType PoseT> Eigen::Affine3f toEigenPose(const PoseT& pose);
    template<coloradar::PclPoseType PoseT> PoseT fromEigenPose(const Eigen::Affine3f& pose);
    template<coloradar::OctoPoseType PoseT> Eigen::Affine3f toEigenPose(const PoseT& pose);
    template<coloradar::OctoPoseType PoseT> PoseT fromEigenPose(const Eigen::Affine3f& pose);

    template<coloradar::PointType PointT, coloradar::CloudType CloudT> std::shared_ptr<CloudT> readLidarPointCloud(const std::filesystem::path& binPath);

    template<typename PointT, typename CloudT> void filterFov(std::shared_ptr<CloudT>& cloud, const float& horizontalFov, const float& verticalFov, const float& range);

    bool parseBoolYamlKey(const YAML::Node &nodeValue, bool defaultValue);
    int parseIntYamlKey(const YAML::Node &nodeValue, int defaultValue);
    float parseFloatYamlKey(const YAML::Node &nodeValue, float defaultValue);

    template<coloradar::PclCloudType CloudT> std::vector<float> flattenLidarCloud(const std::shared_ptr<CloudT>& cloud, bool collapseElevation, bool removeIntensity);
    template<coloradar::PclCloudType CloudT> std::vector<float> flattenRadarCloud(const std::shared_ptr<CloudT>& cloud, const int numElevationBins, const bool hasDoppler);

    void saveVectorToHDF5(const std::string& name, const H5::H5File& file, const std::vector<double>& vec);
    void savePoseToHDF5(const std::string& name, const H5::H5File& file, const Eigen::Affine3f& pose);
    void savePosesToHDF5(const std::string& name, const H5::H5File& file, const std::vector<Eigen::Affine3f>& poses);
    void saveCloudToHDF5(const std::string& name, const H5::H5File& file, const std::vector<float>& flatCloud, const hsize_t& numDims);
    void saveCloudsToHDF5(const std::string& name, const H5::H5File& file, const std::vector<float>& flatClouds, const hsize_t& numFrames, const std::vector<hsize_t>& cloudSizes, const hsize_t& numDims);
    void saveDatacubesToHDF5(const std::string& name, const H5::H5File& file, const std::vector<int16_t>& flatDatacubes, const hsize_t& numFrames, const hsize_t datacubeSize);
    void saveHeatmapsToHDF5(const std::string& name, const H5::H5File& file, const std::vector<float>& flatHeatmaps, const hsize_t& numFrames, const int numAzimuthBins, const int numRangeBins, const int numElevationBins, const bool hasDoppler);

    std::vector<double> readH5Timestamps(const H5::H5File& file, const std::string& datasetName);
    Eigen::Affine3f readH5Pose(const H5::H5File& file, const std::string& datasetName);
    std::vector<Eigen::Affine3f> readH5Poses(const H5::H5File& file, const std::string& datasetName);
    std::vector<std::shared_ptr<std::vector<int16_t>>> readH5Datacubes(const H5::H5File& file, const std::string& datasetName);
    std::vector<std::shared_ptr<std::vector<float>>> readH5Heatmaps(const H5::H5File& file, const std::string& datasetName);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> readH5LidarClouds(const H5::H5File& file, const std::string& baseName);
    pcl::PointCloud<pcl::PointXYZI>::Ptr readH5SingleCloud(const H5::H5File& file, const std::string& datasetName);
}

#include "hpp/internal.hpp"


#endif
