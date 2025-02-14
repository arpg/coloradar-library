#ifndef COLORADAR_RUN_H
#define COLORADAR_RUN_H

#include "utils.h"

#include <H5Cpp.h>


namespace coloradar {

class ColoradarPlusRun {
protected:
    std::filesystem::path runDirPath_;
    std::filesystem::path posesDirPath_;
    std::filesystem::path imuDirPath_;
    std::filesystem::path lidarScansDirPath_;
    std::filesystem::path lidarCloudsDirPath_;
    std::filesystem::path lidarMapsDirPath_;
    std::filesystem::path cascadeScansDirPath_;
    std::filesystem::path cascadeCubesDirPath_;
    std::filesystem::path cascadeHeatmapsDirPath_;
    std::filesystem::path cascadeCloudsDirPath_;

    std::vector<double> poseTimestamps_;
    std::vector<double> imuTimestamps_;
    std::vector<double> lidarTimestamps_;
    std::vector<double> cascadeCubeTimestamps_;
    std::vector<double> cascadeTimestamps_;
    std::vector<double> readTimestamps(const std::filesystem::path& path);

    RadarConfig* cascadeConfig_;
    std::vector<int16_t> getDatacube(const std::filesystem::path& binFilePath, RadarConfig* config);
    std::vector<float> getHeatmap(const std::filesystem::path& binFilePath, RadarConfig* config);
    void createRadarPointclouds(RadarConfig* config, const std::filesystem::path& heatmapDirPath, const std::filesystem::path& pointcloudDirPath, const float& intensityThresholdPercent = 0.0);
    pcl::PointCloud<RadarPoint> getRadarPointcloud(const std::filesystem::path& binFilePath, RadarConfig* config, const float& intensityThresholdPercent = 0.0);

    void saveVectorToHDF5(const std::string& name, H5::H5File& file, const std::vector<double>& vec);
    void savePosesToHDF5(const std::string& name, H5::H5File& file, const std::vector<Eigen::Affine3f>& poses);
    void saveHeatmapToHDF5(const int& idx, H5::H5File& file, const std::vector<float>& heatmap, const int& numAzimuthBins, const int& numElevationBins, const int& numRangeBins, const int& numDims);
    void saveRadarCloudToHDF5(const int& idx, H5::H5File& file, const pcl::PointCloud<coloradar::RadarPoint>& cloud, bool collapseElevation = false);
    void saveLidarCloudToHDF5(const std::string& name, H5::H5File& file, const pcl::PointCloud<pcl::PointXYZI>& cloud, bool includeIntensity = false, bool collapseElevation = false);

public:
    const std::string name;

    ColoradarPlusRun(const std::filesystem::path& runPath, RadarConfig* cascadeRadarConfig);

    const std::vector<double>& poseTimestamps() const;
    const std::vector<double>& imuTimestamps() const;
    const std::vector<double>& lidarTimestamps() const;
    const std::vector<double>& cascadeCubeTimestamps() const;
    const std::vector<double>& cascadeTimestamps() const;

    std::vector<int16_t> getCascadeDatacube(const std::filesystem::path& binFilePath);
    std::vector<int16_t> getCascadeDatacube(const int& cubeIdx);
    std::vector<float> getCascadeHeatmap(const std::filesystem::path& binFilePath);
    std::vector<float> getCascadeHeatmap(const int& hmIdx);

    void createCascadePointclouds(const float& intensityThresholdPercent = 0.0);
    pcl::PointCloud<RadarPoint> getCascadePointcloud(const std::filesystem::path& binFilePath, const float& intensityThresholdPercent = 0.0);
    pcl::PointCloud<RadarPoint> getCascadePointcloud(const int& cloudIdx, const float& intensityThresholdPercent = 0.0);

    template<PoseType PoseT> std::vector<PoseT> getPoses();
    template<PoseType PoseT> std::vector<PoseT> interpolatePoses(const std::vector<PoseT>& poses, const std::vector<double>& poseTimestamps, const std::vector<double>& targetTimestamps);

    template<PclCloudType CloudT> CloudT getLidarPointCloud(const std::filesystem::path& binPath);
    template<OctomapCloudType CloudT> CloudT getLidarPointCloud(const std::filesystem::path& binPath);
    template<CloudType CloudT> CloudT getLidarPointCloud(const int& cloudIdx);

    octomap::OcTree buildLidarOctomap(
        const double& mapResolution,
        const float& lidarTotalHorizontalFov,
        const float& lidarTotalVerticalFov,
        const float& lidarMaxRange,
        Eigen::Affine3f baseToLidarTransform = Eigen::Affine3f::Identity()
    );
    void saveLidarOctomap(const octomap::OcTree& tree);
    pcl::PointCloud<pcl::PointXYZI> readLidarOctomap();
    void createLidarOctomap(
        const double& mapResolution,
        const float& lidarTotalHorizontalFov,
        const float& lidarTotalVerticalFov,
        const float& lidarMaxRange,
        Eigen::Affine3f baseToLidarTransform = Eigen::Affine3f::Identity()
    );

    void sampleMapFrames(
        const float& horizontalFov,
        const float& verticalFov,
        const float& range,
        const Eigen::Affine3f& baseToSensorTransform = Eigen::Affine3f::Identity(),
        std::vector<Eigen::Affine3f> basePoses = {}
    );
    pcl::PointCloud<pcl::PointXYZI> readMapFrame(const int& frameIdx);

//    std::filesystem::path exportToFile(
//        std::filesystem::path destination = "",
//
//        const bool& includeCascadeHeatmaps = false,
//        const bool& includeCascadePointclouds = false,
//        const int& cascadeAzimuthMaxBin = -1,
//        const int& cascadeElevationMaxBin = -1,
//        const int& cascadeRangeMaxBin = -1,
//        const bool& removeCascadeDopplerDim = false,
//        const bool& collapseCascadeElevation = false,
//        const int& collapseCascadeElevationMinZ = -100,
//        const int& collapseCascadeElevationMaxZ = 100,
//        const float& cascadeCloudIntensityThresholdPercent = 0,
//
//        const bool& includeLidarFrames = false,
//        const float& lidarFrameTotalHorizontalFov = 360,
//        const float& lidarFrameTotalVerticalFov = 180,
//        const float& lidarFrameMaxRange = 100,
//        const bool& collapseLidarFrameElevation = false,
//        const float& collapseLidarFrameElevationMinZ = -100,
//        const float& collapseLidarFrameElevationMaxZ = 100,
//
//        const bool& includeLidarMap = false,
//        const bool& collapseMapElevation = false,
//        const float& collapseMapElevationMinZ = -100,
//        const float& collapseMapElevationMaxZ = 100,
//
//        const bool& includeMapFrames = false,
//        const float& mapSampleTotalHorizontalFov = 360,
//        const float& mapSampleTotalVerticalFov = 180,
//        const float& mapSampleMaxRange = 100,
//        const Eigen::Affine3f& mapSamplingPreTransform = Eigen::Affine3f::Identity(),
//        std::vector<Eigen::Affine3f> mapSamplingPoses = {},
//        const bool& collapseMapSampleElevation = false,
//        const float& collapseMapSampleElevationMinZ = -100,
//        const float& collapseMapSampleElevationMaxZ = 100,
//
//        const bool& removeLidarIntensity = false,
//
//        const bool& includeTruePoses = true,
//        const bool& includeCascadePoses = true,
//        const bool& includeLidarPoses = true,
//        const bool& includeTrueTimestamps = true,
//        const bool& includeCascadeTimestamps = true,
//        const bool& includeLidarTimestamps = true
//    );

    virtual ~ColoradarPlusRun() = default;
};


class ColoradarRun : public ColoradarPlusRun {
protected:
    std::filesystem::path singleChipScansDirPath_;
    std::filesystem::path singleChipCubesDirPath_;
    std::filesystem::path singleChipHeatmapsDirPath_;
    std::filesystem::path singleChipCloudsDirPath_;

    std::vector<double> singleChipCubeTimestamps_;
    std::vector<double> singleChipTimestamps_;

    RadarConfig* singleChipConfig_;

public:
    ColoradarRun(const std::filesystem::path& runPath, RadarConfig* cascadeRadarConfig, RadarConfig* singleChipRadarConfig);

    const std::vector<double>& singleChipCubeTimestamps() const;
    const std::vector<double>& singleChipTimestamps() const;

    std::vector<int16_t> getSingleChipDatacube(const std::filesystem::path& binFilePath);
    std::vector<int16_t> getSingleChipDatacube(const int& cubeIdx);
    std::vector<float> getSingleChipHeatmap(const std::filesystem::path& binFilePath);
    std::vector<float> getSingleChipHeatmap(const int& hmIdx);
    pcl::PointCloud<RadarPoint> getSingleChipPointcloud(const std::filesystem::path& binFilePath, const float& intensityThresholdPercent = 0.0);
    pcl::PointCloud<RadarPoint> getSingleChipPointcloud(const int& cloudIdx, const float& intensityThresholdPercent = 0.0);
    // void createSingleChipPointclouds(const float& intensityThresholdPercent = 0.0);
};

}

#include "coloradar_run.hpp"

#endif
