#ifndef COLORADAR_PLUS_RUN_H
#define COLORADAR_PLUS_RUN_H

#include "run/base_run.h"


namespace coloradar {

class ColoradarPlusRun : public Run {
protected:
    // ATTRIBUTES
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
    

    // src/run/coloradar_plus_run.cpp
    std::vector<double> readTimestamps(const std::filesystem::path& path) const;
    std::vector<Eigen::Affine3f> readPoses(const std::filesystem::path& path) const;
    std::shared_ptr<std::vector<int16_t>> readDatacube(const std::filesystem::path& binFilePath, const std::shared_ptr<RadarConfig>& config) const;
    std::shared_ptr<std::vector<float>> readHeatmap(const std::filesystem::path& binFilePath, const std::shared_ptr<RadarConfig>& config) const;
    pcl::PointCloud<RadarPoint>::Ptr readRadarPointcloud(
        std::shared_ptr<RadarConfig> config,
        const std::filesystem::path& binFilePath,
        const double intensityThreshold = 0.0
    ) const;
    void createRadarPointclouds(
        const std::shared_ptr<RadarConfig>& config, 
        const std::filesystem::path& heatmapDirPath, 
        const std::filesystem::path& pointcloudDirPath, 
        const double intensityThreshold = 0.0
    );
    
public:
    // src/run/coloradar_plus_run.cpp
    ColoradarPlusRun(const std::filesystem::path& runPath, std::shared_ptr<RadarConfig> cascadeRadarConfig = nullptr);
    virtual ~ColoradarPlusRun() = default;

    virtual pcl::PointCloud<pcl::PointXYZI>::Ptr getLidarPointCloud(const int cloudIdx) const override;
    virtual std::shared_ptr<std::vector<int16_t>> getCascadeDatacube(const int cubeIdx) const override;
    virtual std::shared_ptr<std::vector<float>> getCascadeHeatmap(const int hmIdx) const override;
    virtual pcl::PointCloud<RadarPoint>::Ptr getCascadePointcloud(const int cloudIdx, const double intensityThreshold = 0.0) const override;
    virtual pcl::PointCloud<pcl::PointXYZI>::Ptr getLidarOctomap() const override;
    virtual pcl::PointCloud<pcl::PointXYZI>::Ptr getMapSample(const int sampleIdx) const override;
    virtual void saveMapSample(const int sampleIdx, const pcl::PointCloud<pcl::PointXYZI>::Ptr sample) override;

    std::shared_ptr<std::vector<int16_t>> getCascadeDatacube(const std::filesystem::path& binFilePath) const;
    std::shared_ptr<std::vector<float>> getCascadeHeatmap(const std::filesystem::path& binFilePath) const;
    pcl::PointCloud<RadarPoint>::Ptr getCascadePointcloud(const std::filesystem::path& binFilePath, const double intensityThreshold = 0.0) const;
    void createCascadePointclouds(const double intensityThreshold = 0.0);

    octomap::OcTree buildLidarOctomap(
        const double& mapResolution,
        const float& lidarTotalHorizontalFov,
        const float& lidarTotalVerticalFov,
        const float& lidarMaxRange,
        Eigen::Affine3f baseToLidarTransform = Eigen::Affine3f::Identity()
    );
    void createLidarOctomap(
        const double& mapResolution,
        const float& lidarTotalHorizontalFov,
        const float& lidarTotalVerticalFov,
        const float& lidarMaxRange,
        Eigen::Affine3f baseToLidarTransform = Eigen::Affine3f::Identity()
    );
    void createMapSamples(
        const float horizontalFov,
        const float verticalFov,
        const float range,
        const std::vector<double> sensorTimestamps = {},
        const Eigen::Affine3f& baseToSensorTransform = Eigen::Affine3f::Identity()
    );
    void saveLidarOctomap(const octomap::OcTree& tree);
    pcl::PointCloud<pcl::PointXYZI>::Ptr readMapSample(const int sampleIdx) const;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> readMapSamples(const int numSamples = -1) const;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> sampleMapFrames(const float horizontalFov, const float verticalFov, const float range, const std::vector<Eigen::Affine3f>& mapFramePoses);
    void saveMapSamples(const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& samples);

    // include/run/coloradar_plus_run.hpp
    template<CloudType CloudT> std::shared_ptr<CloudT> getLidarPointCloud(const int cloudIdx) const;
    template<PclCloudType CloudT> std::shared_ptr<CloudT> getLidarPointCloud(const std::filesystem::path& binPath) const;
    template<OctomapCloudType CloudT> std::shared_ptr<CloudT> getLidarPointCloud(const std::filesystem::path& binPath) const;
};


}

#include "run/coloradar_plus_run.hpp"

#endif
