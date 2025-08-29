#ifndef COLORADAR_RUN_H
#define COLORADAR_RUN_H

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
    

    // run/coloradar_run_init.cpp
    std::vector<double> readTimestamps(const std::filesystem::path& path) const;
    std::vector<Eigen::Affine3f> readPoses(const std::filesystem::path& path) const;
    
    // run/coloradar_run_data.cpp
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
    // run/coloradar_run_init.cpp
    ColoradarPlusRun(const std::filesystem::path& runPath, std::shared_ptr<RadarConfig> cascadeRadarConfig = nullptr);
    virtual ~ColoradarPlusRun() = default;

    // run/coloradar_run_data.cpp
    virtual pcl::PointCloud<pcl::PointXYZI>::Ptr getLidarPointCloud(const int cloudIdx) const override;
    virtual std::shared_ptr<std::vector<int16_t>> getCascadeDatacube(const int cubeIdx) const override;
    virtual std::shared_ptr<std::vector<float>> getCascadeHeatmap(const int hmIdx) const override;
    virtual pcl::PointCloud<RadarPoint>::Ptr getCascadePointcloud(const int cloudIdx, const double intensityThreshold = 0.0) const override;
    virtual pcl::PointCloud<pcl::PointXYZI>::Ptr getLidarOctomap() const override;

    template<CloudType CloudT> std::shared_ptr<CloudT> getLidarPointCloud(const int cloudIdx) const;
    template<PclCloudType CloudT> std::shared_ptr<CloudT> getLidarPointCloud(const std::filesystem::path& binPath) const;
    template<OctomapCloudType CloudT> std::shared_ptr<CloudT> getLidarPointCloud(const std::filesystem::path& binPath) const;
    
    // pcl::PointCloud<pcl::PointXYZI>::Ptr getLidarPointCloud(const std::filesystem::path& binFilePath) const;
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
    void saveLidarOctomap(const octomap::OcTree& tree);
    void createLidarOctomap(
        const double& mapResolution,
        const float& lidarTotalHorizontalFov,
        const float& lidarTotalVerticalFov,
        const float& lidarMaxRange,
        Eigen::Affine3f baseToLidarTransform = Eigen::Affine3f::Identity()
    );
    pcl::PointCloud<pcl::PointXYZI>::Ptr readMapSample(const int& sampleIdx) const;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> readMapSamples(const int& numSamples = -1) const;
    pcl::PointCloud<pcl::PointXYZI>::Ptr sampleMapFrame(const float& horizontalFov, const float& verticalFov, const float& range, const Eigen::Affine3f& mapFramePose, const pcl::PointCloud<pcl::PointXYZI>::Ptr& mapCloud);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> sampleMapFrames(const float& horizontalFov, const float& verticalFov, const float& range, const std::vector<Eigen::Affine3f>& mapFramePoses);
    void saveMapSample(const pcl::PointCloud<pcl::PointXYZI>::Ptr& sample, const int& sampleIdx);
    void saveMapSamples(const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& samples);
    void createMapSamples(
        const float& horizontalFov,
        const float& verticalFov,
        const float& range,
        const std::vector<double>& sensorTimestamps = {},
        const Eigen::Affine3f& baseToSensorTransform = Eigen::Affine3f::Identity()
    );
};


class ColoradarRun : public ColoradarPlusRun {
protected:
    std::filesystem::path singleChipScansDirPath_;
    std::filesystem::path singleChipCubesDirPath_;
    std::filesystem::path singleChipHeatmapsDirPath_;
    std::filesystem::path singleChipCloudsDirPath_;

    std::vector<double> singleChipCubeTimestamps_;
    std::vector<double> singleChipTimestamps_;

    std::shared_ptr<RadarConfig> singleChipConfig_;

public:
    ColoradarRun(const std::filesystem::path& runPath, std::shared_ptr<RadarConfig> cascadeRadarConfig, std::shared_ptr<RadarConfig> singleChipRadarConfig);

    const std::shared_ptr<RadarConfig> singleChipConfig() const { return singleChipConfig_; }
    const std::vector<double>& singleChipCubeTimestamps() const { return singleChipCubeTimestamps_; }
    const std::vector<double>& singleChipTimestamps() const { return singleChipTimestamps_; }

    std::shared_ptr<std::vector<int16_t>> getSingleChipDatacube(const std::filesystem::path& binFilePath);
    std::shared_ptr<std::vector<int16_t>> getSingleChipDatacube(const int& cubeIdx);
    std::shared_ptr<std::vector<float>> getSingleChipHeatmap(const std::filesystem::path& binFilePath);
    std::shared_ptr<std::vector<float>> getSingleChipHeatmap(const int& hmIdx);
    pcl::PointCloud<RadarPoint>::Ptr getSingleChipPointcloud(const std::filesystem::path& binFilePath, const double intensityThreshold = 0.0);
    pcl::PointCloud<RadarPoint>::Ptr getSingleChipPointcloud(const int& cloudIdx, const double intensityThreshold = 0.0);
    // void createSingleChipPointclouds(const float& intensityThreshold = 0.0);
};

}

#include "run/coloradar_run.hpp"

#endif
