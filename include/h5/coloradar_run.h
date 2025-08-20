#ifndef COLORADAR_RUN_H
#define COLORADAR_RUN_H

#include "radar_configs.h"
#include "dataset_configs.h"


namespace coloradar {

class H5Run {
protected:
    // ATTRIBUTES
    std::vector<double> poseTimestamps_;
    std::vector<double> imuTimestamps_;
    std::vector<double> lidarTimestamps_;
    std::vector<double> cascadeCubeTimestamps_;
    std::vector<double> cascadeTimestamps_;

    RadarConfig* cascadeConfig_;

    std::vector<std::shared_ptr<Eigen::Affine3f>> poses_;
    std::vector<std::shared_ptr<std::vector<int16_t>>> cascadeDatacubes_;
    std::vector<std::shared_ptr<std::vector<float>>> cascadeHeatmaps_;
    std::vector<pcl::PointCloud<RadarPoint>::Ptr> cascadePointclouds_;

    std::shared_ptr<octomap::OcTree> lidarOctomap_;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> mapSamples_;


    // h5/coloradar_run.cpp
    // std::vector<double> readTimestamps(const std::filesystem::path& path);
    // std::vector<int16_t> getDatacube(const std::filesystem::path& binFilePath, RadarConfig* config) const;
    // std::vector<float> getHeatmap(const std::filesystem::path& binFilePath, RadarConfig* config) const;
    // void createRadarPointclouds(RadarConfig* config, const std::filesystem::path& heatmapDirPath, const std::filesystem::path& pointcloudDirPath, const double intensityThreshold = 0.0);
    // pcl::PointCloud<RadarPoint>::Ptr getRadarPointcloud(const std::filesystem::path& binFilePath, RadarConfig* config, const double intensityThreshold = 0.0);

public:
    const std::string name;

    // H5Run(const std::filesystem::path& runPath, RadarConfig* cascadeRadarConfig);

    const std::vector<double>& poseTimestamps() const;
    const std::vector<double>& imuTimestamps() const;
    const std::vector<double>& lidarTimestamps() const;
    const std::vector<double>& cascadeCubeTimestamps() const;
    const std::vector<double>& cascadeTimestamps() const;

    template<PoseType PoseT> std::vector<PoseT> getPoses() const;


    std::vector<int16_t> getCascadeDatacube(const int cubeIdx) const;
    std::vector<float> getCascadeHeatmap(const int hmIdx) const;
    void createCascadePointclouds(const double intensityThreshold = 0.0);
    pcl::PointCloud<RadarPoint>::Ptr getCascadePointcloud(const int& cloudIdx, const double intensityThreshold = 0.0);

    template<CloudType CloudT> std::shared_ptr<CloudT> getLidarPointCloud(const int cloudIdx) const;

    octomap::OcTree buildLidarOctomap(
        const double& mapResolution,
        const float& lidarTotalHorizontalFov,
        const float& lidarTotalVerticalFov,
        const float& lidarMaxRange,
        Eigen::Affine3f baseToLidarTransform = Eigen::Affine3f::Identity()
    );

    pcl::PointCloud<pcl::PointXYZI>::Ptr sampleMapFrame(const float& horizontalFov, const float& verticalFov, const float& range, const Eigen::Affine3f& mapFramePose, const pcl::PointCloud<pcl::PointXYZI>::Ptr& mapCloud);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> sampleMapFrames(const float& horizontalFov, const float& verticalFov, const float& range, const std::vector<Eigen::Affine3f>& mapFramePoses);
    void createMapSamples(
        const float& horizontalFov,
        const float& verticalFov,
        const float& range,
        const std::vector<double>& sensorTimestamps = {},
        const Eigen::Affine3f& baseToSensorTransform = Eigen::Affine3f::Identity()
    );

    virtual ~H5Run() = default;
};


}

#include "h5_hpp/coloradar_run.hpp"

#endif
