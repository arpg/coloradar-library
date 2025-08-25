#ifndef H5_COLORADAR_RUN_H
#define H5_COLORADAR_RUN_H

#include "radar_configs.h"
#include "dataset_configs.h"


namespace coloradar {

class H5Run {
protected:
    // ATTRIBUTES
    std::string name_;
    std::shared_ptr<RadarConfig> cascadeConfig_;
    
    std::vector<double> poseTimestamps_;
    std::vector<double> imuTimestamps_;
    std::vector<double> lidarTimestamps_;
    std::vector<double> cascadeCubeTimestamps_;
    std::vector<double> cascadeTimestamps_;

    std::vector<std::shared_ptr<Eigen::Affine3f>> poses_;
    std::vector<std::shared_ptr<std::vector<int16_t>>> cascadeDatacubes_;
    std::vector<std::shared_ptr<std::vector<float>>> cascadeHeatmaps_;
    std::vector<pcl::PointCloud<RadarPoint>::Ptr> cascadePointclouds_;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> lidarPointclouds_;

    std::shared_ptr<octomap::OcTree> lidarOctomap_;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> mapSamples_;

public:
    H5Run(
        std::string name,
        std::shared_ptr<RadarConfig> cascadeRadarConfig = nullptr,
        std::vector<double> poseTimestamps = {},
        std::vector<std::shared_ptr<Eigen::Affine3f>> poses = {},
        std::vector<double> imuTimestamps = {},
        std::vector<double> lidarTimestamps = {},
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> lidarPointclouds = {},
        std::vector<double> cascadeCubeTimestamps = {},
        std::vector<std::shared_ptr<std::vector<int16_t>>> cascadeDatacubes = {},
        std::vector<double> cascadeTimestamps = {},
        std::vector<std::shared_ptr<std::vector<float>>> cascadeHeatmaps = {},
        std::vector<pcl::PointCloud<RadarPoint>::Ptr> cascadePointclouds = {}
    );

    const std::string& name() const noexcept { return name_; }
    std::shared_ptr<RadarConfig> cascadeConfig() const noexcept { return cascadeConfig_; }

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

// #include "h5_hpp/coloradar_run.hpp"

#endif
