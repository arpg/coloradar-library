#ifndef BASE_RUN_H
#define BASE_RUN_H

#include "radar_configs.h"
#include "dataset_configs.h"


namespace coloradar {

class Run {
protected:
    // ATTRIBUTES
    std::string name_;
    std::shared_ptr<RadarConfig> cascadeConfig_;
    
    std::vector<double> poseTimestamps_;
    std::vector<double> imuTimestamps_;
    std::vector<double> lidarTimestamps_;
    std::vector<double> cascadeCubeTimestamps_;
    std::vector<double> cascadeTimestamps_;

    std::vector<Eigen::Affine3f> poses_;

public:
    Run(std::string name, std::shared_ptr<RadarConfig> cascadeRadarConfig = nullptr) : name_(std::move(name)), cascadeConfig_(std::move(cascadeRadarConfig)) {}
    virtual ~Run() = default;

    const std::string& name() const noexcept { return name_; }
    std::shared_ptr<RadarConfig> cascadeConfig() const noexcept { return cascadeConfig_; }
    const std::vector<double>& poseTimestamps() const { return poseTimestamps_; }
    const std::vector<double>& imuTimestamps() const { return imuTimestamps_; }
    const std::vector<double>& lidarTimestamps() const { return lidarTimestamps_; }
    const std::vector<double>& cascadeCubeTimestamps() const { return cascadeCubeTimestamps_; }
    const std::vector<double>& cascadeTimestamps() const { return cascadeTimestamps_; }
    
    // include/run/base_run.hpp
    template<PoseType PoseT> std::vector<PoseT> getPoses() const;

    // src/run/base_run.cpp
    pcl::PointCloud<pcl::PointXYZI>::Ptr sampleMapFrame(
        const float horizontalFov, const float verticalFov, const float range, 
        const Eigen::Affine3f& mapFramePose, 
        const pcl::PointCloud<pcl::PointXYZI>::Ptr mapCloud
    );

    // abstract methods
    virtual pcl::PointCloud<pcl::PointXYZI>::Ptr getLidarPointCloud(const int cloudIdx) const = 0;
    virtual std::shared_ptr<std::vector<int16_t>> getCascadeDatacube(const int cubeIdx) const = 0;
    virtual std::shared_ptr<std::vector<float>> getCascadeHeatmap(const int hmIdx) const = 0;
    virtual pcl::PointCloud<RadarPoint>::Ptr getCascadePointcloud(const int cloudIdx, const double intensityThreshold = 0.0) const = 0;
    virtual pcl::PointCloud<pcl::PointXYZI>::Ptr getLidarOctomap() const = 0;
    virtual pcl::PointCloud<pcl::PointXYZI>::Ptr getMapSample(const int sampleIdx) const = 0;
    virtual void saveMapSample(const int sampleIdx, const pcl::PointCloud<pcl::PointXYZI>::Ptr sample) = 0;  
};


}

#include "run/base_run.hpp"

#endif
