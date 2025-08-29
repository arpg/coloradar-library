#ifndef H5_RUN_H
#define H5_RUN_H

#include "run/base_run.h"


namespace coloradar {

class H5Run : public Run {
protected:
    // ATTRIBUTES
    std::vector<std::shared_ptr<std::vector<int16_t>>> cascadeDatacubes_;
    std::vector<std::shared_ptr<std::vector<float>>> cascadeHeatmaps_;
    std::vector<pcl::PointCloud<RadarPoint>::Ptr> cascadePointclouds_;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> lidarPointclouds_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidarOctomap_;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> mapSamples_;

public:
    H5Run(std::string name, std::shared_ptr<RadarConfig> cascadeRadarConfig = nullptr) : Run(std::move(name), std::move(cascadeRadarConfig)) {}
    virtual ~H5Run() = default;

    // run/h5_run_data.cpp
    void setData(
        std::vector<double> poseTimestamps = {},
        std::vector<double> imuTimestamps = {},
        std::vector<double> lidarTimestamps = {},
        std::vector<double> cascadeCubeTimestamps = {},
        std::vector<double> cascadeTimestamps = {},
        std::vector<Eigen::Affine3f> poses = {},
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> lidarPointclouds = {},
        std::vector<std::shared_ptr<std::vector<int16_t>>> cascadeDatacubes = {},
        std::vector<std::shared_ptr<std::vector<float>>> cascadeHeatmaps = {},
        std::vector<pcl::PointCloud<RadarPoint>::Ptr> cascadePointclouds = {}
    );
    virtual std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> getLidarPointCloud(const int cloudIdx) const override;
    virtual std::shared_ptr<std::vector<int16_t>> getCascadeDatacube(const int cubeIdx) const override;
    virtual std::shared_ptr<std::vector<float>> getCascadeHeatmap(const int hmIdx) const override;
    virtual pcl::PointCloud<RadarPoint>::Ptr getCascadePointcloud(const int cloudIdx, const double intensityThreshold = 0.0) const override;
    virtual pcl::PointCloud<pcl::PointXYZI>::Ptr getLidarOctomap() const override;

    template<CloudType CloudT> std::shared_ptr<CloudT> getLidarPointCloud(const int cloudIdx) const;
    
    // run/h5_run.hpp
    // template<CloudType CloudT> std::shared_ptr<CloudT> getLidarPointCloud(const int cloudIdx) const override;
};


}

#include "run/h5_run.hpp"

#endif
