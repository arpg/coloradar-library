#include "run/h5_run.h"


namespace coloradar {


void H5Run::setData(
    std::vector<double> poseTimestamps,
    std::vector<double> imuTimestamps,
    std::vector<double> lidarTimestamps,
    std::vector<double> cascadeCubeTimestamps,
    std::vector<double> cascadeTimestamps,
    std::vector<Eigen::Affine3f> poses,
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> lidarPointclouds,
    std::vector<std::shared_ptr<std::vector<int16_t>>> cascadeDatacubes,
    std::vector<std::shared_ptr<std::vector<float>>> cascadeHeatmaps,
    std::vector<pcl::PointCloud<RadarPoint>::Ptr> cascadePointclouds,
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidarOctomap,
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> mapSamples
) {
    // pose data
    if (!poseTimestamps.empty() && !poses.empty() && poseTimestamps.size() != poses.size())
        throw std::invalid_argument("Data length mismatch in run " + name_ + ": 'poseTimestamps' (" + std::to_string(poseTimestamps.size()) + ") vs 'poses' (" + std::to_string(poses.size()) + ").");
    poseTimestamps_ = poseTimestamps;
    poses_ = poses;

    // imu data
    // if (!imuTimestamps.empty() && !imuPoses.empty() && imuTimestamps.size() != imuPoses.size())
    //     throw std::invalid_argument("H5Run::setData() Length Mismatch: 'imuTimestamps' (" + std::to_string(imuTimestamps.size()) + ") vs 'imuPoses' (" + std::to_string(imuPoses.size()) + ").");
    imuTimestamps_ = imuTimestamps;
    // imuPoses_ = imuPoses;

    // lidar data
    if (!lidarTimestamps.empty() && !lidarPointclouds.empty() && lidarTimestamps.size() != lidarPointclouds.size())
        throw std::invalid_argument(
            "Data length mismatch in run " + name_ + ": 'lidarTimestamps' (" + std::to_string(lidarTimestamps.size()) + ")"
            " vs 'lidarPointclouds' (" + std::to_string(lidarPointclouds.size()) + ")."
        );
    lidarTimestamps_ = lidarTimestamps;
    lidarPointclouds_ = lidarPointclouds;
    lidarOctomap_ = lidarOctomap;
    mapSamples_ = mapSamples;

    // datacube data
    if (!cascadeCubeTimestamps.empty() && !cascadeDatacubes.empty() && cascadeCubeTimestamps.size() != cascadeDatacubes.size())
        throw std::invalid_argument(
            "Data length mismatch in run " + name_ + ": 'cascadeCubeTimestamps' (" + std::to_string(cascadeCubeTimestamps.size()) + ")"
            " vs 'cascadeDatacubes' (" + std::to_string(cascadeDatacubes.size()) + ")."
        );
    if (!cascadeDatacubes.empty()) {
        if (!cascadeDatacubes[0]) throw std::invalid_argument("cascadeDatacubes[0] is null.");
        const std::size_t refSize = cascadeDatacubes[0]->size();
        for (std::size_t i = 1; i < cascadeDatacubes.size(); ++i) {
            if (!cascadeDatacubes[i]) throw std::invalid_argument("cascadeDatacubes[" + std::to_string(i) + "] is null.");
            if (cascadeDatacubes[i]->size() != refSize) throw std::invalid_argument("Expected datacube size " + std::to_string(refSize) + ", got " + std::to_string(cascadeDatacubes[i]->size()) + " at index " + std::to_string(i) + ".");
        }
    }
    cascadeCubeTimestamps_ = cascadeCubeTimestamps;
    cascadeDatacubes_ = cascadeDatacubes;

    // heatmap data
    int numHeatmaps = 0;
    if (!cascadeTimestamps.empty()) numHeatmaps = cascadeHeatmaps.size();
    if (!cascadeHeatmaps.empty()) {
        if (numHeatmaps == 0) numHeatmaps = cascadeHeatmaps.size();
        else if (cascadeHeatmaps.size() != numHeatmaps) 
            throw std::invalid_argument("Expected " + std::to_string(numHeatmaps) + " cascade heatmaps, got " + std::to_string(cascadeHeatmaps.size()) + ".");
    }
    if (!cascadePointclouds.empty()) {
        if (numHeatmaps == 0) numHeatmaps = cascadePointclouds.size();
        else if (cascadePointclouds.size() != numHeatmaps) 
            throw std::invalid_argument("Expected " + std::to_string(numHeatmaps) + " cascade pointclouds, got " + std::to_string(cascadePointclouds.size()) + ".");
    }
    if (!cascadeHeatmaps.empty()) {
        if (!cascadeHeatmaps[0]) throw std::invalid_argument("cascadeHeatmaps[0] is null.");
        const std::size_t refSize = cascadeHeatmaps[0]->size();
        for (std::size_t i = 1; i < cascadeHeatmaps.size(); ++i) {
            if (!cascadeHeatmaps[i]) throw std::invalid_argument("cascadeHeatmaps[" + std::to_string(i) + "] is null.");
            if (cascadeHeatmaps[i]->size() != refSize) throw std::invalid_argument("Expected heatmap size " + std::to_string(refSize) + ", got " + std::to_string(cascadeHeatmaps[i]->size()) + " at index " + std::to_string(i) + ".");
        }
    }
    cascadeTimestamps_ = cascadeTimestamps;
    cascadeHeatmaps_ = cascadeHeatmaps;
    cascadePointclouds_ = cascadePointclouds;
}


pcl::PointCloud<pcl::PointXYZI>::Ptr H5Run::getLidarPointCloud(const int cloudIdx) const {
    // if (lidarPointclouds_.empty()) throw std::runtime_error("No lidar pointclouds found in run " + name_);
    if (cloudIdx < 0 || cloudIdx >= lidarPointclouds_.size()) throw std::invalid_argument("H5Run: 'cloudIdx' out of bounds [0, " + std::to_string(lidarPointclouds_.size()) + ")");
    return lidarPointclouds_[cloudIdx];
}


std::shared_ptr<std::vector<int16_t>> H5Run::getCascadeDatacube(const int cubeIdx) const {
    // if (cascadeDatacubes_.empty()) throw std::runtime_error("No cascade datacubes found in run " + name_);
    if (cubeIdx < 0 || cubeIdx >= cascadeDatacubes_.size()) throw std::invalid_argument("H5Run: 'cubeIdx' out of bounds [0, " + std::to_string(cascadeDatacubes_.size()) + ")");
    return cascadeDatacubes_[cubeIdx];
}


std::shared_ptr<std::vector<float>> H5Run::getCascadeHeatmap(const int hmIdx) const {
    // if (cascadeHeatmaps_.empty()) throw std::runtime_error("No cascade heatmaps found in run " + name_);
    if (hmIdx < 0 || hmIdx >= cascadeHeatmaps_.size()) throw std::invalid_argument("H5Run: 'hmIdx' out of bounds [0, " + std::to_string(cascadeHeatmaps_.size()) + ")");
    return cascadeHeatmaps_[hmIdx];
}


pcl::PointCloud<RadarPoint>::Ptr H5Run::getCascadePointcloud(const int cloudIdx, const double intensityThreshold) const {
    // if (cascadePointclouds_.empty()) throw std::runtime_error("No cascade pointclouds found in run " + name_);
    if (cloudIdx < 0 || cloudIdx >= cascadePointclouds_.size()) throw std::invalid_argument("H5Run: 'cloudIdx' out of bounds [0, " + std::to_string(cascadePointclouds_.size()) + ")");
    if (intensityThreshold < 0.0) throw std::invalid_argument("H5Run: 'intensityThreshold' must be non-negative");
    if (intensityThreshold > 0.0) {
        pcl::PointCloud<RadarPoint>::Ptr cloud(new pcl::PointCloud<RadarPoint>());
        for (const auto& point : *cascadePointclouds_[cloudIdx]) {
            if (point.intensity >= intensityThreshold) {
                cloud->points.push_back(point);
            }
        }
        return cloud;
    }
    return cascadePointclouds_[cloudIdx];
}


pcl::PointCloud<pcl::PointXYZI>::Ptr H5Run::getLidarOctomap() const {
    // if (!lidarOctomap_) throw std::runtime_error("No lidar octomap found in run " + name_);
    return lidarOctomap_;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr H5Run::getMapSample(const int sampleIdx) const {
    // if (!mapSamples_.empty()) throw std::runtime_error("No map samples found in run " + name_);
    if (sampleIdx < 0 || sampleIdx >= mapSamples_.size()) throw std::invalid_argument("H5Run: 'sampleIdx' out of bounds [0, " + std::to_string(mapSamples_.size()) + ")");
    return mapSamples_[sampleIdx];
}

void H5Run::saveMapSample(const int sampleIdx, const pcl::PointCloud<pcl::PointXYZI>::Ptr sample) {
    if (sampleIdx < 0) throw std::invalid_argument("H5Run: 'sampleIdx' out of bounds [0, " + std::to_string(mapSamples_.size()) + ")");
    if (sampleIdx >= mapSamples_.size()) mapSamples_.resize(sampleIdx + 1);
    mapSamples_[sampleIdx] = sample;
}

}