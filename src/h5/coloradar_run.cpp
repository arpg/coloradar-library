#include "h5/coloradar_run.h"


namespace coloradar {

H5Run::H5Run(
    std::string name,
    std::shared_ptr<RadarConfig> cascadeRadarConfig,
    std::vector<double> poseTimestamps,
    std::vector<std::shared_ptr<Eigen::Affine3f>> poses,
    std::vector<double> imuTimestamps,
    std::vector<double> lidarTimestamps,
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> lidarPointclouds,
    std::vector<double> cascadeCubeTimestamps,
    std::vector<std::shared_ptr<std::vector<int16_t>>> cascadeDatacubes,
    std::vector<double> cascadeTimestamps,
    std::vector<std::shared_ptr<std::vector<float>>> cascadeHeatmaps,
    std::vector<pcl::PointCloud<RadarPoint>::Ptr> cascadePointclouds
)
: name_(std::move(name)),
  cascadeConfig_(std::move(cascadeRadarConfig)),
  poseTimestamps_(std::move(poseTimestamps)),
  imuTimestamps_(std::move(imuTimestamps)),
  lidarTimestamps_(std::move(lidarTimestamps)),
  cascadeCubeTimestamps_(std::move(cascadeCubeTimestamps)),
  cascadeTimestamps_(std::move(cascadeTimestamps)),
  poses_(std::move(poses)),
  cascadeDatacubes_(std::move(cascadeDatacubes)),
  cascadeHeatmaps_(std::move(cascadeHeatmaps)),
  cascadePointclouds_(std::move(cascadePointclouds)),
  lidarPointclouds_(std::move(lidarPointclouds))
{
    // required arguments
    if (name_.empty()) throw std::invalid_argument("H5Run: 'name' must be non-empty.");

    // pose data
    if (!poseTimestamps_.empty() && !poses_.empty() && poseTimestamps_.size() != poses_.size())
        throw std::invalid_argument(
            "Length mismatch: poseTimestamps (" + std::to_string(poseTimestamps_.size()) + ") vs poses (" + std::to_string(poses_.size()) + ")."
        );

    // lidar data
    if (!lidarTimestamps_.empty() && !lidarPointclouds_.empty() && lidarTimestamps_.size() != lidarPointclouds_.size())
        throw std::invalid_argument(
            "Length mismatch: lidarTimestamps (" + std::to_string(lidarTimestamps_.size()) + ") vs lidarPointclouds (" + std::to_string(lidarPointclouds_.size()) + ")."
        );

    // datacube data
    if (!cascadeCubeTimestamps_.empty() && !cascadeDatacubes_.empty() && cascadeCubeTimestamps_.size() != cascadeDatacubes_.size())
        throw std::invalid_argument(
            "Length mismatch: cascadeCubeTimestamps (" + std::to_string(cascadeCubeTimestamps_.size()) + ") vs cascadeDatacubes (" + std::to_string(cascadeDatacubes_.size()) + ")."
        );

    if (!cascadeDatacubes_.empty()) {
        if (!cascadeDatacubes_[0]) throw std::invalid_argument("cascadeDatacubes[0] is null.");
        const std::size_t refSize = cascadeDatacubes_[0]->size();
        for (std::size_t i = 1; i < cascadeDatacubes_.size(); ++i) {
            if (!cascadeDatacubes_[i]) throw std::invalid_argument("cascadeDatacubes[" + std::to_string(i) + "] is null.");
            if (cascadeDatacubes_[i]->size() != refSize) throw std::invalid_argument(
                "Expected datacube size " + std::to_string(refSize) + ", got " + std::to_string(cascadeDatacubes_[i]->size()) + " at index " + std::to_string(i) + "."
            );
        }
    }

    // heatmap data
    int numHeatmaps = 0;
    if (!cascadeTimestamps_.empty()) numHeatmaps = cascadeHeatmaps_.size();
    if (!cascadeHeatmaps_.empty()) {
        if (numHeatmaps == 0) numHeatmaps = cascadeHeatmaps_.size();
        else if (cascadeHeatmaps_.size() != numHeatmaps) 
            throw std::invalid_argument("Expected " + std::to_string(numHeatmaps) + " cascade heatmaps, got " + std::to_string(cascadeHeatmaps_.size()) + ".");
    }
    if (!cascadePointclouds_.empty()) {
        if (numHeatmaps == 0) numHeatmaps = cascadePointclouds_.size();
        else if (cascadePointclouds_.size() != numHeatmaps) 
            throw std::invalid_argument("Expected " + std::to_string(numHeatmaps) + " cascade pointclouds, got " + std::to_string(cascadePointclouds_.size()) + ".");
    }

    if (!cascadeHeatmaps_.empty()) {
        if (!cascadeHeatmaps_[0]) throw std::invalid_argument("cascadeHeatmaps[0] is null.");
        const std::size_t refSize = cascadeHeatmaps_[0]->size();
        for (std::size_t i = 1; i < cascadeHeatmaps_.size(); ++i) {
            if (!cascadeHeatmaps_[i]) throw std::invalid_argument("cascadeHeatmaps[" + std::to_string(i) + "] is null.");
            if (cascadeHeatmaps_[i]->size() != refSize) throw std::invalid_argument(
                "Expected heatmap size " + std::to_string(refSize) + ", got " + std::to_string(cascadeHeatmaps_[i]->size()) + " at index " + std::to_string(i) + "."
            );
        }
    }
}

}