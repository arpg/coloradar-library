#ifndef BASE_DATASET_H
#define BASE_DATASET_H

#include "run/base_run.h"


namespace coloradar {


class Dataset {
protected:
    // CONSTANTS
    inline static const std::string poseTimestampsContentName        = "base_timestamps";
    inline static const std::string imuTimestampsContentName         = "imu_timestamps";
    inline static const std::string lidarTimestampsContentName       = "lidar_timestamps";
    inline static const std::string cascadeCubeTimestampsContentName = "cascade_cube_timestamps";
    inline static const std::string cascadeTimestampsContentName     = "cascade_timestamps";

    inline static const std::string transformBaseToCascadeContentName = "transform_base_to_cascade";
    inline static const std::string transformBaseToLidarContentName   = "transform_base_to_lidar";
    inline static const std::string transformBaseToImuContentName     = "transform_base_to_imu";

    inline static const std::string posesContentName        = "base_poses";
    inline static const std::string imuPosesContentName     = "imu_poses";
    inline static const std::string lidarPosesContentName   = "lidar_poses";
    inline static const std::string cascadePosesContentName = "cascade_poses";

    inline static const std::string lidarCloudsContentName     = "lidar_clouds";
    inline static const std::string lidarMapContentName        = "lidar_map";
    inline static const std::string lidarMapSamplesContentName = "lidar_map_samples";

    inline static const std::string cascadeDatacubesContentName = "cascade_datacubes";
    inline static const std::string cascadeHeatmapsContentName  = "cascade_heatmaps";
    inline static const std::string cascadeCloudsContentName    = "cascade_clouds";
        

    // ATTRIBUTES
    Eigen::Affine3f imuTransform_;
    Eigen::Affine3f lidarTransform_;
    Eigen::Affine3f cascadeTransform_;

    std::shared_ptr<RadarConfig> cascadeConfig_;

    std::unique_ptr<BaseDevice> base_device_;
    std::unique_ptr<ImuDevice> imu_;
    std::unique_ptr<CascadeDevice> cascade_;
    std::unique_ptr<LidarDevice> lidar_;
 
    std::vector<std::string> runNames_;
    std::vector<std::shared_ptr<Run>> runs_;
    

    // METHODS
    const std::string getExportArrayName(const std::string contentName, const std::string runName) const { return contentName + "_" + runName; }

public:
    Dataset() = default;
    Dataset(const Dataset&) = delete;
    Dataset& operator=(const Dataset&) = delete;
    Dataset(Dataset&&) noexcept = default;
    Dataset& operator=(Dataset&&) noexcept = default;
    virtual ~Dataset() = default;

    const std::shared_ptr<RadarConfig> cascadeConfig() const { return cascadeConfig_; }
    const Eigen::Affine3f& imuTransform() const { return imuTransform_; }
    const Eigen::Affine3f& lidarTransform() const { return lidarTransform_; }
    const Eigen::Affine3f& cascadeTransform() const { return cascadeTransform_; }
    
    std::vector<std::string> listRuns() const { return runNames_; }
    std::vector<std::shared_ptr<Run>> getRuns() const { return runs_; }
    std::shared_ptr<Run> getRun(const std::string& runName) const { 
        auto it = std::find(runNames_.begin(), runNames_.end(), runName);
        if (it == runNames_.end()) throw std::invalid_argument("Run '" + runName + "' not found in dataset.");
        return runs_[std::distance(runNames_.begin(), it)];
    }
};

}

#endif
