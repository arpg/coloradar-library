#ifndef BASE_DATASET_H
#define BASE_DATASET_H

#include "run/base_run.h"


namespace coloradar {


class Dataset {
protected:
    // CONSTANTS
    inline static constexpr std::string_view poseTimestampsContentName = "base_timestamps";
    inline static constexpr std::string_view imuTimestampsContentName = "imu_timestamps";
    inline static constexpr std::string_view lidarTimestampsContentName = "lidar_timestamps";
    inline static constexpr std::string_view cascadeCubeTimestampsContentName = "cascade_cube_timestamps";
    inline static constexpr std::string_view cascadeTimestampsContentName = "cascade_timestamps";
 
    inline static constexpr std::string_view transformBaseToCascadeContentName = "transform_base_to_cascade";
    inline static constexpr std::string_view transformBaseToLidarContentName = "transform_base_to_lidar";
    inline static constexpr std::string_view transformBaseToImuContentName = "transform_base_to_imu";

    inline static constexpr std::string_view posesContentName = "base_poses";

    inline static constexpr std::string_view lidarCloudsContentName = "lidar_clouds";
    inline static constexpr std::string_view lidarMapContentName = "lidar_map";
    inline static constexpr std::string_view lidarMapSamplesContentName = "lidar_map_samples";

    inline static constexpr std::string_view cascadeDatacubesContentName = "cascade_datacubes";
    inline static constexpr std::string_view cascadeHeatmapsContentName = "cascade_heatmaps"; 
    inline static constexpr std::string_view cascadeCloudsContentName = "cascade_clouds";

    // ATTRIBUTES
    Eigen::Affine3f imuTransform_;
    Eigen::Affine3f lidarTransform_;
    Eigen::Affine3f cascadeTransform_;

    std::shared_ptr<RadarConfig> cascadeConfig_;

    std::unique_ptr<BaseDevice> base_device_;
    std::unique_ptr<ImuDevice> imu_;
    std::unique_ptr<CascadeDevice> cascade_;
    std::unique_ptr<LidarDevice> lidar_;

    std::unordered_map<std::string, std::shared_ptr<Run>> runs_;
    

    // METHODS
    const std::string getExportArrayName(const std::string_view contentName, const std::string_view runName) const { 
        return std::string(contentName) + "_" + std::string(runName); 
    }

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
    
    // dataset/base_dataset.cpp
    std::vector<std::string> listRuns() const;
    std::vector<std::shared_ptr<Run>> getRuns() const;
    std::shared_ptr<Run> getRun(const std::string& runName) const;
};

}

#endif
