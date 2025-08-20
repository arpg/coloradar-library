#ifndef H5_DATASET_H
#define H5_DATASET_H

#include "h5/coloradar_run.h"


namespace coloradar {


class H5Dataset {
protected:
    // ATTRIBUTES
    std::filesystem::path h5SourceFilePath_;

    Eigen::Affine3f imuTransform_;
    Eigen::Affine3f lidarTransform_;
    Eigen::Affine3f cascadeTransform_;

    RadarConfig* cascadeConfig_;

    std::unique_ptr<BaseDevice> base_device_;
    std::unique_ptr<ImuDevice> imu_;
    std::unique_ptr<CascadeDevice> cascade_;
    std::unique_ptr<LidarDevice> lidar_;

    std::unordered_map<std::string, std::shared_ptr<H5Run>> runs_;


    // dataset/h5_init.cpp
    H5Dataset() = default;
    void init(const std::filesystem::path& pathToH5File);
    void postInit();

public:
    // dataset/h5_init.cpp
    explicit H5Dataset(const std::filesystem::path& pathToH5File);
    H5Dataset(const H5Dataset&) = delete;
    H5Dataset& operator=(const H5Dataset&) = delete;
    H5Dataset(H5Dataset&&) noexcept = default;
    H5Dataset& operator=(H5Dataset&&) noexcept = default;

    // dataset/h5_data.cpp
    const Eigen::Affine3f& imuTransform() const;
    const Eigen::Affine3f& lidarTransform() const;
    const Eigen::Affine3f& cascadeTransform() const;
    const RadarConfig* cascadeConfig() const;
    
    std::vector<std::string> listRuns();
    std::vector<std::shared_ptr<H5Run>> getRuns();
    std::shared_ptr<H5Run> getRun(const std::string& runName);
};

}

#endif
