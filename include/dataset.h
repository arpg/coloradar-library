#ifndef DATASET_H
#define DATASET_H

#include "coloradar_run.h"
#include "device.h"
#include "dataset_configs.h"


namespace coloradar {

class ColoradarPlusDataset {
protected:
    std::filesystem::path datasetDirPath_;
    std::filesystem::path calibDirPath_;
    std::filesystem::path transformsDirPath_;
    std::filesystem::path runsDirPath_;

    Eigen::Affine3f imuTransform_;
    Eigen::Affine3f lidarTransform_;
    Eigen::Affine3f cascadeTransform_;

    RadarConfig* cascadeConfig_;

    ColoradarPlusDataset() = default;
    void init(const std::filesystem::path& pathToDataset);
    Eigen::Affine3f loadTransform(const std::filesystem::path& filePath);

    std::unique_ptr<BaseDevice> base_device_;
    std::unique_ptr<ImuDevice> imu_;
    std::unique_ptr<CascadeDevice> cascade_;
    std::unique_ptr<LidarDevice> lidar_;
    // std::vector<std::unique_ptr<BaseDevice>> devices;

    // void readExportConfig(const std::filesystem::path& configPath);

    // void exportConfig(config);
    void exportCascade(const std::vector<ColoradarPlusRun*> &runs, const H5::H5File &datasetFile, const DatasetExportConfig &userConfig, Json::Value finalConfig);
    // void exportLidar(config);
    // void exportImu(config);
    // void exportBaseFrame(config);
    // void exportSingleChip(config); for old dataset
    // void exportCamera(config); later

public:
    // ColoradarPlusDataset(const std::filesystem::path& pathToDataset);
    explicit ColoradarPlusDataset(const std::filesystem::path& pathToDataset);
    ColoradarPlusDataset(const ColoradarPlusDataset&) = delete;
    ColoradarPlusDataset& operator=(const ColoradarPlusDataset&) = delete;
    ColoradarPlusDataset(ColoradarPlusDataset&&) noexcept = default;
    ColoradarPlusDataset& operator=(ColoradarPlusDataset&&) noexcept = default;

    std::vector<std::string> listRuns();
    std::vector<ColoradarPlusRun*> getRuns();

    virtual ColoradarPlusRun* getRun(const std::string& runName);

    const Eigen::Affine3f& imuTransform() const;
    const Eigen::Affine3f& lidarTransform() const;
    const Eigen::Affine3f& cascadeTransform() const;
    const RadarConfig* cascadeConfig() const;

    std::filesystem::path exportToFile(
        std::vector<ColoradarPlusRun*> runs = {},
        std::filesystem::path destination = "",

        const bool& includeCascadeHeatmaps = false,
        const bool& includeCascadePointclouds = false,
        const bool& cascadePointcloudsInGlobalFrame = false,
        const int& cascadeAzimuthMaxBin = -1,
        const int& cascadeElevationMaxBin = -1,
        const int& cascadeRangeMaxBin = -1,
        const bool& removeCascadeDopplerDim = false,
        const bool& collapseCascadeElevation = false,
        const int& collapseCascadeElevationMinZ = -100,
        const int& collapseCascadeElevationMaxZ = 100,
        const float& cascadeCloudIntensityThresholdPercent = 0,

        const bool& includeLidarFrames = false,
        const float& lidarFrameTotalHorizontalFov = 360,
        const float& lidarFrameTotalVerticalFov = 180,
        const float& lidarFrameMaxRange = 100,
        const bool& collapseLidarFrameElevation = false,
        const float& collapseLidarFrameElevationMinZ = -100,
        const float& collapseLidarFrameElevationMaxZ = 100,

        const bool& includeLidarMap = false,
        const bool& collapseMapElevation = false,
        const float& collapseMapElevationMinZ = -100,
        const float& collapseMapElevationMaxZ = 100,
        const Eigen::Affine3f& lidarMapTransform = Eigen::Affine3f::Identity(),

        const bool& includeMapFrames = false,
        const float& mapSampleTotalHorizontalFov = 360,
        const float& mapSampleTotalVerticalFov = 180,
        const float& mapSampleMaxRange = 100,
        const Eigen::Affine3f& mapSamplingBaseToSensorTransform = Eigen::Affine3f::Identity(),
        std::vector<Eigen::Affine3f> mapSamplingBasePoses = {},
        const bool& collapseMapSampleElevation = false,
        const float& collapseMapSampleElevationMinZ = -100,
        const float& collapseMapSampleElevationMaxZ = 100,

        const bool& removeLidarIntensity = false,

        const bool& includeTruePoses = true,
        const bool& includeCascadePoses = true,
        const bool& includeLidarPoses = true,
        const bool& includeTrueTimestamps = true,
        const bool& includeCascadeTimestamps = true,
        const bool& includeLidarTimestamps = true
    );

};

class ColoradarDataset : public ColoradarPlusDataset {
protected:
    Eigen::Affine3f singleChipTransform_;

    RadarConfig* singleChipConfig_;
    std::unique_ptr<SingleChipDevice> single_chip_;

public:
    ColoradarDataset(const std::filesystem::path& pathToDataset);

    virtual ColoradarPlusRun* getRun(const std::string& runName) override;

    const Eigen::Affine3f& singleChipTransform() const;
    const RadarConfig* singleChipConfig() const;
};

}

#endif
