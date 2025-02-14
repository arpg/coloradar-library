#ifndef DEVICE_H
#define DEVICE_H

#include "utils.h"


namespace coloradar {

// BASE CLASSES

class BaseExportConfig {
protected:
    std::string deviceName_;
    bool exportPoses_;
    bool exportTimestamps_;

    virtual void validate() {}
public:
    BaseExportConfig(std::string deviceName = "base", bool exportPoses = false, bool exportTimestamps = false)
        : deviceName_(deviceName), exportPoses_(exportPoses), exportTimestamps_(exportTimestamps) {
        validate();
    }
    const bool& exportPoses() const { return exportPoses_; }
    const bool& exportTimestamps() const { return exportTimestamps_; }
};

class BaseDevice {
protected:
    std::unique_ptr<BaseExportConfig> exportConfig_;

public:
    // static const std::string name;
    static inline const std::string name = "base";
    BaseDevice(std::unique_ptr<BaseExportConfig> exportConfig = std::make_unique<BaseExportConfig>()): exportConfig_(std::move(exportConfig)) {}

    virtual const BaseExportConfig* exportConfig() const { return exportConfig_.get(); }

    virtual ~BaseDevice() = default;
};
// const std::string BaseDevice::name = "base";


// HELPER CLASSES

struct FovConfig {
    bool useDegreeConstraints = true;
    int  azimuthIdx = -1;
    int  elevationIdx = -1;

    float horizontalDegreesTotal = 360.0f;
    float verticalDegreesTotal = 180.0f;

    float rangeMeters = 0.0f;
};


// EXPORT CONFIGS

class RadarExportConfig : public BaseExportConfig {
protected:
    bool collapseElevation_;
    float collapseElevationMinZ_;
    float collapseElevationMaxZ_;
    bool removeDopplerDim_;
    FovConfig fov_;

    bool exportDatacubes_;
    bool exportHeatmaps_;
    bool exportClouds_;
    float intensityThresholdPercent_;
    bool cloudsInGlobalFrame_;

    virtual void validate() override;

public:
    RadarExportConfig(
        std::string deviceName = "radar",
        bool exportPoses = false,
        bool exportTimestamps = false,
        bool collapseElevation = false,
        float collapseElevationMinZ = 0.0f,
        float collapseElevationMaxZ = 0.0f,
        bool removeDopplerDim = false,
        FovConfig fov = FovConfig(),
        bool exportDatacubes = false,
        bool exportHeatmaps = false,
        bool exportClouds = false,
        float intensityThresholdPercent = 0.0f,
        bool cloudsInGlobalFrame = false
    ) : BaseExportConfig(deviceName, exportPoses, exportTimestamps),
        collapseElevation_(collapseElevation),
        collapseElevationMinZ_(collapseElevationMinZ),
        collapseElevationMaxZ_(collapseElevationMaxZ),
        removeDopplerDim_(removeDopplerDim),
        fov_(fov),
        exportDatacubes_(exportDatacubes),
        exportHeatmaps_(exportHeatmaps),
        exportClouds_(exportClouds),
        intensityThresholdPercent_(intensityThresholdPercent),
        cloudsInGlobalFrame_(cloudsInGlobalFrame) {
        validate();
    }

    const bool& collapseElevation() const { return collapseElevation_; }
    const float& collapseElevationMinZ() const { return collapseElevationMinZ_; }
    const float& collapseElevationMaxZ() const { return collapseElevationMaxZ_; }
    const bool& removeDopplerDim() const { return removeDopplerDim_; }
    const FovConfig& fov() const { return fov_; }
    const bool& exportDatacubes() const { return exportDatacubes_; }
    const bool& exportHeatmaps() const { return exportHeatmaps_; }
    const bool& exportClouds() const { return exportClouds_; }
    const float& intensityThresholdPercent() const { return intensityThresholdPercent_; }
    const bool& cloudsInGlobalFrame() const { return cloudsInGlobalFrame_; }
};

class LidarExportConfig : public BaseExportConfig {
protected:
    bool collapseElevation_;
    float collapseElevationMinZ_;
    float collapseElevationMaxZ_;

    bool exportClouds_;
    bool removeIntensityDim_;
    FovConfig cloudFov_;

    bool exportMap_;
    float mapOccupancyThresholdPercent_;
    Eigen::Affine3f mapTransform_;

    bool exportMapSamples_;
    float mapSampleOccupancyThresholdPercent_;
    bool allowResample_;
    bool forceResample_;
    std::unique_ptr<BaseDevice> centerSensor_;
    FovConfig mapSampleFov_;

    virtual void validate() override;

public:
    LidarExportConfig(
        std::string deviceName = "lidar",
        bool exportPoses = false,
        bool exportTimestamps = false,
        bool collapseElevation = false,
        float collapseElevationMinZ = 0.0f,
        float collapseElevationMaxZ = 0.0f,
        bool exportClouds = false,
        bool removeIntensityDim = false,
        FovConfig cloudFov = FovConfig(),
        bool exportMap = false,
        float mapOccupancyThresholdPercent = 0.0f,
        Eigen::Affine3f mapTransform = Eigen::Affine3f::Identity(),
        bool exportMapSamples = false,
        float mapSampleOccupancyThresholdPercent = 0.0f,
        bool allowResample = true,
        bool forceResample = false,
        std::unique_ptr<BaseDevice> centerSensor = std::make_unique<BaseDevice>(),
        FovConfig mapSampleFov = FovConfig()
    ) : BaseExportConfig(deviceName, exportPoses, exportTimestamps),
        collapseElevation_(collapseElevation),
        collapseElevationMinZ_(collapseElevationMinZ),
        collapseElevationMaxZ_(collapseElevationMaxZ),
        exportClouds_(exportClouds),
        removeIntensityDim_(removeIntensityDim),
        cloudFov_(cloudFov),
        exportMap_(exportMap),
        mapOccupancyThresholdPercent_(mapOccupancyThresholdPercent),
        mapTransform_(mapTransform),
        exportMapSamples_(exportMapSamples),
        mapSampleOccupancyThresholdPercent_(mapSampleOccupancyThresholdPercent),
        allowResample_(allowResample),
        forceResample_(forceResample),
        centerSensor_(std::move(centerSensor)),
        mapSampleFov_(mapSampleFov) {
        validate();
    }

    const BaseDevice* centerSensor() const { return centerSensor_.get(); }
};

class ImuExportConfig : public BaseExportConfig {
protected:
    bool exportData_;

public:
    ImuExportConfig(std::string deviceName = "imu", bool exportPoses = false, bool exportTimestamps = false, bool exportData = false)
        : BaseExportConfig(deviceName, exportPoses, exportTimestamps), exportData_(exportData) {}

    const bool& exportData() const { return exportData_; }
};


// DEVICE CLASSES

class RadarDevice : public BaseDevice {
protected:
    std::unique_ptr<RadarExportConfig> exportConfig_;
public:
    static inline const std::string name = "radar";
    RadarDevice(std::unique_ptr<RadarExportConfig> exportConfig = std::make_unique<RadarExportConfig>()): exportConfig_(std::move(exportConfig)) {}

    const RadarExportConfig* exportConfig() const override { return exportConfig_.get(); }
    virtual ~RadarDevice() = default;
};


class SingleChipDevice : public RadarDevice {
public:
    static inline const std::string name = "single_chip_radar";
    SingleChipDevice(std::unique_ptr<RadarExportConfig> exportConfig = std::make_unique<RadarExportConfig>()) : RadarDevice(std::move(exportConfig)) {}
};


class CascadeDevice : public RadarDevice {
public:
    static inline const std::string name = "cascade_radar";
    CascadeDevice(std::unique_ptr<RadarExportConfig> exportConfig = std::make_unique<RadarExportConfig>()) : RadarDevice(std::move(exportConfig)) {}
};


class LidarDevice : public BaseDevice {
protected:
    std::unique_ptr<LidarExportConfig> exportConfig_;
public:
    static inline const std::string name = "lidar";
    LidarDevice(std::unique_ptr<LidarExportConfig> exportConfig = std::make_unique<LidarExportConfig>()): exportConfig_(std::move(exportConfig)) {}

    const LidarExportConfig* exportConfig() const override { return exportConfig_.get(); }
    virtual ~LidarDevice() = default;
};


class ImuDevice : public BaseDevice {
protected:
    std::unique_ptr<ImuExportConfig> exportConfig_;
public:
    static inline const std::string name = "imu";
    ImuDevice(std::unique_ptr<ImuExportConfig> exportConfig = std::make_unique<ImuExportConfig>()): exportConfig_(std::move(exportConfig)) {}

    const ImuExportConfig* exportConfig() const override { return exportConfig_.get(); }
    virtual ~ImuDevice() = default;
};

} // namespace coloradar


#endif // DEVICE_H
