#ifndef DEVICE_H
#define DEVICE_H

#include "utils.h"


namespace coloradar {

bool validateDeviceName(const std::string &deviceName);

// BASE CLASSES

class BaseExportConfig {
protected:
    bool exportPoses_;
    bool exportTimestamps_;

    virtual void validate() {}
public:
    BaseExportConfig(bool exportPoses = false, bool exportTimestamps = false) : exportPoses_(exportPoses), exportTimestamps_(exportTimestamps) { validate(); }
    BaseExportConfig(const BaseExportConfig& other) = default;
    BaseExportConfig& operator=(const BaseExportConfig& other) = default;

    const bool& exportPoses() const { return exportPoses_; }
    const bool& exportTimestamps() const { return exportTimestamps_; }

    virtual void loadFromFile(const YAML::Node& deviceNode);
};

class BaseDevice {
protected:
    std::unique_ptr<BaseExportConfig> exportConfig_;
public:
    virtual std::string name() const { return "base"; }

    BaseDevice(std::unique_ptr<BaseExportConfig> exportConfig = std::make_unique<BaseExportConfig>())
        : exportConfig_(std::move(exportConfig)) {}
        
    BaseDevice(const BaseDevice& other)
        : exportConfig_(other.exportConfig_ ? std::make_unique<BaseExportConfig>(*other.exportConfig_) : nullptr) {}

    BaseDevice& operator=(const BaseDevice& other) {
        if (this != &other) {
            exportConfig_ = other.exportConfig_ ? std::make_unique<BaseExportConfig>(*other.exportConfig_) : nullptr;
        }
        return *this;
    }

    void loadExportConfig(const BaseExportConfig* config) const { if (config) *exportConfig_ = *config; }
    const virtual BaseExportConfig* exportConfig() const { return exportConfig_.get(); }

    virtual ~BaseDevice() = default;
};


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

    ) : BaseExportConfig(exportPoses, exportTimestamps),
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
    RadarExportConfig(const RadarExportConfig& other) = default;
    RadarExportConfig& operator=(const RadarExportConfig& other) = default;

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

    void loadFromFile(const YAML::Node& deviceNode) override;
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
    bool exportMapSamples_;
    bool removeOccupancyDim_;
    bool logOddsToProbability_;
    float occupancyThresholdPercent_;
    bool allowResample_;
    bool forceResample_;
    bool saveSamples_;
    std::unique_ptr<BaseDevice> centerSensor_;
    FovConfig mapSampleFov_;

    virtual void validate() override;

public:
    LidarExportConfig(
        bool exportPoses = false,
        bool exportTimestamps = false,

        bool collapseElevation = false,
        float collapseElevationMinZ = 0.0f,
        float collapseElevationMaxZ = 0.0f,

        bool exportClouds = false,
        bool removeIntensityDim = false,
        FovConfig cloudFov = FovConfig(),

        bool exportMap = false,
        bool exportMapSamples = false,
        bool removeOccupancyDim = false,
        bool logOddsToProbability = false,
        float occupancyThresholdPercent = 0.0f,
        bool allowResample = true,
        bool forceResample = false,
        bool saveSamples = false,
        std::unique_ptr<BaseDevice> centerSensor = std::make_unique<BaseDevice>(),
        FovConfig mapSampleFov = FovConfig()

    ) : BaseExportConfig(exportPoses, exportTimestamps),
        collapseElevation_(collapseElevation),
        collapseElevationMinZ_(collapseElevationMinZ),
        collapseElevationMaxZ_(collapseElevationMaxZ),
        exportClouds_(exportClouds),
        removeIntensityDim_(removeIntensityDim),
        cloudFov_(cloudFov),
        exportMap_(exportMap),
        exportMapSamples_(exportMapSamples),
        removeOccupancyDim_(removeOccupancyDim),
        logOddsToProbability_(logOddsToProbability),
        occupancyThresholdPercent_(occupancyThresholdPercent),
        allowResample_(allowResample),
        forceResample_(forceResample),
        saveSamples_(saveSamples),
        centerSensor_(std::move(centerSensor)),
        mapSampleFov_(mapSampleFov) {
        validate();
    }
    // Copy
    LidarExportConfig(const LidarExportConfig& other) 
    : BaseExportConfig(other),
      collapseElevation_(other.collapseElevation_),
      collapseElevationMinZ_(other.collapseElevationMinZ_),
      collapseElevationMaxZ_(other.collapseElevationMaxZ_),
      exportClouds_(other.exportClouds_),
      removeIntensityDim_(other.removeIntensityDim_),
      cloudFov_(other.cloudFov_),
      exportMap_(other.exportMap_),
      exportMapSamples_(other.exportMapSamples_),
      removeOccupancyDim_(other.removeOccupancyDim_),
      logOddsToProbability_(other.logOddsToProbability_),
      occupancyThresholdPercent_(other.occupancyThresholdPercent_),
      allowResample_(other.allowResample_),
      forceResample_(other.forceResample_),
      saveSamples_(other.saveSamples_),
      centerSensor_(other.centerSensor_ ? std::make_unique<BaseDevice>(*other.centerSensor_) : nullptr),
      mapSampleFov_(other.mapSampleFov_) {}

    // Assignment
    LidarExportConfig& operator=(const LidarExportConfig& other) {
        if (this != &other) {
            BaseExportConfig::operator=(other);
            collapseElevation_ = other.collapseElevation_;
            collapseElevationMinZ_ = other.collapseElevationMinZ_;
            collapseElevationMaxZ_ = other.collapseElevationMaxZ_;
            exportClouds_ = other.exportClouds_;
            removeIntensityDim_ = other.removeIntensityDim_;
            cloudFov_ = other.cloudFov_;
            exportMap_ = other.exportMap_;
            exportMapSamples_ = other.exportMapSamples_;
            removeOccupancyDim_ = other.removeOccupancyDim_;
            logOddsToProbability_ = other.logOddsToProbability_;
            occupancyThresholdPercent_ = other.occupancyThresholdPercent_;
            allowResample_ = other.allowResample_;
            forceResample_ = other.forceResample_;
            saveSamples_ = other.saveSamples_;
            centerSensor_ = other.centerSensor_ ? std::make_unique<BaseDevice>(*other.centerSensor_) : nullptr;
            mapSampleFov_ = other.mapSampleFov_;
        }
        return *this;
    }

    const bool& collapseElevation() const { return collapseElevation_; }
    const float& collapseElevationMinZ() const { return collapseElevationMinZ_; }
    const float& collapseElevationMaxZ() const { return collapseElevationMaxZ_; }
    const bool& exportClouds() const { return exportClouds_; }
    const bool& removeIntensityDim() const { return removeIntensityDim_; }
    const FovConfig& cloudFov() const { return cloudFov_; }
    const bool& exportMap() const { return exportMap_; }
    const bool& exportMapSamples() const { return exportMapSamples_; }
    const bool& removeOccupancyDim() const { return removeOccupancyDim_; }
    const bool& logOddsToProbability() const { return logOddsToProbability_; }
    const float& occupancyThresholdPercent() const { return occupancyThresholdPercent_; }
    const bool& allowResample() const { return allowResample_; }
    const bool& forceResample() const { return forceResample_; }
    const bool& saveSamples() const { return saveSamples_; }
    const BaseDevice* centerSensor() const { return centerSensor_.get(); }
    const FovConfig& mapSampleFov() const { return mapSampleFov_; }

    void loadFromFile(const YAML::Node& deviceNode) override;
};

class ImuExportConfig : public BaseExportConfig {
protected:
    bool exportData_;

public:
    ImuExportConfig(bool exportPoses = false, bool exportTimestamps = false, bool exportData = false) : BaseExportConfig(exportPoses, exportTimestamps), exportData_(exportData) {}

    const bool& exportData() const { return exportData_; }

    void loadFromFile(const YAML::Node& deviceNode) override;
};


// DEVICE CLASSES

class RadarDevice : public BaseDevice {
protected:
    std::unique_ptr<RadarExportConfig> exportConfig_;

public:
    std::string name() const override { return "radar"; }

    RadarDevice(std::unique_ptr<RadarExportConfig> exportConfig = std::make_unique<RadarExportConfig>()) : exportConfig_(std::move(exportConfig)) {}
    RadarDevice(const RadarDevice& other) : BaseDevice(other), exportConfig_(other.exportConfig_ ? std::make_unique<RadarExportConfig>(*other.exportConfig_) : nullptr) {}
    RadarDevice& operator=(const RadarDevice& other) {
        if (this != &other) {
            BaseDevice::operator=(other);
            exportConfig_ = other.exportConfig_ ? std::make_unique<RadarExportConfig>(*other.exportConfig_) : nullptr;
        }
        return *this;
    }

    const RadarExportConfig* exportConfig() const override { return exportConfig_.get(); }
    virtual ~RadarDevice() = default;
};
    
class LidarDevice : public BaseDevice {
protected:
    std::unique_ptr<LidarExportConfig> exportConfig_;

public:
    std::string name() const override { return "lidar"; }

    LidarDevice(std::unique_ptr<LidarExportConfig> exportConfig = std::make_unique<LidarExportConfig>()): exportConfig_(std::move(exportConfig)) {}
    LidarDevice(const LidarDevice& other) : BaseDevice(other), exportConfig_(other.exportConfig_ ? std::make_unique<LidarExportConfig>(*other.exportConfig_) : nullptr) {}
    LidarDevice& operator=(const LidarDevice& other) {
        if (this != &other) {
            BaseDevice::operator=(other);
            exportConfig_ = other.exportConfig_ ? std::make_unique<LidarExportConfig>(*other.exportConfig_) : nullptr;
        }
        return *this;
    }

    const LidarExportConfig* exportConfig() const override { return exportConfig_.get(); }
    virtual ~LidarDevice() = default;
};
    


class SingleChipDevice : public RadarDevice {
public:
    std::string name() const override { return "single_chip_radar"; }
    SingleChipDevice(std::unique_ptr<RadarExportConfig> exportConfig = std::make_unique<RadarExportConfig>()) : RadarDevice(std::move(exportConfig)) {}
};


class CascadeDevice : public RadarDevice {
public:
    std::string name() const override { return "cascade_radar"; }
    CascadeDevice(std::unique_ptr<RadarExportConfig> exportConfig = std::make_unique<RadarExportConfig>()) : RadarDevice(std::move(exportConfig)) {}
};


class ImuDevice : public BaseDevice {
protected:
    std::unique_ptr<ImuExportConfig> exportConfig_;
public:
    std::string name() const override { return "imu"; }
    ImuDevice(std::unique_ptr<ImuExportConfig> exportConfig = std::make_unique<ImuExportConfig>()): exportConfig_(std::move(exportConfig)) {}

    const ImuExportConfig* exportConfig() const override { return exportConfig_.get(); }
    virtual ~ImuDevice() = default;
};

} // namespace coloradar


#endif // DEVICE_H
