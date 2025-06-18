#include "device.h"


bool coloradar::validateDeviceName(const std::string &deviceName) {
    if (deviceName != (new coloradar::CascadeDevice())->name() &&
        deviceName != (new coloradar::SingleChipDevice())->name() &&
        deviceName != (new coloradar::LidarDevice())->name() &&
        deviceName != (new coloradar::ImuDevice())->name() &&
        deviceName != (new coloradar::BaseDevice())->name()) {
        return false;
    }
    return true;
}


namespace {

std::string parseDeviceName(const YAML::Node &nameNode, std::string defaultValue) {
    if (!nameNode) {
        return defaultValue;
    }
    std::string deviceName = nameNode.as<std::string>();
    if (!coloradar::validateDeviceName(deviceName)) {
        std::cerr << "Warning: Unknown device " << deviceName << ", skipping." << std::endl;
    }
   return deviceName;
}

}

namespace coloradar {

void RadarExportConfig::validate() {
    BaseExportConfig::validate();
    if (collapseElevation_) {
        if (collapseElevationMaxZ_ < collapseElevationMinZ_) {
            throw std::runtime_error("'collapse_elevation_max_z_meters' must be greater or equal to 'collapse_elevation_min_z_meters'.");
        }
    }
    if (fov_.useDegreeConstraints) {
        if (fov_.horizontalDegreesTotal < 0 || fov_.horizontalDegreesTotal > 360) {
            throw std::runtime_error("'horizontal_fov_degrees_total' must be a float between 0 and 360.");
        }
        if (fov_.verticalDegreesTotal < 0 || fov_.verticalDegreesTotal > 180) {
            throw std::runtime_error("'vertical_fov_degrees_total' must be a float between 0 and 180.");
        }
    }
    if (exportClouds_) {
        if (intensityThreshold_ < 0) {
            throw std::runtime_error("'intensity_threshold' must be non-negative.");
        }
    }
}


void LidarExportConfig::validate() {
    BaseExportConfig::validate();
    if (collapseElevation_) {
        if (collapseElevationMaxZ_ < collapseElevationMinZ_) {
            throw std::runtime_error("'collapse_elevation_max_z_meters' must be greater or equal to 'collapse_elevation_min_z_meters'.");
        }
    }
    if (exportClouds_) {
        if (!cloudFov_.useDegreeConstraints) {
            throw std::runtime_error("cloud_fov: cannot use idx fov constraints for this device.");
        }
        if (cloudFov_.horizontalDegreesTotal < 0 || cloudFov_.horizontalDegreesTotal > 360) {
            throw std::runtime_error("cloud_fov: 'horizontal_fov_degrees_total' must be a float between 0 and 360.");
        }
        if (cloudFov_.verticalDegreesTotal < 0 || cloudFov_.verticalDegreesTotal > 180) {
            throw std::runtime_error("cloud_fov: 'vertical_fov_degrees_total' must be a float between 0 and 180.");
        }
    }
    if (exportMap_ || exportMapSamples_) {
        if (occupancyThresholdPercent_ < 0 || occupancyThresholdPercent_ > 100) {
            throw std::runtime_error("'map_occupancy_threshold_percent' must be a float between 0 and 100.");
        }
    }
    if (exportMapSamples_) {
        if (!centerSensor_) {
            throw std::runtime_error("center sensor cannot be set empty.");
        }
        if (forceResample_ && !allowResample_) {
            throw std::runtime_error("cannot have 'force_resample' set to True when 'allow_resample' is set to False.");
        }
        if (!mapSampleFov_.useDegreeConstraints) {
            throw std::runtime_error(".map_sample_fov: cannot use idx fov constraints for this device.");
        }
        if (mapSampleFov_.horizontalDegreesTotal < 0 || mapSampleFov_.horizontalDegreesTotal > 360) {
            throw std::runtime_error(".map_sample_fov: 'horizontal_fov_degrees_total' must be a float between 0 and 360.");
        }
        if (mapSampleFov_.verticalDegreesTotal < 0 || mapSampleFov_.verticalDegreesTotal > 180) {
            throw std::runtime_error(".map_sample_fov: 'vertical_fov_degrees_total' must be a float between 0 and 180.");
        }
    }
}

void BaseExportConfig::loadFromFile(const YAML::Node& deviceNode) {
    if (!deviceNode) return;
    exportPoses_ = coloradar::internal::parseBoolYamlKey(deviceNode["export_poses"], exportPoses_);
    exportTimestamps_ = coloradar::internal::parseBoolYamlKey(deviceNode["export_timestamps"], exportTimestamps_);
}

void RadarExportConfig::loadFromFile(const YAML::Node& deviceNode) {
    BaseExportConfig::loadFromFile(deviceNode);

    collapseElevation_ = coloradar::internal::parseBoolYamlKey(deviceNode["collapse_elevation"], collapseElevation_);
    collapseElevationMinZ_ = coloradar::internal::parseFloatYamlKey(deviceNode["collapse_elevation_min_z_meters"], collapseElevationMinZ_);
    collapseElevationMaxZ_ = coloradar::internal::parseFloatYamlKey(deviceNode["collapse_elevation_max_z_meters"], collapseElevationMaxZ_);
    removeDopplerDim_ = coloradar::internal::parseBoolYamlKey(deviceNode["remove_doppler_dim"], removeDopplerDim_);

    int azimuthIdx = coloradar::internal::parseIntYamlKey(deviceNode["fov_azimuth_idx"], fov_.azimuthIdx);
    int elevationIdx = coloradar::internal::parseIntYamlKey(deviceNode["fov_elevation_idx"], fov_.elevationIdx);
    int horizontalDegreesTotal = coloradar::internal::parseFloatYamlKey(deviceNode["horizontal_fov_degrees_total"], fov_.horizontalDegreesTotal);
    int verticalDegreesTotal = coloradar::internal::parseFloatYamlKey(deviceNode["vertical_fov_degrees_total"], fov_.verticalDegreesTotal);
    fov_.rangeMeters = coloradar::internal::parseFloatYamlKey(deviceNode["range_meters"], fov_.rangeMeters);
    if (azimuthIdx >= 0 || elevationIdx >= 0                         // non-default idx values
       || (horizontalDegreesTotal == fov_.horizontalDegreesTotal     // default horizontal degree value
           && verticalDegreesTotal == fov_.verticalDegreesTotal)) {  // default vertical degree value
        fov_.useDegreeConstraints = false;
    }
    fov_.azimuthIdx = azimuthIdx;
    fov_.elevationIdx = elevationIdx;
    fov_.horizontalDegreesTotal = horizontalDegreesTotal;
    fov_.verticalDegreesTotal = verticalDegreesTotal;

    exportDatacubes_ = coloradar::internal::parseBoolYamlKey(deviceNode["export_datacubes"], exportDatacubes_);
    exportHeatmaps_ = coloradar::internal::parseBoolYamlKey(deviceNode["export_heatmaps"], exportHeatmaps_);
    exportClouds_ = coloradar::internal::parseBoolYamlKey(deviceNode["export_clouds"], exportClouds_);
    intensityThreshold_ = coloradar::internal::parseFloatYamlKey(deviceNode["intensity_threshold"], intensityThreshold_);
    cloudsInGlobalFrame_ = coloradar::internal::parseBoolYamlKey(deviceNode["clouds_in_global_frame"], cloudsInGlobalFrame_);
}

void LidarExportConfig::loadFromFile(const YAML::Node& deviceNode) {
    BaseExportConfig::loadFromFile(deviceNode);

    collapseElevation_ = coloradar::internal::parseBoolYamlKey(deviceNode["collapse_elevation"], collapseElevation_);
    collapseElevationMinZ_ = coloradar::internal::parseFloatYamlKey(deviceNode["collapse_elevation_min_z_meters"], collapseElevationMinZ_);
    collapseElevationMaxZ_ = coloradar::internal::parseFloatYamlKey(deviceNode["collapse_elevation_max_z_meters"], collapseElevationMaxZ_);

    exportClouds_ = coloradar::internal::parseBoolYamlKey(deviceNode["export_clouds"], exportClouds_);
    removeIntensityDim_ = coloradar::internal::parseBoolYamlKey(deviceNode["remove_intensity_dim"], removeIntensityDim_);

    auto cloudFovNode = deviceNode["cloud_fov"];
    if (cloudFovNode) {
        cloudFov_.horizontalDegreesTotal = coloradar::internal::parseFloatYamlKey(cloudFovNode["horizontal_degrees_total"], cloudFov_.horizontalDegreesTotal);
        cloudFov_.verticalDegreesTotal = coloradar::internal::parseFloatYamlKey(cloudFovNode["vertical_degrees_total"], cloudFov_.verticalDegreesTotal);
        cloudFov_.rangeMeters = coloradar::internal::parseFloatYamlKey(cloudFovNode["range_meters"], cloudFov_.rangeMeters);
    }

    exportMap_ = coloradar::internal::parseBoolYamlKey(deviceNode["export_map"], exportMap_);
    exportMapSamples_ = coloradar::internal::parseBoolYamlKey(deviceNode["export_map_samples"], exportMapSamples_);
    removeOccupancyDim_ = coloradar::internal::parseBoolYamlKey(deviceNode["remove_occupancy_dim"], removeOccupancyDim_);
    logOddsToProbability_ = coloradar::internal::parseBoolYamlKey(deviceNode["log_odds_to_probability"], logOddsToProbability_);
    occupancyThresholdPercent_ = coloradar::internal::parseFloatYamlKey(deviceNode["occupancy_threshold_percent"], occupancyThresholdPercent_);
    allowResample_ = coloradar::internal::parseBoolYamlKey(deviceNode["allow_resample"], allowResample_);
    forceResample_ = coloradar::internal::parseBoolYamlKey(deviceNode["force_resample"], forceResample_);
    saveSamples_ = coloradar::internal::parseBoolYamlKey(deviceNode["save_samples"], saveSamples_);

    auto mapFovNode = deviceNode["map_sample_fov"];
    mapSampleFov_.horizontalDegreesTotal = coloradar::internal::parseFloatYamlKey(mapFovNode["horizontal_degrees_total"], mapSampleFov_.horizontalDegreesTotal);
    mapSampleFov_.verticalDegreesTotal = coloradar::internal::parseFloatYamlKey(mapFovNode["vertical_degrees_total"], mapSampleFov_.verticalDegreesTotal);
    mapSampleFov_.rangeMeters = coloradar::internal::parseFloatYamlKey(mapFovNode["range_meters"], mapSampleFov_.rangeMeters);

    std::string centerSensorName = parseDeviceName(deviceNode["center_sensor"], centerSensor_->name());
    if (centerSensorName == (new BaseDevice())->name()) {
        centerSensor_ = std::make_unique<BaseDevice>();
    } else if (centerSensorName == (new LidarDevice())->name()) {
        centerSensor_ = std::make_unique<LidarDevice>();
    } else if (centerSensorName == (new CascadeDevice())->name()) {
        centerSensor_ = std::make_unique<CascadeDevice>();
    } else if (centerSensorName == (new SingleChipDevice())->name()) {
        centerSensor_ = std::make_unique<SingleChipDevice>();
    } else if (centerSensorName == (new ImuDevice())->name()) {
        centerSensor_ = std::make_unique<ImuDevice>();
    } else {
        throw std::runtime_error("Error: Unknown center sensor '" + centerSensorName + "'");
    }
}

void ImuExportConfig::loadFromFile(const YAML::Node& deviceNode) {
    BaseExportConfig::loadFromFile(deviceNode);
    exportData_ = coloradar::internal::parseBoolYamlKey(deviceNode["export_data"], exportData_);
}

}