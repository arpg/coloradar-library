#include "device.h"


bool coloradar::validateDeviceName(const std::string &deviceName) {
    if (deviceName != coloradar::CascadeDevice::name && 
        deviceName != coloradar::SingleChipDevice::name && 
        deviceName != coloradar::LidarDevice::name && 
        deviceName != coloradar::ImuDevice::name && 
        deviceName != coloradar::BaseDevice::name) {
        return false;
    }
    return true;
}


namespace {

YAML::Node findNode(const YAML::Node &config, const std::string &nestedKey) {
    std::istringstream iss(nestedKey);
    std::string token;
    YAML::Node node = config;
    while (std::getline(iss, token, '.')) {
        if (!node[token])
            return YAML::Node();
        node = node[token];
    }
    return node;
}

bool parseBoolKey(const YAML::Node &config, const std::string &nestedKey, bool defaultValue) {
    YAML::Node node = findNode(config, nestedKey);
    if (!node.IsDefined() || node.IsNull()) {
        return defaultValue;
    }
    try {
        return node.as<bool>();
    } catch (const YAML::BadConversion &) {
        throw std::runtime_error("Malformed bool value for key: " + nestedKey);
    }
}

int parseIntKey(const YAML::Node &config, const std::string &nestedKey, int defaultValue) {
    YAML::Node node = findNode(config, nestedKey);
    if (!node.IsDefined() || node.IsNull()) {
        return defaultValue;
    }
    try {
        return node.as<int>();
    } catch (const YAML::BadConversion &) {
        throw std::runtime_error("Malformed int value for key: " + nestedKey);
    }
}

float parseFloatKey(const YAML::Node &config, const std::string &nestedKey, float defaultValue) {
    YAML::Node node = findNode(config, nestedKey);
    if (!node.IsDefined() || node.IsNull()) {
        return defaultValue;
    }
    try {
        return node.as<float>();
    } catch (const YAML::BadConversion &) {
        throw std::runtime_error("Malformed float value for key: " + nestedKey);
    }
}

std::string parseDeviceName(const YAML::Node &config, const std::string &nestedKey, std::string defaultValue) {
    YAML::Node node = findNode(config, nestedKey);
    if (!node.IsDefined() || node.IsNull()) {
        return defaultValue;
    }
    std::string deviceName = node.as<std::string>();
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
            throw std::runtime_error("Error in configuring " + deviceName_ + ": 'collapse_elevation_max_z_meters' must be greater or equal to 'collapse_elevation_min_z_meters'.");
        }
    }
    if (fov_.useDegreeConstraints) {
        if (fov_.horizontalDegreesTotal < 0 || fov_.horizontalDegreesTotal > 360) {
            throw std::runtime_error("Error in configuring " + deviceName_ + ": 'horizontal_fov_degrees_total' must be a float between 0 and 360.");
        }
        if (fov_.verticalDegreesTotal < 0 || fov_.verticalDegreesTotal > 180) {
            throw std::runtime_error("Error in configuring " + deviceName_ + ": 'vertical_fov_degrees_total' must be a float between 0 and 180.");
        }
    }
    if (exportClouds_) {
        if (intensityThresholdPercent_ < 0 || intensityThresholdPercent_ > 100) {
            throw std::runtime_error("Error in configuring " + deviceName_ + ": 'intensity_threshold_percent' must be a float between 0 and 100.");
        }
    }
}


void LidarExportConfig::validate() {
    BaseExportConfig::validate();
    if (collapseElevation_) {
        if (collapseElevationMaxZ_ < collapseElevationMinZ_) {
            throw std::runtime_error("Error in configuring " + deviceName_ + ": 'collapse_elevation_max_z_meters' must be greater or equal to 'collapse_elevation_min_z_meters'.");
        }
    }
    if (exportClouds_) {
        if (!cloudFov_.useDegreeConstraints) {
            throw std::runtime_error("Error in configuring " + deviceName_ + ".cloud_fov: cannot use idx fov constraints for this device.");
        }
        if (cloudFov_.horizontalDegreesTotal < 0 || cloudFov_.horizontalDegreesTotal > 360) {
            throw std::runtime_error("Error in configuring " + deviceName_ + ".cloud_fov: 'horizontal_fov_degrees_total' must be a float between 0 and 360.");
        }
        if (cloudFov_.verticalDegreesTotal < 0 || cloudFov_.verticalDegreesTotal > 180) {
            throw std::runtime_error("Error in configuring " + deviceName_ + ".cloud_fov: 'vertical_fov_degrees_total' must be a float between 0 and 180.");
        }
    }
    if (exportMap_ || exportMapSamples_) {
        if (occupancyThresholdPercent_ < 0 || occupancyThresholdPercent_ > 100) {
            throw std::runtime_error("Error in configuring " + deviceName_ + ": 'map_occupancy_threshold_percent' must be a float between 0 and 100.");
        }
    }
    if (exportMapSamples_) {
        if (!centerSensor_) {
            throw std::runtime_error("Error in configuring " + deviceName_ + ": center sensor cannot be set empty.");
        }
        if (forceResample_ && !allowResample_) {
            throw std::runtime_error("Error in configuring " + deviceName_ + ": cannot have 'force_resample' set to True when 'allow_resample' is set to False.");
        }
        if (!mapSampleFov_.useDegreeConstraints) {
            throw std::runtime_error("Error in configuring " + deviceName_ + ".map_sample_fov: cannot use idx fov constraints for this device.");
        }
        if (mapSampleFov_.horizontalDegreesTotal < 0 || mapSampleFov_.horizontalDegreesTotal > 360) {
            throw std::runtime_error("Error in configuring " + deviceName_ + ".map_sample_fov: 'horizontal_fov_degrees_total' must be a float between 0 and 360.");
        }
        if (mapSampleFov_.verticalDegreesTotal < 0 || mapSampleFov_.verticalDegreesTotal > 180) {
            throw std::runtime_error("Error in configuring " + deviceName_ + ".map_sample_fov: 'vertical_fov_degrees_total' must be a float between 0 and 180.");
        }
    }
}

void BaseExportConfig::loadFromFile(const YAML::Node& deviceNode) {
    exportPoses_ = parseBoolKey(deviceNode, "export_poses", exportPoses_);
    exportTimestamps_ = parseBoolKey(deviceNode, "export_timestamps", exportTimestamps_);
}

void RadarExportConfig::loadFromFile(const YAML::Node& deviceNode) {
    collapseElevation_ = parseBoolKey(deviceNode, "collapse_elevation", collapseElevation_);
    collapseElevationMinZ_ = parseFloatKey(deviceNode, "collapse_elevation_min_z_meters", collapseElevationMinZ_);
    collapseElevationMaxZ_ = parseFloatKey(deviceNode, "collapse_elevation_max_z_meters", collapseElevationMaxZ_);
    removeDopplerDim_ = parseBoolKey(deviceNode, "remove_doppler_dim", removeDopplerDim_);

    fov_.azimuthIdx = parseIntKey(deviceNode, "fov_azimuth_idx", fov_.azimuthIdx);
    fov_.elevationIdx = parseIntKey(deviceNode, "fov_elevation_idx", fov_.elevationIdx);
    fov_.horizontalDegreesTotal = parseFloatKey(deviceNode, "horizontal_fov_degrees_total", fov_.horizontalDegreesTotal);
    fov_.verticalDegreesTotal = parseFloatKey(deviceNode, "vertical_fov_degrees_total", fov_.verticalDegreesTotal);
    fov_.rangeMeters = parseFloatKey(deviceNode, "range_meters", fov_.rangeMeters);
    
    exportDatacubes_ = parseBoolKey(deviceNode, "export_datacubes", exportDatacubes_);
    exportHeatmaps_ = parseBoolKey(deviceNode, "export_heatmaps", exportHeatmaps_);
    exportClouds_ = parseBoolKey(deviceNode, "export_clouds", exportClouds_);
    intensityThresholdPercent_ = parseFloatKey(deviceNode, "intensity_threshold_percent", intensityThresholdPercent_);
    cloudsInGlobalFrame_ = parseBoolKey(deviceNode, "clouds_in_global_frame", cloudsInGlobalFrame_);
}

void LidarExportConfig::loadFromFile(const YAML::Node& deviceNode) {
    collapseElevation_ = parseBoolKey(deviceNode, "collapse_elevation", collapseElevation_);
    collapseElevationMinZ_ = parseFloatKey(deviceNode, "collapse_elevation_min_z_meters", collapseElevationMinZ_);
    collapseElevationMaxZ_ = parseFloatKey(deviceNode, "collapse_elevation_max_z_meters", collapseElevationMaxZ_);

    exportClouds_ = parseBoolKey(deviceNode, "export_clouds", exportClouds_);
    removeIntensityDim_ = parseBoolKey(deviceNode, "remove_intensity_dim", removeIntensityDim_);
    cloudFov_.horizontalDegreesTotal = parseFloatKey(deviceNode, "cloud_fov.horizontal_fov_degrees_total", cloudFov_.horizontalDegreesTotal);
    cloudFov_.verticalDegreesTotal = parseFloatKey(deviceNode, "cloud_fov.vertical_fov_degrees_total", cloudFov_.verticalDegreesTotal);
    cloudFov_.rangeMeters = parseFloatKey(deviceNode, "cloud_fov.range_meters", cloudFov_.rangeMeters);

    exportMap_ = parseBoolKey(deviceNode, "export_map", exportMap_);
    exportMapSamples_ = parseBoolKey(deviceNode, "export_map_samples", exportMapSamples_);
    removeOccupancyDim_ = parseBoolKey(deviceNode, "remove_occupancy_dim", removeOccupancyDim_);
    logOddsToProbability_ = parseBoolKey(deviceNode, "log_odds_to_probability", logOddsToProbability_);
    occupancyThresholdPercent_ = parseFloatKey(deviceNode, "occupancy_threshold_percent", occupancyThresholdPercent_);
    allowResample_ = parseBoolKey(deviceNode, "allow_resample", allowResample_);
    forceResample_ = parseBoolKey(deviceNode, "force_resample", forceResample_);
    mapSampleFov_.horizontalDegreesTotal = parseFloatKey(deviceNode, "map_sample_fov.horizontal_fov_degrees_total", mapSampleFov_.horizontalDegreesTotal);
    mapSampleFov_.verticalDegreesTotal = parseFloatKey(deviceNode, "map_sample_fov.vertical_fov_degrees_total", mapSampleFov_.verticalDegreesTotal);
    mapSampleFov_.rangeMeters = parseFloatKey(deviceNode, "map_sample_fov.range_meters", mapSampleFov_.rangeMeters);

    std::string centerSensorName = parseDeviceName(deviceNode, "center_sensor", centerSensor_->name);
    if (centerSensorName == BaseDevice::name) {
        centerSensor_ = std::make_unique<BaseDevice>();
    } else if (centerSensorName == LidarDevice::name) {
        centerSensor_ = std::make_unique<LidarDevice>();
    } else if (centerSensorName == CascadeDevice::name) {
        centerSensor_ = std::make_unique<CascadeDevice>();
    } else if (centerSensorName == SingleChipDevice::name) {
        centerSensor_ = std::make_unique<SingleChipDevice>();
    } else if (centerSensorName == ImuDevice::name) {
        centerSensor_ = std::make_unique<ImuDevice>();
    }
}

void ImuExportConfig::loadFromFile(const YAML::Node& deviceNode) {
    exportData_ = parseBoolKey(deviceNode, "export_data", exportData_);
}

}