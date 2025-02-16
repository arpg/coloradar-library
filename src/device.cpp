#include "device.h"


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
    if (exportMap_) {
        if (mapOccupancyThresholdPercent_ < 0 || mapOccupancyThresholdPercent_ > 100) {
            throw std::runtime_error("Error in configuring " + deviceName_ + ": 'map_occupancy_threshold_percent' must be a float between 0 and 100.");
        }
    }
    if (exportMapSamples_) {
        if (!centerSensor_) {
            throw std::runtime_error("Error in configuring " + deviceName_ + ": center sensor cannot be set empty.");
        }
        if (mapSampleOccupancyThresholdPercent_ < 0 || mapSampleOccupancyThresholdPercent_ > 100) {
            throw std::runtime_error("Error in configuring " + deviceName_ + ": 'map_sample_occupancy_threshold_percent' must be a float between 0 and 100.");
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


}