#include "dataset_configs.h"

#include <chrono>
#include <ctime>


namespace coloradar {

YAML::Node DatasetExportConfig::findNode(const YAML::Node &config, const std::string &nestedKey) {
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

void DatasetExportConfig::validateConfigYaml(const YAML::Node &config) {
    YAML::Node devicesNode = findNode(config, "devices");
    if (!devicesNode.IsDefined() || devicesNode.IsNull()) {
        return;
    }
//    for (const auto &device : devicesNode) {
//        std::string deviceName = device.first.as<std::string>();
//        if (devices_.find(deviceName) == devices_.end()) {
//            std::cerr << "Warning: Unknown device " << deviceName << ", skipping." << std::endl;
//        }
//        else
//    }
}

std::filesystem::path DatasetExportConfig::parseDestination(const YAML::Node &config, const std::filesystem::path &defaultDestination) {
    YAML::Node node = findNode(config, "global.destination");
    std::filesystem::path destination;
    if (!node.IsDefined() || node.IsNull()) {
        destination = defaultDestination;
    } else {
        destination = node.as<std::string>();
    }
    return validateDestination(destination);
}

std::vector<std::string> DatasetExportConfig::parseRuns(const YAML::Node &config) {
    YAML::Node node = findNode(config, "global.runs");
    std::vector<std::string> runs;
    if (!node.IsDefined() || node.IsNull()) {
        return runs;
    }
    if (node.IsScalar()) {
        std::string run = node.as<std::string>();
        runs.push_back(run);
    } else if (node.IsSequence()) {
        for (const auto &n : node) {
            std::string run = n.as<std::string>();
            runs.push_back(run);
        }
    } else {
        throw std::runtime_error("Invalid format for 'runs' key.");
    }
    return validateRuns(runs);
}

//std::string DatasetExportConfig::parseDeviceName(const YAML::Node &config, const std::string &nestedKey, std::string defaultValue) {
//    YAML::Node node = findNode(config, nestedKey);
//    if (!node.IsDefined() || node.IsNull()) {
//        return defaultValue;
//    }
//    std::string deviceName = node.as<std::string>();
//    validateDeviceName(deviceName);
//    return deviceName;
//}

bool DatasetExportConfig::parseBoolKey(const YAML::Node &config, const std::string &nestedKey, bool defaultValue) {
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

int DatasetExportConfig::parseIntKey(const YAML::Node &config, const std::string &nestedKey, int defaultValue) {
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

float DatasetExportConfig::parseFloatKey(const YAML::Node &config, const std::string &nestedKey, float defaultValue) {
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

std::filesystem::path DatasetExportConfig::validateDestination(const std::filesystem::path &destination) {
    std::filesystem::path absoluteDestination = std::filesystem::absolute(destination);
    std::filesystem::path directoriesPath = absoluteDestination.parent_path();
    std::string filename = absoluteDestination.filename().string();
    if (!std::filesystem::exists(directoriesPath)) {
        std::filesystem::create_directories(directoriesPath);
    }
    if (filename.empty()) {
        auto now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);
        char buffer[100];
        std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", std::localtime(&now_c));
        filename = "dataset_" + std::string(buffer) + ".h5";
    }
    if (std::filesystem::path(filename).extension() != ".h5") {
        filename += ".h5";
    }
    return directoriesPath / filename;
}

std::vector<std::string> DatasetExportConfig::validateRuns(const std::vector<std::string> &runs) {
    if (runs.size() == 1 && (runs[0].empty() || runs[0] == "all")) return {};
    for (const auto &run : runs) {
        if (run.empty()) throw std::runtime_error("empty string is not allowed in the list of runs.");
        if (run == "all") throw std::runtime_error("'all' is not allowed in the list of runs.");
    }
    return runs;
}


DatasetExportConfig::DatasetExportConfig(const std::string &yamlFilePath) {
    YAML::Node config = YAML::LoadFile(yamlFilePath);
    validateConfigYaml(config);
    destinationFilePath_ = parseDestination(config, destinationFilePath_);
    runs_ = parseRuns(config);
    exportTransforms_ = parseBoolKey(config, "global.export_transforms", exportTransforms_);

//    cascade_.exportPoses = parseBoolKey(config, "devices.cascade_radar.export_poses", cascade_.exportPoses);
//    cascade_.exportTimestamps = parseBoolKey(config, "devices.cascade_radar.export_timestamps", cascade_.exportTimestamps);
//    cascade_.fov.azimuthIdx = parseIntKey(config, "devices.cascade_radar.fov_azimuth_idx", cascade_.fov.azimuthIdx);
//    cascade_.fov.elevationIdx = parseIntKey(config, "devices.cascade_radar.fov_elevation_idx", cascade_.fov.elevationIdx);
//    cascade_.fov.horizontalDegreesTotal = parseFloatKey(config, "devices.cascade_radar.horizontal_fov_degrees_total", cascade_.fov.horizontalDegreesTotal);
//    cascade_.fov.verticalDegreesTotal = parseFloatKey(config, "devices.cascade_radar.vertical_fov_degrees_total", cascade_.fov.verticalDegreesTotal);
//    cascade_.fov.rangeMeters = parseFloatKey(config, "devices.cascade_radar.range_meters", cascade_.fov.rangeMeters);
//    cascade_.collapseElevation = parseBoolKey(config, "devices.cascade_radar.collapse_elevation", cascade_.collapseElevation);
//    cascade_.collapseElevationMinZ = parseFloatKey(config, "devices.cascade_radar.collapse_elevation_min_z_meters", cascade_.collapseElevationMinZ);
//    cascade_.collapseElevationMaxZ = parseFloatKey(config, "devices.cascade_radar.collapse_elevation_max_z_meters", cascade_.collapseElevationMaxZ);
//    cascade_.removeDopplerDim = parseBoolKey(config, "devices.cascade_radar.remove_doppler_dim", cascade_.removeDopplerDim);
//    cascade_.exportDatacubes = parseBoolKey(config, "devices.cascade_radar.export_datacubes", cascade_.exportDatacubes);
//    cascade_.exportHeatmaps = parseBoolKey(config, "devices.cascade_radar.export_heatmaps", cascade_.exportHeatmaps);
//    cascade_.exportClouds = parseBoolKey(config, "devices.cascade_radar.export_clouds", cascade_.exportClouds);
//    cascade_.intensityThresholdPercent = parseFloatKey(config, "devices.cascade_radar.intensity_threshold_percent", cascade_.intensityThresholdPercent);
//    cascade_.cloudsInGlobalFrame = parseBoolKey(config, "devices.cascade_radar.clouds_in_global_frame", cascade_.cloudsInGlobalFrame);
//
//    lidar_.exportPoses = parseBoolKey(config, "devices.lidar.export_poses", lidar_.exportPoses);
//    lidar_.exportTimestamps = parseBoolKey(config, "devices.lidar.export_timestamps", lidar_.exportTimestamps);
//    lidar_.collapseElevation = parseBoolKey(config, "devices.lidar.collapse_elevation", lidar_.collapseElevation);
//    lidar_.collapseElevationMinZ = parseFloatKey(config, "devices.lidar.collapse_elevation_min_z_meters", lidar_.collapseElevationMinZ);
//    lidar_.collapseElevationMaxZ = parseFloatKey(config, "devices.lidar.collapse_elevation_max_z_meters", lidar_.collapseElevationMaxZ);
//    lidar_.exportClouds = parseBoolKey(config, "devices.lidar.export_clouds", lidar_.exportClouds);
//    lidar_.removeIntensityDim = parseBoolKey(config, "devices.lidar.remove_intensity_dim", lidar_.removeIntensityDim);
//    lidar_.cloudFov.horizontalDegreesTotal = parseFloatKey(config, "devices.lidar.cloud_fov.horizontal_fov_degrees_total", lidar_.cloudFov.horizontalDegreesTotal);
//    lidar_.cloudFov.verticalDegreesTotal = parseFloatKey(config, "devices.lidar.cloud_fov.vertical_fov_degrees_total", lidar_.cloudFov.verticalDegreesTotal);
//    lidar_.cloudFov.rangeMeters = parseFloatKey(config, "devices.lidar.cloud_fov.range_meters", lidar_.cloudFov.rangeMeters);
//    lidar_.exportMap = parseBoolKey(config, "devices.lidar.export_map", lidar_.exportMap);
//    lidar_.mapOccupancyThresholdPercent = parseFloatKey(config, "devices.lidar.map_occupancy_threshold_percent", lidar_.mapOccupancyThresholdPercent);
//    lidar_.exportMapSamples = parseBoolKey(config, "devices.lidar.export_map_samples", lidar_.exportMapSamples);
//    lidar_.mapSampleOccupancyThresholdPercent = parseFloatKey(config, "devices.lidar.map_sample_occupancy_threshold_percent", lidar_.mapSampleOccupancyThresholdPercent);
//    lidar_.allowResample = parseBoolKey(config, "devices.lidar.allow_resample", lidar_.allowResample);
//    lidar_.forceResample = parseBoolKey(config, "devices.lidar.force_resample", lidar_.forceResample);
//    lidar_.centerSensor = parseDeviceName(config, "devices.lidar.center_sensor", lidar_.centerSensor);
//    lidar_.mapSampleFov.horizontalDegreesTotal = parseFloatKey(config, "devices.lidar.map_sample_fov.horizontal_fov_degrees_total", lidar_.mapSampleFov.horizontalDegreesTotal);
//    lidar_.mapSampleFov.verticalDegreesTotal = parseFloatKey(config, "devices.lidar.map_sample_fov.vertical_fov_degrees_total", lidar_.mapSampleFov.verticalDegreesTotal);
//    lidar_.mapSampleFov.rangeMeters = parseFloatKey(config, "devices.lidar.map_sample_fov.range_meters", lidar_.mapSampleFov.rangeMeters);
//
//    base_.exportPoses = parseBoolKey(config, "devices.base_frame.export_poses", base_.exportPoses);
//    base_.exportTimestamps = parseBoolKey(config, "devices.base_frame.export_timestamps", base_.exportTimestamps);
//
//    imu_.exportData = parseBoolKey(config, "devices.imu.export_data", imu_.exportData);
//    imu_.exportPoses = parseBoolKey(config, "devices.imu.export_poses", imu_.exportPoses);
//    imu_.exportTimestamps = parseBoolKey(config, "devices.imu.export_timestamps", imu_.exportTimestamps);
//
//    singleChip_.exportPoses = parseBoolKey(config, "devices.single_chip_radar.export_poses", singleChip_.exportPoses);
//    singleChip_.exportTimestamps = parseBoolKey(config, "devices.single_chip_radar.export_timestamps", singleChip_.exportTimestamps);
//    singleChip_.fov.azimuthIdx = parseIntKey(config, "devices.single_chip_radar.fov_azimuth_idx", singleChip_.fov.azimuthIdx);
//    singleChip_.fov.elevationIdx = parseIntKey(config, "devices.single_chip_radar.fov_elevation_idx", singleChip_.fov.elevationIdx);
//    singleChip_.fov.rangeMeters = parseFloatKey(config, "devices.single_chip_radar.fov_range_meters", singleChip_.fov.rangeMeters);
//    singleChip_.collapseElevation = parseBoolKey(config, "devices.single_chip_radar.collapse_elevation", singleChip_.collapseElevation);
//    singleChip_.collapseElevationMinZ = parseFloatKey(config, "devices.single_chip_radar.collapse_elevation_min_z_meters", singleChip_.collapseElevationMinZ);
//    singleChip_.collapseElevationMaxZ = parseFloatKey(config, "devices.single_chip_radar.collapse_elevation_max_z_meters", singleChip_.collapseElevationMaxZ);
//    singleChip_.removeDopplerDim = parseBoolKey(config, "devices.single_chip_radar.remove_doppler_dim", singleChip_.removeDopplerDim);
//    singleChip_.exportDatacubes = parseBoolKey(config, "devices.single_chip_radar.export_datacubes", singleChip_.exportDatacubes);
//    singleChip_.exportHeatmaps = parseBoolKey(config, "devices.single_chip_radar.export_heatmaps", singleChip_.exportHeatmaps);
//    singleChip_.exportClouds = parseBoolKey(config, "devices.single_chip_radar.export_clouds", singleChip_.exportClouds);
//    singleChip_.intensityThresholdPercent = parseFloatKey(config, "devices.single_chip_radar.intensity_threshold_percent", singleChip_.intensityThresholdPercent);
//    singleChip_.cloudsInGlobalFrame = parseBoolKey(config, "devices.single_chip_radar.clouds_in_global_frame", singleChip_.cloudsInGlobalFrame);
}

//DatasetExportConfig::DatasetExportConfig(
//    const std::filesystem::path &destinationFilePath,
//    const std::vector<std::string> &runs,
//    bool exportTransforms,
//    const RadarExportConfig &cascade,
//    const LidarExportConfig &lidar,
//    const BaseExportConfig &base,
//    const ImuExportConfig &imu,
//    const RadarExportConfig &singleChip
//)
//    : destinationFilePath_(validateDestination(destinationFilePath)),
//      runs_(validateRuns(runs)),
//      exportTransforms_(exportTransforms),
//      cascade_(cascade),
//      lidar_(lidar),
//      base_(base),
//      imu_(imu),
//      singleChip_(singleChip)
//{
//    validateDeviceName(lidar_.centerSensor);
//}

const std::filesystem::path &DatasetExportConfig::destinationFilePath() const {
    return destinationFilePath_;
}

const std::vector<std::string> &DatasetExportConfig::runs() const {
    return runs_;
}

bool DatasetExportConfig::exportTransforms() const {
    return exportTransforms_;
}

//const std::set<std::string> &DatasetExportConfig::devices() const {
//    return devices_;
//}
//
//const RadarExportConfig &DatasetExportConfig::cascade() const {
//    return cascade_;
//}
//
//const LidarExportConfig &DatasetExportConfig::lidar() const {
//    return lidar_;
//}
//
//const BaseExportConfig &DatasetExportConfig::base() const {
//    return base_;
//}
//
//const ImuExportConfig &DatasetExportConfig::imu() const {
//    return imu_;
//}
//
//const RadarExportConfig &DatasetExportConfig::singleChip() const {
//    return singleChip_;
//}

//void DatasetExportConfig::fitRadarParameters(RadarExportConfig* radarConfig, ColoradarPlusDataset* dataset) {
//    if
//}

//void DatasetExportConfig::fitParameters(ColoradarPlusDataset* dataset) {
//    std::vector<std::string> existingRuns = dataset->listRuns();
//    if (runs_.empty()) {
//        runs_ = existingRuns;
//    } else {
//        std::vector<std::string> finalRuns = {};
//        for (const auto &runName : runs_) {
//            if std::find(existingRuns.begin(), existingRuns.end(), runName) != existingRuns.end() {
//                finalRuns.push_back(runName);
//            } else {
//                std::cerr << "Warning: Unknown run " << runName << ", skipping." << std::endl;
//            }
//        }
//        if (finalRuns.empty()) {
//            throw std::runtime_error("No valid runs specified.");
//        }
//        runs_ = finalRuns;
//    }
//
//    for (auto radarConfig : {cascade_, singleChip_}) {
//
//    }
//}

}
