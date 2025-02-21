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

DatasetExportConfig::DatasetExportConfig(
    const std::filesystem::path &destinationFilePath,
    const std::vector<std::string> &runs,
    bool exportTransforms,
    const RadarExportConfig &cascadeCfg,
    const LidarExportConfig &lidarCfg,
    const BaseExportConfig &baseCfg,
    const ImuExportConfig &imuCfg,
    const RadarExportConfig &singleChipCfg
) : destinationFilePath_(validateDestination(destinationFilePath)),
    runs_(validateRuns(runs)),
    exportTransforms_(exportTransforms),
    cascadeCfg_(cascadeCfg),
    lidarCfg_(lidarCfg),
    baseCfg_(baseCfg),
    imuCfg_(imuCfg),
    singleChipCfg_(singleChipCfg) {}


DatasetExportConfig::DatasetExportConfig(const std::string &yamlFilePath) {
    YAML::Node config = YAML::LoadFile(yamlFilePath);
    validateConfigYaml(config);
    destinationFilePath_ = parseDestination(config, destinationFilePath_);
    runs_ = parseRuns(config);
    exportTransforms_ = parseBoolKey(config, "global.export_transforms", exportTransforms_);

    // for (auto* device in {&cascadeCfg_ &baseCfg_, &imuCfg_, &singleChipCfg_}) {
        //if (lidarCfg_.deviceName() == device->name {
            
        //}
    //     device->loadFromFile(config);
    // }

    // set lidar center device
}

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
