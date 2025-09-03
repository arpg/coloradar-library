#include "dataset_configs.h"

#include <chrono>
#include <ctime>


namespace coloradar {


void DatasetExportConfig::validateConfigYaml(const YAML::Node &config) {
    for (auto nodeName : {"global", "devices"}) {
        if (!config[nodeName]) {
            throw std::runtime_error("Missing node: " + std::string(nodeName));
        }
    }
}

std::filesystem::path DatasetExportConfig::parseDestination(const YAML::Node &destinationValue, const std::filesystem::path &defaultDestination) {
    std::filesystem::path destination;
    if (!destinationValue) {
        destination = defaultDestination;
    } else {
        destination = destinationValue.as<std::string>();
    }
    return validateDestination(destination);
}


std::vector<std::string> DatasetExportConfig::parseRuns(const YAML::Node &runsValue) {
    if (!runsValue || runsValue.IsNull()) return {};
    std::vector<std::string> runs;

    if (runsValue.IsScalar()) {
        std::string run = runsValue.as<std::string>();
        runs.push_back(run);
    } else if (runsValue.IsSequence()) {
        for (const auto &n : runsValue) {
            if (!n.IsScalar()) throw std::runtime_error("Invalid 'runs' item: each entry must be a string.");
            std::string run = n.as<std::string>();
            runs.push_back(run);
        }
    } else throw std::runtime_error("Invalid format for 'runs' key. Expected a string or a list of strings.");
    return validateRuns(runs);
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
        if (run.empty()) throw std::runtime_error("Invalid run name: empty string.");
        if (run == "all") throw std::runtime_error("Invalid run name: 'all' is not allowed if other names are specified.");
        for (unsigned char c : run) {
            if (!(std::isalnum(c) || c == '_' || c == '-')) throw std::runtime_error(
                "Invalid run name \"" + run + "\"\nOnly letter, number, '_' or '-' characters are allowed."
                "\nUse correct YAML list formats:\n```\n  runs:\n    - run1\n    - run2\n```\nor as a flow sequence:\n```\n  runs: [run1, run2]\n```"
            );
        }
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
    destinationFilePath_ = parseDestination(config["global"]["destination_file"], destinationFilePath_);
    runs_ = parseRuns(config["global"]["runs"]);
    // std::cout << "DatasetExportConfig(): parsed " << runs_.size() << " runs: ";
    for (const auto &run : runs_) {
        std::cout << run << " ";
    }
    std::cout << std::endl;
    exportTransforms_ = coloradar::internal::parseBoolYamlKey(config["global"]["export_transforms"], exportTransforms_);

    cascadeCfg_.loadFromFile(config["devices"]["cascade_radar"]);
    lidarCfg_.loadFromFile(config["devices"]["lidar"]);
    baseCfg_.loadFromFile(config["devices"]["base"]);
    imuCfg_.loadFromFile(config["devices"]["imu"]);
    singleChipCfg_.loadFromFile(config["devices"]["single_chip_radar"]);

    std::vector<BaseExportConfig*> deviceConfigs = {&cascadeCfg_, &lidarCfg_, &baseCfg_, &imuCfg_, &singleChipCfg_};
    for (auto* deviceCfg : deviceConfigs) {
        if (lidarCfg_.centerSensor() && typeid(*lidarCfg_.centerSensor()->exportConfig()) == typeid(*deviceCfg)) {
            lidarCfg_.centerSensor()->loadExportConfig(deviceCfg);
        }
    }
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

void DatasetExportConfig::initDataset(std::vector<std::string> runNames, std::span<const std::unique_ptr<BaseDevice>> devices) {
    if (!runNames.empty()) {
        runs_ = validateRuns(runNames);
    }
    if (!devices.empty()) {
        devices_ = devices;
    }
}

}
