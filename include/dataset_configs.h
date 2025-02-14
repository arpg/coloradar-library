#ifndef DATASET_CONFIGS_H
#define DATASET_CONFIGS_H

#include "device.h"


namespace coloradar {

class DatasetExportConfig {
protected:
    std::filesystem::path destinationFilePath_ = "dataset.h5";
    std::vector<std::string> runs_ = {};
    bool exportTransforms_ = false;

//    RadarExportConfig        cascade_;
//    LidarExportConfig        lidar_;
//    BaseExportConfig         base_;
//    ImuExportConfig          imu_;
//    RadarExportConfig        singleChip_;

    YAML::Node findNode(const YAML::Node &config, const std::string &key);
    void validateConfigYaml(const YAML::Node &config);
    std::filesystem::path parseDestination(const YAML::Node &config, const std::filesystem::path &defaultDestination);
    std::vector<std::string> parseRuns(const YAML::Node &config);
    bool parseBoolKey(const YAML::Node &config, const std::string &key, bool defaultValue);
    int parseIntKey(const YAML::Node &config, const std::string &key, int defaultValue);
    float parseFloatKey(const YAML::Node &config, const std::string &key, float defaultValue);
    // std::string parseDeviceName(const YAML::Node &config, const std::string &key, std::string defaultValue);

    std::filesystem::path validateDestination(const std::filesystem::path &destination);
    std::vector<std::string> validateRuns(const std::vector<std::string> &runs);

public:
    DatasetExportConfig(const std::string &yamlFilePath);

//    DatasetExportConfig(
//        const std::filesystem::path &destinationFilePath = "dataset.h5",
//        const std::vector<std::string> &runs = {},
//        bool exportTransforms = false,
//        const RadarExportConfig &cascade = RadarExportConfig(),
//        const LidarExportConfig &lidar = LidarExportConfig(),
//        const BaseExportConfig &base = BaseExportConfig(),
//        const ImuExportConfig &imu = ImuExportConfig(),
//        const RadarExportConfig &singleChip = RadarExportConfig()
//    );

    const std::filesystem::path &destinationFilePath() const;
    const std::vector<std::string> &runs() const;
    bool exportTransforms() const;
    // const std::set<std::string> devices() const;

//    const RadarExportConfig        &cascade() const;
//    const LidarExportConfig        &lidar() const;
//    const BaseExportConfig         &base() const;
//    const ImuExportConfig          &imu() const;
//    const RadarExportConfig        &singleChip() const;

    // void fitParameters(ColoradarPlusDataset* dataset);

    // void exportConfig(config);
    // void exportCascade(ColoradarPlusDataset* dataset);
    // void exportLidar(config);
    // void exportImu(config);
    // void exportBaseFrame(config);
    // void exportSingleChip(config); for old dataset
    // void exportCamera(config); later
};

}

#endif
