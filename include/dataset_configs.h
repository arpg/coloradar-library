#ifndef DATASET_CONFIGS_H
#define DATASET_CONFIGS_H

#include "device.h"


namespace coloradar {

class DatasetExportConfig {
protected:
    std::filesystem::path destinationFilePath_ = "dataset.h5";
    std::vector<std::string> runs_ = {};
    bool exportTransforms_ = false;

    RadarExportConfig        cascadeCfg_;
    LidarExportConfig        lidarCfg_;
    ImuExportConfig          imuCfg_;
    BaseExportConfig         baseCfg_;
    RadarExportConfig        singleChipCfg_;

    void validateConfigYaml(const YAML::Node &config);
    std::filesystem::path parseDestination(const YAML::Node &destinationValue, const std::filesystem::path &defaultDestination);
    std::vector<std::string> parseRuns(const YAML::Node &runsValue);

    std::filesystem::path validateDestination(const std::filesystem::path &destination);
    std::vector<std::string> validateRuns(const std::vector<std::string> &runs);

public:
    DatasetExportConfig(const std::string &yamlFilePath);

    DatasetExportConfig(
        const std::filesystem::path &destinationFilePath = "dataset.h5",
        const std::vector<std::string> &runs = {},
        bool exportTransforms = false,
        const RadarExportConfig &cascadeCfg = RadarExportConfig(),
        const LidarExportConfig &lidarCfg = LidarExportConfig(),
        const BaseExportConfig &baseCfg = BaseExportConfig(),
        const ImuExportConfig &imuCfg = ImuExportConfig(),
        const RadarExportConfig &singleChipCfg = RadarExportConfig()
    );

    const std::filesystem::path &destinationFilePath() const;
    const std::vector<std::string> &runs() const;
    bool exportTransforms() const;
    // const std::set<std::string> devices() const;

   const RadarExportConfig        &cascade() const { return cascadeCfg_; }
   const LidarExportConfig        &lidar() const { return lidarCfg_; }
   const BaseExportConfig         &base() const { return baseCfg_; }
   const ImuExportConfig          &imu() const { return imuCfg_; }
   // const RadarExportConfig        &singleChip() const { return ; }
};

}

#endif
