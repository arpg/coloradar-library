#ifndef DATASET_H
#define DATASET_H

#include "coloradar_run.h"


namespace coloradar {

class ColoradarPlusDataset {
protected:
    std::filesystem::path datasetDirPath_;
    std::filesystem::path calibDirPath_;
    std::filesystem::path transformsDirPath_;
    std::filesystem::path runsDirPath_;

    Eigen::Affine3f imuTransform_;
    Eigen::Affine3f lidarTransform_;
    Eigen::Affine3f cascadeTransform_;

    RadarConfig* cascadeConfig_;

    ColoradarPlusDataset() = default;
    void init(const std::filesystem::path& pathToDataset);
    Eigen::Affine3f loadTransform(const std::filesystem::path& filePath);

    std::unique_ptr<BaseDevice> base_device_;
    std::unique_ptr<ImuDevice> imu_;
    std::unique_ptr<CascadeDevice> cascade_;
    std::unique_ptr<LidarDevice> lidar_;
    // std::vector<std::unique_ptr<BaseDevice>> devices;

    std::vector<std::string> exportBaseDevice(std::vector<ColoradarPlusRun*> runs, const H5::H5File &datasetFile);
    std::vector<std::string> exportImu(std::vector<ColoradarPlusRun*> runs, const H5::H5File &datasetFile);
    std::vector<std::string> exportCascade(std::vector<ColoradarPlusRun*> runs, const H5::H5File &datasetFile);
    std::vector<std::string> exportLidar(std::vector<ColoradarPlusRun*> runs, const H5::H5File &datasetFile);
    // void exportSingleChip(config); for old dataset
    // void exportCamera(config); later

public:
    explicit ColoradarPlusDataset(const std::filesystem::path& pathToDataset);
    ColoradarPlusDataset(const ColoradarPlusDataset&) = delete;
    ColoradarPlusDataset& operator=(const ColoradarPlusDataset&) = delete;
    ColoradarPlusDataset(ColoradarPlusDataset&&) noexcept = default;
    ColoradarPlusDataset& operator=(ColoradarPlusDataset&&) noexcept = default;

    std::vector<std::string> listRuns();
    std::vector<ColoradarPlusRun*> getRuns();

    virtual ColoradarPlusRun* getRun(const std::string& runName);

    const Eigen::Affine3f& imuTransform() const;
    const Eigen::Affine3f& lidarTransform() const;
    const Eigen::Affine3f& cascadeTransform() const;
    const RadarConfig* cascadeConfig() const;

    std::filesystem::path exportToFile(const DatasetExportConfig &exportConfig);
    std::filesystem::path exportToFile(const std::string &yamlConfigPath);
//    std::filesystem::path exportToFile(const DatasetExportConfig &exportConfig, std::vector<ColoradarPlusRun*> runs = {});
//    std::filesystem::path exportToFile(const std::string &yamlConfigPath, std::vector<ColoradarPlusRun*> runs = {});
};

class ColoradarDataset : public ColoradarPlusDataset {
protected:
    Eigen::Affine3f singleChipTransform_;

    RadarConfig* singleChipConfig_;
    std::unique_ptr<SingleChipDevice> single_chip_;

public:
    ColoradarDataset(const std::filesystem::path& pathToDataset);

    virtual ColoradarPlusRun* getRun(const std::string& runName) override;

    const Eigen::Affine3f& singleChipTransform() const;
    const RadarConfig* singleChipConfig() const;
};

}

#endif
