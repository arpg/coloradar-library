#ifndef DATASET_H
#define DATASET_H

#include "dataset/base_dataset.h"
#include "coloradar_run.h"


namespace coloradar {

class ColoradarPlusDataset : public Dataset {
protected:
    // ATTRIBUTES
    std::filesystem::path datasetDirPath_;
    std::filesystem::path calibDirPath_;
    std::filesystem::path transformsDirPath_;
    std::filesystem::path runsDirPath_;


    // dataset/parent_init.cpp
    ColoradarPlusDataset() = default;
    void init(const std::filesystem::path& pathToDataset);
    void init(const std::filesystem::path& pathToRunsDir, const std::filesystem::path& pathToCalibDir);
    void postInit();
    
    // dataset/parent_data.cpp
    Eigen::Affine3f loadTransform(const std::filesystem::path& filePath);
    
    // dataset/parent_export.cpp
    std::vector<std::string> exportBaseDevice(const BaseExportConfig &config, std::vector<ColoradarPlusRun*> runs, const H5::H5File &datasetFile);
    std::vector<std::string> exportImu(const ImuExportConfig &config, std::vector<ColoradarPlusRun*> runs, const H5::H5File &datasetFile);
    std::vector<std::string> exportCascade(const RadarExportConfig &config, std::vector<ColoradarPlusRun*> runs, const H5::H5File &datasetFile);
    std::vector<std::string> exportLidar(const LidarExportConfig &config, std::vector<ColoradarPlusRun*> runs, const H5::H5File &datasetFile);

public:
    // dataset/parent_init.cpp
    explicit ColoradarPlusDataset(const std::filesystem::path& pathToDataset);
    explicit ColoradarPlusDataset(const std::filesystem::path& pathToRunsDir, const std::filesystem::path& pathToCalibDir);
    ColoradarPlusDataset(const ColoradarPlusDataset&) = delete;
    ColoradarPlusDataset& operator=(const ColoradarPlusDataset&) = delete;
    ColoradarPlusDataset(ColoradarPlusDataset&&) noexcept = default;
    ColoradarPlusDataset& operator=(ColoradarPlusDataset&&) noexcept = default;

    // dataset/parent_data.cpp
    const Eigen::Affine3f& imuTransform() const;
    const Eigen::Affine3f& lidarTransform() const;
    const Eigen::Affine3f& cascadeTransform() const;
    const std::shared_ptr<RadarConfig> cascadeConfig() const;
    std::vector<std::string> listRuns();
    std::vector<ColoradarPlusRun*> getRuns();
    virtual ColoradarPlusRun* getRun(const std::string& runName);

    // dataset/parent_export.cpp
    std::filesystem::path exportToFile(DatasetExportConfig& exportConfig);
    std::filesystem::path exportToFile(const std::string &yamlConfigPath);
};

class ColoradarDataset : public ColoradarPlusDataset {
protected:
    // ATTRIBUTES
    Eigen::Affine3f singleChipTransform_;
    std::shared_ptr<RadarConfig> singleChipConfig_;
    std::unique_ptr<SingleChipDevice> single_chip_;

    // dataset/child_init.cpp
    void postInit();

public:
    // dataset/child_init.cpp
    ColoradarDataset(const std::filesystem::path& pathToDataset);
    ColoradarDataset(const std::filesystem::path& pathToRunsDir, const std::filesystem::path& pathToCalibDir);

    const Eigen::Affine3f& singleChipTransform() const { return singleChipTransform_; }
    const std::shared_ptr<RadarConfig> singleChipConfig() const { return singleChipConfig_; }

    // dataset/child_data.cpp
    virtual ColoradarPlusRun* getRun(const std::string& runName) override;
};


}

#endif
