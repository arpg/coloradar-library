#ifndef COLORADAR_PLUS_DATASET_H
#define COLORADAR_PLUS_DATASET_H

#include "dataset/base_dataset.h"
#include "run/coloradar_plus_run.h"


namespace coloradar {

    
class ColoradarPlusDataset : public Dataset {
protected:
    // ATTRIBUTES
    std::filesystem::path datasetDirPath_;
    std::filesystem::path calibDirPath_;
    std::filesystem::path transformsDirPath_;
    std::filesystem::path runsDirPath_;


    // src/dataset/coloradar_plus_dataset.cpp
    ColoradarPlusDataset() = default;
    void init(const std::filesystem::path& pathToDataset);
    void init(const std::filesystem::path& pathToRunsDir, const std::filesystem::path& pathToCalibDir);
    void postInit();
    
    Eigen::Affine3f loadTransform(const std::filesystem::path& filePath);

    std::vector<std::string> exportBaseDevice(const BaseExportConfig &config, std::vector<std::shared_ptr<ColoradarPlusRun>> runs, H5::H5File& datasetFile);
    std::vector<std::string> exportImu(const ImuExportConfig &config, std::vector<std::shared_ptr<ColoradarPlusRun>> runs, H5::H5File& datasetFile);
    std::vector<std::string> exportCascade(const RadarExportConfig &config, std::vector<std::shared_ptr<ColoradarPlusRun>> runs, H5::H5File& datasetFile);
    std::vector<std::string> exportLidar(const LidarExportConfig &config, std::vector<std::shared_ptr<ColoradarPlusRun>> runs, H5::H5File& datasetFile);

public:
    // src/dataset/coloradar_plus_dataset.cpp
    explicit ColoradarPlusDataset(const std::filesystem::path& pathToDataset);
    explicit ColoradarPlusDataset(const std::filesystem::path& pathToRunsDir, const std::filesystem::path& pathToCalibDir);
    ColoradarPlusDataset(const ColoradarPlusDataset&) = delete;
    ColoradarPlusDataset& operator=(const ColoradarPlusDataset&) = delete;
    ColoradarPlusDataset(ColoradarPlusDataset&&) noexcept = default;
    ColoradarPlusDataset& operator=(ColoradarPlusDataset&&) noexcept = default;

    std::filesystem::path exportToFile(DatasetExportConfig& exportConfig);
    std::filesystem::path exportToFile(const std::string &yamlConfigPath);
};


}

#endif
