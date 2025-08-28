#include "dataset.h"


namespace coloradar {

// PUBLIC METHODS

ColoradarPlusDataset::ColoradarPlusDataset(const std::filesystem::path& pathToDataset) {
    init(pathToDataset);
    postInit();
}

ColoradarPlusDataset::ColoradarPlusDataset(const std::filesystem::path& pathToRunsDir, const std::filesystem::path& pathToCalibDir) {
    init(pathToRunsDir, pathToCalibDir);
    postInit();
}


// PROTECTED METHODS

void ColoradarPlusDataset::init(const std::filesystem::path& pathToDataset) {
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    datasetDirPath_ = pathToDataset;
    coloradar::internal::checkPathExists(datasetDirPath_);
    runsDirPath_ = datasetDirPath_ / "kitti";
    calibDirPath_ = datasetDirPath_ / "calib";
    init(runsDirPath_, calibDirPath_);
}

void ColoradarPlusDataset::init(const std::filesystem::path& pathToRunsDir, const std::filesystem::path& pathToCalibDir) {
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    runsDirPath_ = pathToRunsDir;
    coloradar::internal::checkPathExists(runsDirPath_);
    calibDirPath_ = pathToCalibDir;
    coloradar::internal::checkPathExists(calibDirPath_);
    
    transformsDirPath_ = calibDirPath_ / "transforms";
    coloradar::internal::checkPathExists(transformsDirPath_);

    imuTransform_ = loadTransform(transformsDirPath_ / "base_to_imu.txt");
    lidarTransform_ = loadTransform(transformsDirPath_ / "base_to_lidar.txt");

    base_device_ = std::make_unique<BaseDevice>();
    imu_ = std::make_unique<ImuDevice>();
    lidar_ = std::make_unique<LidarDevice>();
    cascade_ = std::make_unique<CascadeDevice>();
    cascadeConfig_ = std::make_shared<coloradar::CascadeConfig>(calibDirPath_);
}

void ColoradarPlusDataset::postInit() {
    cascadeTransform_ = loadTransform(transformsDirPath_ / "base_to_radar.txt");
}

}
