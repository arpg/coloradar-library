#include "run/coloradar_run.h"


namespace coloradar {


ColoradarPlusRun::ColoradarPlusRun(
    const std::filesystem::path& runPath,
    std::shared_ptr<RadarConfig> cascadeRadarConfig
) : Run(runPath.filename(), cascadeRadarConfig), runDirPath_(runPath) 
{
    coloradar::internal::checkPathExists(runDirPath_);
    posesDirPath_ = runDirPath_ / "groundtruth";
    coloradar::internal::checkPathExists(posesDirPath_);
    imuDirPath_ = runDirPath_ / "imu";
    coloradar::internal::checkPathExists(imuDirPath_);
    lidarScansDirPath_ = runDirPath_ / "lidar";
    coloradar::internal::checkPathExists(lidarScansDirPath_);
    cascadeScansDirPath_ = runDirPath_ / "cascade";
    coloradar::internal::checkPathExists(cascadeScansDirPath_);

    lidarCloudsDirPath_ = lidarScansDirPath_ / "pointclouds";
    coloradar::internal::checkPathExists(lidarCloudsDirPath_);
    lidarMapsDirPath_ = runDirPath_ / "lidar_maps";

    cascadeCubesDirPath_ = cascadeScansDirPath_ / "adc_samples";
    coloradar::internal::checkPathExists(cascadeCubesDirPath_);
    cascadeHeatmapsDirPath_ = cascadeScansDirPath_ / "heatmaps";
    coloradar::internal::checkPathExists(cascadeHeatmapsDirPath_);
    cascadeCloudsDirPath_ = cascadeScansDirPath_ / "pointclouds";

    poseTimestamps_ = readTimestamps(posesDirPath_ / "timestamps.txt");
    imuTimestamps_ = readTimestamps(imuDirPath_ / "timestamps.txt");
    lidarTimestamps_ = readTimestamps(lidarScansDirPath_ / "timestamps.txt");
    cascadeCubeTimestamps_ = readTimestamps(cascadeCubesDirPath_ / "timestamps.txt");
    cascadeTimestamps_ = readTimestamps(cascadeHeatmapsDirPath_ / "timestamps.txt");
    poses_ = readPoses(posesDirPath_ / "groundtruth_poses.txt");
}


std::vector<double> ColoradarPlusRun::readTimestamps(const std::filesystem::path& path) const {
    coloradar::internal::checkPathExists(path);
    std::vector<double> timestamps;
    std::ifstream infile(path);
    std::string line;
    while (std::getline(infile, line)) {
        timestamps.push_back(std::stod(line));
    }
    return timestamps;
}


std::vector<Eigen::Affine3f> ColoradarPlusRun::readPoses(const std::filesystem::path& path) const {
    coloradar::internal::checkPathExists(path);
    std::ifstream infile(path);
    std::string line;
    std::vector<Eigen::Affine3f> poses;

    while (std::getline(infile, line)) {
        float x, y, z, rotX, rotY, rotZ, rotW;
        std::istringstream iss(line);
        iss >> x >> y >> z >> rotX >> rotY >> rotZ >> rotW;
        Eigen::Translation3f translation(x, y, z);
        Eigen::Quaternionf rotation(rotW, rotX, rotY, rotZ);
        Eigen::Affine3f pose = translation * rotation;
        poses.push_back(pose);
    }
    return poses;
}


}
