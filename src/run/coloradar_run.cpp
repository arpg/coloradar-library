#include "run/coloradar_run.h"


namespace coloradar {


ColoradarRun::ColoradarRun(const std::filesystem::path& runPath, std::shared_ptr<RadarConfig> cascadeRadarConfig, std::shared_ptr<RadarConfig> singleChipRadarConfig) : ColoradarPlusRun(runPath, cascadeRadarConfig), singleChipConfig_(singleChipRadarConfig) {
    singleChipScansDirPath_ = runDirPath_ / "cascade";
    coloradar::internal::checkPathExists(singleChipScansDirPath_);
    singleChipCubesDirPath_ = singleChipScansDirPath_ / "adc_samples";
    coloradar::internal::checkPathExists(singleChipCubesDirPath_);
    singleChipHeatmapsDirPath_ = singleChipScansDirPath_ / "heatmaps";
    coloradar::internal::checkPathExists(cascadeHeatmapsDirPath_);
    singleChipCloudsDirPath_ = singleChipScansDirPath_ / "pointclouds";

    singleChipCubeTimestamps_ = readTimestamps(singleChipCubesDirPath_ / "timestamps.txt");
    singleChipTimestamps_ = readTimestamps(singleChipHeatmapsDirPath_ / "timestamps.txt");
}

std::shared_ptr<std::vector<int16_t>> ColoradarRun::getSingleChipDatacube(const std::filesystem::path& binFilePath) {
    return readDatacube(binFilePath, singleChipConfig_);
}
std::shared_ptr<std::vector<int16_t>> ColoradarRun::getSingleChipDatacube(const int& cubeIdx) {
    std::filesystem::path cubesDir = singleChipCubesDirPath_ / "data";
    int fileIdx = frameIdxToFileIdx(cubeIdx, cubesDir);
    return getSingleChipDatacube(cubesDir / ("frame_" + std::to_string(fileIdx) + ".bin"));
}
std::shared_ptr<std::vector<float>> ColoradarRun::getSingleChipHeatmap(const std::filesystem::path& binFilePath) {
    return readHeatmap(binFilePath, singleChipConfig_);
}
std::shared_ptr<std::vector<float>> ColoradarRun::getSingleChipHeatmap(const int& hmIdx) {
    std::filesystem::path heatmapsDir = singleChipHeatmapsDirPath_ / "data";
    int fileIdx = frameIdxToFileIdx(hmIdx, heatmapsDir);
    return getSingleChipHeatmap(heatmapsDir / ("heatmap_" + std::to_string(fileIdx) + ".bin"));
}

pcl::PointCloud<RadarPoint>::Ptr ColoradarRun::getSingleChipPointcloud(const std::filesystem::path& binFilePath, const double intensityThreshold) {
    return readRadarPointcloud(singleChipConfig_, binFilePath, intensityThreshold);
}
pcl::PointCloud<RadarPoint>::Ptr ColoradarRun::getSingleChipPointcloud(const int& cloudIdx, const double intensityThreshold) {
    std::filesystem::path cloudsDir = singleChipCloudsDirPath_ / "data";
    int fileIdx = frameIdxToFileIdx(cloudIdx, cloudsDir);
    return getSingleChipPointcloud(cloudsDir / ("radar_pointcloud_" + std::to_string(fileIdx) + ".bin"), intensityThreshold);
}


}