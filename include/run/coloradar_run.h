#ifndef COLORADAR_RUN_H
#define COLORADAR_RUN_H

#include "run/coloradar_plus_run.h"


namespace coloradar {


class ColoradarRun : public ColoradarPlusRun {
protected:
    std::filesystem::path singleChipScansDirPath_;
    std::filesystem::path singleChipCubesDirPath_;
    std::filesystem::path singleChipHeatmapsDirPath_;
    std::filesystem::path singleChipCloudsDirPath_;

    std::vector<double> singleChipCubeTimestamps_;
    std::vector<double> singleChipTimestamps_;

    std::shared_ptr<RadarConfig> singleChipConfig_;

public:
    // src/run/coloradar_run.cpp
    ColoradarRun(const std::filesystem::path& runPath, std::shared_ptr<RadarConfig> cascadeRadarConfig, std::shared_ptr<RadarConfig> singleChipRadarConfig);

    const std::shared_ptr<RadarConfig> singleChipConfig() const { return singleChipConfig_; }
    const std::vector<double>& singleChipCubeTimestamps() const { return singleChipCubeTimestamps_; }
    const std::vector<double>& singleChipTimestamps() const { return singleChipTimestamps_; }

    std::shared_ptr<std::vector<int16_t>> getSingleChipDatacube(const std::filesystem::path& binFilePath);
    std::shared_ptr<std::vector<int16_t>> getSingleChipDatacube(const int& cubeIdx);
    std::shared_ptr<std::vector<float>> getSingleChipHeatmap(const std::filesystem::path& binFilePath);
    std::shared_ptr<std::vector<float>> getSingleChipHeatmap(const int& hmIdx);
    pcl::PointCloud<RadarPoint>::Ptr getSingleChipPointcloud(const std::filesystem::path& binFilePath, const double intensityThreshold = 0.0);
    pcl::PointCloud<RadarPoint>::Ptr getSingleChipPointcloud(const int& cloudIdx, const double intensityThreshold = 0.0);
    // void createSingleChipPointclouds(const float& intensityThreshold = 0.0);
};


}


#endif
