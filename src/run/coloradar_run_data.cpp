#include "run/coloradar_run.h"

namespace coloradar {


// PROTECTED METHODS

std::shared_ptr<std::vector<int16_t>> ColoradarPlusRun::readDatacube(const std::filesystem::path& binFilePath, const std::shared_ptr<RadarConfig>& config) const {
    coloradar::internal::checkPathExists(binFilePath);
    std::ifstream file(binFilePath, std::ios::binary);
    if (!file) throw std::runtime_error("Failed to open file: " + binFilePath.string());

    const size_t totalElements = config->numTxAntennas * config->numRxAntennas * config->numChirpsPerFrame * config->numAdcSamplesPerChirp * 2;
    const size_t totalBytes = totalElements * sizeof(int16_t);
    auto datacube = std::make_shared<std::vector<int16_t>>(totalElements);
    if (!file.read(reinterpret_cast<char*>(datacube->data()), totalBytes))
        throw std::runtime_error("Datacube file read error or size mismatch: " + binFilePath.string());
    file.close();
    return datacube;
}

std::shared_ptr<std::vector<float>> ColoradarPlusRun::readHeatmap(const std::filesystem::path& binFilePath, const std::shared_ptr<RadarConfig>& config) const {
    coloradar::internal::checkPathExists(binFilePath);
    std::ifstream file(binFilePath, std::ios::binary);
    if (!file) throw std::runtime_error("Failed to open file: " + binFilePath.string());

    int totalElements = config->numElevationBins * config->numAzimuthBins * config->numPosRangeBins * 2;
    const size_t totalBytes = totalElements * sizeof(float);
    auto heatmap = std::make_shared<std::vector<float>>(totalElements);
    if (!file.read(reinterpret_cast<char*>(heatmap->data()), totalBytes))
        throw std::runtime_error("Heatmap file read error or size mismatch: " + binFilePath.string());
    file.close();
    return heatmap;
}

pcl::PointCloud<RadarPoint>::Ptr ColoradarPlusRun::readRadarPointcloud(
    std::shared_ptr<RadarConfig> config,
    const std::filesystem::path& binFilePath,
    const double intensityThreshold
) const {
    std::ifstream file(binFilePath, std::ios::in | std::ios::binary);
    if (!file.is_open()) {
        throw std::filesystem::filesystem_error("Failed to open file: " + binFilePath.string(), std::make_error_code(std::errc::no_such_file_or_directory));
    }
    auto cloud = pcl::PointCloud<RadarPoint>::Ptr(new pcl::PointCloud<RadarPoint>());

    float x, y, z, i, d;
    while (true) {
        if (!file.read(reinterpret_cast<char*>(&x), sizeof(float))) break;
        if (!file.read(reinterpret_cast<char*>(&y), sizeof(float))) break;
        if (!file.read(reinterpret_cast<char*>(&z), sizeof(float))) break;
        if (!file.read(reinterpret_cast<char*>(&i), sizeof(float))) break;
        if (!file.read(reinterpret_cast<char*>(&d), sizeof(float))) break;
        if (i < intensityThreshold) continue;
        cloud->points.emplace_back();
        RadarPoint& pt = cloud->points.back();
        pt.x = x; pt.y = y; pt.z = z;
        pt.intensity = i;
        pt.doppler   = d;
    }
    cloud->width  = static_cast<uint32_t>(cloud->size());
    return cloud;
}

void ColoradarPlusRun::createRadarPointclouds(
    const std::shared_ptr<RadarConfig>& config,
    const std::filesystem::path& heatmapDirPath,
    const std::filesystem::path& pointcloudDirPath,
    double intensityThreshold)
{
    coloradar::internal::createDirectoryIfNotExists(pointcloudDirPath);
    coloradar::internal::createDirectoryIfNotExists(pointcloudDirPath / "data");
    const std::filesystem::path heatmapDataDir = heatmapDirPath / "data";

    for (const auto& entry : std::filesystem::directory_iterator(heatmapDataDir)) {
        const std::filesystem::path heatmapPath = entry.path();
        if (!entry.is_regular_file() || heatmapPath.extension() != ".bin") continue;

        auto heatmap = readHeatmap(heatmapPath, config);
        auto cloud = config->heatmapToPointcloud(heatmap, intensityThreshold);

        std::string filename = heatmapPath.filename().string();
        const std::string heatmapStr = "heatmap";
        filename.replace(filename.find(heatmapStr), heatmapStr.size(), "radar_pointcloud");
        const std::filesystem::path cloudPath = pointcloudDirPath / "data" / filename;
        std::ofstream file(cloudPath, std::ios::binary);
        if (!file) throw std::runtime_error("Failed to open file: " + cloudPath.string());
        file.exceptions(std::ofstream::failbit | std::ofstream::badbit);

        for (const auto& pt : cloud->points) {
            file.write(reinterpret_cast<const char*>(&pt.x),         sizeof(float));
            file.write(reinterpret_cast<const char*>(&pt.y),         sizeof(float));
            file.write(reinterpret_cast<const char*>(&pt.z),         sizeof(float));
            file.write(reinterpret_cast<const char*>(&pt.intensity), sizeof(float));
            file.write(reinterpret_cast<const char*>(&pt.doppler),   sizeof(float));
        }
    }
}


// PUBLIC METHODS

pcl::PointCloud<pcl::PointXYZI>::Ptr ColoradarPlusRun::getLidarPointCloud(const int cloudIdx) const {
    return getLidarPointCloud<pcl::PointCloud<pcl::PointXYZI>>(cloudIdx);
}

std::shared_ptr<std::vector<int16_t>> ColoradarPlusRun::getCascadeDatacube(const int cubeIdx) const {
    return getCascadeDatacube(cascadeCubesDirPath_ / "data" / ("frame_" + std::to_string(cubeIdx) + ".bin"));
}

std::shared_ptr<std::vector<float>> ColoradarPlusRun::getCascadeHeatmap(const int hmIdx) const {
    return getCascadeHeatmap(cascadeHeatmapsDirPath_ / "data" / ("heatmap_" + std::to_string(hmIdx) + ".bin"));
}

pcl::PointCloud<RadarPoint>::Ptr ColoradarPlusRun::getCascadePointcloud(const int cloudIdx, const double intensityThreshold) const {
    return getCascadePointcloud(cascadeCloudsDirPath_ / "data" / ("radar_pointcloud_" + std::to_string(cloudIdx) + ".bin"), intensityThreshold);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr ColoradarPlusRun::getLidarOctomap() const {
    auto cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    std::filesystem::path mapFilePath = lidarMapsDirPath_ / "map.pcd";
    coloradar::internal::checkPathExists(mapFilePath);
    pcl::io::loadPCDFile<pcl::PointXYZI>(mapFilePath.string(), *cloud);
    return cloud;
}


std::shared_ptr<std::vector<int16_t>> ColoradarPlusRun::getCascadeDatacube(const std::filesystem::path& binFilePath) const {
    return readDatacube(binFilePath, cascadeConfig_);
}

std::shared_ptr<std::vector<float>> ColoradarPlusRun::getCascadeHeatmap(const std::filesystem::path& binFilePath) const {
    return readHeatmap(binFilePath, cascadeConfig_);
}

pcl::PointCloud<RadarPoint>::Ptr ColoradarPlusRun::getCascadePointcloud(const std::filesystem::path& binFilePath, const double intensityThreshold) const {
    return readRadarPointcloud(cascadeConfig_, binFilePath, intensityThreshold);
}

void ColoradarPlusRun::createCascadePointclouds(const double intensityThreshold) {
    createRadarPointclouds(cascadeConfig_, cascadeHeatmapsDirPath_, cascadeCloudsDirPath_, intensityThreshold);
}


octomap::OcTree ColoradarPlusRun::buildLidarOctomap(
    const double& mapResolution,
    const float& lidarTotalHorizontalFov,
    const float& lidarTotalVerticalFov,
    const float& lidarMaxRange,
    Eigen::Affine3f baseToLidarTransform
) {
    float maxRange = lidarMaxRange == 0 ? std::numeric_limits<float>::max() : lidarMaxRange;
    std::vector<octomath::Pose6D> gtPoses = getPoses<octomath::Pose6D>();
    std::vector<octomath::Pose6D> poses = interpolatePoses(gtPoses, poseTimestamps_, lidarTimestamps_);
    auto baseToLidarT = coloradar::internal::fromEigenPose<octomath::Pose6D>(baseToLidarTransform);
    octomap::OcTree tree(mapResolution);

    for (size_t i = 0; i < lidarTimestamps_.size(); ++i) {
        std::shared_ptr<OctoPointcloud> cloud = getLidarPointCloud<OctoPointcloud>(i);
        cloud->filterFov(lidarTotalHorizontalFov, lidarTotalVerticalFov, maxRange);
        auto frameTransform = poses[i] * baseToLidarT;
        cloud->transform(frameTransform);
        tree.insertPointCloud(*cloud, frameTransform.trans());
    }
    return tree;
}

void ColoradarPlusRun::saveLidarOctomap(const octomap::OcTree& tree) {
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> treePcl = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    coloradar::octreeToPcl(tree, treePcl);
    coloradar::internal::createDirectoryIfNotExists(lidarMapsDirPath_);
    std::filesystem::path outputMapFile = lidarMapsDirPath_ / "map.pcd";
    if (treePcl->size() == 0) {
        treePcl->height = 1;
        treePcl->is_dense = true;
    }
    pcl::io::savePCDFile(outputMapFile, *treePcl);
}

void ColoradarPlusRun::createLidarOctomap(
    const double& mapResolution,
    const float& lidarTotalHorizontalFov,
    const float& lidarTotalVerticalFov,
    const float& lidarMaxRange,
    Eigen::Affine3f baseToLidarTransform
) {
    auto map = buildLidarOctomap(mapResolution, lidarTotalHorizontalFov, lidarTotalVerticalFov, lidarMaxRange, baseToLidarTransform);
    saveLidarOctomap(map);
}


pcl::PointCloud<pcl::PointXYZI>::Ptr ColoradarPlusRun::sampleMapFrame(
    const float& horizontalFov,
    const float& verticalFov,
    const float& range,
    const Eigen::Affine3f& mapFramePose,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& mapCloud
) {
    float maxRange = range == 0 ? std::numeric_limits<float>::max() : range;
    pcl::PointCloud<pcl::PointXYZI>::Ptr sample(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*mapCloud, *sample, mapFramePose);
    filterFov(sample, horizontalFov, verticalFov, maxRange);
    return sample;
}


std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> ColoradarPlusRun::sampleMapFrames(
    const float& horizontalFov,
    const float& verticalFov,
    const float& range,
    const std::vector<Eigen::Affine3f>& mapFramePoses
) {
    if (mapFramePoses.empty()) throw std::runtime_error("Empty sampling poses.");
    auto mapCloud = getLidarOctomap();
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> samples;
    samples.reserve(mapFramePoses.size());

    auto startT = std::chrono::high_resolution_clock::now();
    for (const auto& pose : mapFramePoses) {
        samples.push_back(sampleMapFrame(horizontalFov, verticalFov, range, pose, mapCloud));
    }
    double elapsedTime = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - startT).count();
    std::cout << std::fixed << std::setprecision(2)  << name_ << ": sampled " << mapFramePoses.size() << " map frames in " << elapsedTime << " seconds." << std::endl;

    return samples;
}


void ColoradarPlusRun::saveMapSample(const pcl::PointCloud<pcl::PointXYZI>::Ptr& sample, const int& sampleIdx) {
    std::filesystem::path frameFilePath = lidarMapsDirPath_ / ("frame_" + std::to_string(sampleIdx) + ".pcd");
    pcl::io::savePCDFile(frameFilePath, *sample);
}


void ColoradarPlusRun::saveMapSamples(const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& samples) {
    if (samples.empty()) throw std::runtime_error("Empty frames.");
    for (const auto& entry : std::filesystem::directory_iterator(lidarMapsDirPath_)) {
        if (entry.is_regular_file() && entry.path().filename() != "map.pcd") {
            std::filesystem::remove(entry.path());
        }
    }
    for (size_t i = 0; i < samples.size(); ++i) {
        saveMapSample(samples[i], i);
    }
}


void ColoradarPlusRun::createMapSamples(
    const float& horizontalFov,
    const float& verticalFov,
    const float& range,
    const std::vector<double>& sensorTimestamps,
    const Eigen::Affine3f& baseToSensorTransform
) {
    for (const auto& entry : std::filesystem::directory_iterator(lidarMapsDirPath_)) {
        if (entry.is_regular_file() && entry.path().filename() != "map.pcd") {
            std::filesystem::remove(entry.path());
        }
    }
    std::vector<Eigen::Affine3f> basePoses = getPoses<Eigen::Affine3f>();
    if (!sensorTimestamps.empty()) basePoses = interpolatePoses(basePoses, poseTimestamps_, sensorTimestamps);
    auto mapCloud = getLidarOctomap();

    auto startT = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < basePoses.size(); ++i) {
        Eigen::Affine3f mapToSensorT = basePoses[i] * baseToSensorTransform;
        auto sample = sampleMapFrame(horizontalFov, verticalFov, range, mapToSensorT, mapCloud);
        saveMapSample(sample, i);
    }
    double elapsedTime = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - startT).count();
    std::cout << std::fixed << std::setprecision(2) << name_ << ": sampled " << basePoses.size() << " map frames in " << elapsedTime << " seconds." << std::endl;
}


pcl::PointCloud<pcl::PointXYZI>::Ptr ColoradarPlusRun::readMapSample(const int& sampleIdx) const {
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    std::filesystem::path frameFilePath = lidarMapsDirPath_ / ("frame_" + std::to_string(sampleIdx) + ".pcd");
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(frameFilePath.string(), *cloud) == -1) throw std::runtime_error("Failed to load PCD file: " + frameFilePath.string());
    return cloud;
}


std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> ColoradarPlusRun::readMapSamples(const int& numSamples) const {
    std::vector<std::filesystem::path> samplePaths = coloradar::internal::readArrayDirectory(lidarMapsDirPath_, "frame_", ".pcd", numSamples);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> samples;
    samples.reserve(samplePaths.size());

    for (const auto& path : samplePaths) {
        auto sample = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(path.string(), *sample) == -1) {
            throw std::runtime_error("Failed to load PCD file: " + path.string());
        }
        samples.push_back(sample);
    }
    return samples;
}


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
    return getSingleChipDatacube(singleChipCubesDirPath_ / "data" / ("frame_" + std::to_string(cubeIdx) + ".bin"));
}
std::shared_ptr<std::vector<float>> ColoradarRun::getSingleChipHeatmap(const std::filesystem::path& binFilePath) {
    return readHeatmap(binFilePath, singleChipConfig_);
}
std::shared_ptr<std::vector<float>> ColoradarRun::getSingleChipHeatmap(const int& hmIdx) {
    return getSingleChipHeatmap(singleChipHeatmapsDirPath_ / "data" / ("heatmap_" + std::to_string(hmIdx) + ".bin"));
}

pcl::PointCloud<RadarPoint>::Ptr ColoradarRun::getSingleChipPointcloud(const std::filesystem::path& binFilePath, const double intensityThreshold) {
    return readRadarPointcloud(singleChipConfig_, binFilePath, intensityThreshold);
}
pcl::PointCloud<RadarPoint>::Ptr ColoradarRun::getSingleChipPointcloud(const int& cloudIdx, const double intensityThreshold) {
    return getSingleChipPointcloud(singleChipCloudsDirPath_ / "data" / ("radar_pointcloud_" + std::to_string(cloudIdx) + ".bin"), intensityThreshold);
}


}