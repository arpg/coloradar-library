#include "coloradar_run.h"


coloradar::ColoradarPlusRun::ColoradarPlusRun(const std::filesystem::path& runPath, coloradar::RadarConfig* cascadeRadarConfig) : runDirPath_(runPath), name(runDirPath_.filename()), cascadeConfig_(cascadeRadarConfig) {
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
}

std::vector<double> coloradar::ColoradarPlusRun::readTimestamps(const std::filesystem::path& path) {
    coloradar::internal::checkPathExists(path);
    std::vector<double> timestamps;
    std::ifstream infile(path);
    std::string line;
    while (std::getline(infile, line)) {
        timestamps.push_back(std::stod(line));
    }
    return timestamps;
}

const std::vector<double>& coloradar::ColoradarPlusRun::poseTimestamps() const { return poseTimestamps_; }
const std::vector<double>& coloradar::ColoradarPlusRun::imuTimestamps() const { return imuTimestamps_; }
const std::vector<double>& coloradar::ColoradarPlusRun::lidarTimestamps() const { return lidarTimestamps_; }
const std::vector<double>& coloradar::ColoradarPlusRun::cascadeCubeTimestamps() const { return cascadeCubeTimestamps_; }
const std::vector<double>& coloradar::ColoradarPlusRun::cascadeTimestamps() const { return cascadeTimestamps_; }

std::vector<int16_t> coloradar::ColoradarPlusRun::getDatacube(const std::filesystem::path& binFilePath, coloradar::RadarConfig* config) const {
    coloradar::internal::checkPathExists(binFilePath);
    std::ifstream file(binFilePath, std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file: " + binFilePath.string());
    }
    int totalElements = config->numTxAntennas * config->numRxAntennas * config->numChirpsPerFrame * config->numAdcSamplesPerChirp * 2;
    std::vector<int16_t> frameBytes(totalElements);
    file.read(reinterpret_cast<char*>(frameBytes.data()), totalElements * sizeof(int16_t));
    if (file.gcount() != totalElements * sizeof(int16_t)) {
        throw std::runtime_error("Datacube file read error or size mismatch");
    }
    file.close();
    return frameBytes;
}

std::vector<float> coloradar::ColoradarPlusRun::getHeatmap(const std::filesystem::path& binFilePath, coloradar::RadarConfig* config) const {
    coloradar::internal::checkPathExists(binFilePath);
    std::ifstream file(binFilePath, std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file: " + binFilePath.string());
    }
    int totalElements = config->numElevationBins * config->numAzimuthBins * config->numPosRangeBins * 2;
    std::vector<float> heatmap(totalElements);
    file.read(reinterpret_cast<char*>(heatmap.data()), totalElements * sizeof(float));
    if (file.gcount() != totalElements * sizeof(float)) {
        throw std::runtime_error("Heatmap file read error or size mismatch");
    }
    file.close();
    return heatmap;
}


void coloradar::ColoradarPlusRun::createRadarPointclouds(coloradar::RadarConfig* config, const std::filesystem::path& heatmapDirPath, const std::filesystem::path& pointcloudDirPath, const double intensityThreshold) {
    coloradar::internal::createDirectoryIfNotExists(pointcloudDirPath);
    coloradar::internal::createDirectoryIfNotExists(pointcloudDirPath / "data");
    for (auto const& entry : std::filesystem::directory_iterator(heatmapDirPath / "data")) {
        if (!entry.is_directory() && entry.path().extension() == ".bin") {
            std::filesystem::path heatmapPath = entry.path();
            std::vector<float> heatmap = getHeatmap(heatmapPath, config);
            pcl::PointCloud<coloradar::RadarPoint>::Ptr cloud = config->heatmapToPointcloud(heatmap, intensityThreshold);

            std::string filename = heatmapPath.filename();
            filename.replace(filename.find("heatmap"), 7, "radar_pointcloud");
            std::filesystem::path cloudPath = pointcloudDirPath / "data" / filename;
            std::ofstream file(cloudPath, std::ios::out | std::ios::binary);
            if (!file.is_open()) {
                throw std::runtime_error("Failed to open file: " + cloudPath.string());
            }
            for (size_t i = 0; i < cloud->points.size(); ++i) {
                file.write(reinterpret_cast<const char*>(&cloud->points[i].x), sizeof(float));
                file.write(reinterpret_cast<const char*>(&cloud->points[i].y), sizeof(float));
                file.write(reinterpret_cast<const char*>(&cloud->points[i].z), sizeof(float));
                file.write(reinterpret_cast<const char*>(&cloud->points[i].intensity), sizeof(float));
                file.write(reinterpret_cast<const char*>(&cloud->points[i].doppler), sizeof(float));
            }
            file.close();
        }
    }
}


pcl::PointCloud<coloradar::RadarPoint>::Ptr coloradar::ColoradarPlusRun::getRadarPointcloud(const std::filesystem::path& binFilePath, RadarConfig* config, const double intensityThreshold) {
    pcl::PointCloud<coloradar::RadarPoint>::Ptr cloud(new pcl::PointCloud<coloradar::RadarPoint>());
    std::ifstream file(binFilePath, std::ios::in | std::ios::binary);
    if (!file.is_open()) {
        throw std::filesystem::filesystem_error("Failed to open file", binFilePath, std::make_error_code(std::errc::no_such_file_or_directory));
    }
    std::vector<coloradar::RadarPoint> points;

    while (file.good()) {
        coloradar::RadarPoint point;
        file.read(reinterpret_cast<char*>(&point.x), sizeof(float));
        file.read(reinterpret_cast<char*>(&point.y), sizeof(float));
        file.read(reinterpret_cast<char*>(&point.z), sizeof(float));
        file.read(reinterpret_cast<char*>(&point.intensity), sizeof(float));
        file.read(reinterpret_cast<char*>(&point.doppler), sizeof(float));

        if (!file.good()) break;
        if (point.intensity >= intensityThreshold) {
            points.push_back(point);
        }
    }
    file.close();
    return cloud;
}


std::vector<int16_t> coloradar::ColoradarPlusRun::getCascadeDatacube(const std::filesystem::path& binFilePath) const {
    return getDatacube(binFilePath, cascadeConfig_);
}
std::vector<int16_t> coloradar::ColoradarPlusRun::getCascadeDatacube(const int cubeIdx) const {
    return getCascadeDatacube(cascadeCubesDirPath_ / "data" / ("frame_" + std::to_string(cubeIdx) + ".bin"));
}
std::vector<float> coloradar::ColoradarPlusRun::getCascadeHeatmap(const std::filesystem::path& binFilePath) const {
    return getHeatmap(binFilePath, cascadeConfig_);
}
std::vector<float> coloradar::ColoradarPlusRun::getCascadeHeatmap(const int hmIdx) const {
    return getCascadeHeatmap(cascadeHeatmapsDirPath_ / "data" / ("heatmap_" + std::to_string(hmIdx) + ".bin"));
}
void coloradar::ColoradarPlusRun::createCascadePointclouds(const double intensityThreshold) {
    createRadarPointclouds(cascadeConfig_, cascadeHeatmapsDirPath_, cascadeCloudsDirPath_, intensityThreshold);
}
pcl::PointCloud<coloradar::RadarPoint>::Ptr coloradar::ColoradarPlusRun::getCascadePointcloud(const std::filesystem::path& binFilePath, const double intensityThreshold) {
    return getRadarPointcloud(binFilePath, cascadeConfig_, intensityThreshold);
}
pcl::PointCloud<coloradar::RadarPoint>::Ptr coloradar::ColoradarPlusRun::getCascadePointcloud(const int& cloudIdx, const double intensityThreshold) {
    return getCascadePointcloud(cascadeCloudsDirPath_ / "data" / ("radar_pointcloud_" + std::to_string(cloudIdx) + ".bin"), intensityThreshold);
}


octomap::OcTree coloradar::ColoradarPlusRun::buildLidarOctomap(
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

void coloradar::ColoradarPlusRun::saveLidarOctomap(const octomap::OcTree& tree) {
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

pcl::PointCloud<pcl::PointXYZI>::Ptr coloradar::ColoradarPlusRun::readLidarOctomap() const {
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    std::filesystem::path mapFilePath = lidarMapsDirPath_ / "map.pcd";
    coloradar::internal::checkPathExists(mapFilePath);
    pcl::io::loadPCDFile<pcl::PointXYZI>(mapFilePath.string(), *cloud);
    return cloud;
}

void coloradar::ColoradarPlusRun::createLidarOctomap(
    const double& mapResolution,
    const float& lidarTotalHorizontalFov,
    const float& lidarTotalVerticalFov,
    const float& lidarMaxRange,
    Eigen::Affine3f baseToLidarTransform
) {
    auto map = buildLidarOctomap(mapResolution, lidarTotalHorizontalFov, lidarTotalVerticalFov, lidarMaxRange, baseToLidarTransform);
    saveLidarOctomap(map);
}


pcl::PointCloud<pcl::PointXYZI>::Ptr coloradar::ColoradarPlusRun::sampleMapFrame(
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


std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> coloradar::ColoradarPlusRun::sampleMapFrames(
    const float& horizontalFov,
    const float& verticalFov,
    const float& range,
    const std::vector<Eigen::Affine3f>& mapFramePoses
) {
    if (mapFramePoses.empty()) throw std::runtime_error("Empty sampling poses.");
    auto mapCloud = readLidarOctomap();
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> samples;
    samples.reserve(mapFramePoses.size());

    auto startT = std::chrono::high_resolution_clock::now();
    for (const auto& pose : mapFramePoses) {
        samples.push_back(sampleMapFrame(horizontalFov, verticalFov, range, pose, mapCloud));
    }
    double elapsedTime = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - startT).count();
    std::cout << std::fixed << std::setprecision(2)  << name << ": sampled " << mapFramePoses.size() << " map frames in " << elapsedTime << " seconds." << std::endl;

    return samples;
}


void coloradar::ColoradarPlusRun::saveMapSample(const pcl::PointCloud<pcl::PointXYZI>::Ptr& sample, const int& sampleIdx) {
    std::filesystem::path frameFilePath = lidarMapsDirPath_ / ("frame_" + std::to_string(sampleIdx) + ".pcd");
    pcl::io::savePCDFile(frameFilePath, *sample);
}


void coloradar::ColoradarPlusRun::saveMapSamples(const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& samples) {
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


void coloradar::ColoradarPlusRun::createMapSamples(
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
    auto mapCloud = readLidarOctomap();

    auto startT = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < basePoses.size(); ++i) {
        Eigen::Affine3f mapToSensorT = basePoses[i] * baseToSensorTransform;
        auto sample = sampleMapFrame(horizontalFov, verticalFov, range, mapToSensorT, mapCloud);
        saveMapSample(sample, i);
    }
    double elapsedTime = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - startT).count();
    std::cout << std::fixed << std::setprecision(2) << name << ": sampled " << basePoses.size() << " map frames in " << elapsedTime << " seconds." << std::endl;
}


pcl::PointCloud<pcl::PointXYZI>::Ptr coloradar::ColoradarPlusRun::readMapSample(const int& sampleIdx) const {
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    std::filesystem::path frameFilePath = lidarMapsDirPath_ / ("frame_" + std::to_string(sampleIdx) + ".pcd");
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(frameFilePath.string(), *cloud) == -1) throw std::runtime_error("Failed to load PCD file: " + frameFilePath.string());
    return cloud;
}


std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> coloradar::ColoradarPlusRun::readMapSamples(const int& numSamples) const {
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


coloradar::ColoradarRun::ColoradarRun(const std::filesystem::path& runPath, coloradar::RadarConfig* cascadeRadarConfig, coloradar::RadarConfig* singleChipRadarConfig) : coloradar::ColoradarPlusRun(runPath, cascadeRadarConfig), singleChipConfig_(singleChipRadarConfig) {
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

const std::vector<double>& coloradar::ColoradarRun::singleChipCubeTimestamps() const { return singleChipCubeTimestamps_; }
const std::vector<double>& coloradar::ColoradarRun::singleChipTimestamps() const { return singleChipTimestamps_; }

std::vector<int16_t> coloradar::ColoradarRun::getSingleChipDatacube(const std::filesystem::path& binFilePath) {
    return getDatacube(binFilePath, singleChipConfig_);
}
std::vector<int16_t> coloradar::ColoradarRun::getSingleChipDatacube(const int& cubeIdx) {
    return getSingleChipDatacube(singleChipCubesDirPath_ / "data" / ("frame_" + std::to_string(cubeIdx) + ".bin"));
}
std::vector<float> coloradar::ColoradarRun::getSingleChipHeatmap(const std::filesystem::path& binFilePath) {
    return getHeatmap(binFilePath, singleChipConfig_);
}
std::vector<float> coloradar::ColoradarRun::getSingleChipHeatmap(const int& hmIdx) {
    return getSingleChipHeatmap(singleChipHeatmapsDirPath_ / "data" / ("heatmap_" + std::to_string(hmIdx) + ".bin"));
}
pcl::PointCloud<coloradar::RadarPoint>::Ptr coloradar::ColoradarRun::getSingleChipPointcloud(const std::filesystem::path& binFilePath, const float& intensityThreshold) {
    return getRadarPointcloud(binFilePath, singleChipConfig_, intensityThreshold);
}
pcl::PointCloud<coloradar::RadarPoint>::Ptr coloradar::ColoradarRun::getSingleChipPointcloud(const int& cloudIdx, const float& intensityThreshold) {
    return getSingleChipPointcloud(singleChipCloudsDirPath_ / "data" / ("radar_pointcloud_" + std::to_string(cloudIdx) + ".bin"), intensityThreshold);
}
//void coloradar::ColoradarRun::createSingleChipPointclouds(const float& intensityThreshold) {
//    createRadarPointclouds(singleChipConfig_, singleChipHeatmapsDirPath_, singleChipCloudsDirPath_, intensityThreshold);
//}
