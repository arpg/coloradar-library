#include "coloradar_run.h"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <fstream>
#include <sstream>


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

std::vector<int16_t> coloradar::ColoradarPlusRun::getDatacube(const std::filesystem::path& binFilePath, coloradar::RadarConfig* config) {
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

std::vector<float> coloradar::ColoradarPlusRun::getHeatmap(const std::filesystem::path& binFilePath, coloradar::RadarConfig* config) {
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

void coloradar::ColoradarPlusRun::createRadarPointclouds(coloradar::RadarConfig* config, const std::filesystem::path& heatmapDirPath, const std::filesystem::path& pointcloudDirPath, const float& intensityThresholdPercent) {
    coloradar::internal::createDirectoryIfNotExists(pointcloudDirPath);
    coloradar::internal::createDirectoryIfNotExists(pointcloudDirPath / "data");
    for (auto const& entry : std::filesystem::directory_iterator(heatmapDirPath / "data")) {
        if (!entry.is_directory() && entry.path().extension() == ".bin") {
            std::filesystem::path heatmapPath = entry.path();
            std::vector<float> heatmap = getHeatmap(heatmapPath, config);
            pcl::PointCloud<coloradar::RadarPoint> cloud = coloradar::heatmapToPointcloud(heatmap, config, intensityThresholdPercent);

            std::string filename = heatmapPath.filename();
            filename.replace(filename.find("heatmap"), 7, "radar_pointcloud");
            std::filesystem::path cloudPath = pointcloudDirPath / "data" / filename;
            std::ofstream file(cloudPath, std::ios::out | std::ios::binary);
            if (!file.is_open()) {
                throw std::runtime_error("Failed to open file: " + cloudPath.string());
            }
            for (size_t i = 0; i < cloud.points.size(); ++i) {
                file.write(reinterpret_cast<const char*>(&cloud.points[i].x), sizeof(float));
                file.write(reinterpret_cast<const char*>(&cloud.points[i].y), sizeof(float));
                file.write(reinterpret_cast<const char*>(&cloud.points[i].z), sizeof(float));
                file.write(reinterpret_cast<const char*>(&cloud.points[i].intensity), sizeof(float));
                file.write(reinterpret_cast<const char*>(&cloud.points[i].doppler), sizeof(float));
            }
            file.close();
        }
    }
}

pcl::PointCloud<coloradar::RadarPoint> coloradar::ColoradarPlusRun::getRadarPointcloud(const std::filesystem::path& binFilePath, RadarConfig* config, const float& intensityThresholdPercent) {
    pcl::PointCloud<coloradar::RadarPoint> cloud;
    std::ifstream file(binFilePath, std::ios::in | std::ios::binary);
    if (!file.is_open()) {
        throw std::filesystem::filesystem_error("Failed to open file", binFilePath, std::make_error_code(std::errc::no_such_file_or_directory));
    }
    float maxIntensity = 0.0f;
    std::vector<coloradar::RadarPoint> points;

    while (file.good()) {
        coloradar::RadarPoint point;
        file.read(reinterpret_cast<char*>(&point.x), sizeof(float));
        file.read(reinterpret_cast<char*>(&point.y), sizeof(float));
        file.read(reinterpret_cast<char*>(&point.z), sizeof(float));
        file.read(reinterpret_cast<char*>(&point.intensity), sizeof(float));
        file.read(reinterpret_cast<char*>(&point.doppler), sizeof(float));

        if (!file.good()) break;
        points.push_back(point);
        if (point.intensity > maxIntensity) {
            maxIntensity = point.intensity;
        }
    }
    file.close();

    float intensityThreshold = maxIntensity * (intensityThresholdPercent / 100.0f);
    for (const auto& point : points) {
        if (point.intensity >= intensityThreshold) {
            cloud.points.push_back(point);
        }
    }
    return cloud;
}


std::vector<int16_t> coloradar::ColoradarPlusRun::getCascadeDatacube(const std::filesystem::path& binFilePath) {
    return getDatacube(binFilePath, cascadeConfig_);
}
std::vector<int16_t> coloradar::ColoradarPlusRun::getCascadeDatacube(const int& cubeIdx) {
    return getCascadeDatacube(cascadeCubesDirPath_ / "data" / ("frame_" + std::to_string(cubeIdx) + ".bin"));
}
std::vector<float> coloradar::ColoradarPlusRun::getCascadeHeatmap(const std::filesystem::path& binFilePath) {
    return getHeatmap(binFilePath, cascadeConfig_);
}
std::vector<float> coloradar::ColoradarPlusRun::getCascadeHeatmap(const int& hmIdx) {
    return getCascadeHeatmap(cascadeHeatmapsDirPath_ / "data" / ("heatmap_" + std::to_string(hmIdx) + ".bin"));
}
void coloradar::ColoradarPlusRun::createCascadePointclouds(const float& intensityThresholdPercent) {
    createRadarPointclouds(cascadeConfig_, cascadeHeatmapsDirPath_, cascadeCloudsDirPath_, intensityThresholdPercent);
}
pcl::PointCloud<coloradar::RadarPoint> coloradar::ColoradarPlusRun::getCascadePointcloud(const std::filesystem::path& binFilePath, const float& intensityThresholdPercent) {
    return getRadarPointcloud(binFilePath, cascadeConfig_, intensityThresholdPercent);
}
pcl::PointCloud<coloradar::RadarPoint> coloradar::ColoradarPlusRun::getCascadePointcloud(const int& cloudIdx, const float& intensityThresholdPercent) {
    return getCascadePointcloud(cascadeCloudsDirPath_ / "data" / ("radar_pointcloud_" + std::to_string(cloudIdx) + ".bin"), intensityThresholdPercent);
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
        OctoPointcloud cloud = getLidarPointCloud<coloradar::OctoPointcloud>(i);
        cloud.filterFov(lidarTotalHorizontalFov, lidarTotalVerticalFov, maxRange);
        auto frameTransform = poses[i] * baseToLidarT;
        cloud.transform(frameTransform);
        tree.insertPointCloud(cloud, frameTransform.trans());
    }
    return tree;
}

void coloradar::ColoradarPlusRun::saveLidarOctomap(const octomap::OcTree& tree) {
    pcl::PointCloud<pcl::PointXYZI> treePcl;
    coloradar::octreeToPcl(tree, treePcl);
    coloradar::internal::createDirectoryIfNotExists(lidarMapsDirPath_);
    std::filesystem::path outputMapFile = lidarMapsDirPath_ / "map.pcd";
    if (treePcl.size() == 0) {
        treePcl.height = 1;
        treePcl.is_dense = true;
    }
    pcl::io::savePCDFile(outputMapFile, treePcl);
}

pcl::PointCloud<pcl::PointXYZI> coloradar::ColoradarPlusRun::readLidarOctomap() {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    std::filesystem::path mapFilePath = lidarMapsDirPath_ / "map.pcd";
    coloradar::internal::checkPathExists(mapFilePath);
    pcl::io::loadPCDFile<pcl::PointXYZI>(mapFilePath.string(), cloud);
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

void coloradar::ColoradarPlusRun::sampleMapFrames(
    const float& horizontalFov,
    const float& verticalFov,
    const float& range,
    const Eigen::Affine3f& baseToSensorTransform,
    std::vector<Eigen::Affine3f> basePoses
) {
    for (const auto& entry : std::filesystem::directory_iterator(lidarMapsDirPath_)) {
        if (entry.is_regular_file() && entry.path().filename() != "map.pcd") {
            std::filesystem::remove(entry.path());
        }
    }
    float maxRange = range == 0 ? std::numeric_limits<float>::max() : range;
    if (basePoses.empty())
        basePoses = getPoses<Eigen::Affine3f>();
    pcl::PointCloud<pcl::PointXYZI> mapCloud = readLidarOctomap();
    for (size_t i = 0; i < basePoses.size(); ++i) {
        Eigen::Affine3f sensorToMapT = baseToSensorTransform.inverse() * basePoses[i].inverse();
        pcl::PointCloud<pcl::PointXYZI> centeredCloud;
        pcl::transformPointCloud(mapCloud, centeredCloud, sensorToMapT);
        filterFov(centeredCloud, horizontalFov, verticalFov, maxRange);
        std::filesystem::path frameFilePath = lidarMapsDirPath_ / ("frame_" + std::to_string(i) + ".pcd");
        pcl::io::savePCDFile(frameFilePath, centeredCloud);
    }
}

pcl::PointCloud<pcl::PointXYZI> coloradar::ColoradarPlusRun::readMapFrame(const int& frameIdx) {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    std::filesystem::path frameFilePath = lidarMapsDirPath_ / ("frame_" + std::to_string(frameIdx) + ".pcd");
    coloradar::internal::checkPathExists(frameFilePath);
    pcl::io::loadPCDFile<pcl::PointXYZI>(frameFilePath.string(), cloud);
    return cloud;
}



void coloradar::ColoradarPlusRun::saveVectorToHDF5(const std::string& name, H5::H5File& file, const std::vector<double>& vec) {
    hsize_t dims[1] = { vec.size() };
    H5::DataSpace dataspace(1, dims);
    H5::DataSet dataset = file.createDataSet(name, H5::PredType::NATIVE_DOUBLE, dataspace);
}

void coloradar::ColoradarPlusRun::savePosesToHDF5(const std::string& name, H5::H5File& file, const std::vector<Eigen::Affine3f>& poses) {
    hsize_t dims[2] = { poses.size(), 7 };  // N * (x, y, z, qx, qy, qz, qw)
    H5::DataSpace dataspace(2, dims);
    H5::DataSet dataset = file.createDataSet(name, H5::PredType::NATIVE_DOUBLE, dataspace);
    std::vector<float> poseData;
    poseData.reserve(poses.size() * 7);
    for (const auto& pose : poses) {
        poseData.push_back(pose.translation().x());
        poseData.push_back(pose.translation().y());
        poseData.push_back(pose.translation().z());
        Eigen::Quaternionf rot(pose.rotation());
        poseData.push_back(rot.x());
        poseData.push_back(rot.y());
        poseData.push_back(rot.z());
        poseData.push_back(rot.w());
    }
    dataset.write(poseData.data(), H5::PredType::NATIVE_FLOAT);
}

void coloradar::ColoradarPlusRun::saveHeatmapToHDF5(const int& idx, H5::H5File& file, const std::vector<float>& heatmap, const int& numAzimuthBins, const int& numElevationBins, const int& numRangeBins, const int& numDims) {
    const size_t expectedSize = numElevationBins * numAzimuthBins * numRangeBins * numDims;
    if (heatmap.size() != expectedSize) {
        throw std::runtime_error("Heatmap size does not match the expected dimensions. Expected size: " + std::to_string(expectedSize) + ", Actual size: " + std::to_string(heatmap.size()));
    }
    std::vector<float> reorganizedHeatmap(expectedSize);
    for (int el = 0; el < numElevationBins; ++el) {
        for (int az = 0; az < numAzimuthBins; ++az) {
            for (int r = 0; r < numRangeBins; ++r) {
                for (int d = 0; d < numDims; ++d) {
                    size_t originalIndex = (((el * numAzimuthBins + az) * numRangeBins) + r) * numDims + d;
                    size_t reorganizedIndex = (((az * numRangeBins + r) * numElevationBins) + el) * numDims + d;
                    reorganizedHeatmap[reorganizedIndex] = heatmap[originalIndex];
                }
            }
        }
    }
    std::vector<hsize_t> dims;
    if (numAzimuthBins > 1) dims.push_back(static_cast<hsize_t>(numAzimuthBins));
    if (numRangeBins > 1) dims.push_back(static_cast<hsize_t>(numRangeBins));
    if (numElevationBins > 1) dims.push_back(static_cast<hsize_t>(numElevationBins));
    if (numDims > 1) dims.push_back(static_cast<hsize_t>(numDims));
    if (dims.empty()) dims.push_back(1);

    H5::DataSpace dataspace(dims.size(), dims.data());
    H5::PredType datatype = H5::PredType::NATIVE_FLOAT;
    H5::DataSet dataset = file.createDataSet("heatmap_" + std::to_string(idx), datatype, dataspace);
    dataset.write(reorganizedHeatmap.data(), H5::PredType::NATIVE_FLOAT);
}

void coloradar::ColoradarPlusRun::saveRadarCloudToHDF5(const int& idx, H5::H5File& file, const pcl::PointCloud<coloradar::RadarPoint>& cloud, bool collapseElevation) {
    size_t numPoints = cloud.size();
    size_t numDims = collapseElevation ? 3 : 4;
    std::vector<float> data;
    if (numPoints > 0) {
        data.resize(numPoints * numDims);
        if (collapseElevation) {
            for (size_t i = 0; i < numPoints; ++i) {
                data[i * numDims + 0] = cloud[i].x;
                data[i * numDims + 1] = cloud[i].y;
                data[i * numDims + 2] = cloud[i].intensity;
            }
        } else {
            for (size_t i = 0; i < numPoints; ++i) {
                data[i * numDims + 0] = cloud[i].x;
                data[i * numDims + 1] = cloud[i].y;
                data[i * numDims + 2] = cloud[i].z;
                data[i * numDims + 3] = cloud[i].intensity;
            }
        }
    }
    hsize_t dims[2] = {numPoints, numDims};
    H5::DataSpace dataspace(2, dims);
    H5::PredType datatype = H5::PredType::NATIVE_FLOAT;
    std::string datasetName = "radar_cloud_" + std::to_string(idx);
    H5::DataSet dataset = file.createDataSet(datasetName, datatype, dataspace);
    if (numPoints > 0) {
        dataset.write(data.data(), H5::PredType::NATIVE_FLOAT);
    }
}

void coloradar::ColoradarPlusRun::saveLidarCloudToHDF5(const std::string& name, H5::H5File& file, const pcl::PointCloud<pcl::PointXYZI>& cloud, bool includeIntensity, bool collapseElevation) {
    size_t numPoints = cloud.size();
    size_t numDims = collapseElevation ? 3 : 4;
    if (!includeIntensity) numDims--;
    std::vector<float> data;
    if (numPoints > 0) {
        data.resize(numPoints * numDims);
        if (collapseElevation) {
            if (includeIntensity) {
                for (size_t i = 0; i < numPoints; ++i) {
                    data[i * numDims + 0] = cloud[i].x;
                    data[i * numDims + 1] = cloud[i].y;
                    data[i * numDims + 2] = cloud[i].intensity;
                }
            } else {
                for (size_t i = 0; i < numPoints; ++i) {
                    data[i * numDims + 0] = cloud[i].x;
                    data[i * numDims + 1] = cloud[i].y;
                }
            }
        } else if (includeIntensity) {
            for (size_t i = 0; i < numPoints; ++i) {
                data[i * numDims + 0] = cloud[i].x;
                data[i * numDims + 1] = cloud[i].y;
                data[i * numDims + 2] = cloud[i].z;
                data[i * numDims + 3] = cloud[i].intensity;
            }
        } else {
            for (size_t i = 0; i < numPoints; ++i) {
                data[i * numDims + 0] = cloud[i].x;
                data[i * numDims + 1] = cloud[i].y;
                data[i * numDims + 2] = cloud[i].z;
            }
        }
    }
    hsize_t dims[2] = {numPoints, numDims};
    H5::DataSpace dataspace(2, dims);
    H5::PredType datatype = H5::PredType::NATIVE_FLOAT;
    H5::DataSet dataset = file.createDataSet(name, datatype, dataspace);
    if (numPoints > 0) {
        dataset.write(data.data(), H5::PredType::NATIVE_FLOAT);
    }
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
pcl::PointCloud<coloradar::RadarPoint> coloradar::ColoradarRun::getSingleChipPointcloud(const std::filesystem::path& binFilePath, const float& intensityThresholdPercent) {
    return getRadarPointcloud(binFilePath, singleChipConfig_, intensityThresholdPercent);
}
pcl::PointCloud<coloradar::RadarPoint> coloradar::ColoradarRun::getSingleChipPointcloud(const int& cloudIdx, const float& intensityThresholdPercent) {
    return getSingleChipPointcloud(singleChipCloudsDirPath_ / "data" / ("radar_pointcloud_" + std::to_string(cloudIdx) + ".bin"), intensityThresholdPercent);
}
//void coloradar::ColoradarRun::createSingleChipPointclouds(const float& intensityThresholdPercent) {
//    createRadarPointclouds(singleChipConfig_, singleChipHeatmapsDirPath_, singleChipCloudsDirPath_, intensityThresholdPercent);
//}
