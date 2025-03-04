#include "dataset.h"


coloradar::ColoradarPlusDataset::ColoradarPlusDataset(const std::filesystem::path& pathToDataset) {
    init(pathToDataset);
    cascadeTransform_ = loadTransform(transformsDirPath_ / "base_to_radar.txt");
}

void coloradar::ColoradarPlusDataset::init(const std::filesystem::path& pathToDataset) {
    datasetDirPath_ = pathToDataset;
    coloradar::internal::checkPathExists(datasetDirPath_);
    calibDirPath_ = datasetDirPath_ / "calib";
    coloradar::internal::checkPathExists(calibDirPath_);
    transformsDirPath_ = calibDirPath_ / "transforms";
    coloradar::internal::checkPathExists(transformsDirPath_);
    runsDirPath_ = datasetDirPath_ / "kitti";
    coloradar::internal::checkPathExists(runsDirPath_);

    imuTransform_ = loadTransform(transformsDirPath_ / "base_to_imu.txt");
    lidarTransform_ = loadTransform(transformsDirPath_ / "base_to_lidar.txt");

    base_device_ = std::make_unique<BaseDevice>();
    imu_ = std::make_unique<ImuDevice>();
    lidar_ = std::make_unique<LidarDevice>();
    cascade_ = std::make_unique<CascadeDevice>();
    cascadeConfig_ = new coloradar::CascadeConfig(calibDirPath_);
}

Eigen::Affine3f coloradar::ColoradarPlusDataset::loadTransform(const std::filesystem::path& filePath) {
    coloradar::internal::checkPathExists(filePath);
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    Eigen::Vector3f translation;
    Eigen::Quaternionf rotation;

    std::ifstream file(filePath);
    std::string line;
    std::getline(file, line);
    std::istringstream iss(line);
    iss >> translation.x() >> translation.y() >> translation.z();

    std::getline(file, line);
    iss.str(line);
    iss.clear();
    iss >> rotation.x() >> rotation.y() >> rotation.z() >> rotation.w();

    transform.translate(translation);
    transform.rotate(rotation);
    return transform;
}

std::vector<std::string> coloradar::ColoradarPlusDataset::listRuns() {
    std::vector<std::string> runs;
    for (const auto& entry : std::filesystem::directory_iterator(runsDirPath_)) {
        if (entry.is_directory()) {
            runs.push_back(entry.path().filename().string());
        }
    }
    return runs;
}

coloradar::ColoradarPlusRun* coloradar::ColoradarPlusDataset::getRun(const std::string& runName) {
    return new coloradar::ColoradarPlusRun(runsDirPath_ / runName, cascadeConfig_);
}

std::vector<coloradar::ColoradarPlusRun*> coloradar::ColoradarPlusDataset::getRuns() {
    std::vector<std::string> runNames = listRuns();
    std::vector<coloradar::ColoradarPlusRun*> runs(runNames.size());
    for (size_t i = 0; i < runNames.size(); ++i)
        runs[i] = getRun(runNames[i]);
    return runs;
}

const Eigen::Affine3f& coloradar::ColoradarPlusDataset::imuTransform() const { return imuTransform_; }
const Eigen::Affine3f& coloradar::ColoradarPlusDataset::lidarTransform() const { return lidarTransform_; }
const Eigen::Affine3f& coloradar::ColoradarPlusDataset::cascadeTransform() const { return cascadeTransform_; }
const coloradar::RadarConfig* coloradar::ColoradarPlusDataset::cascadeConfig() const { return cascadeConfig_; }


std::vector<float> flattenHeatmap(const std::vector<float>& heatmap, const int& numAzimuthBins, const int& numElevationBins, const int& numRangeBins, const int& numDims) {
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
    return reorganizedHeatmap;
}


namespace coloradar {

std::vector<float> flattenLidarCloud(const pcl::PointCloud<pcl::PointXYZI>& cloud, bool collapseElevation, bool removeIntensity) {
    size_t numPoints = cloud.size();
    std::vector<float> data;
    if (numPoints <= 0) return data;

    size_t numDims = collapseElevation ? 3 : 4;
    if (removeIntensity) numDims--;

    data.resize(numPoints * numDims);
    for (size_t i = 0; i < numPoints; ++i) {
        data[i * numDims + 0] = cloud[i].x;
        data[i * numDims + 1] = cloud[i].y;
        if (collapseElevation) {
            if (!removeIntensity) data[i * numDims + 2] = cloud[i].intensity;
        } else {
            data[i * numDims + 2] = cloud[i].z;
            if (!removeIntensity) data[i * numDims + 3] = cloud[i].intensity;
        }
    }
    return data;
}

std::vector<float> flattenRadarCloud(const pcl::PointCloud<coloradar::RadarPoint>& cloud, const RadarConfig* config) {
    size_t numPoints = cloud.size();
    std::vector<float> data;
    if (numPoints == 0) return data;

    bool hasZ = config->numElevationBins > 0;
    size_t numDims = hasZ ? 5 : 4;
    if (!config->hasDoppler) numDims -= 1;

    data.resize(numPoints * numDims);
    for (size_t i = 0; i < numPoints; ++i) {
        data[i * numDims + 0] = cloud[i].x;
        data[i * numDims + 1] = cloud[i].y;
        if (hasZ) {
            data[i * numDims + 2] = cloud[i].z;
            data[i * numDims + 3] = cloud[i].intensity;
            if (config->hasDoppler) data[i * numDims + 4] = cloud[i].doppler;
        } else {
            data[i * numDims + 2] = cloud[i].intensity;
            if (config->hasDoppler) data[i * numDims + 3] = cloud[i].doppler;
        }
    }
    return data;
}

void saveCloudToHDF5(const std::string& name, const H5::H5File& file, const std::vector<float>& flatCloud, const hsize_t& numDims) {
    hsize_t numPoints = flatCloud.size() / numDims;
    hsize_t dims[2] = {numPoints, numDims};
    H5::DataSpace dataspace(2, dims);
    H5::PredType datatype = H5::PredType::NATIVE_FLOAT;
    H5::DataSet dataset = file.createDataSet(name, datatype, dataspace);
    if (numPoints > 0) {
        dataset.write(flatCloud.data(), H5::PredType::NATIVE_FLOAT);
    }
}

void saveCloudsToHDF5(const std::string& name, const H5::H5File& file, const std::vector<float>& flatClouds, const hsize_t& numFrames, const std::vector<hsize_t>& cloudSizes, const hsize_t& numDims) {
    if (cloudSizes.size() != numFrames) {
        throw std::invalid_argument("cloudSizes size must match the number of frames.");
    }
    std::string sizeDatasetName = name + "_sizes";
    hsize_t sizeDims[1] = {numFrames};
    H5::DataSpace sizeDataspace(1, sizeDims);
    H5::DataSet sizeDataset = file.createDataSet(sizeDatasetName, H5::PredType::NATIVE_HSIZE, sizeDataspace);
    sizeDataset.write(cloudSizes.data(), H5::PredType::NATIVE_HSIZE);

    hsize_t totalPoints = std::accumulate(cloudSizes.begin(), cloudSizes.end(), static_cast<hsize_t>(0));
    hsize_t cloudDims[2] = {totalPoints, static_cast<hsize_t>(numDims)};
    H5::DataSpace cloudDataspace(2, cloudDims);
    H5::DataSet cloudDataset = file.createDataSet(name, H5::PredType::NATIVE_FLOAT, cloudDataspace);
    cloudDataset.write(flatClouds.data(), H5::PredType::NATIVE_FLOAT);
}

void saveHeatmapsToHDF5(const std::string& name, const H5::H5File& file, const std::vector<float>& flatHeatmaps, const hsize_t& numFrames, const RadarConfig* config) {
    std::vector<hsize_t> dims = {numFrames, static_cast<hsize_t>(config->numAzimuthBins), static_cast<hsize_t>(config->nRangeBins())};
    if (config->numElevationBins > 1) {
        dims.push_back(static_cast<hsize_t>(config->numElevationBins));
    }
    if (config->hasDoppler) {
        dims.push_back(2);
    }
    H5::DataSpace dataspace(dims.size(), dims.data());
    H5::PredType datatype = H5::PredType::NATIVE_FLOAT;
    H5::DataSet dataset = file.createDataSet(name, datatype, dataspace);
    dataset.write(flatHeatmaps.data(), H5::PredType::NATIVE_FLOAT);
}

void saveDatacubesToHDF5(const std::string& name, const H5::H5File& file, const std::vector<int16_t>& flatDatacubes, const hsize_t& numFrames, const RadarConfig* config) {
    hsize_t datacubeSize = static_cast<hsize_t>(config->numElevationBins * config->numAzimuthBins * config->nRangeBins() * 2);
    std::vector<hsize_t> dims = {numFrames, datacubeSize};
    H5::DataSpace dataspace(dims.size(), dims.data());
    H5::PredType datatype = H5::PredType::NATIVE_FLOAT;
    H5::DataSet dataset = file.createDataSet(name, datatype, dataspace);
    dataset.write(flatDatacubes.data(), H5::PredType::NATIVE_INT16);
}

void savePosesToHDF5(const std::string& name, const H5::H5File& file, const std::vector<Eigen::Affine3f>& poses) {
    hsize_t dims[2] = { poses.size(), 7 };
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

void saveVectorToHDF5(const std::string& name, const H5::H5File& file, const std::vector<double>& vec) {
    hsize_t dims[1] = { vec.size() };
    H5::DataSpace dataspace(1, dims);
    H5::DataSet dataset = file.createDataSet(name, H5::PredType::NATIVE_DOUBLE, dataspace);
}

std::vector<std::string> ColoradarPlusDataset::exportBaseDevice(const BaseExportConfig &config, std::vector<ColoradarPlusRun*> runs, const H5::H5File &datasetFile) {
    // Constants
    const std::string posesContentName = "base_poses",
                      timestampsContentName = "base_timestamps";

    std::vector<std::string> content;

    for (auto* run : runs) {
        std::vector<double> timestamps = run->poseTimestamps();
        hsize_t numFrames = timestamps.size();
        std::vector<Eigen::Affine3f> basePoses = run->getPoses<Eigen::Affine3f>();

        // timestamps
        if (config.exportTimestamps()) {
            saveVectorToHDF5(timestampsContentName + "_" + run->name, datasetFile, timestamps);
            content.push_back(timestampsContentName);
        }

        // poses
        if (config.exportPoses()) {
            savePosesToHDF5(posesContentName + "_" + run->name, datasetFile, basePoses);
            content.push_back(posesContentName);
        }
    }
    std::cout << "Finished exporting base device." << std::endl;
    return content;
}

std::vector<std::string> ColoradarPlusDataset::exportImu(const ImuExportConfig &config, std::vector<ColoradarPlusRun*> runs, const H5::H5File &datasetFile) {
    // Constants
    const std::string posesContentName = "imu_poses",
                      timestampsContentName = "imu_timestamps";

    std::vector<std::string> content;

    for (auto* run : runs) {
        std::vector<double> timestamps = run->imuTimestamps();
        hsize_t numFrames = timestamps.size();
        std::vector<Eigen::Affine3f> basePoses = run->interpolatePoses(run->getPoses<Eigen::Affine3f>(), run->poseTimestamps(), timestamps);
        std::vector<Eigen::Affine3f> sensorPoses(numFrames);
        for (int i = 0; i < numFrames; ++i) {
            sensorPoses[i] = basePoses[i] * imuTransform_;
        }

        // timestamps
        if (config.exportTimestamps()) {
            saveVectorToHDF5(timestampsContentName + "_" + run->name, datasetFile, timestamps);
            content.push_back(timestampsContentName);
        }

        // poses
        if (config.exportPoses()) {
            savePosesToHDF5(posesContentName + "_" + run->name, datasetFile, sensorPoses);
            content.push_back(posesContentName);
        }

        // data: TBD
    }
    std::cout << "Finished exporting imu." << std::endl;
    return content;
}

std::vector<std::string> ColoradarPlusDataset::exportCascade(const RadarExportConfig &config, std::vector<ColoradarPlusRun*> runs, const H5::H5File &datasetFile) {
    // Constants
    const std::string datacubeContentName = "cascade_datacubes",
                      heatmapContentName = "cascade_heatmaps",
                      cloudContentName = "cascade_clouds",
                      posesContentName = "cascade_poses",
                      timestampsContentName = "cascade_timestamps";

    std::vector<std::string> content;

    // FOV
    float range = cascadeConfig_->clipRange(config.fov().rangeMeters);
    // std::cout << "range" << range << std::endl;
    int rangeMaxBin = cascadeConfig_->rangeToRangeIdx(range);
    // std::cout << "rangeMaxBin " << rangeMaxBin << std::endl;
    int azimuthMaxBin = config.fov().azimuthIdx;
    // std::cout << "azimuthMaxBin " << azimuthMaxBin << std::endl;
    int elevationMaxBin = config.fov().elevationIdx;
    // std::cout << "elevationMaxBin " << elevationMaxBin << std::endl;
    float horizontalFov = config.fov().horizontalDegreesTotal;
    // std::cout << "horizontalFov " << horizontalFov << std::endl;
    float verticalFov = config.fov().verticalDegreesTotal;
    // std::cout << "verticalFov " << verticalFov << std::endl;
    if (config.fov().useDegreeConstraints) {
        azimuthMaxBin = cascadeConfig_->horizontalFovToAzimuthIdx(config.fov().horizontalDegreesTotal);
        elevationMaxBin = cascadeConfig_->verticalFovToElevationIdx(config.fov().verticalDegreesTotal);
    }
    else {
        azimuthMaxBin = cascadeConfig_->clipAzimuthMaxBin(azimuthMaxBin);
        elevationMaxBin = cascadeConfig_->clipElevationMaxBin(elevationMaxBin);
        horizontalFov = cascadeConfig_->azimuthIdxToFovDegrees(azimuthMaxBin);
        verticalFov = cascadeConfig_->elevationIdxToFovDegrees(elevationMaxBin);
    }

    for (auto* run : runs) {
        std::vector<double> timestamps = run->cascadeTimestamps();
        hsize_t numFrames = timestamps.size();
        std::vector<Eigen::Affine3f> basePoses = run->interpolatePoses(run->getPoses<Eigen::Affine3f>(), run->poseTimestamps(), timestamps);
        std::vector<Eigen::Affine3f> sensorPoses(numFrames);
        for (int i = 0; i < numFrames; ++i) {
            sensorPoses[i] = basePoses[i] * cascadeTransform_;
        }

        // timestamps
        if (config.exportTimestamps()) {
            saveVectorToHDF5(timestampsContentName + "_" + run->name, datasetFile, timestamps);
            content.push_back(timestampsContentName);
        }

        // poses
        if (config.exportPoses()) {
            savePosesToHDF5(posesContentName + "_" + run->name, datasetFile, sensorPoses);
            content.push_back(posesContentName);
        }

        // datacubes
        if (config.exportDatacubes()) {
            std::vector<int16_t> datacubesFlat;
            for (size_t i = 0; i < numFrames; ++i) {
                auto datacube = run->getCascadeDatacube(i);
                std::copy(datacube.begin(), datacube.end(), datacubesFlat.begin() + i * datacube.size());
            }
            saveDatacubesToHDF5(datacubeContentName + "_" + run->name, datasetFile, datacubesFlat, numFrames, cascadeConfig_);
            content.push_back(datacubeContentName);
        }
        
        // heatmaps
        if (config.exportHeatmaps()) {
            CascadeConfig* heatmapConfig = new CascadeConfig(*dynamic_cast<CascadeConfig*>(cascadeConfig_));
            std::vector<float> heatmapsFlat;
            for (size_t i = 0; i < numFrames; ++i) {
                auto heatmap = run->getCascadeHeatmap(i);
                heatmap = heatmapConfig->clipHeatmap(heatmap, azimuthMaxBin, elevationMaxBin, rangeMaxBin);
                if (config.collapseElevation()) {
                    heatmap = heatmapConfig->collapseHeatmapElevation(heatmap, config.collapseElevationMinZ(), config.collapseElevationMaxZ());
                }
                if (config.removeDopplerDim()) {
                    heatmap = heatmapConfig->removeDoppler(heatmap);
                }
                heatmap = heatmapConfig->swapHeatmapDimensions(heatmap);
                heatmapsFlat.insert(heatmapsFlat.end(), heatmap.begin(), heatmap.end());
            }
            saveHeatmapsToHDF5(heatmapContentName + "_" + run->name, datasetFile, heatmapsFlat, numFrames, heatmapConfig);
            content.push_back(heatmapContentName);
            heatmapsFlat.clear();
        }

        // clouds
        if (config.exportClouds()) {
            hsize_t numDims = cascadeConfig_->numElevationBins > 0 ? 5 : 4;  // x, y, (z), intensity, doppler
            if (!cascadeConfig_->hasDoppler) numDims -= 1;

            bool buildClouds = false;
            try {
                auto cloud = run->getCascadePointcloud(0);
            } catch (const std::filesystem::filesystem_error& e) {
                buildClouds = true;
            }
            std::vector<float> cloudsFlat;
            std::vector<hsize_t> cloudSizes;
            for (size_t i = 0; i < numFrames; ++i) {
                pcl::PointCloud<RadarPoint> cloud;
                if (buildClouds) {
                    cloud = heatmapToPointcloud(run->getCascadeHeatmap(i), cascadeConfig_, config.intensityThresholdPercent());
                }
                else {
                    cloud = run->getCascadePointcloud(i, config.intensityThresholdPercent());
                }
                filterFov(cloud, horizontalFov, verticalFov, range);
                if (config.cloudsInGlobalFrame()) {
                    pcl::transformPointCloud(cloud, cloud, sensorPoses[i]);
                }
                if (config.collapseElevation()) {
                    collapseElevation(cloud, config.collapseElevationMinZ(), config.collapseElevationMaxZ());
                }
                cloudSizes[i] = cloud.size();
                std::vector<float> cloudFlat = flattenRadarCloud(cloud, cascadeConfig_);
                cloudsFlat.insert(cloudsFlat.end(), cloudFlat.begin(), cloudFlat.end());
            }
            saveCloudsToHDF5(cloudContentName + "_" + run->name, datasetFile, cloudsFlat, numFrames, cloudSizes, numDims);
            content.push_back(cloudContentName);
        }
    }
    std::cout << "Finished exporting cascade." << std::endl;
    return content;
}

std::vector<std::string> ColoradarPlusDataset::exportLidar(const LidarExportConfig &config, std::vector<ColoradarPlusRun*> runs, const H5::H5File &datasetFile) {
    // Constants
    const std::string cloudContentName = "lidar_clouds",
                      mapContentName = "lidar_map",
                      mapSampleContentName = "lidar_map_samples",
                      posesContentName = "lidar_poses",
                      timestampsContentName = "lidar_timestamps";

    std::vector<std::string> content;

    for (auto* run : runs) {
        std::vector<double> truePoseTimestamps = run->poseTimestamps();
        std::vector<double> timestamps = run->lidarTimestamps();
        hsize_t numFrames = timestamps.size();
        auto truePoses = run->getPoses<Eigen::Affine3f>();
        std::vector<Eigen::Affine3f> basePoses = run->interpolatePoses(truePoses, truePoseTimestamps, timestamps);
        std::vector<Eigen::Affine3f> sensorPoses(numFrames);
        for (int i = 0; i < numFrames; ++i) {
            sensorPoses[i] = basePoses[i] * lidarTransform_;
        }

        // timestamps
        if (config.exportTimestamps()) {
            saveVectorToHDF5(timestampsContentName + "_" + run->name, datasetFile, timestamps);
            content.push_back(timestampsContentName);
        }

        // poses
        if (config.exportPoses()) {
            savePosesToHDF5(posesContentName + "_" + run->name, datasetFile, sensorPoses);
            content.push_back(posesContentName);
        }

        // clouds
        if (config.exportClouds()) {
            hsize_t numDims = config.collapseElevation() ? 3 : 4;  // x, y, (z), intensity
            if (config.removeIntensityDim()) numDims -= 1;

            std::vector<float> cloudsFlat;
            std::vector<hsize_t> cloudSizes;
            for (size_t i = 0; i < numFrames; ++i) {
                auto cloud = run->getLidarPointCloud<pcl::PointCloud<pcl::PointXYZI>>(i);
                filterFov(
                    cloud,
                    config.cloudFov().horizontalDegreesTotal,
                    config.cloudFov().verticalDegreesTotal,
                    config.cloudFov().rangeMeters
                );
                if (config.collapseElevation()) {
                    collapseElevation(cloud, config.collapseElevationMinZ(), config.collapseElevationMaxZ());
                }
                cloudSizes[i] = cloud.size();
                std::vector<float> cloudFlat = flattenLidarCloud(cloud, config.collapseElevation(), config.removeIntensityDim());
                cloudsFlat.insert(cloudsFlat.end(), cloudFlat.begin(), cloudFlat.end());
            }
            saveCloudsToHDF5(cloudContentName + "_" + run->name, datasetFile, cloudsFlat, numFrames, cloudSizes, numDims);
            content.push_back(cloudContentName);
        }
        
        // map and samples
        if (config.exportMap() || config.exportMapSamples()) {
            pcl::PointCloud<pcl::PointXYZI> map = run->readLidarOctomap();
            hsize_t numDims = config.collapseElevation() ? 3 : 4;  // x, y, (z), occupancy
            if (config.removeOccupancyDim()) numDims -= 1;

            if (config.collapseElevation()) {
                collapseElevation(map, config.collapseElevationMinZ(), config.collapseElevationMaxZ());
            }
            filterOccupancy(map, config.occupancyThresholdPercent() / 100.0, config.logOddsToProbability());

            if (config.exportMap()) {
                std::vector<float> mapFlat = flattenLidarCloud(map, config.collapseElevation(), config.removeOccupancyDim());
                saveCloudToHDF5(mapContentName + "_" + run->name, datasetFile, mapFlat, numDims);
                content.push_back(mapContentName);
            }

            if (config.exportMapSamples()) {
                std::vector<double> centerTimestamps;
                Eigen::Affine3f centerTransform;
                if (config.centerSensor()->name == CascadeDevice::name) {
                    centerTimestamps = run->cascadeTimestamps();
                    centerTransform = cascadeTransform_;
                } else if (config.centerSensor()->name == LidarDevice::name) {
                    centerTimestamps = run->lidarTimestamps();
                    centerTransform = lidarTransform_;
                } else if (config.centerSensor()->name == ImuDevice::name) {
                    centerTimestamps = run->imuTimestamps();
                    centerTransform = imuTransform_;
                } else {
                    centerTimestamps = run->poseTimestamps();
                    centerTransform = Eigen::Affine3f::Identity();
                }
                hsize_t numSamples = centerTimestamps.size();
                std::vector<Eigen::Affine3f> basePosesCenterTs = run->interpolatePoses(truePoses, truePoseTimestamps, centerTimestamps);
                std::vector<Eigen::Affine3f> centerPoses(numSamples);
                for (int i = 0; i < numSamples; ++i) {
                    centerPoses[i] = basePosesCenterTs[i] * centerTransform;
                }

                bool resample = false;
                if (config.forceResample()) {
                    resample = true;
                } else if (config.allowResample()) {
                    try {
                        auto sample = run->readMapSample(0);
                    } catch (const std::filesystem::filesystem_error& e) {
                        std::cout << run->name << ": map samples not found, resampling... " << std::endl;
                        resample = true;
                    }
                }

                std::vector<float> samplesFlat;
                std::vector<hsize_t> sampleSizes(numSamples);
                for (size_t i = 0; i < numSamples; ++i) {
                    pcl::PointCloud<pcl::PointXYZI> sample;
                    if (resample) {
                        sample = run->sampleMapFrame(
                            config.mapSampleFov().horizontalDegreesTotal,
                            config.mapSampleFov().verticalDegreesTotal,
                            config.mapSampleFov().rangeMeters,
                            centerPoses[i],
                            map
                        );
                        if (config.saveSamples()) {
                            run->saveMapSample(sample, i);
                        }
                    } else {
                        sample = run->readMapSample(i);
                    }
                    sampleSizes[i] = sample.size();
                    auto sampleFlat = flattenLidarCloud(sample, config.collapseElevation(), config.removeOccupancyDim());
                    samplesFlat.insert(samplesFlat.end(), sampleFlat.begin(), sampleFlat.end());
                }
                saveCloudsToHDF5(mapSampleContentName + "_" + run->name, datasetFile, samplesFlat, numSamples, sampleSizes, numDims);
                content.push_back(mapSampleContentName);
            }
        }
    }
    std::cout << "Finished exporting lidar." << std::endl;
    return content;
}

std::filesystem::path ColoradarPlusDataset::exportToFile(const DatasetExportConfig &exportConfig) {
    std::vector<ColoradarPlusRun*> runObjects;
    if (exportConfig.runs().empty()) {
        runObjects = getRuns();
    } else {
        for (auto runName : exportConfig.runs()) {
            runObjects.push_back(getRun(runName));
        }
    }
    std::cout << "Exporting " << runObjects.size() << " runs." << std::endl;
    H5::H5File datasetFile(exportConfig.destinationFilePath(), H5F_ACC_TRUNC);
    Json::Value finalConfig;
    finalConfig["runs"] = Json::arrayValue;
    finalConfig["radar_config"] = cascadeConfig_->toJson();
    finalConfig["data_content"] = Json::arrayValue;

    for (const auto &contentList : {
        exportBaseDevice(exportConfig.base(), runObjects, datasetFile),
        exportImu(exportConfig.imu(), runObjects, datasetFile),
        exportCascade(exportConfig.cascade(), runObjects, datasetFile),
        exportLidar(exportConfig.lidar(), runObjects, datasetFile)
    }) {
        for (const auto &content : contentList) {
            finalConfig["data_content"].append(content);
        }
    }

    std::string configString = Json::writeString(Json::StreamWriterBuilder(), finalConfig);
    H5::StrType strType(H5::PredType::C_S1, H5T_VARIABLE);
    H5::DataSpace dataspace(H5S_SCALAR);
    H5::DataSet configDataset = datasetFile.createDataSet("config", strType, dataspace);
    configDataset.write(configString, strType);
    return exportConfig.destinationFilePath();
}

std::filesystem::path ColoradarPlusDataset::exportToFile(const std::string &yamlConfigPath) {
    DatasetExportConfig exportConfig = DatasetExportConfig(yamlConfigPath);
    return exportToFile(exportConfig);
}

}

coloradar::ColoradarDataset::ColoradarDataset(const std::filesystem::path& pathToDataset) {
    init(pathToDataset);
    cascadeTransform_ = loadTransform(transformsDirPath_ / "base_to_cascade.txt");
    singleChipTransform_ = loadTransform(transformsDirPath_ / "base_to_single_chip.txt");

    single_chip_ = std::make_unique<SingleChipDevice>();
    singleChipConfig_ = new coloradar::SingleChipConfig(calibDirPath_);
}

coloradar::ColoradarPlusRun* coloradar::ColoradarDataset::getRun(const std::string& runName) {
    return new coloradar::ColoradarRun(runsDirPath_ / runName, cascadeConfig_, singleChipConfig_);
}

const Eigen::Affine3f& coloradar::ColoradarDataset::singleChipTransform() const { return singleChipTransform_; }
const coloradar::RadarConfig* coloradar::ColoradarDataset::singleChipConfig() const { return singleChipConfig_; }
