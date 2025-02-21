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

void ColoradarPlusDataset::exportCascade(const std::vector<ColoradarPlusRun*> &runs, const H5::H5File &datasetFile) {
    // Constants
    const std::string datacubeContentName = "cascade_datacubes",
                      heatmapContentName = "cascade_heatmaps",
                      cloudContentName = "cascade_clouds",
                      posesContentName = "cascade_poses",
                      timestampsContentName = "cascade_timestamps";
                      
    auto exportCfg = cascade_->exportConfig();

    // FOV
    float range = cascadeConfig_->clipRange(exportCfg->fov().rangeMeters);
    int rangeMaxBin = cascadeConfig_->rangeToRangeIdx(range);
    int azimuthMaxBin = exportCfg->fov().azimuthIdx;
    int elevationMaxBin = exportCfg->fov().elevationIdx;
    float horizontalFov = exportCfg->fov().horizontalDegreesTotal;
    float verticalFov = exportCfg->fov().verticalDegreesTotal;
    if (exportCfg->fov().useDegreeConstraints) {
        azimuthMaxBin = cascadeConfig_->horizontalFovToAzimuthIdx(exportCfg->fov().horizontalDegreesTotal);
        elevationMaxBin = cascadeConfig_->verticalFovToElevationIdx(exportCfg->fov().verticalDegreesTotal);
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
        std::vector<Eigen::Affine3f> sensorPoses;
        for (int i = 0; i < numFrames; ++i) {
            sensorPoses[i] = basePoses[i] * cascadeTransform_;
        }

        // timestamps
        if (exportCfg->exportTimestamps()) {
            saveVectorToHDF5(timestampsContentName + "_" + run->name, datasetFile, timestamps);
        }

        // poses
        if (exportCfg->exportPoses()) {
            savePosesToHDF5(posesContentName + "_" + run->name, datasetFile, sensorPoses);
        }

        // datacubes
        if (exportCfg->exportDatacubes()) {
            std::vector<int16_t> datacubesFlat;
            for (size_t i = 0; i < numFrames; ++i) {
                auto datacube = run->getCascadeDatacube(i);
                std::copy(datacube.begin(), datacube.end(), datacubesFlat.begin() + i * datacube.size());
            }
            saveDatacubesToHDF5(datacubeContentName + "_" + run->name, datasetFile, datacubesFlat, numFrames, cascadeConfig_);
        }
        
        // heatmaps
        if (exportCfg->exportHeatmaps()) {
            CascadeConfig* heatmapConfig = new CascadeConfig(*dynamic_cast<CascadeConfig*>(cascadeConfig_));
            std::vector<float> heatmapsFlat;
            for (size_t i = 0; i < numFrames; ++i) {
                auto heatmap = run->getCascadeHeatmap(i);
                heatmap = heatmapConfig->clipHeatmap(heatmap, azimuthMaxBin, elevationMaxBin, rangeMaxBin);
                if (exportCfg->collapseElevation()) {
                    heatmap = heatmapConfig->collapseHeatmapElevation(heatmap, exportCfg->collapseElevationMinZ(), exportCfg->collapseElevationMaxZ());
                }
                if (exportCfg->removeDopplerDim()) {
                    heatmap = heatmapConfig->removeDoppler(heatmap);
                }
                heatmap = heatmapConfig->swapHeatmapDimensions(heatmap);
                heatmapsFlat.insert(heatmapsFlat.end(), heatmap.begin(), heatmap.end());
            }
            saveHeatmapsToHDF5(heatmapContentName + "_" + run->name, datasetFile, heatmapsFlat, numFrames, heatmapConfig);
        }

        // clouds
        if (exportCfg->exportClouds()) {
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
                    cloud = heatmapToPointcloud(run->getCascadeHeatmap(i), cascadeConfig_, exportCfg->intensityThresholdPercent());
                }
                else {
                    cloud = run->getCascadePointcloud(i, exportCfg->intensityThresholdPercent());
                }
                filterFov(cloud, horizontalFov, verticalFov, range);
                if (exportCfg->cloudsInGlobalFrame()) {
                    pcl::transformPointCloud(cloud, cloud, sensorPoses[i]);
                }
                if (exportCfg->collapseElevation()) {
                    collapseElevation(cloud, exportCfg->collapseElevationMinZ(), exportCfg->collapseElevationMaxZ());
                }
                cloudSizes[i] = cloud.size();
                std::vector<float> cloudFlat = flattenRadarCloud(cloud, cascadeConfig_);
                cloudsFlat.insert(cloudsFlat.end(), cloudFlat.begin(), cloudFlat.end());
            }
            saveCloudsToHDF5(cloudContentName + "_" + run->name, datasetFile, cloudsFlat, numFrames, cloudSizes, numDims);
        }
    }
}

void ColoradarPlusDataset::exportLidar(const std::vector<ColoradarPlusRun*> &runs, const H5::H5File &datasetFile) {
    // Constants
    const std::string cloudContentName = "lidar_clouds",
                      mapContentName = "lidar_map",
                      mapSampleContentName = "lidar_map_samples",
                      posesContentName = "lidar_poses",
                      timestampsContentName = "lidar_timestamps";

    auto exportCfg = lidar_->exportConfig();

    for (auto* run : runs) {
        std::vector<double> timestamps = run->lidarTimestamps();
        hsize_t numFrames = timestamps.size();
        std::vector<Eigen::Affine3f> basePoses = run->interpolatePoses(run->getPoses<Eigen::Affine3f>(), run->poseTimestamps(), timestamps);
        std::vector<Eigen::Affine3f> sensorPoses;
        for (int i = 0; i < numFrames; ++i) {
            sensorPoses[i] = basePoses[i] * lidarTransform_;
        }

        // timestamps
        if (exportCfg->exportTimestamps()) {
            saveVectorToHDF5(timestampsContentName + "_" + run->name, datasetFile, timestamps);
        }

        // poses
        if (exportCfg->exportPoses()) {
            savePosesToHDF5(posesContentName + "_" + run->name, datasetFile, sensorPoses);
        }

        // clouds
        if (exportCfg->exportClouds()) {
            hsize_t numDims = exportCfg->collapseElevation() ? 3 : 4;  // x, y, (z), intensity
            if (exportCfg->removeIntensityDim()) numDims -= 1;

            std::vector<float> cloudsFlat;
            std::vector<hsize_t> cloudSizes;
            for (size_t i = 0; i < numFrames; ++i) {
                auto cloud = run->getLidarPointCloud<pcl::PointCloud<pcl::PointXYZI>>(i);
                filterFov(
                    cloud,
                    exportCfg->cloudFov().horizontalDegreesTotal, 
                    exportCfg->cloudFov().verticalDegreesTotal, 
                    exportCfg->cloudFov().rangeMeters
                );
                if (exportCfg->collapseElevation()) {
                    collapseElevation(cloud, exportCfg->collapseElevationMinZ(), exportCfg->collapseElevationMaxZ());
                }
                cloudSizes[i] = cloud.size();
                std::vector<float> cloudFlat = flattenLidarCloud(cloud, exportCfg->collapseElevation(), exportCfg->removeIntensityDim());
                cloudsFlat.insert(cloudsFlat.end(), cloudFlat.begin(), cloudFlat.end());
            }
            saveCloudsToHDF5(cloudContentName + "_" + run->name, datasetFile, cloudsFlat, numFrames, cloudSizes, numDims);
        }
        
        // map and samples
        if (exportCfg->exportMap() || exportCfg->exportMapSamples()) {
            pcl::PointCloud<pcl::PointXYZI> map = run->readLidarOctomap();
            hsize_t numDims = exportCfg->collapseElevation() ? 3 : 4;  // x, y, (z), occupancy
            if (exportCfg->removeOccupancyDim()) numDims -= 1;

            if (exportCfg->exportMap()) {
                if (exportCfg->collapseElevation()) {
                    collapseElevation(map, exportCfg->collapseElevationMinZ(), exportCfg->collapseElevationMaxZ());
                }
                filterOccupancy(map, exportCfg->occupancyThresholdPercent() / 100.0, exportCfg->logOddsToProbability());
                std::vector<float> mapFlat = flattenLidarCloud(map, exportCfg->collapseElevation(), exportCfg->removeOccupancyDim());
                saveCloudToHDF5(mapContentName + "_" + run->name, datasetFile, mapFlat, numDims);
            }

            if (exportCfg->exportMapSamples()) {
//                hsize_t numSamples = config->centerCensor()->name;
//                std::vector<float> samplesFlat;
//                std::vector<hsize_t> cloudSizes(numFrames);
//                for (size_t i = 0; i < numCascadeFrames; ++i) {
//                pcl::PointCloud<pcl::PointXYZI> lidarMapSample;
//                try {
//                    lidarMapSample = run->readMapFrame(i);
//                } catch (const std::filesystem::filesystem_error& e) {
//                    run->sampleMapFrames(mapSampleTotalHorizontalFov, mapSampleTotalVerticalFov, mapSampleMaxRange, mapSamplingBaseToSensorTransform, mapSamplingBasePoses);
//                    lidarMapSample = run->readMapFrame(i);
//                }
//                if (collapseMapSampleElevation)
//                    coloradar::collapseElevation(lidarMapSample, collapseMapSampleElevationMinZ, collapseMapSampleElevationMaxZ);
//                cloudSizes[i] = lidarMapSample.size();
//                auto cloudFlat = flattenLidarCloud(lidarMapSample, collapseMapSampleElevation, true);
//                framesFlat.insert(framesFlat.end(), cloudFlat.begin(), cloudFlat.end());
//                }
                // saveCloudsToHDF5(mapFrameContentName + "_" + run->name, datasetFile, framesFlat, numCascadeFrames, mapFrameNumDims, cloudSizes);
            }
        }
    }
}

}


//std::filesystem::path coloradar::ColoradarPlusDataset::exportToFile(
//    std::vector<ColoradarPlusRun*> runs,
//    std::filesystem::path destination,
//
//    const bool& includeCascadeHeatmaps,
//    const bool& includeCascadePointclouds,
//    const bool& cascadePointcloudsInGlobalFrame,
//    const int& cascadeAzimuthMaxBin,
//    const int& cascadeElevationMaxBin,
//    const int& cascadeRangeMaxBin,
//    const bool& removeCascadeDopplerDim,
//    const bool& collapseCascadeElevation,
//    const int& collapseCascadeElevationMinZ,
//    const int& collapseCascadeElevationMaxZ,
//    const float& cascadeCloudIntensityThresholdPercent,
//
//    const bool& includeLidarFrames,
//    const float& lidarFrameTotalHorizontalFov,
//    const float& lidarFrameTotalVerticalFov,
//    const float& lidarFrameMaxRange,
//    const bool& collapseLidarFrameElevation,
//    const float& collapseLidarFrameElevationMinZ,
//    const float& collapseLidarFrameElevationMaxZ,
//
//    const bool& includeLidarMap,
//    const bool& collapseMapElevation,
//    const float& collapseMapElevationMinZ,
//    const float& collapseMapElevationMaxZ,
//    const Eigen::Affine3f& lidarMapTransform,
//
//    const bool& includeMapFrames,
//    const float& mapSampleTotalHorizontalFov,
//    const float& mapSampleTotalVerticalFov,
//    const float& mapSampleMaxRange,
//    const Eigen::Affine3f& mapSamplingBaseToSensorTransform,
//    std::vector<Eigen::Affine3f> mapSamplingBasePoses,
//    const bool& collapseMapSampleElevation,
//    const float& collapseMapSampleElevationMinZ,
//    const float& collapseMapSampleElevationMaxZ,
//
//    const bool& removeLidarIntensity,
//
//    const bool& includeTruePoses,
//    const bool& includeCascadePoses,
//    const bool& includeLidarPoses,
//    const bool& includeTrueTimestamps,
//    const bool& includeCascadeTimestamps,
//    const bool& includeLidarTimestamps
//)
//{
//    const std::string cascadeHeatmapContentName = "cascade_heatmaps", cascadeCloudContentName = "cascade_clouds",
//                      lidarFrameContentName = "lidar_frames", lidarMapContentName = "lidar_map", mapFrameContentName = "lidar_map_frames",
//                      truePosesContentName = "true_poses", cascadePosesContentName = "cascade_poses", lidarPosesContentName = "lidar_poses",
//                      trueTimestampsContentName = "true_timestamps", cascadeTimestampsContentName = "cascade_timestamps", lidarTimestampsContentName = "lidar_timestamps";
//    if (runs.empty()) {
//        runs = getRuns();
//    }
//    if (destination.empty()) {
//        destination = "dataset.h5";
//    }
//    std::filesystem::path destinationAbs = std::filesystem::absolute(destination);
//    H5::H5File datasetFile(destinationAbs, H5F_ACC_TRUNC);
//    Json::Value finalConfig;
//    finalConfig["runs"] = Json::arrayValue;
//    finalConfig["data_content"] = Json::arrayValue;
//    finalConfig["radar_config"] = cascadeConfig_->toJson();
//    if (includeCascadeHeatmaps) finalConfig["data_content"].append(cascadeHeatmapContentName);
//    if (includeCascadePointclouds) finalConfig["data_content"].append(cascadeCloudContentName);
//    if (includeLidarFrames) finalConfig["data_content"].append(lidarFrameContentName);
//    if (includeLidarMap) finalConfig["data_content"].append(lidarMapContentName);
//    if (includeMapFrames) finalConfig["data_content"].append(mapFrameContentName);
//    if (includeTruePoses) finalConfig["data_content"].append(truePosesContentName);
//    if (includeCascadePoses) finalConfig["data_content"].append(cascadePosesContentName);
//    if (includeLidarPoses) finalConfig["data_content"].append(lidarPosesContentName);
//    if (includeTrueTimestamps) finalConfig["data_content"].append(trueTimestampsContentName);
//    if (includeCascadeTimestamps) finalConfig["data_content"].append(cascadeTimestampsContentName);
//    if (includeLidarTimestamps) finalConfig["data_content"].append(lidarTimestampsContentName);
//
//    int lidarFrameNumDims = (collapseLidarFrameElevation ? 3 : 4) - removeLidarIntensity,
//        lidarMapNumDims = collapseMapElevation ? 3 : 4,
//        mapFrameNumDims = collapseMapSampleElevation ? 3 : 4;
//    int cascadeCloudNumDims = collapseCascadeElevation ? 3 : 4;
//    int cascadeNumAzimuthBins = cascadeAzimuthMaxBin >= 0 && (cascadeAzimuthMaxBin + 1) * 2 < cascadeConfig_->numAzimuthBins ?
//                                (cascadeAzimuthMaxBin + 1) * 2 :
//                                cascadeConfig_->numAzimuthBins;
//    int cascadeNumElevationBins = cascadeElevationMaxBin >= 0 && (cascadeElevationMaxBin + 1) * 2 < cascadeConfig_->numElevationBins ?
//                                  (cascadeElevationMaxBin + 1) * 2 :
//                                  cascadeConfig_->numElevationBins;
//    int cascadeNumRangeBins = cascadeRangeMaxBin >= 0 && cascadeRangeMaxBin + 1 < cascadeConfig_->numPosRangeBins ?
//                              cascadeRangeMaxBin + 1 :
//                              cascadeConfig_->numPosRangeBins;
//    int cascadeNumDims = removeCascadeDopplerDim ? 1 : 2;
//    int elStartIdx = (cascadeConfig_->numElevationBins - cascadeNumElevationBins) / 2;
//    int elEndIdx = elStartIdx + cascadeNumElevationBins;
//    std::vector<float> elevationBins(cascadeConfig_->elevationBins.begin() + elStartIdx, cascadeConfig_->elevationBins.begin() + elEndIdx);
//    float cascadeHorizontalFov, cascadeVerticalFov, cascadeRange;
//    coloradar::convertRadarBinsToFov(cascadeAzimuthMaxBin, cascadeElevationMaxBin, cascadeRangeMaxBin, cascadeConfig_, cascadeHorizontalFov, cascadeVerticalFov, cascadeRange);
//    std::vector<hsize_t> heatmapDims;
//    if (cascadeNumAzimuthBins > 1) heatmapDims.push_back(static_cast<hsize_t>(cascadeNumAzimuthBins));
//    if (cascadeNumRangeBins > 1) heatmapDims.push_back(static_cast<hsize_t>(cascadeNumRangeBins));
//    if (cascadeNumElevationBins > 1) heatmapDims.push_back(static_cast<hsize_t>(cascadeNumElevationBins));
//    if (cascadeNumDims > 1) heatmapDims.push_back(static_cast<hsize_t>(cascadeNumDims));
//    if (heatmapDims.empty()) heatmapDims.push_back(1);
//
////    finalConfig["data_dimensions"][cascadeHeatmapContentName]["cascade_num_azimuth_bins"] = cascadeNumAzimuthBins;
////    finalConfig["data_dimensions"][cascadeHeatmapContentName]["cascade_num_elevation_bins"] = collapseCascadeElevation ? 1 : cascadeNumElevationBins;
////    finalConfig["data_dimensions"][cascadeHeatmapContentName]["cascade_num_range_bins"] = cascadeNumRangeBins;
////    finalConfig["data_dimensions"][cascadeHeatmapContentName]["cascade_num_dimensions"] = cascadeNumDim;
//
//    for (auto* run : runs) {
//        finalConfig["runs"].append(run->name);
//        int numCascadeFrames = run->cascadeTimestamps().size();
//        int numLidarFrames = run->lidarTimestamps().size();
//        bool savedHeatmaps = false;
////        finalConfig["data_dimensions"]['cascade_num_frames'][run->name] = numCascadeFrames;
////        finalConfig["data_dimensions"]['lidar_num_frames'][run->name] = numLidarFrames;
//
//        std::vector<Eigen::Affine3f> poses = run->getPoses<Eigen::Affine3f>();
//        std::vector<Eigen::Affine3f> cascadePoses = run->interpolatePoses(poses, run->poseTimestamps(), run->cascadeTimestamps());
//        std::vector<Eigen::Affine3f> lidarPoses = run->interpolatePoses(poses, run->poseTimestamps(), run->lidarTimestamps());
//        for (int i = 0; i < numCascadeFrames; ++i) {
//            cascadePoses[i] = cascadePoses[i] * cascadeTransform_;
//        }
//        for (int i = 0; i < numLidarFrames; ++i) {
//            lidarPoses[i] = lidarPoses[i] * lidarTransform_;
//        }
//
//        if (includeCascadePointclouds) {
//            std::vector<float> framesFlat;
//            std::vector<float> heatmapsFlat;
//            std::vector<hsize_t> cloudSizes(numCascadeFrames);
//
//            for (size_t i = 0; i < numCascadeFrames; ++i) {
//                Eigen::Affine3f cascadeCloudTransform = cascadePointcloudsInGlobalFrame ? cascadePoses[i] : Eigen::Affine3f::Identity();
//                pcl::PointCloud<coloradar::RadarPoint> rawCascadeCloud;
//                try {
//                    rawCascadeCloud = run->getCascadePointcloud(i, cascadeCloudIntensityThresholdPercent);
//                } catch (const std::filesystem::filesystem_error& e) {
//                    auto heatmap = run->getCascadeHeatmap(i);
//                    rawCascadeCloud = coloradar::heatmapToPointcloud(heatmap, cascadeConfig_, cascadeCloudIntensityThresholdPercent);
//                    if (includeCascadeHeatmaps) {
//                        // finalConfig["data_content"].append(cascadeHeatmapContentName);
//                        heatmap = coloradar::clipHeatmapImage(heatmap, cascadeAzimuthMaxBin, cascadeElevationMaxBin, cascadeRangeMaxBin, cascadeConfig_);
//                        if (collapseCascadeElevation)
//                            heatmap = coloradar::collapseHeatmapElevation(heatmap, collapseCascadeElevationMinZ, collapseCascadeElevationMaxZ, elevationBins, cascadeNumAzimuthBins, cascadeNumRangeBins);
//                        if (removeCascadeDopplerDim)
//                            heatmap = coloradar::removeDoppler(heatmap);
//                        auto heatmapFlat = flattenHeatmap(heatmap, cascadeNumAzimuthBins, collapseCascadeElevation? 1 : cascadeNumElevationBins, cascadeNumRangeBins, cascadeNumDims);
//                        heatmapsFlat.insert(heatmapsFlat.end(), heatmapFlat.begin(), heatmapFlat.end());
//                        savedHeatmaps = true;
//                    }
//                }
//                pcl::PointCloud<coloradar::RadarPoint> cascadeCloud;
//                pcl::transformPointCloud(rawCascadeCloud, cascadeCloud, cascadeCloudTransform);
//                coloradar::filterFov(cascadeCloud, cascadeHorizontalFov, cascadeVerticalFov, cascadeRange);
//                if (collapseCascadeElevation) {
//                    coloradar::collapseElevation(cascadeCloud, collapseCascadeElevationMinZ, collapseCascadeElevationMaxZ);
//                }
//                cloudSizes[i] = cascadeCloud.size();
//                // auto cloudFlat = flattenRadarCloud(cascadeCloud, collapseCascadeElevation);
//                // framesFlat.insert(framesFlat.end(), cloudFlat.begin(), cloudFlat.end());
//            }
//            // saveCloudsToHDF5(cascadeCloudContentName + "_" + run->name, datasetFile, framesFlat, numCascadeFrames, cascadeCloudNumDims, cloudSizes);
//            // if (savedHeatmaps) saveHeatmapsToHDF5(cascadeHeatmapContentName + "_" + run->name, datasetFile, heatmapsFlat, numCascadeFrames, heatmapDims);
//        }
//        if (includeCascadeHeatmaps && !savedHeatmaps) {
//            // finalConfig["data_content"].append(cascadeHeatmapContentName);
//            std::vector<float> heatmapsFlat;
//            for (size_t i = 0; i < numCascadeFrames; ++i) {
//                auto heatmap = run->getCascadeHeatmap(i);
//                heatmap = coloradar::clipHeatmapImage(heatmap, cascadeAzimuthMaxBin, cascadeElevationMaxBin, cascadeRangeMaxBin, cascadeConfig_);
//                if (collapseCascadeElevation)
//                    heatmap = coloradar::collapseHeatmapElevation(heatmap, collapseCascadeElevationMinZ, collapseCascadeElevationMaxZ, elevationBins, cascadeNumAzimuthBins, cascadeNumRangeBins);
//                if (removeCascadeDopplerDim)
//                    heatmap = coloradar::removeDoppler(heatmap);
//                auto heatmapFlat = flattenHeatmap(heatmap, cascadeNumAzimuthBins, collapseCascadeElevation? 1 : cascadeNumElevationBins, cascadeNumRangeBins, cascadeNumDims);
//                heatmapsFlat.insert(heatmapsFlat.end(), heatmapFlat.begin(), heatmapFlat.end());
//            }
//            // saveHeatmapsToHDF5(cascadeHeatmapContentName + "_" + run->name, datasetFile, heatmapsFlat, numCascadeFrames, heatmapDims);
//        }
//        if (includeLidarFrames) {
//            // finalConfig["data_content"].append(lidarFrameContentName);
//            std::vector<float> framesFlat;
//            std::vector<hsize_t> cloudSizes(numLidarFrames);
//            for (size_t i = 0; i < numLidarFrames; ++i) {
//                auto lidarFrame = run->getLidarPointCloud<pcl::PointCloud<pcl::PointXYZI>>(i);
//                coloradar::filterFov(lidarFrame, lidarFrameTotalHorizontalFov, lidarFrameTotalVerticalFov, lidarFrameMaxRange);
//                if (collapseLidarFrameElevation)
//                    coloradar::collapseElevation(lidarFrame, collapseLidarFrameElevationMinZ, collapseLidarFrameElevationMaxZ);
//                cloudSizes[i] = lidarFrame.size();
//                auto cloudFlat = flattenLidarCloud(lidarFrame, collapseLidarFrameElevation, !removeLidarIntensity);
//                framesFlat.insert(framesFlat.end(), cloudFlat.begin(), cloudFlat.end());
//            }
//            // saveCloudsToHDF5(lidarFrameContentName + "_" + run->name, datasetFile, framesFlat, numLidarFrames, lidarFrameNumDims, cloudSizes);
//        }
//        if (includeLidarMap) {
//            pcl::PointCloud<pcl::PointXYZI> mapRaw = run->readLidarOctomap();
//            pcl::PointCloud<pcl::PointXYZI> map;
//            pcl::transformPointCloud(mapRaw, map, lidarMapTransform);
//            if (collapseMapElevation)
//                coloradar::collapseElevation(map, collapseMapElevationMinZ, collapseMapElevationMaxZ);
//            std::vector<float> mapFlat = flattenLidarCloud(map, collapseMapElevation, true);
//            saveCloudToHDF5(lidarMapContentName + "_" + run->name, datasetFile, mapFlat, lidarMapNumDims);
//        }
//        if (includeMapFrames) {
//            std::vector<float> framesFlat;
//            std::vector<hsize_t> cloudSizes(numCascadeFrames);
//            for (size_t i = 0; i < numCascadeFrames; ++i) {
//                pcl::PointCloud<pcl::PointXYZI> lidarMapSample;
//                try {
//                    lidarMapSample = run->readMapFrame(i);
//                } catch (const std::filesystem::filesystem_error& e) {
//                    run->sampleMapFrames(mapSampleTotalHorizontalFov, mapSampleTotalVerticalFov, mapSampleMaxRange, mapSamplingBaseToSensorTransform, mapSamplingBasePoses);
//                    lidarMapSample = run->readMapFrame(i);
//                }
//                if (collapseMapSampleElevation)
//                    coloradar::collapseElevation(lidarMapSample, collapseMapSampleElevationMinZ, collapseMapSampleElevationMaxZ);
//                cloudSizes[i] = lidarMapSample.size();
//                auto cloudFlat = flattenLidarCloud(lidarMapSample, collapseMapSampleElevation, true);
//                framesFlat.insert(framesFlat.end(), cloudFlat.begin(), cloudFlat.end());
//            }
//            // saveCloudsToHDF5(mapFrameContentName + "_" + run->name, datasetFile, framesFlat, numCascadeFrames, mapFrameNumDims, cloudSizes);
//        }
//        if (includeTruePoses) {
//            savePosesToHDF5(truePosesContentName + "_" + run->name, datasetFile, poses);
//        }
//        if (includeCascadePoses) {
//            savePosesToHDF5(cascadePosesContentName + "_" + run->name, datasetFile, cascadePoses);
//        }
//        if (includeLidarPoses) {
//            savePosesToHDF5(lidarPosesContentName + "_" + run->name, datasetFile, lidarPoses);
//        }
//        if (includeTrueTimestamps) {
//            saveVectorToHDF5(trueTimestampsContentName + "_" + run->name, datasetFile, run->poseTimestamps());
//        }
//        if (includeCascadeTimestamps) {
//            saveVectorToHDF5(cascadeTimestampsContentName + "_" + run->name, datasetFile, run->cascadeTimestamps());
//        }
//        if (includeLidarTimestamps) {
//            saveVectorToHDF5(lidarTimestampsContentName + "_" + run->name, datasetFile, run->lidarTimestamps());
//        }
//    }
//
//    std::string configString = Json::writeString(Json::StreamWriterBuilder(), finalConfig);
//    H5::StrType strType(H5::PredType::C_S1, H5T_VARIABLE);
//    H5::DataSpace dataspace(H5S_SCALAR);
//    H5::DataSet configDataset = datasetFile.createDataSet("config", strType, dataspace);
//    configDataset.write(configString, strType);
//
//    return destinationAbs;
//}


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
