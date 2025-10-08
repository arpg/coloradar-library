#include "dataset/coloradar_plus_dataset.h"


namespace coloradar {


// CONSTRUCTOR METHODS

ColoradarPlusDataset::ColoradarPlusDataset(const std::filesystem::path& pathToDataset) {
    init(pathToDataset);
    postInit();
}

ColoradarPlusDataset::ColoradarPlusDataset(const std::filesystem::path& pathToRunsDir, const std::filesystem::path& pathToCalibDir) {
    init(pathToRunsDir, pathToCalibDir);
    postInit();
}

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

    for (const auto& entry : std::filesystem::directory_iterator(runsDirPath_)) {
        std::string entryName = entry.path().filename().string();
        if (entry.is_directory() && coloradar::internal::toLower(entryName).find("run") != std::string::npos) {
            runs_.push_back(std::make_shared<ColoradarPlusRun>(runsDirPath_ / entryName, cascadeConfig_));
            runNames_.push_back(entryName);
        }
    }
}

void ColoradarPlusDataset::postInit() {
    cascadeTransform_ = loadTransform(transformsDirPath_ / "base_to_radar.txt");
}


// PUBLIC METHODS

std::filesystem::path ColoradarPlusDataset::exportToFile(const std::string &yamlConfigPath) {
    DatasetExportConfig exportConfig = DatasetExportConfig(yamlConfigPath);
    return exportToFile(exportConfig);
}

Json::Value affineToJson(const Eigen::Affine3f& transform) {
    Json::Value transformJson(Json::objectValue);
    Eigen::Vector3f trans = transform.translation();
    Json::Value transJson(Json::arrayValue);
    transJson.append(trans.x());
    transJson.append(trans.y());
    transJson.append(trans.z());
    transformJson["translation"] = transJson;
    Eigen::Quaternionf rot(transform.linear());
    Json::Value rotJson(Json::arrayValue);
    rotJson.append(rot.x());
    rotJson.append(rot.y());
    rotJson.append(rot.z());
    rotJson.append(rot.w());
    transformJson["rotation"] = rotJson;
    return transformJson;
}

std::filesystem::path ColoradarPlusDataset::exportToFile(DatasetExportConfig& exportConfig) {
    Json::Value finalConfig;
    finalConfig["runs"] = Json::arrayValue;
    finalConfig["radar_config"] = cascadeConfig_->toJson();
    finalConfig["data_content"] = Json::arrayValue;

    auto exportRunNames = exportConfig.runs().empty() ? listRuns() : exportConfig.runs();
    exportConfig.initDataset(exportRunNames);
    std::vector<std::shared_ptr<ColoradarPlusRun>> exportRunObjects;
    for (auto runName : exportRunNames) {
        exportRunObjects.push_back(std::dynamic_pointer_cast<ColoradarPlusRun>(getRun(runName)));
        finalConfig["runs"].append(runName);
    }
    std::cout << "Exporting " << finalConfig["runs"].size() << " run(s)." << std::endl;

    H5::H5File datasetFile(exportConfig.destinationFilePath(), H5F_ACC_TRUNC);
    std::vector<std::string> baseContent, imuContent, cascadeContent, lidarContent;
    try {
        baseContent = exportBaseDevice(exportConfig.base(), exportRunObjects, datasetFile);
    } catch (const std::exception& e) {
        std::cerr << "Error exporting base device data: " << e.what() << std::endl;
        throw;
    }
    try {
        imuContent = exportImu(exportConfig.imu(), exportRunObjects, datasetFile);
    } catch (const std::exception& e) {
        std::cerr << "Error exporting IMU data: " << e.what() << std::endl;
        throw;
    }
    try {
        cascadeContent = exportCascade(exportConfig.cascade(), exportRunObjects, datasetFile, &finalConfig);
    } catch (const std::exception& e) {
        std::cerr << "Error exporting cascade radar data: " << e.what() << std::endl;
        throw;
    }
    try {
        lidarContent = exportLidar(exportConfig.lidar(), exportRunObjects, datasetFile);
    } catch (const std::exception& e) {
        std::cerr << "Error exporting LiDAR data: " << e.what() << std::endl;
        throw;
    }

    for (const auto &contentList : {baseContent, imuContent, cascadeContent, lidarContent}) {
        if (contentList.empty()) continue;
        for (const auto &content : contentList) {
            finalConfig["data_content"].append(content);
        }
    }

    if (exportConfig.exportTransforms()) {
        std::cout << "Exporting transforms..." << std::endl;
        Json::Value transformsJson(Json::objectValue);
        transformsJson[transformBaseToCascadeContentName] = affineToJson(cascadeTransform_);
        transformsJson[transformBaseToLidarContentName] = affineToJson(lidarTransform_);
        transformsJson[transformBaseToImuContentName] = affineToJson(imuTransform_);
        finalConfig["transforms"] = transformsJson;
    }

    std::string configString = Json::writeString(Json::StreamWriterBuilder(), finalConfig);
    const hsize_t dims[1] = { static_cast<hsize_t>(configString.size()) };
    H5::DataSpace dataspace(1, dims);
    H5::DataSet configDataset = datasetFile.createDataSet("config", H5::PredType::STD_U8LE, dataspace);
    const unsigned char* bytes = reinterpret_cast<const unsigned char*>(configString.data());
    configDataset.write(bytes, H5::PredType::NATIVE_UINT8);
    configDataset.close();

    std::cout << "Closing file..." << std::endl;
    datasetFile.close();
    return exportConfig.destinationFilePath();
}


// PROTECTED METHODS

Eigen::Affine3f ColoradarPlusDataset::loadTransform(const std::filesystem::path& filePath) {
    coloradar::internal::checkPathExists(filePath);

    Eigen::Vector3f translation;
    std::ifstream file(filePath);
    std::string line;
    std::getline(file, line);
    std::istringstream iss(line);
    iss >> translation.x() >> translation.y() >> translation.z();

    Eigen::Quaternionf rotation;
    std::getline(file, line);
    iss.str(line);
    iss.clear();
    iss >> rotation.x() >> rotation.y() >> rotation.z() >> rotation.w();

    Eigen::Affine3f transform = Eigen::Translation3f(translation) * rotation;
    return transform;
}

std::vector<std::string> ColoradarPlusDataset::exportBaseDevice(const BaseExportConfig &config, std::vector<std::shared_ptr<ColoradarPlusRun>> runs, H5::H5File& datasetFile) {
    std::vector<std::string> content;
    if (!config.exportTimestamps() && !config.exportPoses()) return content;
    std::cout << "Exporting base data..." << std::endl;

    // Content types
    if (config.exportTimestamps()) content.push_back(poseTimestampsContentName);
    if (config.exportPoses()) content.push_back(posesContentName);

    for (auto run : runs) {
        std::vector<double> timestamps = run->poseTimestamps();
        hsize_t numFrames = timestamps.size();
        std::vector<Eigen::Affine3f> basePoses = run->getPoses<Eigen::Affine3f>();

        // timestamps
        if (config.exportTimestamps()) {
            coloradar::internal::saveVectorToHDF5(poseTimestampsContentName + "_" + run->name(), datasetFile, timestamps);
        }

        // poses
        if (config.exportPoses()) {
            coloradar::internal::savePosesToHDF5(posesContentName + "_" + run->name(), datasetFile, basePoses);
        }
    }
    datasetFile.flush(H5F_SCOPE_GLOBAL);
    return content;
}

std::vector<std::string> ColoradarPlusDataset::exportImu(const ImuExportConfig &config, std::vector<std::shared_ptr<ColoradarPlusRun>> runs, H5::H5File& datasetFile) {
    std::vector<std::string> content;
    if (!config.exportTimestamps() && !config.exportPoses()) return content;
    std::cout << "Exporting IMU data..." << std::endl;

    // Content types
    if (config.exportTimestamps()) content.push_back(imuTimestampsContentName);
    if (config.exportPoses()) content.push_back(imuPosesContentName);

    for (auto run : runs) {
        std::vector<double> timestamps = run->imuTimestamps();
        hsize_t numFrames = timestamps.size();
        std::vector<Eigen::Affine3f> basePoses = interpolatePoses(run->getPoses<Eigen::Affine3f>(), run->poseTimestamps(), timestamps);
        std::vector<Eigen::Affine3f> sensorPoses(numFrames);
        for (int i = 0; i < numFrames; ++i) {
            sensorPoses[i] = basePoses[i] * imuTransform_;
        }

        // timestamps
        if (config.exportTimestamps()) {
            coloradar::internal::saveVectorToHDF5(imuTimestampsContentName + "_" + run->name(), datasetFile, timestamps);
        }

        // poses
        if (config.exportPoses()) {
            coloradar::internal::savePosesToHDF5(imuPosesContentName + "_" + run->name(), datasetFile, sensorPoses);
        }

        // data: TBD
    }
    datasetFile.flush(H5F_SCOPE_GLOBAL);
    return content;
}

std::vector<std::string> ColoradarPlusDataset::exportCascade(
    const RadarExportConfig &config,
    std::vector<std::shared_ptr<ColoradarPlusRun>> runs, 
    H5::H5File& datasetFile,
    Json::Value* finalConfig
) {
    std::vector<std::string> content;
    if (!config.exportTimestamps() && !config.exportPoses() && !config.exportDatacubes() && !config.exportHeatmaps() && !config.exportClouds()) return content;
    std::cout << "Exporting cascade data..." << std::endl;

    // Content types
    if (config.exportTimestamps()) {
        content.push_back(cascadeCubeTimestampsContentName);
        content.push_back(cascadeTimestampsContentName);
    } 
    if (config.exportPoses()) content.push_back(cascadePosesContentName);
    if (config.exportClouds()) content.push_back(cascadeCloudsContentName);
    if (config.exportDatacubes()) content.push_back(cascadeDatacubesContentName);
    if (config.exportHeatmaps()) content.push_back(cascadeHeatmapsContentName);

    // FOV
    float range = cascadeConfig_->clipRange(config.heatmapCloudFov().rangeMeters);
    int rangeMaxBin = cascadeConfig_->rangeToRangeIdx(range);
    int azimuthMaxBin = config.heatmapCloudFov().azimuthIdx;
    int elevationMaxBin = config.heatmapCloudFov().elevationIdx;
    float horizontalFov = config.heatmapCloudFov().horizontalDegreesTotal;
    float verticalFov = config.heatmapCloudFov().verticalDegreesTotal;
    if (config.heatmapCloudFov().useDegreeConstraints) {
        azimuthMaxBin = cascadeConfig_->horizontalFovToAzimuthIdx(config.heatmapCloudFov().horizontalDegreesTotal);
        elevationMaxBin = cascadeConfig_->verticalFovToElevationIdx(config.heatmapCloudFov().verticalDegreesTotal);
    }
    else {
        azimuthMaxBin = cascadeConfig_->clipAzimuthMaxBin(azimuthMaxBin);
        elevationMaxBin = cascadeConfig_->clipElevationMaxBin(elevationMaxBin);
        horizontalFov = cascadeConfig_->azimuthIdxToFovDegrees(azimuthMaxBin);
        verticalFov = cascadeConfig_->elevationIdxToFovDegrees(elevationMaxBin);
    }
    std::shared_ptr<CascadeConfig> heatmapConfig;

    for (auto run : runs) {
        std::vector<double> timestamps = run->cascadeTimestamps();
        hsize_t numFrames = timestamps.size();
        std::vector<Eigen::Affine3f> basePoses = interpolatePoses(run->getPoses<Eigen::Affine3f>(), run->poseTimestamps(), timestamps);
        std::vector<Eigen::Affine3f> sensorPoses(numFrames);

        // timestamps
        if (config.exportTimestamps()) {
            coloradar::internal::saveVectorToHDF5(cascadeCubeTimestampsContentName + "_" + run->name(), datasetFile, run->cascadeCubeTimestamps());
            coloradar::internal::saveVectorToHDF5(cascadeTimestampsContentName + "_" + run->name(), datasetFile, timestamps);
        }

        // poses
        if (config.exportPoses()) {
            coloradar::internal::savePosesToHDF5(cascadePosesContentName + "_" + run->name(), datasetFile, sensorPoses);
        }

        // datacubes
        if (config.exportDatacubes()) {
            hsize_t numCubeFrames = run->cascadeCubeTimestamps().size();
            hsize_t datacubeSize = static_cast<hsize_t>(cascadeConfig_->numElevationBins() * cascadeConfig_->numAzimuthBins() * cascadeConfig_->nRangeBins() * 2);
            std::vector<int16_t> datacubesFlat;
            for (size_t i = 0; i < numCubeFrames; ++i) {
                auto datacube = run->getCascadeDatacube(i);
                datacubesFlat.insert(datacubesFlat.end(), datacube->begin(), datacube->end());
            }
            coloradar::internal::saveDatacubesToHDF5(cascadeDatacubesContentName + "_" + run->name(), datasetFile, datacubesFlat, numCubeFrames, datacubeSize);
        }
        
        // heatmaps
        if (config.exportHeatmaps()) {
            std::vector<float> heatmapsFlat;
            for (size_t i = 0; i < numFrames; ++i) {
                heatmapConfig = dynamic_pointer_cast<CascadeConfig>(cascadeConfig_->clone());
                auto heatmap = run->getCascadeHeatmap(i);
                auto heatmapResult = heatmapConfig->clipHeatmap(heatmap, azimuthMaxBin, elevationMaxBin, rangeMaxBin);
                heatmap = heatmapResult.heatmap;
                heatmapConfig = std::dynamic_pointer_cast<CascadeConfig>(heatmapResult.newConfig);
                if (config.collapseElevation()) {
                    heatmapResult = heatmapConfig->collapseHeatmapElevation(heatmap, config.collapseElevationMinZ(), config.collapseElevationMaxZ());
                    heatmap = heatmapResult.heatmap;
                    heatmapConfig = std::dynamic_pointer_cast<CascadeConfig>(heatmapResult.newConfig);
                }
                if (config.removeDopplerDim()) {
                    heatmapResult = heatmapConfig->removeDoppler(heatmap);
                    heatmap = heatmapResult.heatmap;
                    heatmapConfig = std::dynamic_pointer_cast<CascadeConfig>(heatmapResult.newConfig);
                }
                heatmapsFlat.insert(heatmapsFlat.end(), heatmap->begin(), heatmap->end());
            }
            coloradar::internal::saveHeatmapsToHDF5(
                cascadeHeatmapsContentName + "_" + run->name(), 
                datasetFile, heatmapsFlat, numFrames, 
                heatmapConfig->numAzimuthBins(), 
                heatmapConfig->nRangeBins(), 
                heatmapConfig->numElevationBins(),
                heatmapConfig->hasDoppler()
            );
            // std::vector<float>().swap(heatmapsFlat); 
        }
        if (heatmapConfig) {
            if (!finalConfig) throw std::runtime_error("exportCascade(): finalConfig is nullptr");
            (*finalConfig)["heatmap_radar_config"] = heatmapConfig->toJson();
        }

        // clouds
        if (config.exportClouds()) {
            hsize_t numDims = cascadeConfig_->numElevationBins() > 0 ? 5 : 4;  // x, y, (z), intensity, doppler
            if (!cascadeConfig_->hasDoppler()) numDims -= 1;

            bool buildClouds = false;
            try {
                auto cloud = run->getCascadePointcloud(0);
            } catch (const std::filesystem::filesystem_error& e) {
                buildClouds = true;
            }
            std::vector<float> cloudsFlat;
            std::vector<hsize_t> cloudSizes(numFrames);
            for (size_t i = 0; i < numFrames; ++i) {
                std::shared_ptr<pcl::PointCloud<RadarPoint>> cloud;
                if (buildClouds) {
                    cloud = cascadeConfig_->heatmapToPointcloud(run->getCascadeHeatmap(i), config.intensityThreshold());
                }
                else {
                    cloud = run->getCascadePointcloud(i, config.intensityThreshold());
                }
                filterFov(cloud, horizontalFov, verticalFov, range);
                if (config.cloudsInGlobalFrame()) {
                    pcl::transformPointCloud(*cloud, *cloud, sensorPoses[i]);
                }
                if (config.collapseElevation()) {
                    collapseElevation(cloud, config.collapseElevationMinZ(), config.collapseElevationMaxZ());
                }
                cloudSizes[i] = cloud->size();
                std::vector<float> cloudFlat = coloradar::internal::flattenRadarCloud(cloud, cascadeConfig_->numElevationBins(), cascadeConfig_->hasDoppler());
                cloudsFlat.insert(cloudsFlat.end(), cloudFlat.begin(), cloudFlat.end());
            }
            coloradar::internal::saveCloudsToHDF5(cascadeCloudsContentName + "_" + run->name(), datasetFile, cloudsFlat, numFrames, cloudSizes, numDims);
        }
    }
    datasetFile.flush(H5F_SCOPE_GLOBAL);
    return content;
}

std::vector<std::string> ColoradarPlusDataset::exportLidar(const LidarExportConfig &config, std::vector<std::shared_ptr<ColoradarPlusRun>> runs, H5::H5File& datasetFile) {
    std::vector<std::string> content;
    if (!config.exportTimestamps() && !config.exportPoses() && !config.exportMap() && !config.exportMapSamples() && !config.exportClouds()) return content;
    std::cout << "Exporting lidar data..." << std::endl;

    // Content types
    if (config.exportTimestamps()) content.push_back(lidarTimestampsContentName);
    if (config.exportPoses()) content.push_back(lidarPosesContentName);
    if (config.exportClouds()) content.push_back(lidarCloudsContentName);
    if (config.exportMap()) content.push_back(lidarMapContentName);
    if (config.exportMapSamples()) content.push_back(lidarMapSamplesContentName);

    for (auto run : runs) {
        std::vector<double> truePoseTimestamps = run->poseTimestamps();
        std::vector<double> timestamps = run->lidarTimestamps();
        hsize_t numFrames = timestamps.size();
        auto truePoses = run->getPoses<Eigen::Affine3f>();
        std::vector<Eigen::Affine3f> basePoses = interpolatePoses(truePoses, truePoseTimestamps, timestamps);
        std::vector<Eigen::Affine3f> sensorPoses(numFrames);
        for (int i = 0; i < numFrames; ++i) {
            sensorPoses[i] = basePoses[i] * lidarTransform_;
        }

        // timestamps
        if (config.exportTimestamps()) {
            coloradar::internal::saveVectorToHDF5(lidarTimestampsContentName + "_" + run->name(), datasetFile, timestamps);
        }

        // poses
        if (config.exportPoses()) {
            coloradar::internal::savePosesToHDF5(lidarPosesContentName + "_" + run->name(), datasetFile, sensorPoses);
        }

        // clouds
        if (config.exportClouds()) {
            hsize_t numDims = config.collapseElevation() ? 3 : 4;  // x, y, (z), intensity
            if (config.removeIntensityDim()) numDims -= 1;
            float range = config.cloudFov().rangeMeters > 0 ? config.cloudFov().rangeMeters : std::numeric_limits<float>::infinity();

            std::vector<float> cloudsFlat;
            std::vector<hsize_t> cloudSizes(numFrames);
            for (size_t i = 0; i < numFrames; ++i) {
                auto cloud = run->getLidarPointCloud(i);
                filterFov(cloud, config.cloudFov().horizontalDegreesTotal, config.cloudFov().verticalDegreesTotal, range);
                if (config.collapseElevation()) {
                    collapseElevation(cloud, config.collapseElevationMinZ(), config.collapseElevationMaxZ());
                }
                cloudSizes[i] = cloud->size();
                auto cloudFlat = coloradar::internal::flattenLidarCloud(cloud, config.collapseElevation(), config.removeIntensityDim());
                cloudsFlat.insert(cloudsFlat.end(), cloudFlat.begin(), cloudFlat.end());
            }
            coloradar::internal::saveCloudsToHDF5(lidarCloudsContentName + "_" + run->name(), datasetFile, cloudsFlat, numFrames, cloudSizes, numDims);
        }
        
        // map and samples
        if (config.exportMap() || config.exportMapSamples()) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr map = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
            bool buildMap = false;
            if (config.forceMapRebuild()) {
                buildMap = true;
            } else if (config.allowMapRebuild()) {
                try {
                    map = run->getLidarOctomap();
                } catch (...) {
                    std::cout << run->name() << ": lidar octomap not found." << std::endl;
                    buildMap = true;
                }
            }
            if (buildMap) {
                std::cout << run->name() << ": building octomap... " << std::endl;
                octomap::OcTree octree = run->buildLidarOctomap(
                    config.mapResolution(), 
                    config.mapInputCloudFov().horizontalDegreesTotal,
                    config.mapInputCloudFov().verticalDegreesTotal,
                    config.mapInputCloudFov().rangeMeters, 
                    lidarTransform_
                );
                std::cout << run->name() << ": finished building octomap." << std::endl;
                octreeToPcl(octree, map);
                if (config.saveMap()) {
                    run->saveLidarOctomap(octree);
                }
            } else {
                map = run->getLidarOctomap();
            }
            
            hsize_t numDims = config.collapseElevation() ? 3 : 4;  // x, y, (z), occupancy
            if (config.removeOccupancyDim()) numDims -= 1;
            if (config.collapseElevation()) {
                collapseElevation(map, config.collapseElevationMinZ(), config.collapseElevationMaxZ());
            }
            filterOccupancy(map, config.occupancyThresholdPercent() / 100.0, config.logOddsToProbability());

            if (config.exportMap()) {
                std::vector<float> mapFlat = coloradar::internal::flattenLidarCloud(map, config.collapseElevation(), config.removeOccupancyDim());
                coloradar::internal::saveCloudToHDF5(lidarMapContentName + "_" + run->name(), datasetFile, mapFlat, numDims);
            }

            if (config.exportMapSamples()) {
                std::vector<double> centerTimestamps;
                Eigen::Affine3f centerTransform;
                if (config.centerSensor()->name() == (new CascadeDevice())->name()) {
                    centerTimestamps = run->cascadeTimestamps();
                    centerTransform = cascadeTransform_;
                } else if (config.centerSensor()->name() == (new LidarDevice())->name()) {
                    centerTimestamps = run->lidarTimestamps();
                    centerTransform = lidarTransform_;
                } else if (config.centerSensor()->name() == (new ImuDevice())->name()) {
                    centerTimestamps = run->imuTimestamps();
                    centerTransform = imuTransform_;
                } else {
                    centerTimestamps = run->poseTimestamps();
                    centerTransform = Eigen::Affine3f::Identity();
                }
                hsize_t numSamples = centerTimestamps.size();
                std::vector<Eigen::Affine3f> basePosesCenterTs = interpolatePoses(truePoses, truePoseTimestamps, centerTimestamps);
                std::vector<Eigen::Affine3f> centerPoses(numSamples);
                for (int i = 0; i < numSamples; ++i) {
                    centerPoses[i] = basePosesCenterTs[i] * centerTransform;
                }

                bool resample = false;
                if (config.forceResample()) {
                    resample = true;
                } else if (config.allowResample()) {
                    try {
                        auto sample = run->getMapSample(0);
                    } catch (...) {
                        std::cout << run->name() << ": map samples not found, resampling... " << std::endl;
                        resample = true;
                    }
                }

                std::vector<float> samplesFlat;
                std::vector<hsize_t> sampleSizes(numSamples);
                for (size_t i = 0; i < numSamples; ++i) {
                    pcl::PointCloud<pcl::PointXYZI>::Ptr sample;
                    if (resample) {
                        sample = run->sampleMapFrame(
                            config.mapSampleFov().horizontalDegreesTotal,
                            config.mapSampleFov().verticalDegreesTotal,
                            config.mapSampleFov().rangeMeters,
                            centerPoses[i],
                            map
                        );
                        if (config.saveSamples()) {
                            run->saveMapSample(i, sample);
                        }
                    } else {
                        sample = run->getMapSample(i);
                    }
                    sampleSizes[i] = sample->size();
                    auto sampleFlat = coloradar::internal::flattenLidarCloud(sample, config.collapseElevation(), config.removeOccupancyDim());
                    samplesFlat.insert(samplesFlat.end(), sampleFlat.begin(), sampleFlat.end());
                }
                coloradar::internal::saveCloudsToHDF5(lidarMapSamplesContentName + "_" + run->name(), datasetFile, samplesFlat, numSamples, sampleSizes, numDims);
            }
        }
    }
    datasetFile.flush(H5F_SCOPE_GLOBAL);
    return content;
}



}
