#include "dataset/dataset.h"


namespace coloradar {


// PUBLIC METHODS

std::filesystem::path ColoradarPlusDataset::exportToFile(DatasetExportConfig& exportConfig) {
    Json::Value finalConfig;
    finalConfig["runs"] = Json::arrayValue;
    finalConfig["radar_config"] = cascadeConfig_->toJson();
    finalConfig["data_content"] = Json::arrayValue;

    std::vector<std::shared_ptr<Run>> runObjects;
    if (exportConfig.runs().empty()) {
        runObjects = getRuns();
        exportConfig.initDataset(listRuns());
        for (auto runName : exportConfig.runs()) {
            finalConfig["runs"].append(runName);
        }
    } else {
        for (auto runName : exportConfig.runs()) {
            runObjects.push_back(getRun(runName));
            finalConfig["runs"].append(runName);
        }
    }
    std::cout << "Exporting " << finalConfig["runs"].size() << " runs." << std::endl;
    H5::H5File datasetFile(exportConfig.destinationFilePath(), H5F_ACC_TRUNC);

    auto baseContent = exportBaseDevice(exportConfig.base(), runObjects, datasetFile);
    auto imuContent = exportImu(exportConfig.imu(), runObjects, datasetFile);
    auto cascadeContent = exportCascade(exportConfig.cascade(), runObjects, datasetFile);
    auto lidarContent = exportLidar(exportConfig.lidar(), runObjects, datasetFile);
    for (const auto &contentList : {baseContent, imuContent, cascadeContent, lidarContent}) {
        if (contentList.empty()) continue;
        for (const auto &content : contentList) {
            finalConfig["data_content"].append(content);
        }
    }

    if (exportConfig.exportTransforms()) {
        coloradar::internal::savePoseToHDF5(transformBaseToCascadeContentName, datasetFile, cascadeTransform_);
        coloradar::internal::savePoseToHDF5(transformBaseToLidarContentName, datasetFile, lidarTransform_);
        coloradar::internal::savePoseToHDF5(transformBaseToImuContentName, datasetFile, imuTransform_);
        finalConfig["data_content"].append(transformBaseToCascadeContentName);
        finalConfig["data_content"].append(transformBaseToLidarContentName);
        finalConfig["data_content"].append(transformBaseToImuContentName);
    }

    std::string configString = Json::writeString(Json::StreamWriterBuilder(), finalConfig);
    H5::StrType strType(H5::PredType::C_S1, H5T_VARIABLE);
    H5::DataSpace dataspace(H5S_SCALAR);
    H5::DataSet configDataset = datasetFile.createDataSet("config", strType, dataspace);
    configDataset.write(configString, strType);
    datasetFile.close();
    return exportConfig.destinationFilePath();
}

std::filesystem::path ColoradarPlusDataset::exportToFile(const std::string &yamlConfigPath) {
    DatasetExportConfig exportConfig = DatasetExportConfig(yamlConfigPath);
    return exportToFile(exportConfig);
}


// PROTECTED METHODS

std::vector<std::string> ColoradarPlusDataset::exportBaseDevice(const BaseExportConfig &config, std::vector<std::shared_ptr<Run>> runs, const H5::H5File &datasetFile) {
    std::vector<std::string> content;
    if (!config.exportTimestamps() && !config.exportPoses()) return content;
    std::cout << "Exporting base data..." << std::endl;

    // Content types
    if (config.exportTimestamps()) content.push_back(poseTimestampsContentName);
    if (config.exportPoses()) content.push_back(posesContentName);

    // std::cout << "Finished runs: ";
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
        // std::cout << run->name() << " ";
    }
    // std::cout << std::endl;
    return content;
}

std::vector<std::string> ColoradarPlusDataset::exportImu(const ImuExportConfig &config, std::vector<std::shared_ptr<Run>> runs, const H5::H5File &datasetFile) {
    std::vector<std::string> content;
    if (!config.exportTimestamps() && !config.exportPoses()) return content;
    std::cout << "Exporting IMU data..." << std::endl;

    // Content types
    if (config.exportTimestamps()) content.push_back(imuTimestampsContentName);
    if (config.exportPoses()) content.push_back(imuPosesContentName);

    // std::cout << "Finished runs: ";
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
        // std::cout << run->name() << " ";
    }
    // std::cout << std::endl;
    return content;
}

std::vector<std::string> ColoradarPlusDataset::exportCascade(const RadarExportConfig &config, std::vector<std::shared_ptr<Run>> runs, const H5::H5File &datasetFile) {
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
    float range = cascadeConfig_->clipRange(config.fov().rangeMeters);
    int rangeMaxBin = cascadeConfig_->rangeToRangeIdx(range);
    int azimuthMaxBin = config.fov().azimuthIdx;
    int elevationMaxBin = config.fov().elevationIdx;
    float horizontalFov = config.fov().horizontalDegreesTotal;
    float verticalFov = config.fov().verticalDegreesTotal;
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

    // std::cout << "Finished runs: ";
    for (auto run : runs) {
        std::vector<double> timestamps = run->cascadeTimestamps();
        hsize_t numFrames = timestamps.size();
        std::vector<Eigen::Affine3f> basePoses = interpolatePoses(run->getPoses<Eigen::Affine3f>(), run->poseTimestamps(), timestamps);
        std::vector<Eigen::Affine3f> sensorPoses(numFrames);
        for (int i = 0; i < numFrames; ++i) {
            sensorPoses[i] = basePoses[i] * cascadeTransform_;
        }

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
            hsize_t datacubeSize = static_cast<hsize_t>(cascadeConfig_->numElevationBins * cascadeConfig_->numAzimuthBins * cascadeConfig_->nRangeBins() * 2);
            std::vector<int16_t> datacubesFlat;
            for (size_t i = 0; i < numFrames; ++i) {
                auto datacube = run->getCascadeDatacube(i);
                datacubesFlat.insert(datacubesFlat.end(), datacube->begin(), datacube->end());
            }
            coloradar::internal::saveDatacubesToHDF5(cascadeDatacubesContentName + "_" + run->name(), datasetFile, datacubesFlat, numFrames, datacubeSize);
        }
        
        // heatmaps
        if (config.exportHeatmaps()) {
            auto heatmapConfig = std::make_shared<CascadeConfig>(*static_cast<CascadeConfig*>(cascadeConfig_.get()));
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
                heatmapsFlat.insert(heatmapsFlat.end(), heatmap->begin(), heatmap->end());
            }
            coloradar::internal::saveHeatmapsToHDF5(cascadeHeatmapsContentName + "_" + run->name(), datasetFile, heatmapsFlat, numFrames, heatmapConfig->numAzimuthBins, heatmapConfig->nRangeBins(), heatmapConfig->numElevationBins, heatmapConfig->hasDoppler);
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
                std::vector<float> cloudFlat = coloradar::internal::flattenRadarCloud(cloud, cascadeConfig_->numElevationBins, cascadeConfig_->hasDoppler);
                cloudsFlat.insert(cloudsFlat.end(), cloudFlat.begin(), cloudFlat.end());
            }
            coloradar::internal::saveCloudsToHDF5(cascadeCloudsContentName + "_" + run->name(), datasetFile, cloudsFlat, numFrames, cloudSizes, numDims);
        }
        // std::cout << run->name() << " ";
    }
    // std::cout << std::endl;
    return content;
}


std::vector<std::string> ColoradarPlusDataset::exportLidar(const LidarExportConfig &config, std::vector<std::shared_ptr<Run>> runs, const H5::H5File &datasetFile) {
    std::vector<std::string> content;
    if (!config.exportTimestamps() && !config.exportPoses() && !config.exportMap() && !config.exportMapSamples() && !config.exportClouds()) return content;
    std::cout << "Exporting lidar data..." << std::endl;

    // Content types
    if (config.exportTimestamps()) content.push_back(lidarTimestampsContentName);
    if (config.exportPoses()) content.push_back(lidarPosesContentName);
    if (config.exportClouds()) content.push_back(lidarCloudsContentName);
    if (config.exportMap()) content.push_back(lidarMapContentName);
    if (config.exportMapSamples()) content.push_back(lidarMapSamplesContentName);

    // std::cout << "Finished runs: ";
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
                auto cloudFlat = coloradar::internal::flattenLidarCloud(cloud, config.collapseElevation(), config.removeIntensityDim());\
                cloudsFlat.insert(cloudsFlat.end(), cloudFlat.begin(), cloudFlat.end());
            }
            coloradar::internal::saveCloudsToHDF5(lidarCloudsContentName + "_" + run->name(), datasetFile, cloudsFlat, numFrames, cloudSizes, numDims);
        }
        
        // map and samples
        if (config.exportMap() || config.exportMapSamples()) {
            auto map = run->getLidarOctomap();
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
                    } catch (const std::filesystem::filesystem_error& e) {
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
        // std::cout << run->name() << " ";
    }
    // std::cout << std::endl;
    return content;
}


}