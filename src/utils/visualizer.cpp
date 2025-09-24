#define PCL_NO_PRECOMPILE
#include "utils/visualizer.h"


template class pcl::visualization::PointCloudGeometryHandlerXYZ<coloradar::RadarPoint>;


namespace coloradar {


DatasetVisualizer::DatasetVisualizer(
    const std::shared_ptr<CascadeConfig> cascadeRadarConfig,
    const Eigen::Affine3f baseToLidarTransform,
    const Eigen::Affine3f baseToCascadeTransform,
    const int frameIncrement, 
    const double cascadeRadarIntensityThreshold,
    const std::string cameraConfigPath
):  
    cascadeRadarConfig(std::make_shared<CascadeConfig>(*cascadeRadarConfig)),
    clippedCascadeRadarConfig(std::make_shared<CascadeConfig>(*cascadeRadarConfig)),
    baseToLidarTransform(baseToLidarTransform),
    baseToCascadeTransform(baseToCascadeTransform),
    frameIncrement(frameIncrement),
    cascadeRadarIntensityThreshold(cascadeRadarIntensityThreshold),
    cameraConfigPath(cameraConfigPath), 
    viewer(new pcl::visualization::PCLVisualizer("3D Viewer")),
    vtkImage(vtkSmartPointer<vtkImageData>::New()),
    imageActor(vtkSmartPointer<vtkImageActor>::New())
{
    reset();
    
    // init radar config
    auto sampleHeatmap = std::make_shared<std::vector<float>>(cascadeRadarConfig->heatmapSize(), 0.0f);
    clippedCascadeRadarConfig = std::make_shared<CascadeConfig>(*cascadeRadarConfig); // copy config
    clippedCascadeRadarConfig->clipHeatmap(sampleHeatmap, cascadeRadarConfig->numAzimuthBins, 0, static_cast<int>(cascadeRadarConfig->nRangeBins() * 0.75), true); // clip sample heatmap to modify the config
    clippedCascadeRadarConfig->precomputePointcloudTemplate(); // update cloud template
}


void DatasetVisualizer::reset() {
    run = nullptr;
    mapIsPrebuilt = false;
    lidarMapCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    basePosesLidarTs.clear();
    basePosesCascadeTs.clear();
    numSteps = 0;
    currentStep = -1;
    currentMapStep = -1;
    currentMessage.clear();
    messageTime = std::chrono::steady_clock::now();
    viewer->removeAllPointClouds();
    viewer->removeAllCoordinateSystems();
    viewer->removeAllShapes();
    viewer->resetCamera();
    viewer->initCameraParameters();

    vtkImage->Initialize();
    imageActor->GetMapper()->SetInputData(vtkImage);
}


void DatasetVisualizer::visualize(const std::shared_ptr<Run> run, const bool usePrebuiltMap) {
    reset();
    this->run = run;

    // init poses and use cascade timestamps as reference
    auto basePoses = run->getPoses<Eigen::Affine3f>();
    if (basePoses.empty()) throw std::runtime_error("Base poses are empty.");
    basePosesLidarTs = interpolatePoses(basePoses, run->poseTimestamps(), run->lidarTimestamps());
    basePosesCascadeTs = interpolatePoses(basePoses, run->poseTimestamps(), run->cascadeTimestamps());
    cascadeToLidarFrameIndices.resize(basePosesCascadeTs.size());
    for (size_t cascadeIdx = 0; cascadeIdx < basePosesCascadeTs.size(); cascadeIdx++) {
        auto cascadeTimestamp = run->cascadeTimestamps()[cascadeIdx];
        auto lidarIdx = findClosestTimestampIndex(cascadeTimestamp, run->lidarTimestamps());
        cascadeToLidarFrameIndices[cascadeIdx] = lidarIdx;
    }
    // baseToLidarFrameIndices.resize(basePoses.size());
    // baseToCascadeFrameIndices.resize(basePoses.size());
    // for (size_t baseIdx = 0; baseIdx < basePoses.size(); baseIdx++) {
    //     auto baseTimestamp = run->poseTimestamps()[baseIdx];
    //     auto lidarFrameIdx = findClosestTimestampIndex(baseTimestamp, run->lidarTimestamps(), "before");
    //     auto cascadeFrameIdx = findClosestTimestampIndex(baseTimestamp, run->cascadeTimestamps(), "before");
    //     baseToLidarFrameIndices[baseIdx] = lidarFrameIdx;
    //     baseToCascadeFrameIndices[baseIdx] = cascadeFrameIdx;
    // }
    
    // init visualization parameters
    // numSteps = baseToLidarFrameIndices.size();
    numSteps = basePosesCascadeTs.size();
    mapIsPrebuilt = usePrebuiltMap;
    // std::cout << "visualize(): numSteps = " << numSteps << std::endl;
    // std::cout << "visualize(): usePrebuiltMap = " << usePrebuiltMap << std::endl;
    if (usePrebuiltMap) {
        lidarMapCloud = run->getLidarOctomap();
        if (lidarMapCloud->empty()) throw std::runtime_error("Prebuilt map cloud is empty.");
        filterOccupancy(lidarMapCloud, 0, true);
        renderLidarMap();
    }
    
    // display frame 0
    step(1);
    
    // start viewer
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(2.0);
    if (std::filesystem::exists(cameraConfigPath)) {
        viewer->loadCameraParameters(cameraConfigPath);
        showMessage("Camera parameters loaded from " + std::filesystem::absolute(cameraConfigPath).string());
    }
    viewer->registerKeyboardCallback(std::bind(&DatasetVisualizer::keyboardCallback, this, std::placeholders::_1));
    renderLegend();
    while (!viewer->wasStopped()) {
        viewer->spin();
    }
}


void DatasetVisualizer::keyboardCallback(const pcl::visualization::KeyboardEvent &event) {
    if (!event.keyDown()) return;
    std::string keySym = event.getKeySym();
    for (const auto& control : getControls()) {
        if (keySym == control.button) {
            control.action();
            break;
        }
    }
}


std::vector<Control> DatasetVisualizer::getControls() {
    return {
        {"Up", "<Up Arrow>", "next frame", [this]() { 
            step(frameIncrement);
        }},
        {"Down", "<Down Arrow>", "previous frame", [this]() {
            step(-frameIncrement);
        }},
        {"s", "s", "save camera parameters", [this]() {
            viewer->saveCameraParameters(cameraConfigPath);
            showMessage("Camera parameters saved to " + std::filesystem::absolute(cameraConfigPath).string());
        }},
        {"l", "l", "load camera parameters", [this]() {
            viewer->loadCameraParameters(cameraConfigPath);
            showMessage("Camera parameters loaded from " + std::filesystem::absolute(cameraConfigPath).string());
        }}
    };
}


void DatasetVisualizer::renderLegend() {
    std::stringstream legend;
    legend << "Step: " << (currentStep + 1) << " / " << numSteps << "\n";
    legend << "Keyboard Controls:\n";
    for (const auto& control : getControls()) {
        legend << "- [" << control.name << "]: " << control.description << "\n";
    }
    viewer->removeText3D("keyboard_legend");
    viewer->addText(legend.str(), 30, 30, 35, 1.0, 1.0, 1.0, "keyboard_legend");
    if (!currentMessage.empty()) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - messageTime).count();
        if (elapsed < 3) {
            viewer->removeText3D("message_text");
            viewer->addText(currentMessage, 30, 10, 25, 1.0, 1.0, 1.0, "message_text");
        } else {
            currentMessage.clear();
            viewer->removeText3D("message_text");
        }
    }
}


void DatasetVisualizer::showMessage(const std::string& message) {
    currentMessage = message;
    messageTime = std::chrono::steady_clock::now();
    renderLegend();
}


void DatasetVisualizer::step(const int increment) {
    if (increment == 0) throw std::runtime_error("Step increment must be non-zero.");
    int prevStep = currentStep;
    int targetStep = std::clamp(currentStep + increment, 0, numSteps - 1);
    if (targetStep == currentStep) return;
    currentStep = targetStep;

    if (!mapIsPrebuilt) {
        // update lidar map if accumulating
        updateLidarMap(currentStep);
        renderLidarMap();
    }
    // int cascadeFrameIdx = baseToCascadeFrameIndices[currentStep];
    auto cascadeCloud = readCascadeCloud(currentStep);
    renderRadarCloud(cascadeCloud);
    renderLegend();
}


void DatasetVisualizer::updateLidarMap(const int step) {
    if (step >= numSteps) throw std::runtime_error("Step " + std::to_string(step) + " is out of range [0, " + std::to_string(numSteps) + ").");
    if (currentMapStep >= step) return;
    for (int i = currentMapStep + 1; i <= step; i++) {
        int lidarFrameIdx = cascadeToLidarFrameIndices[i];
        auto lidarCloud = readLidarCloud(lidarFrameIdx);
        lidarCloud = downsampleLidarCloud(lidarCloud);
        *lidarMapCloud += *lidarCloud;
    }
    currentMapStep = step;
}


pcl::PointCloud<RadarPoint>::Ptr DatasetVisualizer::readCascadeCloud(const int scanIdx) {
    auto cascadeHeatmap = run->getCascadeHeatmap(scanIdx);
    std::cout << "readCascadeCloud(): scanIdx = " << scanIdx << ", cascadeHeatmap->size() = " << cascadeHeatmap->size() << ", hasDoppler: " << cascadeRadarConfig->hasDoppler << std::endl;
    if (cascadeHeatmap->size() >= 5) {
        for (int i = 0; i < 5; ++i) {
            std::cout << "Original heatmap element " << i << ": " << cascadeHeatmap->at(i) << std::endl;
        }
    }
    int targetNumRangeBins = static_cast<int>(cascadeRadarConfig->nRangeBins() * 0.75);
    if (cascadeRadarConfig->numElevationBins > 0 || cascadeRadarConfig->nRangeBins() > targetNumRangeBins) {
        cascadeHeatmap = cascadeRadarConfig->clipHeatmap(cascadeHeatmap, cascadeRadarConfig->numAzimuthBins, 0, targetNumRangeBins, false); // clip without config update
    }
    if (cascadeHeatmap->size() >= 5) {
        for (int i = 0; i < 5; ++i) {
            std::cout << "Clipped heatmap element " << i << ": " << cascadeHeatmap->at(i) << std::endl;
        }
    }
    auto cascadeCloud = clippedCascadeRadarConfig->heatmapToPointcloud(cascadeHeatmap, cascadeRadarIntensityThreshold); // use clipped config to convert heatmap to pointcloud
    // cascadeCloud = extractTopNIntensity(cascadeCloud, 500);
    cascadeCloud = normalizeRadarCloudIntensity(cascadeCloud);
    
    auto map_T_base = basePosesCascadeTs[scanIdx];  // read map_T_base as "base to map"
    auto base_T_cascade = baseToCascadeTransform; // read base_T_cascade as "cascade to base"
    auto map_T_cascade = map_T_base * base_T_cascade; // read map_T_cascade as "cascade to map"
    pcl::transformPointCloud(*cascadeCloud, *cascadeCloud, map_T_cascade);                                  // map frame
    std::cout << "readCascadeCloud(): cascadeCloud->size() = " << cascadeCloud->size() << std::endl;
    if (cascadeCloud->size() >= 5) {
        for (int i = 0; i < 5; ++i) {
            const auto& point = cascadeCloud->points[i];
            std::cout << "Radar cloud point " << i << ": " << point.x << ", " << point.y << ", " << point.z << std::endl;
        }
    }
    return cascadeCloud;
}


pcl::PointCloud<pcl::PointXYZI>::Ptr DatasetVisualizer::readLidarCloud(const int scanIdx) {
    auto lidarCloud = run->getLidarPointCloud(scanIdx);
    auto map_T_base = basePosesLidarTs[scanIdx];
    auto base_T_lidar = baseToLidarTransform;
    auto map_T_lidar = map_T_base * base_T_lidar;
    pcl::transformPointCloud(*lidarCloud, *lidarCloud, map_T_lidar);     // map frame
    return lidarCloud;
}


pcl::PointCloud<pcl::PointXYZI>::Ptr DatasetVisualizer::downsampleLidarCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud, const float leafSize) const {
    auto downsampledCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::VoxelGrid<pcl::PointXYZI> voxelGrid;
    voxelGrid.setInputCloud(inputCloud);
    voxelGrid.setLeafSize(leafSize, leafSize, leafSize);
    voxelGrid.filter(*downsampledCloud);
    return downsampledCloud;
}


pcl::PointCloud<RadarPoint>::Ptr DatasetVisualizer::normalizeRadarCloudIntensity(const pcl::PointCloud<RadarPoint>::Ptr& inputCloud) const {
    auto normalizedCloud = std::make_shared<pcl::PointCloud<RadarPoint>>();
    normalizedCloud->reserve(inputCloud->size());

    float minIntensity = std::numeric_limits<float>::max();
    float maxIntensity = std::numeric_limits<float>::lowest();
    for (const auto& point : *inputCloud) {
        if (point.intensity < minIntensity) minIntensity = point.intensity;
        if (point.intensity > maxIntensity) maxIntensity = point.intensity;
    }
    float range = maxIntensity - minIntensity;
    if (range == 0) range = 1.0f;

    for (const auto& point : *inputCloud) {
        RadarPoint normalizedPoint = point;
        normalizedPoint.intensity = (point.intensity - minIntensity) / range;
        normalizedCloud->push_back(normalizedPoint);
    }
    return normalizedCloud;
}


pcl::PointCloud<RadarPoint>::Ptr DatasetVisualizer::extractTopNIntensity(const pcl::PointCloud<RadarPoint>::Ptr& inputCloud, size_t N) const {
    auto sorted = *inputCloud;
    std::nth_element(sorted.begin(), sorted.begin() + N, sorted.end(), 
        [](const RadarPoint& a, const RadarPoint& b) { return a.intensity > b.intensity; });
    sorted.resize(std::min(N, sorted.size()));

    auto output = std::make_shared<pcl::PointCloud<RadarPoint>>();
    output->insert(output->end(), sorted.begin(), sorted.end());
    return output;
}


void DatasetVisualizer::renderLidarMap() {
    viewer->removePointCloud(lidarMapCloudName);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> mapColorHandler(
        lidarMapCloud, 
        mapIsPrebuilt ? 255 : 180, 
        mapIsPrebuilt ? 255 : 180, 
        mapIsPrebuilt ? 255 : 180
    );
    viewer->addPointCloud<pcl::PointXYZI>(lidarMapCloud, mapColorHandler, lidarMapCloudName);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.1, lidarMapCloudName);
}


void DatasetVisualizer::renderRadarCloud(const pcl::PointCloud<RadarPoint>::Ptr& cloud) {
    viewer->removePointCloud(cascadeRadarCloudName);
    pcl::visualization::PointCloudColorHandlerGenericField<RadarPoint> radarColorHandler(cloud, "intensity");
    viewer->addPointCloud<RadarPoint>(cloud, radarColorHandler, cascadeRadarCloudName);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, cascadeRadarCloudName);
}


}

