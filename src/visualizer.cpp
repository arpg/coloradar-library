#define PCL_NO_PRECOMPILE
#include "visualizer.h"


template class pcl::visualization::PointCloudGeometryHandlerXYZ<coloradar::RadarPoint>;


namespace coloradar {


DatasetVisualizer::DatasetVisualizer(
    const RadarConfig* cascadeRadarConfig,
    const Eigen::Affine3f baseToLidarTransform,
    const Eigen::Affine3f baseToCascadeTransform,
    const int frameIncrement, 
    const double cascadeRadarIntensityThreshold,
    const std::string cameraConfigPath
) :  
    cascadeRadarConfig(cascadeRadarConfig),
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
}


void DatasetVisualizer::reset() {
    run = nullptr;
    mapIsPrebuilt = false;
    lidarMapCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    lidarPoses.clear();
    cascadePoses.clear();
    numSteps = 0;
    currentStep = -1;

    viewer->removeAllPointClouds();
    viewer->removeAllCoordinateSystems();
    viewer->removeAllShapes();
    viewer->resetCamera();

    vtkImage->Initialize();
    imageActor->GetMapper()->SetInputData(vtkImage);
}


void DatasetVisualizer::visualize(const ColoradarPlusRun* run, const bool usePrebuiltMap) {
    reset();
    this->run = run;
    initPoses();

    numSteps = baseToLidarFrameIndices.size();
    mapIsPrebuilt = usePrebuiltMap;
    if (usePrebuiltMap) {
        lidarMapCloud = run->readLidarOctomap();
        filterOccupancy(lidarMapCloud, 0, true);
    }
    step(1);

    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(2.0);
    viewer->initCameraParameters();
    viewer->registerKeyboardCallback(std::bind(&DatasetVisualizer::keyboardCallback, this, std::placeholders::_1));
    while (!viewer->wasStopped()) {
        viewer->spin();
    }
}


void DatasetVisualizer::initPoses() {
    auto basePoses = run->getPoses<Eigen::Affine3f>();
    lidarPoses = interpolatePoses(basePoses, run->poseTimestamps(), run->lidarTimestamps());
    cascadePoses = interpolatePoses(basePoses, run->poseTimestamps(), run->cascadeTimestamps());
    baseToLidarFrameIndices.resize(basePoses.size());
    baseToCascadeFrameIndices.resize(basePoses.size());

    for (size_t baseIdx = 0; baseIdx < basePoses.size(); baseIdx++) {
        auto baseTimestamp = run->poseTimestamps()[baseIdx];
        auto lidarFrameIdx = findClosestTimestampIndex(baseTimestamp, run->lidarTimestamps(), "before");
        auto cascadeFrameIdx = findClosestTimestampIndex(baseTimestamp, run->cascadeTimestamps(), "before");
        baseToLidarFrameIndices[baseIdx] = lidarFrameIdx;
        baseToCascadeFrameIndices[baseIdx] = cascadeFrameIdx;
    }
}


void DatasetVisualizer::keyboardCallback(const pcl::visualization::KeyboardEvent &event) {
    std::cout.precision(20);
    if (!event.keyDown()) return;
    std::cout << "Key pressed: " << event.getKeySym() << std::endl;

    if (event.getKeySym() == "Up") {
        this->step(frameIncrement);
    } else if (event.getKeySym() == "Down") {
        this->step(-frameIncrement);
    } else if (event.getKeySym() == "s") {
        viewer->saveCameraParameters(cameraConfigPath);
        std::cout << "Camera parameters saved." << std::endl;
    } else if (event.getKeySym() == "l") {
        viewer->loadCameraParameters(cameraConfigPath);
        std::cout << "Camera parameters loaded." << std::endl;
    }
}


void DatasetVisualizer::step(const int increment) {
    if (increment == 0) throw std::runtime_error("Step increment must be non-zero.");
    int targetStep = std::clamp(currentStep + increment, 0, numSteps - 1);
    std::cout << "targetStep: " << targetStep << std::endl;
    if (targetStep == currentStep) return;
    currentStep = targetStep;
    
    if (!mapIsPrebuilt) {
        int lidarFrameIdx = baseToLidarFrameIndices[currentStep];
        auto lidarCloud = readLidarCloud(lidarFrameIdx);
        std::cout << "using lidar frame " << lidarFrameIdx << " for frame " << currentStep << std::endl;
        std::cout << "lidarCloud size: " << lidarCloud->size() << std::endl;
        lidarCloud = downsampleLidarCloud(lidarCloud);
        std::cout << "lidarCloud size after downsampling: " << lidarCloud->size() << std::endl;
        *lidarMapCloud += *lidarCloud;
        std::cout << "lidarMapCloud size: " << lidarMapCloud->size() << std::endl;
        renderLidarMap();
    }

    int cascadeFrameIdx = baseToCascadeFrameIndices[currentStep];
    auto cascadeCloud = readCascadeCloud(cascadeFrameIdx);
    std::cout << "using cascade frame " << cascadeFrameIdx << " for frame " << currentStep << std::endl;
    std::cout << "cascadeCloud size: " << cascadeCloud->size() << std::endl << std::endl;
    renderRadarCloud(cascadeCloud);
    
    imageActor->GetMapper()->SetInputData(vtkImage);
}


pcl::PointCloud<RadarPoint>::Ptr DatasetVisualizer::readCascadeCloud(const int scanIdx) {
    auto cascadeHeatmap = run->getCascadeHeatmap(scanIdx);
    auto heatmapConfig = new CascadeConfig(*static_cast<CascadeConfig*>(cascadeRadarConfig));            
    cascadeHeatmap = heatmapConfig->clipHeatmap(cascadeHeatmap, heatmapConfig->numAzimuthBeams, 0, heatmapConfig->nRangeBins(), false);
    auto cascadeCloud = heatmapConfig->heatmapToPointcloud(cascadeHeatmap, cascadeRadarIntensityThreshold);
    // cascadeCloud = extractTopNIntensity(cascadeCloud, 500);
    cascadeCloud = normalizeRadarCloudIntensity(cascadeCloud);

    auto cascadePose = cascadePoses[scanIdx];                                                                 // base frame, cascade timestamp
    auto cascadeToMapT = cascadePose * baseToCascadeTransform.inverse();
    pcl::transformPointCloud(*cascadeCloud, *cascadeCloud, cascadeToMapT);                                        // map frame
    return cascadeCloud;
}


pcl::PointCloud<pcl::PointXYZI>::Ptr DatasetVisualizer::readLidarCloud(const int scanIdx) {
    auto lidarCloud = run->getLidarPointCloud<pcl::PointCloud<pcl::PointXYZI>>(scanIdx);  // lidar frame
    auto lidarPose = lidarPoses[scanIdx];                                             // base frame, lidar timestamp
    auto lidarToMapT = lidarPose * baseToLidarTransform.inverse();
    pcl::transformPointCloud(*lidarCloud, *lidarCloud, lidarToMapT);                      // map frame
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


pcl::PointCloud<RadarPoint>::Ptr DatasetVisualizer::extractTopNIntensity(const pcl::PointCloud<RadarPoint>::Ptr& inputCloud, size_t N) const {
    auto sorted = *inputCloud;
    std::nth_element(sorted.begin(), sorted.begin() + N, sorted.end(), 
        [](const RadarPoint& a, const RadarPoint& b) { return a.intensity > b.intensity; });
    sorted.resize(std::min(N, sorted.size()));

    auto output = std::make_shared<pcl::PointCloud<RadarPoint>>();
    output->insert(output->end(), sorted.begin(), sorted.end());
    return output;
}


}
