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
    const std::string& cameraConfigPath
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
    numSteps = lidarPoses.size();
    mapIsPrebuilt = usePrebuiltMap;
    if (usePrebuiltMap) {
        lidarMapCloud = run->readLidarOctomap();
    } else {
        throw std::runtime_error("Accumulated map not implemented.");
    }

    renderLidarMap();
    viewer->setBackgroundColor(1, 1, 1);
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
        renderLidarMap();
    }

    int lastCascadeTimestampIdx = findClosestTimestampIndex(run->lidarTimestamps()[targetStep], run->cascadeTimestamps(), false, true);
    auto cascadeCloud = cascadeRadarConfig->heatmapToPointcloud(run->getCascadeHeatmap(lastCascadeTimestampIdx), cascadeRadarIntensityThreshold);  // cascade frame
    auto cascadePose = cascadePoses[targetStep];                                                                                                   // base frame, cascade timestamp
    auto cascadeToMapT = cascadePose * baseToCascadeTransform.inverse();
    pcl::transformPointCloud(*cascadeCloud, *cascadeCloud, cascadeToMapT);                                                                         // map frame    
    std::cout << "cascadeCloud size: " << cascadeCloud->size() << std::endl;
    renderRadarCloud(cascadeCloud);
    
    imageActor->GetMapper()->SetInputData(vtkImage);
}


void DatasetVisualizer::renderLidarMap() {
    viewer->removePointCloud(lidarMapCloudName);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> mapColorHandler(lidarMapCloud, "z");
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
