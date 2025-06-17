#include "visualizer.h"


// void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void *data) {
//     std::cout.precision(20);
//     if (event.keyDown()) {
//         if (event.getKeySym() == "Up") {
//             dataset->currentFileIndex += dataset->FileIncrement;
//             updateDisplay(*dataset);  // Pass dereferenced Dataset
//         } else if (event.getKeySym() == "Down") {
//             if (dataset->currentFileIndex > 0) {
//                 dataset->currentFileIndex -= dataset->FileIncrement;
//                 updateDisplay(*dataset);  // Pass dereferenced Dataset
//             }
//         } else if (event.getKeySym() == "s") {
//             viewer->saveCameraParameters(camera_config_path);
//             std::cout << "Camera parameters saved." << std::endl;
//         } else if (event.getKeySym() == "l") {
//             viewer->loadCameraParameters(camera_config_path);
//             std::cout << "Camera parameters loaded." << std::endl;
//         }
//     }
// }


namespace coloradar {


DatasetVisualizer::DatasetVisualizer()
 : viewer(new pcl::visualization::PCLVisualizer("3D Viewer")),
   vtkImage(vtkSmartPointer<vtkImageData>::New()),
   imageActor(vtkSmartPointer<vtkImageActor>::New()) 
{
    reset();
}


void DatasetVisualizer::reset() {
    stepCounter = -1;
    numSteps = 0;
    numPoseFrames = 0;
    numRadarFrames = 0;
    numLidarFrames = 0;

    viewer->removeAllPointClouds();
    viewer->removeAllCoordinateSystems();
    viewer->removeAllShapes();
    viewer->resetCamera();

    vtkImage->Initialize();
    imageActor->GetMapper()->SetInputData(vtkImage);

    lidarMapCloud->clear();
}


void DatasetVisualizer::visualize(const ColoradarPlusRun* run, const RadarConfig* radarConfig, const bool usePrebuiltMap) {
    reset();
    auto poses = run->getPoses<Eigen::Affine3f>();
    numPoseFrames = poses.size();
    numRadarFrames = run->cascadeTimestamps().size();
    numLidarFrames = run->lidarTimestamps().size();
    numSteps = numPoseFrames;
    if (usePrebuiltMap) {
        lidarMapCloud = run->readLidarOctomap();
    } else {
        throw std::runtime_error("Accumulated map not implemented.");
    }

    viewer->setBackgroundColor(1, 1, 1);
    viewer->addCoordinateSystem(2.0);
    viewer->initCameraParameters();
    // viewer->registerKeyboardCallback(keyboardEventOccurred, static_cast<void*>(&dataset));
    while (!viewer->wasStopped()) {
        viewer->spin();
    }
}

void DatasetVisualizer::step(int increment) {
    if (increment == 0) throw std::runtime_error("Step increment must be non-zero.");
    if (stepCounter + increment >= numSteps or stepCounter + increment < 0) return;

    pcl::PointCloud<pcl::PointXYZI>::Ptr radarCloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidarCloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledLidarCloud(new pcl::PointCloud<pcl::PointXYZI>());
}

void updateDisplay(
    pcl::PointCloud<pcl::PointXYZI>::Ptr radarCloud,
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidarCloud,
    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledLidarCloud
) {
    viewer->removePointCloud("current_scan");
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(lidarCloud, "z");
    viewer->addPointCloud<pcl::PointXYZI>(lidarCloud, intensity_distribution, "current_scan");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.1, "current_scan");

    std::string accum_cloud_name = "accumulated_ds_cloud_" + std::to_string(stepCounter);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> accumulated_ds_intensity_distribution(downsampledLidarCloud, "z");
    viewer->addPointCloud<pcl::PointXYZI>(downsampledLidarCloud, accumulated_ds_intensity_distribution, accum_cloud_name);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.1, accum_cloud_name);

    viewer->removePointCloud("current_radar_points");
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> radar_intensity_cloud(radarCloud, "intensity");
    viewer->addPointCloud<pcl::PointXYZI>(radarCloud, radar_intensity_cloud, "current_radar_points");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "current_radar_points");

    imageActor->GetMapper()->SetInputData(vtkImage);
}


}
