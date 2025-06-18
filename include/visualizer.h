#ifndef VISUALIZER_H
#define VISUALIZER_H

#include "coloradar_run.h"


namespace coloradar {


class DatasetVisualizer {

protected:
    // constructor parameters
    const RadarConfig* cascadeRadarConfig;
    const Eigen::Affine3f& baseToLidarTransform;
    const Eigen::Affine3f& baseToCascadeTransform;
    int frameIncrement;
    double cascadeRadarIntensityThreshold;
    std::string cameraConfigPath;
    
    // constants
    const std::string lidarMapCloudName = "lidar_map_cloud";
    const std::string cascadeRadarCloudName = "current_cascade_cloud";

    // static variables
    pcl::visualization::PCLVisualizer::Ptr viewer;
    vtkSmartPointer<vtkImageData> vtkImage;
    vtkSmartPointer<vtkImageActor> imageActor;

    // dynamic variables
    const ColoradarPlusRun* run;
    bool mapIsPrebuilt;
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidarMapCloud;
    std::vector<Eigen::Affine3f> lidarPoses;
    std::vector<Eigen::Affine3f> cascadePoses;
    int numSteps;
    int currentStep;

    // methods
    void reset();
    void initPoses();
    void step(const int increment = 1);
    void keyboardCallback(const pcl::visualization::KeyboardEvent &event);
    void renderLidarMap();
    void renderRadarCloud(const pcl::PointCloud<RadarPoint>::Ptr& cloud);

public:
    DatasetVisualizer(
        const RadarConfig* cascadeRadarConfig,
        const Eigen::Affine3f baseToLidarTransform,
        const Eigen::Affine3f baseToCascadeTransform,
        const int frameIncrement = 1, 
        const double cascadeRadarIntensityThreshold = 0.0,
        const std::string& cameraConfigPath = "camera_config.txt"
    );
    virtual ~DatasetVisualizer() = default;

    void visualize(const ColoradarPlusRun* run, const bool usePrebuiltMap = true);
};

}

#endif
