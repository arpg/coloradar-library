#ifndef VISUALIZER_H
#define VISUALIZER_H

#include "coloradar_run.h"


namespace coloradar {


class DatasetVisualizer {

protected:
    // constructor parameters
    const CascadeConfig cascadeRadarConfig;
    const Eigen::Affine3f baseToLidarTransform;
    const Eigen::Affine3f baseToCascadeTransform;
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
    

    // dynamic variables, vary per run
    const ColoradarPlusRun* run;
    std::vector<Eigen::Affine3f> basePoses;
    std::vector<Eigen::Affine3f> lidarPoses;
    std::vector<Eigen::Affine3f> cascadePoses;
    std::vector<int> baseToLidarFrameIndices;
    std::vector<int> baseToCascadeFrameIndices;
    bool mapIsPrebuilt;
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidarMapCloud;
    int numSteps;
    int currentStep;

    // methods
    void reset();
    void initPoses();
    void step(const int increment = 1);
    void keyboardCallback(const pcl::visualization::KeyboardEvent &event);
    pcl::PointCloud<RadarPoint>::Ptr readCascadeCloud(const int scanIdx);
    pcl::PointCloud<pcl::PointXYZI>::Ptr readLidarCloud(const int scanIdx);
    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampleLidarCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud, const float leafSize = 0.1) const;
    pcl::PointCloud<RadarPoint>::Ptr normalizeRadarCloudIntensity(const pcl::PointCloud<RadarPoint>::Ptr& inputCloud) const;
    pcl::PointCloud<RadarPoint>::Ptr extractTopNIntensity(const pcl::PointCloud<RadarPoint>::Ptr& inputCloud, size_t N) const;
    void renderLidarMap();
    void renderRadarCloud(const pcl::PointCloud<RadarPoint>::Ptr& cloud);

public:
    DatasetVisualizer(
        const CascadeConfig cascadeRadarConfig,
        const Eigen::Affine3f baseToLidarTransform,
        const Eigen::Affine3f baseToCascadeTransform,
        const int frameIncrement = 1, 
        const double cascadeRadarIntensityThreshold = 0.0,
        const std::string cameraConfigPath = "camera_config.txt"
    );
    virtual ~DatasetVisualizer() = default;

    void visualize(const ColoradarPlusRun* run, const bool usePrebuiltMap = false);
};

}

#endif
