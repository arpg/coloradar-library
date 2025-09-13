#ifndef VISUALIZER_H
#define VISUALIZER_H

#include "run/base_run.h"


namespace coloradar {


struct Control {
    std::string button;
    std::string name;
    std::string description;
    std::function<void()> action;
};


class DatasetVisualizer {

protected:
    // constructor parameters
    std::shared_ptr<CascadeConfig> cascadeRadarConfig;
    std::shared_ptr<CascadeConfig> clippedCascadeRadarConfig;
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
    std::shared_ptr<Run> run;
    std::vector<Eigen::Affine3f> basePoses;
    std::vector<Eigen::Affine3f> basePosesLidarTs;
    std::vector<Eigen::Affine3f> basePosesCascadeTs;
    std::vector<int> cascadeToLidarFrameIndices;
    std::vector<int> baseToLidarFrameIndices;
    std::vector<int> baseToCascadeFrameIndices;
    bool mapIsPrebuilt;
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidarMapCloud;
    int numSteps;
    int currentStep;
    int currentMapStep;
    std::string currentMessage;
    std::chrono::steady_clock::time_point messageTime;

    // methods
    void reset();
    void step(const int increment = 1);
    void keyboardCallback(const pcl::visualization::KeyboardEvent &event);
    pcl::PointCloud<RadarPoint>::Ptr readCascadeCloud(const int scanIdx);
    pcl::PointCloud<pcl::PointXYZI>::Ptr readLidarCloud(const int scanIdx);
    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampleLidarCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud, const float leafSize = 0.3) const;
    pcl::PointCloud<RadarPoint>::Ptr normalizeRadarCloudIntensity(const pcl::PointCloud<RadarPoint>::Ptr& inputCloud) const;
    pcl::PointCloud<RadarPoint>::Ptr extractTopNIntensity(const pcl::PointCloud<RadarPoint>::Ptr& inputCloud, size_t N) const;
    void updateLidarMap(const int lastStep);
    void renderLidarMap();
    void renderRadarCloud(const pcl::PointCloud<RadarPoint>::Ptr& cloud);
    std::vector<Control> getControls();
    void renderLegend();
    void showMessage(const std::string& message);

public:
    DatasetVisualizer(
        const std::shared_ptr<CascadeConfig> cascadeRadarConfig,
        const Eigen::Affine3f baseToLidarTransform,
        const Eigen::Affine3f baseToCascadeTransform,
        const int frameIncrement = 1, 
        const double cascadeRadarIntensityThreshold = 0.0,
        const std::string cameraConfigPath = "camera_config.txt"
    );
    virtual ~DatasetVisualizer() = default;

    void visualize(const std::shared_ptr<Run> run, const bool usePrebuiltMap = false);
};

}

#endif
