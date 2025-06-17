#ifndef VISUALIZER_H
#define VISUALIZER_H

#include "coloradar_run.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkSmartPointer.h>
#include <vtkImageData.h>
#include <vtkImageActor.h>
#include <vtkImageMapper3D.h>


namespace coloradar {

class DatasetVisualizer {
protected:
    pcl::visualization::PCLVisualizer::Ptr viewer;
    vtkSmartPointer<vtkImageData> vtkImage;
    vtkSmartPointer<vtkImageActor> imageActor;

    pcl::PointCloud<pcl::PointXYZI>::Ptr lidarMapCloud;
    int stepCounter;
    int numSteps;
    int numPoseFrames;
    int numRadarFrames;
    int numLidarFrames;

    void step(int increment = 1);
    void reset();
    void updatePointCloudDisplay(
        pcl::PointCloud<pcl::PointXYZI>::Ptr radarCloud,
        pcl::PointCloud<pcl::PointXYZI>::Ptr lidarCloud,
        pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledLidarCloud
    );

public:
    explicit DatasetVisualizer();
    virtual ~DatasetVisualizer() = default;

    void visualize(
        const ColoradarPlusRun* run, 
        const RadarConfig* radarConfig,
        const bool usePrebuiltMap = false
    );

};

}

#endif
