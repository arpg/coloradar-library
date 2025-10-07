#include "run/base_run.h"


namespace coloradar {


pcl::PointCloud<pcl::PointXYZI>::Ptr Run::sampleMapFrame(
    const float horizontalFov, const float verticalFov, const float range, 
    const Eigen::Affine3f& mapFramePose, 
    const pcl::PointCloud<pcl::PointXYZI>::Ptr mapCloud
) {
    float maxRange = range == 0 ? std::numeric_limits<float>::max() : range;
    pcl::PointCloud<pcl::PointXYZI>::Ptr sample(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*mapCloud, *sample, mapFramePose);
    filterFov(sample, horizontalFov, verticalFov, maxRange);
    return sample;
}


}