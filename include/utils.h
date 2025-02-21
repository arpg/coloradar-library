#ifndef UTILS_H
#define UTILS_H

#include "internal.h"


namespace coloradar {

struct RadarPoint {
    EIGEN_ALIGN16
    PCL_ADD_POINT4D;
    float intensity;
    float doppler;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template <Pcl4dPointType PointT, template <PclCloudType> class CloudT> void octreeToPcl(const octomap::OcTree& tree, CloudT<PointT>& cloud);
template <PclPointType PointT, template <PclCloudType> class CloudT> void filterFov(CloudT<PointT>& cloud, const float& horizontalFov, const float& verticalFov, const float& range);
template <Pcl4dPointType PointT, template <typename> class CloudT> void collapseElevation(CloudT<PointT>& cloud, const float& elevationMinMeters, const float& elevationMaxMeters);
template <PclPointType PointT, template <typename> class CloudT> void collapseElevation(CloudT<PointT>& cloud);
template <Pcl4dPointType PointT, template <typename> class CloudT> void filterOccupancy(CloudT<PointT>& cloud, const float& probabilityThreshold = 0.0, const bool& saveProbabilities = false);


class OctoPointcloud : public octomap::Pointcloud {
public:
    OctoPointcloud() = default;
    OctoPointcloud(const OctoPointcloud& other) : octomap::Pointcloud(other) {}
    template <PclPointType PointT, template <PclCloudType> class CloudT> OctoPointcloud(const CloudT<PointT>& cloud);

    template <PclCloudType CloudT> CloudT toPcl();

    void filterFov(const float& horizontalFovTan, const float& verticalFovTan, const float& range);
    void transform(const Eigen::Affine3f& transformMatrix);
    using octomap::Pointcloud::transform;
};

}

#include "utils_hpp/pcl_functions.hpp"
#include "utils_hpp/octo_pointcloud.hpp"

#endif
