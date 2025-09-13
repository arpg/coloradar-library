#ifndef UTILS_H
#define UTILS_H

#include "internal_utils/h5_utils.h"
#include "internal_utils/internal_utils.h"


namespace coloradar {


int frameIdxToFileIdx(const int frameIdx, const std::filesystem::path& framesDirPath);
const int findClosestTimestampIndex(const double targetTimestamp, const std::vector<double>& timestamps, const std::string& preference = "none");
template<PoseType PoseT> std::vector<PoseT> interpolatePoses(const std::vector<PoseT>& poses, const std::vector<double>& poseTimestamps, const std::vector<double>& targetTimestamps);


struct RadarPoint {
    EIGEN_ALIGN16
    PCL_ADD_POINT4D;
    float intensity;
    float doppler;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


// PCL clouds
template <PclPointType PointT, template <PclCloudType> class CloudT> void filterFov(std::shared_ptr<CloudT<PointT>>& cloud, const float& horizontalFov, const float& verticalFov, const float& range);
template <Pcl4dPointType PointT, template <typename> class CloudT> void collapseElevation(std::shared_ptr<CloudT<PointT>>& cloud, const float& elevationMinMeters, const float& elevationMaxMeters);
template <PclPointType PointT, template <typename> class CloudT> void collapseElevation(std::shared_ptr<CloudT<PointT>>& cloud);
template <Pcl4dPointType PointT, template <PclCloudType> class CloudT> void octreeToPcl(const octomap::OcTree& tree, std::shared_ptr<CloudT<PointT>>& cloud);
template <Pcl4dPointType PointT, template <typename> class CloudT> void filterOccupancy(std::shared_ptr<CloudT<PointT>>& cloud, const float& probabilityThreshold = 0.0, const bool& saveProbabilities = false);
template <typename PointT, template <typename> class CloudT> pcl::PointCloud<RadarPoint>::Ptr toRadarCloud(const std::shared_ptr<CloudT<PointT>> cloud);

class OctoPointcloud : public octomap::Pointcloud, public std::enable_shared_from_this<OctoPointcloud> {
public:
    OctoPointcloud() = default;
    OctoPointcloud(std::shared_ptr<OctoPointcloud>& other) : octomap::Pointcloud(*other) {}
    template <PclPointType PointT, template <PclCloudType> class CloudT> OctoPointcloud(std::shared_ptr<CloudT<PointT>>& cloud);

    template <PclCloudType CloudT> std::shared_ptr<CloudT> toPcl();

    void filterFov(const float& horizontalFovTan, const float& verticalFovTan, const float& range);
    void transform(const Eigen::Affine3f& transformMatrix);
    using octomap::Pointcloud::transform;
};

}


#ifndef RADAR_POINT_REGISTERED
#define RADAR_POINT_REGISTERED

POINT_CLOUD_REGISTER_POINT_STRUCT(coloradar::RadarPoint,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (float, doppler, doppler))

#endif


#include "utils/basic_utils.hpp"
#include "utils/octo_pointcloud.hpp"

#endif
