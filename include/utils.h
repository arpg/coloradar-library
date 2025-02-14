#ifndef UTILS_H
#define UTILS_H

#include "radar_configs.h"

#include <filesystem>
#include <vector>
#include <Eigen/Dense>


namespace coloradar {

struct RadarPoint
{
  PCL_ADD_POINT4D;
  float intensity;
  float doppler;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;


template <Pcl4dPointType PointT, template <PclCloudType> class CloudT> void octreeToPcl(const octomap::OcTree& tree, CloudT<PointT>& cloud);
template <PclPointType PointT, template <PclCloudType> class CloudT> void filterFov(CloudT<PointT>& cloud, const float& horizontalFov, const float& verticalFov, const float& range);
pcl::PointCloud<RadarPoint> heatmapToPointcloud(const std::vector<float>& heatmap, coloradar::RadarConfig* config, const float& intensityThresholdPercent = 0.0);
template <Pcl4dPointType PointT, template <typename> class CloudT> void collapseElevation(CloudT<PointT>& cloud, const float& elevationMinMeters, const float& elevationMaxMeters);
template <PclPointType PointT, template <typename> class CloudT> void collapseElevation(CloudT<PointT>& cloud);

void convertRadarBinsToFov(int azimuthMaxBin, int elevationMaxBin, int rangeMaxBin, const RadarConfig* config, float& horizontalFov, float& verticalFov, float& range);
void convertFovToRadarBins(const float& horizontalFov, const float& verticalFov, const float& range, const RadarConfig* config, int& azimuthMaxBin, int& elevationMaxBin, int& rangeMaxBin);
std::vector<float> clipHeatmapImage(const std::vector<float>& image, const float& horizontalFov, const float& verticalFov, const float& range, const RadarConfig* config);
std::vector<float> clipHeatmapImage(const std::vector<float>& image, int azimuthMaxBin, int elevationMaxBin, int rangeMaxBin, const RadarConfig* config);

void convertBinsToElevationRange(int elevationMinBin, int elevationMaxBin, const std::vector<float>& elevationBins, float& elevationMinDeg, float& elevationMaxDeg);
void convertElevationRangeToBins(float elevationMinDeg, float elevationMaxDeg, const std::vector<float>& elevationBins, int& elevationMinBin, int& elevationMaxBin);
std::vector<float> collapseHeatmapElevation(const std::vector<float>& image,  const float& elevationMinMeters, const float& elevationMaxMeters, const std::vector<float>& elevationBins, const int& numAzimuthBins,  const int& numRangeBins);
// std::vector<float> collapseHeatmapElevation(const std::vector<float>& image, float elevationMinDeg, float elevationMaxDeg,  const std::vector<float>& elevationBins, const int& numAzimuthBins, const int& numElevationBins, const int& numRangeBins);
std::vector<float> removeDoppler(const std::vector<float>& image);


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

#include "pcl_functions.hpp"
#include "octo_pointcloud.hpp"

#endif
