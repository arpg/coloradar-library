#ifndef H5_UTILS_H
#define H5_UTILS_H

#include "types.h"


namespace coloradar::internal {


template<typename T> struct H5TypeMap;

template<typename T> void saveVectorToHDF5(const std::string& name, H5::H5File& file, const std::vector<T>& vec);
void savePoseToHDF5(const std::string& name, H5::H5File& file, const Eigen::Affine3f& pose);
void savePosesToHDF5(const std::string& name, H5::H5File& file, const std::vector<Eigen::Affine3f>& poses);
void saveCloudToHDF5(const std::string& name, H5::H5File& file, const std::vector<float>& flatCloud, hsize_t numDims);
void saveCloudsToHDF5(const std::string& name, H5::H5File& file, const std::vector<float>& flatClouds, hsize_t numFrames, const std::vector<hsize_t>& cloudSizes, hsize_t numDims);
void saveDatacubesToHDF5(const std::string& name, H5::H5File& file, const std::vector<int16_t>& flatDatacubes, hsize_t numFrames, hsize_t datacubeSize);
void saveHeatmapsToHDF5(const std::string& name, H5::H5File& file, const std::vector<float>& flatHeatmaps, hsize_t numFrames, int numAzimuthBins, int numRangeBins, int numElevationBins, bool hasDoppler);

template<coloradar::PclCloudType CloudT> std::vector<float> flattenLidarCloud(const std::shared_ptr<CloudT>& cloud, bool collapseElevation, bool removeIntensity);
template<coloradar::PclCloudType CloudT> std::vector<float> flattenRadarCloud(const std::shared_ptr<CloudT>& cloud, const int numElevationBins, const bool hasDoppler);

std::vector<double> readH5Timestamps(const H5::H5File& file, const std::string& datasetName);
Eigen::Affine3f readH5Pose(const H5::H5File& file, const std::string& datasetName);
std::vector<Eigen::Affine3f> readH5Poses(const H5::H5File& file, const std::string& datasetName);
std::vector<std::shared_ptr<std::vector<int16_t>>> readH5Datacubes(const H5::H5File& file, const std::string& datasetName);
std::vector<std::shared_ptr<std::vector<float>>> readH5Heatmaps(const H5::H5File& file, const std::string& datasetName);
std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> readH5LidarClouds(const H5::H5File& file, const std::string& baseName);
pcl::PointCloud<pcl::PointXYZI>::Ptr readH5SingleCloud(const H5::H5File& file, const std::string& datasetName);

}


#include "internal_utils/h5_utils.hpp"

#endif