#ifndef COLORADAR_RUN_HPP
#define COLORADAR_RUN_HPP


namespace coloradar {


template<CloudType CloudT>
std::shared_ptr<CloudT> ColoradarPlusRun::getLidarPointCloud(const int cloudIdx) const {
    std::filesystem::path cloudsDir = lidarCloudsDirPath_;
    int fileIdx = frameIdxToFileIdx(cloudIdx, cloudsDir);
    std::filesystem::path pclBinFilePath = cloudsDir / ("lidar_pointcloud_" + std::to_string(fileIdx) + ".bin");
    return getLidarPointCloud<CloudT>(pclBinFilePath);
}

template<PclCloudType CloudT>
std::shared_ptr<CloudT> ColoradarPlusRun::getLidarPointCloud(const std::filesystem::path& binPath) const {
    return coloradar::internal::readLidarPointCloud<typename CloudT::PointType, CloudT>(binPath);
}

template<OctomapCloudType CloudT>
std::shared_ptr<CloudT> ColoradarPlusRun::getLidarPointCloud(const std::filesystem::path& binPath) const {
    return coloradar::internal::readLidarPointCloud<octomap::point3d, CloudT>(binPath);
}


}


#endif
