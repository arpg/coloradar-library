#ifndef H5_COLORADAR_RUN_HPP
#define H5_COLORADAR_RUN_HPP


namespace coloradar {

template<PoseType PoseT>
std::vector<PoseT> H5Run::getPoses() const {
    if (poses_.empty()) throw std::runtime_error("No poses found in run " + name_);
    std::vector<PoseT> outPoses;
    outPoses.reserve(poses_.size());
    for (const auto& pose : poses_) {
        outPoses.push_back(coloradar::internal::fromEigenPose<PoseT>(pose));
    }
    return outPoses;
}

template<CloudType CloudT>
std::shared_ptr<CloudT> H5Run::getLidarPointCloud(const int cloudIdx) const {
    if (lidarPointclouds_.empty()) 
        throw std::runtime_error("No lidar point clouds found in run " + name_);
    if (cloudIdx < 0 || cloudIdx >= lidarPointclouds_.size()) 
        throw std::runtime_error("Invalid lidar point cloud index: " + std::to_string(cloudIdx) + " (size: " + std::to_string(lidarPointclouds_.size()) + ")");
    return lidarPointclouds_[cloudIdx];
}

}

#endif
