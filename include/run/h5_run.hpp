#ifndef H5_COLORADAR_RUN_HPP
#define H5_COLORADAR_RUN_HPP


namespace coloradar {


template<CloudType CloudT>
std::shared_ptr<CloudT> H5Run::getLidarPointCloud(const int cloudIdx) const {
    if (lidarPointclouds_.empty()) throw std::runtime_error("No lidar point clouds found in run " + name_);
    if (cloudIdx < 0 || cloudIdx >= lidarPointclouds_.size()) throw std::invalid_argument("H5Run: 'cloudIdx' out of bounds [0, " + std::to_string(lidarPointclouds_.size()) + ")");
    return std::dynamic_pointer_cast<CloudT>(lidarPointclouds_[cloudIdx]);
}

}

#endif
