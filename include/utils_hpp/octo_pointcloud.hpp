#ifndef OCTO_POINTCLOUD_HPP
#define OCTO_POINTCLOUD_HPP


template <coloradar::PclPointType PointT, template <coloradar::PclCloudType> class CloudT>
coloradar::OctoPointcloud::OctoPointcloud(std::shared_ptr<CloudT<PointT>>& cloud) {
    this->clear();
    this->reserve(cloud->size());
    for (const auto& point : cloud->points) {
        this->push_back(octomap::point3d(point.x, point.y, point.z));
    }
}

template <coloradar::PclCloudType CloudT>
std::shared_ptr<CloudT> coloradar::OctoPointcloud::toPcl() {
    using PointT = typename CloudT::PointType;
    std::shared_ptr<CloudT> cloud(new CloudT);
    cloud->reserve(this->size());
    for (size_t i = 0; i < this->size(); ++i) {
        const octomap::point3d& point = this->getPoint(i);
        cloud->push_back(PointT(point.x(), point.y(), point.z()));
    }
    return cloud;
}

#endif
