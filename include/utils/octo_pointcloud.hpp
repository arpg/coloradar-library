#ifndef OCTO_POINTCLOUD_HPP
#define OCTO_POINTCLOUD_HPP


namespace coloradar {

template <PclPointType PointT, template <PclCloudType> class CloudT>
OctoPointcloud::OctoPointcloud(std::shared_ptr<CloudT<PointT>>& cloud) {
    this->clear();
    this->reserve(cloud->size());
    for (const auto& point : cloud->points) {
        this->push_back(octomap::point3d(point.x, point.y, point.z));
    }
}

template <PclCloudType CloudT>
std::shared_ptr<CloudT> OctoPointcloud::toPcl() {
    using PointT = typename CloudT::PointType;
    std::shared_ptr<CloudT> cloud(new CloudT);
    cloud->reserve(this->size());
    for (size_t i = 0; i < this->size(); ++i) {
        const octomap::point3d& point = this->getPoint(i);
        cloud->push_back(PointT(point.x(), point.y(), point.z()));
    }
    return cloud;
}

}

#endif
