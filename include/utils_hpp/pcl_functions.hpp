#ifndef PCL_FUNCTIONS_HPP
#define PCL_FUNCTIONS_HPP


template <coloradar::Pcl4dPointType PointT, template <coloradar::PclCloudType> class CloudT>
void coloradar::octreeToPcl(const octomap::OcTree& tree, std::shared_ptr<CloudT<PointT>>& cloud) {
    cloud->clear();
    for (auto it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it) {
        PointT point;
        octomap::point3d coords = it.getCoordinate();
        point.x = coords.x();
        point.y = coords.y();
        point.z = coords.z();
        point.intensity = it->getLogOdds();
        cloud->push_back(point);
    }
}


template <coloradar::PclPointType PointT, template <coloradar::PclCloudType> class CloudT>
void coloradar::filterFov(std::shared_ptr<CloudT<PointT>>& cloud, const float& horizontalFov, const float& verticalFov, const float& range) {
    coloradar::internal::filterFov<PointT, CloudT<PointT>>(cloud, horizontalFov, verticalFov, range);
}


struct PointKey {
    float x;
    float y;
    int precision;

    PointKey(float x, float y, int precision)
        : x(round(x * std::pow(10, precision)) / std::pow(10, precision)),
          y(round(y * std::pow(10, precision)) / std::pow(10, precision)),
          precision(precision) {}

    bool operator==(const PointKey& other) const {
        return x == other.x && y == other.y;
    }
};
struct PointKeyHash {
    std::size_t operator()(const PointKey& key) const {
        std::hash<float> hash_fn;
        return hash_fn(key.x) ^ (hash_fn(key.y) << 1);
    }
};


template <coloradar::Pcl4dPointType PointT, template <typename> class CloudT>
void coloradar::collapseElevation(std::shared_ptr<CloudT<PointT>>& cloud, const float& elevationMinMeters, const float& elevationMaxMeters) {
    const int precision = 4;
    if (elevationMinMeters > elevationMaxMeters)
        throw std::invalid_argument("Invalid elevation range: elevationMin must be less or equal to elevationMax.");
    std::unordered_map<PointKey, PointT, PointKeyHash> maxIntensityMap;
    maxIntensityMap.reserve(cloud->size());
    for (const auto& point : *cloud) {
        if (point.z >= elevationMinMeters && point.z <= elevationMaxMeters) {
            PointKey key(point.x, point.y, precision);
            auto iter = maxIntensityMap.find(key);
            if (iter == maxIntensityMap.end() || point.intensity > iter->second.intensity) {
                maxIntensityMap[key] = point;
            }
        }
    }
    cloud->clear();
    cloud->reserve(maxIntensityMap.size());
    for (const auto& [key, point] : maxIntensityMap) {
        PointT collapsedPoint = point;
        collapsedPoint.z = 0.0f;
        cloud->push_back(collapsedPoint);
    }
}

template <coloradar::PclPointType PointT, template <typename> class CloudT>
void coloradar::collapseElevation(std::shared_ptr<CloudT<PointT>>& cloud) {
    const int precision = 4;
    std::unordered_map<PointKey, PointT, PointKeyHash> uniquePointsMap;
    uniquePointsMap.reserve(cloud->size());
    for (const auto& point : *cloud) {
        PointKey key(point.x, point.y, precision);
        if (uniquePointsMap.find(key) == uniquePointsMap.end()) {
            uniquePointsMap[key] = point;
        }
    }
    cloud->clear();
    cloud->reserve(uniquePointsMap.size());
    for (const auto& [key, point] : uniquePointsMap) {
        PointT collapsedPoint = point;
        collapsedPoint.z = 0.0f;
        cloud->push_back(collapsedPoint);
    }
}

template <coloradar::Pcl4dPointType PointT, template <typename> class CloudT>
void coloradar::filterOccupancy(std::shared_ptr<CloudT<PointT>>& cloud, const float& probabilityThreshold, const bool& saveProbabilities) {
    if (probabilityThreshold < 0.0f || probabilityThreshold > 1.0f) {
        throw std::out_of_range("Invalid probability threshold: expected value from 0 to 1, got " + std::to_string(probabilityThreshold));
    }
    if (probabilityThreshold == 0.0f && !saveProbabilities) return;

    CloudT<PointT> filteredCloud;
    filteredCloud.reserve(cloud->size());
    for (const auto& point : *cloud) {
        float probability = 1.0f / (1.0f + std::exp(-point.intensity));
        if (probability >= probabilityThreshold) {
            PointT newPoint = point;
            if (saveProbabilities) newPoint.intensity = probability;
            filteredCloud.push_back(newPoint);
        }
    }
    cloud->swap(filteredCloud);
}


#endif
