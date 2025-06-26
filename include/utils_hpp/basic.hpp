#ifndef BASIC_HPP
#define BASIC_HPP


namespace coloradar {


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


template<coloradar::PoseType PoseT>
std::vector<PoseT> interpolatePoses(const std::vector<PoseT>& poses, const std::vector<double>& poseTimestamps, const std::vector<double>& targetTimestamps) {
    if (poses.empty()) throw std::runtime_error("Cannot interpolate empty poses.");
    if (poseTimestamps.empty()) throw std::runtime_error("Cannot interpolate over empty pose timestamps.");
    if (targetTimestamps.empty()) throw std::runtime_error("Cannot interpolate over empty target timestamps.");
    if (poses.size() != poseTimestamps.size()) throw std::runtime_error("The length of poses and poses' timestamps vectors must match.");

    std::vector<PoseT> interpolatedPoses;
    size_t tsStartIdx = 0, tsEndIdx = poseTimestamps.size() - 1;
    size_t tsIdx = tsStartIdx;

    for (size_t targetTsIdx = 0; targetTsIdx < targetTimestamps.size(); ++targetTsIdx) {
        if (targetTimestamps[targetTsIdx] < poseTimestamps[tsStartIdx]) {
            interpolatedPoses.push_back(poses[tsStartIdx]);
            continue;
        }
        if (targetTimestamps[targetTsIdx] > poseTimestamps[tsEndIdx]) {
            interpolatedPoses.push_back(poses[tsEndIdx]);
            continue;
        }
        while (tsIdx + 1 <= tsEndIdx && poseTimestamps[tsIdx + 1] < targetTimestamps[targetTsIdx])
            tsIdx++;
        double denominator = poseTimestamps[tsIdx + 1] - poseTimestamps[tsIdx];
        double ratio = denominator > 0.0f ? (targetTimestamps[targetTsIdx] - poseTimestamps[tsIdx]) / denominator : 0.0f;

        Eigen::Vector3f t1 = coloradar::internal::toEigenTrans(poses[tsIdx]);
        Eigen::Vector3f t2 = coloradar::internal::toEigenTrans(poses[tsIdx + 1]);
        Eigen::Vector3f interpolatedTransEig = (1.0f - ratio) * t1 + ratio * t2;
        auto interpolatedTrans = coloradar::internal::fromEigenTrans<typename coloradar::PoseTraits<PoseT>::TranslationType>(interpolatedTransEig);

        Eigen::Quaternionf q1 = coloradar::internal::toEigenQuat(poses[tsIdx]);
        Eigen::Quaternionf q2 = coloradar::internal::toEigenQuat(poses[tsIdx + 1]);
        Eigen::Quaternionf interpolatedRotEig = q1.slerp(ratio, q2);
        auto interpolatedRot = coloradar::internal::fromEigenQuat<typename coloradar::PoseTraits<PoseT>::RotationType>(interpolatedRotEig);

        PoseT interpolatedPose = coloradar::internal::makePose<PoseT>(interpolatedTrans, interpolatedRot);
        interpolatedPoses.push_back(interpolatedPose);
    }
    return interpolatedPoses;
}


}

#endif
