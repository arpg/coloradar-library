#ifndef COLORADAR_RUN_HPP
#define COLORADAR_RUN_HPP

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <fstream>
#include <sstream>


template<coloradar::PoseType PoseT>
std::vector<PoseT> coloradar::ColoradarPlusRun::getPoses() {
    std::filesystem::path posesFilePath = posesDirPath_ / "groundtruth_poses.txt";
    coloradar::internal::checkPathExists(posesFilePath);

    std::vector<PoseT> poses;
    std::ifstream infile(posesFilePath);
    std::string line;

    while (std::getline(infile, line)) {
        float x, y, z, rotX, rotY, rotZ, rotW;
        std::istringstream iss(line);
        iss >> x >> y >> z >> rotX >> rotY >> rotZ >> rotW;
        typename coloradar::PoseTraits<PoseT>::TranslationType translation(x, y, z);
        typename coloradar::PoseTraits<PoseT>::RotationType rotation(rotW, rotX, rotY, rotZ);
        PoseT pose = coloradar::internal::makePose<PoseT>(translation, rotation);
        // std::cout << pose.translation().x() << " " << pose.translation().y() << " " << pose.translation().z();
        poses.push_back(pose);
    }
    return poses;
}


template<coloradar::PoseType PoseT>
std::vector<PoseT> coloradar::ColoradarPlusRun::interpolatePoses(const std::vector<PoseT>& poses, const std::vector<double>& poseTimestamps, const std::vector<double>& targetTimestamps) {
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


template<coloradar::PclCloudType CloudT>
CloudT coloradar::ColoradarPlusRun::getLidarPointCloud(const std::filesystem::path& binPath) {
    return coloradar::internal::readLidarPointCloud<typename CloudT::PointType, CloudT>(binPath);
}

template<coloradar::OctomapCloudType CloudT>
CloudT coloradar::ColoradarPlusRun::getLidarPointCloud(const std::filesystem::path& binPath) {
    return coloradar::internal::readLidarPointCloud<octomap::point3d, CloudT>(binPath);
}

template<coloradar::CloudType CloudT>
CloudT coloradar::ColoradarPlusRun::getLidarPointCloud(const int& cloudIdx) {
    std::filesystem::path pclBinFilePath = lidarCloudsDirPath_ / ("lidar_pointcloud_" + std::to_string(cloudIdx) + ".bin");
    return getLidarPointCloud<CloudT>(pclBinFilePath);
}

#endif
