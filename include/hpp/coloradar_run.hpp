#ifndef COLORADAR_RUN_HPP
#define COLORADAR_RUN_HPP


template<coloradar::PoseType PoseT>
std::vector<PoseT> coloradar::ColoradarPlusRun::getPoses() const {
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
        poses.push_back(pose);
    }
    return poses;
}


template<coloradar::PclCloudType CloudT>
std::shared_ptr<CloudT> coloradar::ColoradarPlusRun::getLidarPointCloud(const std::filesystem::path& binPath) {
    return coloradar::internal::readLidarPointCloud<typename CloudT::PointType, CloudT>(binPath);
}

template<coloradar::OctomapCloudType CloudT>
std::shared_ptr<CloudT> coloradar::ColoradarPlusRun::getLidarPointCloud(const std::filesystem::path& binPath) {
    return coloradar::internal::readLidarPointCloud<octomap::point3d, CloudT>(binPath);
}

template<coloradar::CloudType CloudT>
std::shared_ptr<CloudT> coloradar::ColoradarPlusRun::getLidarPointCloud(const int& cloudIdx) {
    std::filesystem::path pclBinFilePath = lidarCloudsDirPath_ / ("lidar_pointcloud_" + std::to_string(cloudIdx) + ".bin");
    return getLidarPointCloud<CloudT>(pclBinFilePath);
}

#endif
