#include "h5/dataset.h"


namespace coloradar {

H5Dataset::H5Dataset(const std::filesystem::path& pathToH5File) {
    // read file
    coloradar::internal::checkPathExists(pathToH5File);
    h5SourceFilePath_ = pathToH5File;
    H5::H5File file(h5SourceFilePath_.string(), H5F_ACC_RDONLY);
    H5::DataSet datasetConfig = file.openDataSet("config");

    // read config
    H5::StrType strType(H5::PredType::C_S1, H5T_VARIABLE);
    std::string configStr;
    datasetConfig.read(configStr, strType);
    Json::CharReaderBuilder b;
    std::unique_ptr<Json::CharReader> reader(b.newCharReader());
    Json::Value root;
    std::string errs;
    const char* begin = configStr.data();
    const char* end   = begin + configStr.size();
    if (!reader->parse(begin, end, &root, &errs)) {
        throw std::runtime_error(std::string("Failed to parse JSON from HDF5 'config': ") + errs);
    }
    if (!root.isObject()) {
        throw std::runtime_error("Invalid config JSON: expected an object at top level.");
    }

    // extract runs
    std::vector<std::string> configRuns;
    if (!root.isMember("runs") || !root["runs"].isArray()) {
        throw std::runtime_error("Invalid config JSON: 'runs' must be an array.");
    }
    const Json::Value& runsNode = root["runs"];
    configRuns.reserve(runsNode.size());
    for (const auto& v : runsNode) {
        if (!v.isString()) {
            throw std::runtime_error("Invalid config JSON: 'runs' must be an array of strings.");
        }
        configRuns.push_back(v.asString());
    }

    // extract data_content
    std::set<std::string, std::less<>> configDataContent;
    if (!root.isMember("data_content") || !root["data_content"].isArray()) {
        throw std::runtime_error("Invalid config JSON: 'data_content' must be an array.");
    }
    const Json::Value& dcNode = root["data_content"];
    for (const auto& v : dcNode) {
        if (!v.isString()) {
            throw std::runtime_error("Invalid config JSON: 'data_content' must be an array of strings.");
        }
        configDataContent.insert(v.asString());
    }

    // extract cascade radar config
    if (root.isMember("radar_config") && root["radar_config"].isObject()) {
        cascadeConfig_ = std::make_shared<coloradar::CascadeConfig>(root["radar_config"]);
    }

    for (const auto& run : configRuns) {
        std::vector<double> poseTimestamps = {}, imuTimestamps = {}, lidarTimestamps = {}, cascadeCubeTimestamps = {}, cascadeTimestamps = {};
        std::vector<Eigen::Affine3f> poses = {};
        std::vector<std::shared_ptr<std::vector<int16_t>>> cascadeDatacubes = {};
        std::vector<std::shared_ptr<std::vector<float>>> cascadeHeatmaps = {};
        std::vector<pcl::PointCloud<RadarPoint>::Ptr> cascadePointclouds = {};
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> lidarPointclouds = {};
        std::shared_ptr<octomap::OcTree> lidarOctomap = {};
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> mapSamples = {};

        if (configDataContent.find(poseTimestampsContentName) != configDataContent.end()) {
            poseTimestamps = coloradar::internal::readH5Timestamps(file, getExportArrayName(poseTimestampsContentName, run));
        }
        if (configDataContent.find(imuTimestampsContentName) != configDataContent.end()) {
            imuTimestamps = coloradar::internal::readH5Timestamps(file, getExportArrayName(imuTimestampsContentName, run));
        }
        if (configDataContent.find(lidarTimestampsContentName) != configDataContent.end()) {
            lidarTimestamps = coloradar::internal::readH5Timestamps(file, getExportArrayName(lidarTimestampsContentName, run));
        }
        if (configDataContent.find(cascadeCubeTimestampsContentName) != configDataContent.end()) {
            cascadeCubeTimestamps = coloradar::internal::readH5Timestamps(file, getExportArrayName(cascadeCubeTimestampsContentName, run));
        }
        if (configDataContent.find(cascadeTimestampsContentName) != configDataContent.end()) {
            cascadeTimestamps = coloradar::internal::readH5Timestamps(file, getExportArrayName(cascadeTimestampsContentName, run));
        }
        if (configDataContent.find(posesContentName) != configDataContent.end()) {
            poses = coloradar::internal::readH5Poses(file, getExportArrayName(posesContentName, run));
        }
        if (configDataContent.find(lidarCloudsContentName) != configDataContent.end()) {
            lidarPointclouds = coloradar::internal::readH5LidarClouds(file, getExportArrayName(lidarCloudsContentName, run));
        }
        runs_[run] = std::make_shared<H5Run>(
            run, cascadeConfig_, 
            poseTimestamps, poses,
            imuTimestamps, lidarTimestamps, lidarPointclouds,
            cascadeCubeTimestamps, cascadeDatacubes,
            cascadeTimestamps, cascadeHeatmaps, cascadePointclouds
        );
    }

    // print info
    std::cout << "data_content: ";
    for (const auto& c : configDataContent) std::cout << c << " ";
    std::cout << std::endl;

    if (cascadeConfig_) {
        std::cout << "cascade nRangeBins = " << cascadeConfig_->nRangeBins() << std::endl;
    }

    // Print timestamp array lengths for each run
    std::cout << "\nRun details:" << std::endl;
    for (const auto& [name, run] : runs_) {
        std::cout << "Run " << name << ": poses=" << run->poseTimestamps().size() 
                // << " (" << run->getPoses<Eigen::Affine3f>().size() << " poses)"
                 << ", imu=" << run->imuTimestamps().size()
                 << ", lidar=" << run->lidarTimestamps().size()
                 // << " (" << run->getLidarPointCloud<pcl::PointCloud<pcl::PointXYZI>>(0)->size() << " clouds)"
                 << ", cascade_cubes=" << run->cascadeCubeTimestamps().size()
                 << ", cascade=" << run->cascadeTimestamps().size() << std::endl;
    }
}


}