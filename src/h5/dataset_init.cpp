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
    std::vector<std::string> configDataContent;
    if (!root.isMember("data_content") || !root["data_content"].isArray()) {
        throw std::runtime_error("Invalid config JSON: 'data_content' must be an array.");
    }
    const Json::Value& dcNode = root["data_content"];
    configDataContent.reserve(dcNode.size());
    for (const auto& v : dcNode) {
        if (!v.isString()) {
            throw std::runtime_error("Invalid config JSON: 'data_content' must be an array of strings.");
        }
        configDataContent.push_back(v.asString());
    }

    // extract cascade radar config
    if (root.isMember("radar_config") && root["radar_config"].isObject()) {
        cascadeConfig_ = std::make_shared<coloradar::CascadeConfig>(root["radar_config"]);
    }

    // print info
    std::cout << "Runs (" << configRuns.size() << "): ";
    for (const auto& r : configRuns) std::cout << r << " ";
    std::cout << std::endl;

    std::cout << "data_content: ";
    for (const auto& c : configDataContent) std::cout << c << " ";
    std::cout << std::endl;

    if (cascadeConfig_) {
        std::cout << "cascade nRangeBins = " << cascadeConfig_->nRangeBins() << std::endl;
    }

}


}