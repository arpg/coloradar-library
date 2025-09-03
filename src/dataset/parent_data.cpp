#include "dataset/dataset.h"


namespace coloradar {

// PUBLIC METHODS

// const Eigen::Affine3f& ColoradarPlusDataset::imuTransform() const { return imuTransform_; }
// const Eigen::Affine3f& ColoradarPlusDataset::lidarTransform() const { return lidarTransform_; }
// const Eigen::Affine3f& ColoradarPlusDataset::cascadeTransform() const { return cascadeTransform_; }
// const std::shared_ptr<RadarConfig> ColoradarPlusDataset::cascadeConfig() const { return cascadeConfig_; }

// std::vector<std::string> ColoradarPlusDataset::listRuns() {
//     std::vector<std::string> runs;
//     for (const auto& entry : std::filesystem::directory_iterator(runsDirPath_)) {
//         std::string entryName = entry.path().filename().string();
//         if (entry.is_directory() && coloradar::internal::toLower(entryName).find("run") != std::string::npos) {
//             runs.push_back(entryName);
//         }
//     }
//     return runs;
// }

// std::vector<ColoradarPlusRun*> ColoradarPlusDataset::getRuns() {
//     std::vector<std::string> runNames = listRuns();
//     std::vector<ColoradarPlusRun*> runs(runNames.size());
//     for (size_t i = 0; i < runNames.size(); ++i)
//         runs[i] = getRun(runNames[i]);
//     return runs;
// }

// ColoradarPlusRun* ColoradarPlusDataset::getRun(const std::string& runName) {
//     return new ColoradarPlusRun(runsDirPath_ / runName, cascadeConfig_);
// }


// PROTECTED METHODS

Eigen::Affine3f ColoradarPlusDataset::loadTransform(const std::filesystem::path& filePath) {
    coloradar::internal::checkPathExists(filePath);

    Eigen::Vector3f translation;
    std::ifstream file(filePath);
    std::string line;
    std::getline(file, line);
    std::istringstream iss(line);
    iss >> translation.x() >> translation.y() >> translation.z();

    Eigen::Quaternionf rotation;
    std::getline(file, line);
    iss.str(line);
    iss.clear();
    iss >> rotation.x() >> rotation.y() >> rotation.z() >> rotation.w();

    Eigen::Affine3f transform = Eigen::Translation3f(translation) * rotation;
    return transform;
}

}