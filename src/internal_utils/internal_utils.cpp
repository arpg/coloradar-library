#include "internal_utils/internal_utils.h"


void coloradar::internal::checkPathExists(const std::filesystem::path& path) {
    // std::cout << "Validating path: " << path << std::endl;
    if (!std::filesystem::exists(path)) {
         throw std::filesystem::filesystem_error("Failed to open file", path, std::make_error_code(std::errc::no_such_file_or_directory));
    }
}
void coloradar::internal::createDirectoryIfNotExists(const std::filesystem::path& dirPath) {
    if (!std::filesystem::exists(dirPath)) {
        std::filesystem::create_directories(dirPath);
    }
}
std::vector<std::filesystem::path> coloradar::internal::readArrayDirectory(
    const std::filesystem::path& directoryPath,
    const std::string& filePrefix,
    const std::string& fileExtension,
    const int& arraySize
) {
    std::string extension = (fileExtension[0] == '.') ? fileExtension : "." + fileExtension;
    std::map<int, std::filesystem::path> filePaths;

    // List files
    for (const auto& entry : std::filesystem::directory_iterator(directoryPath)) {
        if (entry.path().extension() == extension) {                                // Empty extension is ok
            std::string filename = entry.path().stem().string();                    // Name string without extension
            if (filename.rfind(filePrefix, 0) == 0) {                               // Make sure it starts with prefix, empty prefix is ok
                try {
                    int index = std::stoi(filename.substr(filePrefix.size()));      // Extract file index
                    filePaths[index] = entry.path();
                } catch (const std::exception&) {
                    continue;                                                       // Ignore malformed filenames
                }
            }
        }
    }
    size_t numFiles = arraySize >= 0 ? arraySize : filePaths.size();
    if (numFiles != filePaths.size()) {
        throw std::runtime_error(
            "Expected " + std::to_string(numFiles) + " files (prefix '" + filePrefix +
            "', extension '" + extension + "'), but found " + std::to_string(filePaths.size()) +
            " in directory: " + directoryPath.string());
    }
    std::vector<std::filesystem::path> files(numFiles);
    for (int i = 0; i < numFiles; ++i) {
        if (!filePaths.count(i)) {
            throw std::runtime_error("Missing expected file: '" + filePrefix + std::to_string(i) + extension + "' in directory: " + directoryPath.string());
        }
        files[i] = filePaths[i];
    }
    return files;
}

std::string coloradar::internal::toLower(std::string s) {
    for (auto& c : s) c = std::tolower(c);
    return s;
}


template<> octomath::Vector3 coloradar::internal::fromEigenTrans(const Eigen::Vector3f& r) { return octomath::Vector3(r.x(), r.y(), r.z()); }
template<> octomath::Quaternion coloradar::internal::fromEigenQuat(const Eigen::Quaternionf& r) { return octomath::Quaternion(r.w(), r.x(), r.y(), r.z()); }


bool coloradar::internal::parseBoolYamlKey(const YAML::Node &nodeValue, bool defaultValue) {
    if (!nodeValue) {
        return defaultValue;
    }
    try {
        return nodeValue.as<bool>();
    } catch (const YAML::BadConversion &) {
        throw std::runtime_error("Malformed bool value");
    }
}

int coloradar::internal::parseIntYamlKey(const YAML::Node &nodeValue, int defaultValue) {
    if (!nodeValue) {
        return defaultValue;
    }
    try {
        return nodeValue.as<int>();
    } catch (const YAML::BadConversion &) {
        throw std::runtime_error("Malformed int value");
    }
}

float coloradar::internal::parseFloatYamlKey(const YAML::Node &nodeValue, float defaultValue) {
    if (!nodeValue) {
        return defaultValue;
    }
    try {
        return nodeValue.as<float>();
    } catch (const YAML::BadConversion &) {
        throw std::runtime_error("Malformed float value");
    }
}
