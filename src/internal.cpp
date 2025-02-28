#include "internal.h"


void coloradar::internal::checkPathExists(const std::filesystem::path& path) {
    if (!std::filesystem::exists(path)) {
         throw std::filesystem::filesystem_error("Failed to open file", path, std::make_error_code(std::errc::no_such_file_or_directory));
    }
}
void coloradar::internal::createDirectoryIfNotExists(const std::filesystem::path& dirPath) {
    if (!std::filesystem::exists(dirPath)) {
        std::filesystem::create_directories(dirPath);
    }
}

template<> octomath::Vector3 coloradar::internal::fromEigenTrans(const Eigen::Vector3f& r) { return octomath::Vector3(r.x(), r.y(), r.z()); }
template<> octomath::Quaternion coloradar::internal::fromEigenQuat(const Eigen::Quaternionf& r) { return octomath::Quaternion(r.w(), r.x(), r.y(), r.z()); }

Eigen::Vector3f coloradar::internal::sphericalToCartesian(const double& azimuth, const double& elevation, const double& range) {
    float x = range * cos(elevation) * cos(azimuth);
    float y = range * cos(elevation) * sin(azimuth);
    float z = range * sin(elevation);
    return Eigen::Vector3f(x, y, z);
}


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
