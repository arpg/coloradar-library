#include "internal.h"


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


template<> octomath::Vector3 coloradar::internal::fromEigenTrans(const Eigen::Vector3f& r) { return octomath::Vector3(r.x(), r.y(), r.z()); }
template<> octomath::Quaternion coloradar::internal::fromEigenQuat(const Eigen::Quaternionf& r) { return octomath::Quaternion(r.w(), r.x(), r.y(), r.z()); }

// Eigen::Vector3f coloradar::internal::sphericalToCartesian(const double& azimuth, const double& elevation, const double& range) {
//     float x = range * cos(elevation) * cos(azimuth);
//     float y = range * cos(elevation) * sin(azimuth);
//     float z = range * sin(elevation);
//     return Eigen::Vector3f(x, y, z);
// }


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



namespace coloradar::internal {

std::string toLower(std::string s) {
    for (auto& c : s) c = std::tolower(c);
    return s;
}


// SMALL DATA

void saveVectorToHDF5(const std::string& name, const H5::H5File& file, const std::vector<double>& vec) {
    hsize_t dims[1] = { vec.size() };
    H5::DataSpace dataspace(1, dims);
    H5::DataSet dataset = file.createDataSet(name, H5::PredType::NATIVE_DOUBLE, dataspace);
}

void savePosesToHDF5(const std::string& name, const H5::H5File& file, const std::vector<Eigen::Affine3f>& poses) {
    hsize_t dims[2] = { poses.size(), 7 };
    H5::DataSpace dataspace(2, dims);
    H5::DataSet dataset = file.createDataSet(name, H5::PredType::NATIVE_DOUBLE, dataspace);
    std::vector<float> poseData;
    poseData.reserve(poses.size() * 7);
    for (const auto& pose : poses) {
        poseData.push_back(pose.translation().x());
        poseData.push_back(pose.translation().y());
        poseData.push_back(pose.translation().z());
        Eigen::Quaternionf rot(pose.rotation());
        poseData.push_back(rot.x());
        poseData.push_back(rot.y());
        poseData.push_back(rot.z());
        poseData.push_back(rot.w());
    }
    dataset.write(poseData.data(), H5::PredType::NATIVE_FLOAT);
}


// CLOUD DATA

void saveCloudToHDF5(const std::string& name, const H5::H5File& file, const std::vector<float>& flatCloud, const hsize_t& numDims) {
    hsize_t numPoints = flatCloud.size() / numDims;
    hsize_t dims[2] = {numPoints, numDims};
    H5::DataSpace dataspace(2, dims);
    H5::PredType datatype = H5::PredType::NATIVE_FLOAT;
    H5::DataSet dataset = file.createDataSet(name, datatype, dataspace);
    if (numPoints > 0) {
        dataset.write(flatCloud.data(), H5::PredType::NATIVE_FLOAT);
    }
}

void saveCloudsToHDF5(const std::string& name, const H5::H5File& file, const std::vector<float>& flatClouds, const hsize_t& numFrames, const std::vector<hsize_t>& cloudSizes, const hsize_t& numDims) {
    if (cloudSizes.size() != numFrames) {
        throw std::invalid_argument("cloudSizes size must match the number of frames.");
    }
    std::string sizeDatasetName = name + "_sizes";
    hsize_t sizeDims[1] = {numFrames};
    H5::DataSpace sizeDataspace(1, sizeDims);
    H5::DataSet sizeDataset = file.createDataSet(sizeDatasetName, H5::PredType::NATIVE_HSIZE, sizeDataspace);
    sizeDataset.write(cloudSizes.data(), H5::PredType::NATIVE_HSIZE);

    hsize_t totalPoints = std::accumulate(cloudSizes.begin(), cloudSizes.end(), static_cast<hsize_t>(0));
    hsize_t cloudDims[2] = {totalPoints, static_cast<hsize_t>(numDims)};
    H5::DataSpace cloudDataspace(2, cloudDims);
    H5::DataSet cloudDataset = file.createDataSet(name, H5::PredType::NATIVE_FLOAT, cloudDataspace);
    cloudDataset.write(flatClouds.data(), H5::PredType::NATIVE_FLOAT);
}


// RADAR-SPECIFIC DATA

void saveDatacubesToHDF5(const std::string& name, const H5::H5File& file, const std::vector<int16_t>& flatDatacubes, const hsize_t& numFrames, const hsize_t datacubeSize) {
    std::vector<hsize_t> dims = {numFrames, datacubeSize};
    H5::DataSpace dataspace(dims.size(), dims.data());
    H5::PredType datatype = H5::PredType::NATIVE_FLOAT;
    H5::DataSet dataset = file.createDataSet(name, datatype, dataspace);
    dataset.write(flatDatacubes.data(), H5::PredType::NATIVE_INT16);
}

void saveHeatmapsToHDF5(const std::string& name, const H5::H5File& file, const std::vector<float>& flatHeatmaps, const hsize_t& numFrames, const int numAzimuthBins, const int numRangeBins, const int numElevationBins, const bool hasDoppler) {
    std::vector<hsize_t> dims = {numFrames};
    if (numElevationBins > 1) {
        dims.push_back(static_cast<hsize_t>(numElevationBins));
    }
    dims.push_back(static_cast<hsize_t>(numAzimuthBins));
    dims.push_back(static_cast<hsize_t>(numRangeBins));
    if (hasDoppler) {
        dims.push_back(2);
    }
    H5::DataSpace dataspace(dims.size(), dims.data());
    H5::PredType datatype = H5::PredType::NATIVE_FLOAT;
    H5::DataSet dataset = file.createDataSet(name, datatype, dataspace);
    dataset.write(flatHeatmaps.data(), H5::PredType::NATIVE_FLOAT);
}


}
