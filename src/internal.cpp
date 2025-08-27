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


std::vector<double> readH5Timestamps(const H5::H5File& file, const std::string& datasetName) {
    return readH5Vector1D<double>(file, datasetName);
}

std::vector<Eigen::Affine3f> readH5Poses(const H5::H5File& file, const std::string& datasetName) {
    size_t rows=0, cols=0;
    auto flat = readH5Matrix2D<float>(file, datasetName, rows, cols);
    if (cols != 7) {
        throw std::runtime_error(datasetName + ": expected pose rows of length 7 [x y z qx qy qz qw]");
    }
    std::vector<Eigen::Affine3f> out;
    out.reserve(rows);
    for (size_t i = 0; i < rows; ++i) {
        const float* p = &flat[i * 7];
        Eigen::Quaternionf q(p[6], p[3], p[4], p[5]); // (w, x, y, z)
        Eigen::Affine3f A = Eigen::Affine3f::Identity();
        A.linear() = q.normalized().toRotationMatrix();
        A.translation() = Eigen::Vector3f(p[0], p[1], p[2]);
        out.push_back(A);
    }
    return out;
}

std::vector<std::shared_ptr<std::vector<int16_t>>>
readH5Datacubes(const H5::H5File& file, const std::string& datasetName) {
    size_t rows=0, cols=0;
    // Writer used INT16 data with a FLOAT declared type; HDF5 will convert on read.
    auto flat = readH5Matrix2D<int16_t>(file, datasetName, rows, cols);
    std::vector<std::shared_ptr<std::vector<int16_t>>> frames;
    frames.reserve(rows);
    for (size_t i = 0; i < rows; ++i) {
        auto vec = std::make_shared<std::vector<int16_t>>(cols);
        std::copy_n(flat.data() + i*cols, cols, vec->data());
        frames.push_back(std::move(vec));
    }
    return frames;
}

static inline std::pair<std::vector<float>, std::vector<hsize_t>>
readAllDimsFloat(const H5::H5File& file, const std::string& name) {
    H5::DataSet ds = file.openDataSet(name);
    H5::DataSpace sp = ds.getSpace();

    int rank = sp.getSimpleExtentNdims();
    if (rank < 1) throw std::runtime_error(name + ": expected rank >= 1");

    std::vector<hsize_t> dims(static_cast<size_t>(rank));
    sp.getSimpleExtentDims(dims.data());

    size_t total = 1;
    for (auto d : dims) total *= static_cast<size_t>(d);

    std::vector<float> flat(total);
    if (total) ds.read(flat.data(), H5::PredType::NATIVE_FLOAT);
    return {std::move(flat), std::move(dims)};
}

std::vector<std::shared_ptr<std::vector<float>>>
readH5Heatmaps(const H5::H5File& file, const std::string& datasetName) {
    auto [flat, dims] = readAllDimsFloat(file, datasetName);
    if (dims.empty()) return {};

    const size_t numFrames = static_cast<size_t>(dims[0]);
    size_t perFrame = 1;
    for (size_t i = 1; i < dims.size(); ++i) perFrame *= static_cast<size_t>(dims[i]);

    if (perFrame == 0) return {};

    std::vector<std::shared_ptr<std::vector<float>>> frames;
    frames.reserve(numFrames);
    for (size_t i = 0; i < numFrames; ++i) {
        auto vec = std::make_shared<std::vector<float>>(perFrame);
        std::copy_n(flat.data() + i*perFrame, perFrame, vec->data());
        frames.push_back(std::move(vec));
    }
    return frames;
}

static inline std::vector<hsize_t>
readSizes1D(const H5::H5File& file, const std::string& name) {
    return readH5Vector1D<hsize_t>(file, name);
}

std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>
readH5LidarClouds(const H5::H5File& file, const std::string& baseName) {
    size_t rows=0, D=0;
    auto flat = readH5Matrix2D<float>(file, baseName, rows, D);
    if (!(D == 3 || D == 4)) {
        throw std::runtime_error(baseName + ": expected 3 or 4 columns (XYZ or XYZ+I)");
    }
    auto sizes = readSizes1D(file, baseName + "_sizes");
    size_t totalPoints = std::accumulate(sizes.begin(), sizes.end(), static_cast<size_t>(0));
    if (totalPoints != rows) {
        throw std::runtime_error(baseName + ": total points mismatch with sizes array");
    }

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> out;
    out.reserve(sizes.size());

    size_t off = 0;
    for (hsize_t n : sizes) {
        auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        cloud->resize(static_cast<size_t>(n));
        for (size_t k = 0; k < static_cast<size_t>(n); ++k) {
            pcl::PointXYZI p{};
            p.x = flat[(off+k)*D + 0];
            p.y = flat[(off+k)*D + 1];
            p.z = (D >= 3) ? flat[(off+k)*D + 2] : 0.f;
            p.intensity = (D >= 4) ? flat[(off+k)*D + 3] : 0.f;
            (*cloud)[k] = p;
        }
        off += static_cast<size_t>(n);
        out.push_back(std::move(cloud));
    }
    return out;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr
readH5SingleCloud(const H5::H5File& file, const std::string& datasetName) {
    size_t rows=0, D=0;
    auto flat = readH5Matrix2D<float>(file, datasetName, rows, D);
    if (!(D == 3 || D == 4)) {
        throw std::runtime_error(datasetName + ": expected 3 or 4 columns (XYZ or XYZ+I)");
    }
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    cloud->resize(rows);
    for (size_t i = 0; i < rows; ++i) {
        pcl::PointXYZI p{};
        p.x = flat[i*D + 0];
        p.y = flat[i*D + 1];
        p.z = (D >= 3) ? flat[i*D + 2] : 0.f;
        p.intensity = (D >= 4) ? flat[i*D + 3] : 0.f;
        (*cloud)[i] = p;
    }
    return cloud;
}


}
