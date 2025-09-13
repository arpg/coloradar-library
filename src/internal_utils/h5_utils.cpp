#include "internal_utils/h5_utils.h"   


namespace coloradar::internal {



void savePoseToHDF5(const std::string& name, H5::H5File& file, const Eigen::Affine3f& pose) {
    const hsize_t dims[1] = { 7 };
    H5::DataSpace space(1, dims);
    H5::DataSet ds = file.createDataSet(name, H5::PredType::NATIVE_FLOAT, space);
    float buf[7];
    const Eigen::Vector3f t = pose.translation();
    const Eigen::Quaternionf q(pose.rotation());
    buf[0] = t.x(); buf[1] = t.y(); buf[2] = t.z();
    buf[3] = q.x(); buf[4] = q.y(); buf[5] = q.z(); buf[6] = q.w();
    ds.write(buf, H5::PredType::NATIVE_FLOAT);
    ds.close();
    space.close();
}


void savePosesToHDF5(const std::string& name, H5::H5File& file, const std::vector<Eigen::Affine3f>& poses) {
    const hsize_t dims[2] = { poses.size(), 7 };
    H5::DataSpace space(2, dims);
    H5::DataSet ds = file.createDataSet(name, H5::PredType::NATIVE_FLOAT, space);
    std::vector<float> buf(poses.size() * 7);
    size_t idx = 0;
    for (const auto& p : poses) {
        const Eigen::Vector3f t = p.translation();
        const Eigen::Quaternionf q(p.rotation());
        buf[idx++] = t.x();
        buf[idx++] = t.y();
        buf[idx++] = t.z();
        buf[idx++] = q.x();
        buf[idx++] = q.y();
        buf[idx++] = q.z();
        buf[idx++] = q.w();
    }
    ds.write(buf.data(), H5::PredType::NATIVE_FLOAT);
    ds.close();
    space.close();
}


void saveCloudToHDF5(const std::string& name, H5::H5File& file, const std::vector<float>& flatCloud, hsize_t numDims) {
    if (numDims == 0) throw std::invalid_argument("saveCloudToHDF5(): expected numDims > 0, got " + std::to_string(numDims));
    if (flatCloud.size() % numDims != 0) throw std::invalid_argument("saveCloudToHDF5(): flatCloud.size() must be divisible by numDims, got " + std::to_string(flatCloud.size()) + " and " + std::to_string(numDims));
    const hsize_t numPoints = flatCloud.size() / numDims;
    const hsize_t dims[2] = { numPoints, numDims };
    H5::DataSpace space(2, dims);
    H5::DataSet ds = file.createDataSet(name, H5::PredType::NATIVE_FLOAT, space);
    if (numPoints > 0) ds.write(flatCloud.data(), H5::PredType::NATIVE_FLOAT);
    ds.close();
    space.close();
}


void saveCloudsToHDF5(const std::string& name, H5::H5File& file, const std::vector<float>& flatClouds, hsize_t numFrames, const std::vector<hsize_t>& cloudSizes, hsize_t numDims) {
    if (cloudSizes.size() != numFrames) throw std::invalid_argument("saveCloudsToHDF5(): expected cloudSizes.size() == numFrames, got " + std::to_string(cloudSizes.size()) + " != " + std::to_string(numFrames));
    if (numDims == 0) throw std::invalid_argument("saveCloudsToHDF5(): expected numDims > 0, got " + std::to_string(numDims));
    std::string sizeDatasetName = name + "_sizes";
    hsize_t sizeDims[1] = {numFrames};
    H5::DataSpace sizeDataspace(1, sizeDims);
    H5::DataSet sizeDataset = file.createDataSet(sizeDatasetName, H5::PredType::NATIVE_HSIZE, sizeDataspace);
    sizeDataset.write(cloudSizes.data(), H5::PredType::NATIVE_HSIZE);

    hsize_t totalPoints = std::accumulate(cloudSizes.begin(), cloudSizes.end(), hsize_t(0));
    hsize_t cloudDims[2] = {totalPoints, numDims};
    H5::DataSpace cloudSpace(2, cloudDims);
    H5::DataSet cloudDataset = file.createDataSet(name, H5::PredType::NATIVE_FLOAT, cloudSpace);
    if (!flatClouds.empty()) cloudDataset.write(flatClouds.data(), H5::PredType::NATIVE_FLOAT);
    cloudDataset.close();
    cloudSpace.close();
}


void saveDatacubesToHDF5(const std::string& name, H5::H5File& file, const std::vector<int16_t>& flatDatacubes, hsize_t numFrames, hsize_t datacubeSize) {
    std::vector<hsize_t> dims = {numFrames, datacubeSize};
    H5::DataSpace dataspace(dims.size(), dims.data());
    H5::PredType datatype = H5::PredType::NATIVE_FLOAT;
    H5::DataSet dataset = file.createDataSet(name, datatype, dataspace);
    dataset.write(flatDatacubes.data(), H5::PredType::NATIVE_INT16);
    dataset.close();
    dataspace.close();
}


void saveHeatmapsToHDF5(const std::string& name, H5::H5File& file, const std::vector<float>& flatHeatmaps, hsize_t numFrames, int numAzimuthBins, int numRangeBins, int numElevationBins, bool hasDoppler) {
    std::vector<hsize_t> dims = {numFrames};
    if (numElevationBins > 1) dims.push_back(numElevationBins);
    dims.push_back(numAzimuthBins);
    dims.push_back(numRangeBins);
    if (hasDoppler) dims.push_back(2);

    H5::DataSpace space(dims.size(), dims.data());
    H5::DataSet dataset = file.createDataSet(name, H5::PredType::NATIVE_FLOAT, space);
    if (!flatHeatmaps.empty()) dataset.write(flatHeatmaps.data(), H5::PredType::NATIVE_FLOAT);
    dataset.close();
    space.close();
}



std::vector<double> readH5Timestamps(const H5::H5File& file, const std::string& datasetName) {
    return readH5Vector1D<double>(file, datasetName);
}


Eigen::Affine3f readH5Pose(const H5::H5File& file, const std::string& datasetName) {
    H5::DataSet ds = file.openDataSet(datasetName);
    H5::DataSpace sp = ds.getSpace();
    int rank = sp.getSimpleExtentNdims();
    if (rank != 1) throw std::runtime_error(datasetName + ": expected rank-1 dataset (length 7).");

    hsize_t dim = 0;
    sp.getSimpleExtentDims(&dim, nullptr);
    if (dim != 7) throw std::runtime_error(datasetName + ": expected 7 elements [x y z qx qy qz qw], got " + std::to_string(dim));

    std::array<float, 7> poseData{};
    ds.read(poseData.data(), H5::PredType::NATIVE_FLOAT);
    Eigen::Vector3f translation(poseData[0], poseData[1], poseData[2]);
    Eigen::Quaternionf rotation(poseData[6], poseData[3], poseData[4], poseData[5]); // (w, x, y, z)
    Eigen::Affine3f pose = Eigen::Translation3f(translation) * rotation;
    return pose;
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
        Eigen::Vector3f translation(p[0], p[1], p[2]);
        Eigen::Quaternionf rotation(p[6], p[3], p[4], p[5]);
        Eigen::Affine3f pose = Eigen::Translation3f(translation) * rotation;
        out.push_back(pose);
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

std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> readH5LidarClouds(const H5::H5File& file, const std::string& baseName) {
    size_t rows=0, D=0;
    auto flat = readH5Matrix2D<float>(file, baseName, rows, D);
    if (D < 3) throw std::runtime_error("readH5LidarClouds(): expected 3 or more columns (XYZ or XYZ+I) in dataset " + baseName);

    auto sizes = readSizes1D(file, baseName + "_sizes");
    size_t totalPoints = std::accumulate(sizes.begin(), sizes.end(), static_cast<size_t>(0));
    if (totalPoints != rows) throw std::runtime_error("readH5LidarClouds(): total points mismatch with sizes array in dataset " + baseName);

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
        out.push_back(cloud);
    }
    return out;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr readH5SingleCloud(const H5::H5File& file, const std::string& datasetName) {
    size_t rows=0, D=0;
    auto flat = readH5Matrix2D<float>(file, datasetName, rows, D);
    if (D < 3) throw std::runtime_error("readH5SingleCloud(): expected 3 or more columns (XYZ or XYZ+I) in dataset " + datasetName);
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

// std::vector<pcl::PointCloud<RadarPoint>::Ptr> readH5RadarClouds(const H5::H5File& file, const std::string& datasetName) {
//     size_t rows=0, D=0;
//     auto flat = readH5Matrix2D<float>(file, datasetName, rows, D);
//     if (D!=3 && D != 4 && D != 5) throw std::runtime_error(datasetName + ": expected 3, 4, or 5 columns (XYZ, XYZ+I, or XYZ+I+D)");

//     auto sizes = readSizes1D(file, datasetName + "_sizes");
//     size_t totalPoints = std::accumulate(sizes.begin(), sizes.end(), static_cast<size_t>(0));
//     if (totalPoints != rows) throw std::runtime_error(datasetName + ": total points mismatch with sizes array");

//     std::vector<pcl::PointCloud<RadarPoint>::Ptr> out;
//     out.reserve(sizes.size());
//     size_t off = 0;
//     for (hsize_t n : sizes) 
//     {
//         auto cloud = std::make_shared<pcl::PointCloud<RadarPoint>>();
//         cloud->resize(static_cast<size_t>(n));
//         for (size_t k = 0; k < static_cast<size_t>(n); ++k) {
//             RadarPoint p{};
//             p.x = flat[(off+k)*D + 0];
//             p.y = flat[(off+k)*D + 1];
//             p.z = (D >= 3) ? flat[(off+k)*D + 2] : 0.f;
//             p.intensity = (D >= 4) ? flat[(off+k)*D + 3] : 0.f;
//             p.doppler = (D >= 5) ? flat[(off+k)*D + 4] : 0.f;
//             (*cloud)[k] = p;
//         }
//         off += static_cast<size_t>(n);
//         out.push_back(cloud);
//     }
//     return out;
// }


}
