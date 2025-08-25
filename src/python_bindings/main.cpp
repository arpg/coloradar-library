#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/complex.h>
#include <pybind11/stl/filesystem.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/stl_bind.h>
#include "coloradar_tools.h"


namespace py = pybind11;


PYBIND11_MAKE_OPAQUE(pcl::PointCloud<pcl::PointXYZI>);
PYBIND11_MAKE_OPAQUE(std::vector<coloradar::RadarPoint>);


bool isNumpyArrayEmpty(const py::array_t<float>& array) {
    if (array.is_none()) {
        return true;
    }
    if (array.ndim() != 2) {
        return true;
    }
    auto buffer = array.unchecked<2>();
    if (buffer.shape(0) == 0 || buffer.shape(1) == 0) {
        return true;
    }
    for (ssize_t i = 0; i < buffer.shape(0); ++i) {
        for (ssize_t j = 0; j < buffer.shape(1); ++j) {
            if (std::isnan(buffer(i, j))) {
                return true;
            }
        }
    }
    return false;
}

py::array_t<float> poseToNumpy(const Eigen::Affine3f& pose) {
    py::array_t<float> result(py::array_t<float>::ShapeContainer({static_cast<py::ssize_t>(7)}));
    auto result_buffer = result.mutable_unchecked<1>();
    result_buffer(0) = pose.translation().x();
    result_buffer(1) = pose.translation().y();
    result_buffer(2) = pose.translation().z();
    Eigen::Quaternionf quaternion(pose.rotation());
    result_buffer(3) = quaternion.x();
    result_buffer(4) = quaternion.y();
    result_buffer(5) = quaternion.z();
    result_buffer(6) = quaternion.w();
    return result;
}

py::array_t<float> posesToNumpy(const std::vector<Eigen::Affine3f>& poses) {
    py::array_t<float> result(py::array_t<float>::ShapeContainer({static_cast<long int>(poses.size()), 7}));
    auto result_buffer = result.mutable_unchecked<2>();
    for (size_t i = 0; i < poses.size(); ++i) {
        const auto& pose = poses[i];
        result_buffer(i, 0) = pose.translation().x();
        result_buffer(i, 1) = pose.translation().y();
        result_buffer(i, 2) = pose.translation().z();
        Eigen::Quaternionf quaternion(pose.rotation());
        result_buffer(i, 3) = quaternion.x();
        result_buffer(i, 4) = quaternion.y();
        result_buffer(i, 5) = quaternion.z();
        result_buffer(i, 6) = quaternion.w();
    }
    return result;
}

Eigen::Affine3f numpyToPose(const py::array_t<float>& array) {
    if (array.ndim() != 1 || array.shape(0) != 7) {
        throw std::runtime_error("Input array must have shape (7,)");
    }
    auto array_data = array.unchecked<1>();
    Eigen::Vector3f translation(array_data(0), array_data(1), array_data(2));
    Eigen::Quaternionf rotation(array_data(6), array_data(3), array_data(4), array_data(5));
    Eigen::Affine3f pose = Eigen::Translation3f(translation) * rotation;
    return pose;
}

std::vector<Eigen::Affine3f> numpyToPoses(const py::array_t<float>& array) {
    if (array.ndim() != 2 || array.shape(1) != 7) {
        throw std::runtime_error("Input array must have shape (N, 7)");
    }
    std::vector<Eigen::Affine3f> poses;
    poses.reserve(array.shape(0));
    auto array_data = array.unchecked<2>();
    for (ssize_t i = 0; i < array.shape(0); ++i) {
        Eigen::Vector3f translation(array_data(i, 0), array_data(i, 1), array_data(i, 2));
        Eigen::Quaternionf rotation(array_data(i, 6), array_data(i, 3), array_data(i, 4), array_data(i, 5));
        Eigen::Affine3f pose = Eigen::Translation3f(translation) * rotation;
        poses.push_back(pose);
    }
    return poses;
}


py::array_t<float> radarCloudToNumpy(const pcl::PointCloud<coloradar::RadarPoint>::Ptr& cloud) {
    py::array_t<float>::ShapeContainer shape({static_cast<long int>(cloud->points.size()), 5});
    py::array_t<float> result(shape);
    auto result_buffer = result.mutable_unchecked<2>();
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        const auto& point = cloud->points[i];
        result_buffer(i, 0) = point.x;
        result_buffer(i, 1) = point.y;
        result_buffer(i, 2) = point.z;
        result_buffer(i, 3) = point.intensity;
        result_buffer(i, 4) = point.doppler;
    }
    return result;
}


py::array_t<float> pointcloudToNumpy(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    py::array_t<float>::ShapeContainer shape({static_cast<long int>(cloud->points.size()), 4});
    py::array_t<float> result(shape);
    auto result_buffer = result.mutable_unchecked<2>();
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        const auto& point = cloud->points[i];
        result_buffer(i, 0) = point.x;
        result_buffer(i, 1) = point.y;
        result_buffer(i, 2) = point.z;
        result_buffer(i, 3) = point.intensity;
    }
    return result;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr numpyToPointcloud(const py::array_t<float>& array) {
    if (isNumpyArrayEmpty(array)) {
        throw std::runtime_error("Input NumPy array is empty or invalid.");
    }
    if (array.ndim() != 2 || array.shape(1) != 4) {
        throw std::runtime_error("Input NumPy array must have shape (N, 4) where N is the number of points.");
    }
    auto array_data = array.unchecked<2>();
    auto cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    cloud->points.reserve(array.shape(0));
    for (ssize_t i = 0; i < array.shape(0); ++i) {
        pcl::PointXYZI point;
        point.x = array_data(i, 0);
        point.y = array_data(i, 1);
        point.z = array_data(i, 2);
        point.intensity = array_data(i, 3);
        cloud->points.push_back(point);
    }
    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = false;
    return cloud;
}


template<typename T>
py::array_t<T> vectorToNumpy(const std::vector<T>& vec) {
    typename py::array_t<T>::ShapeContainer shape({static_cast<long int>(vec.size())});
    py::array_t<T> result(shape);
    auto result_buffer = result.template mutable_unchecked<1>();
    for (size_t i = 0; i < vec.size(); ++i) {
        result_buffer(i) = vec[i];
    }
    return result;
}

template <typename T>
std::vector<T> numpyToVector(const py::array_t<T>& array) {
    auto buffer = array.template unchecked<1>();
    std::vector<T> vec(buffer.shape(0));
    for (size_t i = 0; i < buffer.shape(0); ++i) {
        vec[i] = buffer(i);
    }
    return vec;
}


pcl::PointCloud<coloradar::RadarPoint>::Ptr heatmapToPointcloudBinding(
    const py::array_t<float>& heatmap_array,
    coloradar::RadarConfig& config,
    float intensityThreshold
) {
    auto buf = heatmap_array.request();
    if (buf.ndim != 1) {
        throw std::runtime_error("Heatmap array must be 1-dimensional");
    }
    const size_t expected_size = static_cast<size_t>(config.numElevationBins * config.numAzimuthBins * config.nRangeBins() * 2);
    if (buf.size != expected_size) throw std::runtime_error("Heatmap size mismatch: expected " + std::to_string(expected_size) + ", got " + std::to_string(buf.size));

    const float* data_ptr = static_cast<const float*>(buf.ptr);
    std::vector<float> heatmap(data_ptr, data_ptr + buf.size);
    return config.heatmapToPointcloud(heatmap, intensityThreshold);
}



PYBIND11_MODULE(coloradar_dataset_lib, m) {
    py::class_<pcl::PointXYZI>(m, "PointXYZI")
        .def(py::init<>())
        .def_readwrite("x", &pcl::PointXYZI::x)
        .def_readwrite("y", &pcl::PointXYZI::y)
        .def_readwrite("z", &pcl::PointXYZI::z)
        .def_readwrite("intensity", &pcl::PointXYZI::intensity);

    py::class_<pcl::PointCloud<pcl::PointXYZI>>(m, "Pointcloud4d")
        .def(py::init<>())
        .def_readwrite("points", &pcl::PointCloud<pcl::PointXYZI>::points)
        .def("width", [](const pcl::PointCloud<pcl::PointXYZI>& pc) { return pc.width; })
        .def("height", [](const pcl::PointCloud<pcl::PointXYZI>& pc) { return pc.height; })
        .def("is_dense", [](const pcl::PointCloud<pcl::PointXYZI>& pc) { return pc.is_dense; });

    m.def("find_closest_timestamp_index", &coloradar::findClosestTimestampIndex,
          py::arg("target_timestamp"),
          py::arg("timestamps"),
          py::arg("preference") = "none",
          "Finds the closest timestamp index in a list with optional direction constraints."
    );

    m.def("interpolate_poses", [](const py::array_t<float>& poses_array,
                                  const std::vector<double>& pose_timestamps,
                                  const std::vector<double>& target_timestamps) {
        std::vector<Eigen::Affine3f> poses = numpyToPoses(poses_array);
        std::vector<Eigen::Affine3f> result = coloradar::interpolatePoses<Eigen::Affine3f>(
            poses, pose_timestamps, target_timestamps);
        return posesToNumpy(result);
    }, py::arg("poses"), py::arg("pose_timestamps"), py::arg("target_timestamps"), 
       "Interpolates poses to match new timestamps. Expects input shape (N, 7) [x, y, z, qx, qy, qz, qw]."
    );


    // RadarConfig
    using RC = coloradar::RadarConfig;

    py::class_<RC, std::shared_ptr<RC>>(m, "RadarConfig")
        .def_readonly("num_elevation_bins", &RC::numElevationBins)
        .def_readonly("num_azimuth_bins",   &RC::numAzimuthBins)
        .def_readonly("range_bin_width",    &RC::rangeBinWidth)
        .def_readonly("azimuth_bins",       &RC::azimuthBins)
        .def_readonly("elevation_bins",     &RC::elevationBins)
        .def_readonly("design_frequency",   &RC::designFrequency)
        .def_readonly("num_tx_antennas",    &RC::numTxAntennas)
        .def_readonly("num_rx_antennas",    &RC::numRxAntennas)
        .def_readonly("tx_centers",         &RC::txCenters)
        .def_readonly("rx_centers",         &RC::rxCenters)
        .def_readonly("num_adc_samples_per_chirp", &RC::numAdcSamplesPerChirp)
        .def_readonly("num_chirps_per_frame",      &RC::numChirpsPerFrame)
        .def_readonly("adc_sample_frequency",      &RC::adcSampleFrequency)
        .def_readonly("start_frequency",           &RC::startFrequency)
        .def_readonly("idle_time",                 &RC::idleTime)
        .def_readonly("adc_start_time",            &RC::adcStartTime)
        .def_readonly("ramp_end_time",             &RC::rampEndTime)
        .def_readonly("frequency_slope",           &RC::frequencySlope)
        .def_readonly("num_doppler_bins",          &RC::numDopplerBins)
        .def_readonly("coupling_calib_matrix",     &RC::couplingCalibMatrix)
        .def_readonly("calib_adc_sample_frequency",&RC::calibAdcSampleFrequency)
        .def_readonly("calib_frequency_slope",     &RC::calibFrequencySlope)
        .def_readonly("frequency_calib_matrix",    &RC::frequencyCalibMatrix)
        .def_readonly("phase_calib_matrix",        &RC::phaseCalibMatrix)
        .def_readonly("num_azimuth_beams",         &RC::numAzimuthBeams)
        .def_readonly("num_elevation_beams",       &RC::numElevationBeams)
        .def_readonly("azimuth_aperture_len",      &RC::azimuthApertureLen)
        .def_readonly("elevation_aperture_len",    &RC::elevationApertureLen)
        .def_readonly("num_angles",                &RC::numAngles)
        .def_readonly("num_virtual_elements",      &RC::numVirtualElements)
        .def_readonly("virtual_array_map",         &RC::virtualArrayMap)
        .def_readonly("azimuth_angles",            &RC::azimuthAngles)
        .def_readonly("elevation_angles",          &RC::elevationAngles)
        .def_readonly("doppler_bin_width",         &RC::dopplerBinWidth)

        .def("n_range_bins", &RC::nRangeBins)
        .def("max_range",    &RC::maxRange)
        .def("to_json",      &RC::toJson)
        .def("from_json", static_cast<void (RC::*)(const std::string&)>(&RC::fromJson), py::arg("json_string"))
        .def("clip_azimuth_max_bin",        &RC::clipAzimuthMaxBin,        py::arg("az_max_bin"))
        .def("clip_elevation_max_bin",      &RC::clipElevationMaxBin,      py::arg("el_max_bin"))
        .def("clip_range_max_bin",          &RC::clipRangeMaxBin,          py::arg("range_max_bin"))
        .def("clip_range",                  &RC::clipRange,                py::arg("range"))
        .def("azimuth_idx_to_fov_degrees",  &RC::azimuthIdxToFovDegrees,   py::arg("az_max_bin"))
        .def("elevation_idx_to_fov_degrees",&RC::elevationIdxToFovDegrees, py::arg("el_max_bin"))
        .def("range_idx_to_range",          &RC::rangeIdxToRange,          py::arg("range_max_bin"))
        .def("horizontal_fov_to_azimuth_idx",&RC::horizontalFovToAzimuthIdx, py::arg("horizontal_fov"))
        .def("vertical_fov_to_elevation_idx",&RC::verticalFovToElevationIdx, py::arg("vertical_fov"))
        .def("range_to_range_idx",          &RC::rangeToRangeIdx,          py::arg("range"))

        .def("clip_heatmap", [](coloradar::RadarConfig& self, const std::vector<float>& heatmap, int azimuth_max_bin, int elevation_max_bin, int range_max_bin, bool update_config) {
            auto clipped = self.clipHeatmap(heatmap, azimuth_max_bin, elevation_max_bin, range_max_bin, update_config);
            return vectorToNumpy(clipped);
        }, py::arg("heatmap"), py::arg("azimuth_max_bin"), py::arg("elevation_max_bin"), py::arg("range_max_bin"), py::arg("update_config") = true)

        .def("clip_heatmap", [](coloradar::RadarConfig& self, const std::vector<float>& heatmap, float horizontal_fov, float vertical_fov, float range, bool update_config) {
            auto clipped = self.clipHeatmap(heatmap, horizontal_fov, vertical_fov, range, update_config);
            return vectorToNumpy(clipped);
        }, py::arg("heatmap"), py::arg("horizontal_fov"), py::arg("vertical_fov"), py::arg("range"), py::arg("update_config") = true)

        .def("collapse_heatmap_elevation", [](coloradar::RadarConfig& self, const std::vector<float>& image, double elevation_min_meters, double elevation_max_meters, bool update_config) {
            auto result = self.collapseHeatmapElevation(image, elevation_min_meters, elevation_max_meters, update_config);
            return vectorToNumpy(result);
        }, py::arg("image"), 
           py::arg("elevation_min_meters") = -100.0, 
           py::arg("elevation_max_meters") = 100.0, 
           py::arg("update_config") = true
        )

        .def("remove_doppler", [](coloradar::RadarConfig& self, const std::vector<float>& image, bool update_config) {
            auto result = self.removeDoppler(image, update_config);
            return vectorToNumpy(result);
        }, py::arg("image"), 
           py::arg("update_config") = true
        )

        .def("swap_heatmap_dimensions", [](coloradar::RadarConfig& self, const std::vector<float>& heatmap) {
            auto result = self.swapHeatmapDimensions(heatmap);
            return vectorToNumpy(result);
        }, py::arg("heatmap"))

        .def("heatmap_to_pointcloud", [](coloradar::RadarConfig& self, const py::array_t<float>& heatmap, float intensityThreshold = 0.0f) {
            auto cloud = heatmapToPointcloudBinding(heatmap, self, intensityThreshold);
            return radarCloudToNumpy(cloud);
        }, py::arg("heatmap"),
           py::arg("intensity_threshold") = 0.0f
    );

    // SingleChipConfig
    py::class_<coloradar::SingleChipConfig, coloradar::RadarConfig, std::shared_ptr<coloradar::SingleChipConfig>>(m, "SingleChipConfig")
        .def(py::init<const std::filesystem::path&, const int&, const int&>(), py::arg("calib_dir"), py::arg("num_azimuth_beams") = 64, py::arg("num_elevation_beams") = 8)
        .def("__copy__", [](const coloradar::SingleChipConfig &self) { return std::make_shared<coloradar::SingleChipConfig>(self); })
        .def("__deepcopy__", [](const coloradar::SingleChipConfig &self, py::dict) { return std::make_shared<coloradar::SingleChipConfig>(self); })
        .def(py::init([](const std::string& json_str) {
            Json::CharReaderBuilder b;
            std::unique_ptr<Json::CharReader> reader(b.newCharReader());
            Json::Value root; std::string errs;
            const char* begin = json_str.data(); const char* end = begin + json_str.size();
            if (!reader->parse(begin, end, &root, &errs)) {
                throw std::invalid_argument(std::string("SingleChipConfig(json_string): parse error: ") + errs);
            }
            return std::make_shared<coloradar::SingleChipConfig>(root);
        }), py::arg("json_string"));

    // CascadeConfig
    py::class_<coloradar::CascadeConfig, coloradar::RadarConfig, std::shared_ptr<coloradar::CascadeConfig>>(m, "CascadeConfig")
        .def(py::init<const std::filesystem::path&, const int&, const int&>(), py::arg("calib_dir"), py::arg("num_azimuth_beams") = 128, py::arg("num_elevation_beams") = 32)
        .def("__copy__", [](const coloradar::CascadeConfig &self) { return std::make_shared<coloradar::CascadeConfig>(self); })
        .def("__deepcopy__", [](const coloradar::CascadeConfig &self, py::dict) { return std::make_shared<coloradar::CascadeConfig>(self); })
        .def(py::init([](const std::string& json_str) {
            Json::CharReaderBuilder b;
            std::unique_ptr<Json::CharReader> reader(b.newCharReader());
            Json::Value root; std::string errs;
            const char* begin = json_str.data(); const char* end = begin + json_str.size();
            if (!reader->parse(begin, end, &root, &errs)) {
                throw std::invalid_argument(std::string("CascadeConfig(json_string): parse error: ") + errs);
            }
            return std::make_shared<coloradar::CascadeConfig>(root);
        }), py::arg("json_string"));

    // ColoradarPlusRun
    py::class_<coloradar::ColoradarPlusRun>(m, "ColoradarPlusRun")
        .def(py::init<const std::filesystem::path&, coloradar::RadarConfig*>())
        .def_readonly("name", &coloradar::ColoradarPlusRun::name)
        .def("pose_timestamps", [](coloradar::ColoradarPlusRun& self) { return vectorToNumpy(self.poseTimestamps()); })
        .def("imu_timestamps", [](coloradar::ColoradarPlusRun& self) { return vectorToNumpy(self.imuTimestamps()); })
        .def("lidar_timestamps", [](coloradar::ColoradarPlusRun& self) { return vectorToNumpy(self.lidarTimestamps()); })
        .def("cascade_cube_timestamps", [](coloradar::ColoradarPlusRun& self) { return vectorToNumpy(self.cascadeCubeTimestamps()); })
        .def("cascade_timestamps", [](coloradar::ColoradarPlusRun& self) { return vectorToNumpy(self.cascadeTimestamps()); })
        
        .def("get_lidar_pointcloud",
            static_cast<std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> (coloradar::ColoradarPlusRun::*)(const std::filesystem::path&) const>
            (&coloradar::ColoradarPlusRun::getLidarPointCloud<pcl::PointCloud<pcl::PointXYZI>>))

        .def("get_lidar_pointcloud",
            static_cast<std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> (coloradar::ColoradarPlusRun::*)(const int) const>
            (&coloradar::ColoradarPlusRun::getLidarPointCloud<pcl::PointCloud<pcl::PointXYZI>>))

        .def("get_cascade_datacube", [](coloradar::ColoradarPlusRun& self, const std::filesystem::path& binFilePath) { return vectorToNumpy(self.getCascadeDatacube(binFilePath)); })
        .def("get_cascade_datacube", [](coloradar::ColoradarPlusRun& self, const int& cubeIdx) { return vectorToNumpy(self.getCascadeDatacube(cubeIdx)); })
        .def("get_cascade_heatmap", [](coloradar::ColoradarPlusRun& self, const std::filesystem::path& binFilePath) { return vectorToNumpy(self.getCascadeHeatmap(binFilePath)); })
        .def("get_cascade_heatmap", [](coloradar::ColoradarPlusRun& self, const int& hmIdx) { return vectorToNumpy(self.getCascadeHeatmap(hmIdx)); })
        .def("create_cascade_pointclouds", &coloradar::ColoradarPlusRun::createCascadePointclouds, py::arg("intensity_threshold") = 0)
        .def("get_cascade_pointcloud", [](coloradar::ColoradarPlusRun& self, std::filesystem::path binFilePath, float intensityThreshold = 0.0f) {
            pcl::PointCloud<coloradar::RadarPoint>::Ptr cloud = self.getCascadePointcloud(binFilePath, intensityThreshold); return radarCloudToNumpy(cloud);
        }, py::arg("bin_file_path"), py::arg("intensity_threshold") = 0)
        .def("get_cascade_pointcloud", [](coloradar::ColoradarPlusRun& self, int cloudIdx, float intensityThreshold = 0.0f) {
            pcl::PointCloud<coloradar::RadarPoint>::Ptr cloud = self.getCascadePointcloud(cloudIdx, intensityThreshold); return radarCloudToNumpy(cloud);
        }, py::arg("cloud_idx"), py::arg("intensity_threshold") = 0)

        .def("create_lidar_octomap", [](coloradar::ColoradarPlusRun& self, const double mapResolution, const float lidarTotalHorizontalFov, const float lidarTotalVerticalFov, const float lidarMaxRange, const py::array_t<float>& baseToLidarTransformArray) {
            self.createLidarOctomap(mapResolution, lidarTotalHorizontalFov, lidarTotalVerticalFov, lidarMaxRange, numpyToPose(baseToLidarTransformArray));
        }, py::arg("map_resolution") = 0.5, py::arg("lidar_total_horizontal_fov") = 360, py::arg("lidar_total_vertical_fov") = 180, py::arg("lidar_max_range") = 100, py::arg("base_to_lidar_transform") = poseToNumpy(Eigen::Affine3f::Identity()))
        .def("get_lidar_octomap", [](coloradar::ColoradarPlusRun& self) { auto octree = self.readLidarOctomap(); return pointcloudToNumpy(octree); })

        .def("get_map_sample", [](coloradar::ColoradarPlusRun& self, int frameIdx) { return pointcloudToNumpy(self.readMapSample(frameIdx)); }, py::arg("frame_idx"))
        .def("sample_map_frame", [](coloradar::ColoradarPlusRun& self,
                                    float horizontalFov, float verticalFov, float range,
                                    const py::array_t<float>& mapFramePoseArray,
                                    const py::array_t<float>& mapCloudArray) {
            Eigen::Affine3f mapFramePose = numpyToPose(mapFramePoseArray);
            pcl::PointCloud<pcl::PointXYZI>::Ptr mapCloud = numpyToPointcloud(mapCloudArray);
            auto sampledFrame = self.sampleMapFrame(horizontalFov, verticalFov, range, mapFramePose, mapCloud);
            return pointcloudToNumpy(sampledFrame);
        }, py::arg("horizontal_fov") = 360, py::arg("vertical_fov") = 180, py::arg("range") = 100, py::arg("map_frame_pose"), py::arg("map_cloud"))
        .def("sample_map_frames", [](coloradar::ColoradarPlusRun& self,
                                     float horizontalFov, float verticalFov, float range,
                                     const py::array_t<float>& mapFramePosesArray) {
            auto frames = self.sampleMapFrames(horizontalFov, verticalFov, range, numpyToPoses(mapFramePosesArray));
            std::vector<py::array_t<float>> numpyFrames;
            for (auto& frame : frames) { numpyFrames.push_back(pointcloudToNumpy(frame)); }
            return numpyFrames;
        }, py::arg("horizontal_fov") = 360, py::arg("vertical_fov") = 180, py::arg("range") = 100, py::arg("map_frame_poses"))

        .def("create_map_samples", [](coloradar::ColoradarPlusRun& self,
                                     float horizontalFov, float verticalFov, float range,
                                     const py::array_t<double>& sensorTimestamps,
                                     const py::array_t<float>& baseToSensorTransformArray) {
            self.createMapSamples(horizontalFov, verticalFov, range, numpyToVector(sensorTimestamps), numpyToPose(baseToSensorTransformArray));
        }, py::arg("horizontal_fov") = 360, py::arg("vertical_fov") = 180, py::arg("range") = 100,
           py::arg("sensor_timestamps") = py::array_t<double>(),
           py::arg("base_to_sensor_transform") = poseToNumpy(Eigen::Affine3f::Identity()))

        .def("get_poses", [](coloradar::ColoradarPlusRun& self) {
            std::vector<Eigen::Affine3f> poses = self.getPoses<Eigen::Affine3f>(); return posesToNumpy(poses);
        }, "Returns poses as an Nx7 numpy array [x, y, z, qx, qy, qz, qw]");

    // ColoradarRun
    py::class_<coloradar::ColoradarRun, coloradar::ColoradarPlusRun>(m, "ColoradarRun")
        .def(py::init<const std::filesystem::path&, coloradar::RadarConfig*, coloradar::RadarConfig*>())
        .def("single_chip_cube_timestamps", [](coloradar::ColoradarRun& self) { return vectorToNumpy(self.singleChipCubeTimestamps()); })
        .def("single_chip_timestamps", [](coloradar::ColoradarRun& self) { return vectorToNumpy(self.singleChipTimestamps()); })
        .def("get_single_chip_datacube", [](coloradar::ColoradarRun& self, const std::filesystem::path& binFilePath) { return vectorToNumpy(self.getSingleChipDatacube(binFilePath)); })
        .def("get_single_chip_datacube", [](coloradar::ColoradarRun& self, const int& cubeIdx) { return vectorToNumpy(self.getSingleChipDatacube(cubeIdx)); })
        .def("get_single_chip_heatmap", [](coloradar::ColoradarRun& self, const std::filesystem::path& binFilePath) { return vectorToNumpy(self.getSingleChipHeatmap(binFilePath)); })
        .def("get_single_chip_heatmap", [](coloradar::ColoradarRun& self, const int& hmIdx) { return vectorToNumpy(self.getSingleChipHeatmap(hmIdx)); })
        .def("get_single_chip_pointcloud", [](coloradar::ColoradarRun& self, std::filesystem::path binFilePath, float intensityThreshold = 0.0f) {
            pcl::PointCloud<coloradar::RadarPoint>::Ptr cloud = self.getSingleChipPointcloud(binFilePath, intensityThreshold); return radarCloudToNumpy(cloud);
        }, py::arg("bin_file_path"), py::arg("intensity_threshold") = 0)

        .def("get_single_chip_pointcloud", [](coloradar::ColoradarRun& self, int cloudIdx, float intensityThreshold = 0.0f) {
            pcl::PointCloud<coloradar::RadarPoint>::Ptr cloud = self.getSingleChipPointcloud(cloudIdx, intensityThreshold); return radarCloudToNumpy(cloud);
        }, py::arg("cloud_idx"), py::arg("intensity_threshold") = 0);


    // H5Dataset
    py::class_<coloradar::H5Dataset, std::shared_ptr<coloradar::H5Dataset>>(m, "H5Dataset")
        .def(py::init<const std::filesystem::path&>(), py::arg("h5_path"))
        .def(py::init([](const std::string& h5_path) {
            return std::make_shared<coloradar::H5Dataset>(std::filesystem::path(h5_path));
        }), py::arg("h5_path"));


    // ColoradarPlusDataset
    py::class_<coloradar::ColoradarPlusDataset, std::shared_ptr<coloradar::ColoradarPlusDataset>>(m, "ColoradarPlusDataset")
        .def(py::init<const std::filesystem::path&>())
        .def(py::init<const std::filesystem::path&, const std::filesystem::path&>(), py::arg("runs_dir"), py::arg("calib_dir"))
        .def("list_runs", &coloradar::ColoradarPlusDataset::listRuns)
        .def("get_runs", &coloradar::ColoradarPlusDataset::getRuns, py::return_value_policy::reference)
        .def("get_run", &coloradar::ColoradarPlusDataset::getRun, py::return_value_policy::reference)
        .def("imu_transform", [](coloradar::ColoradarPlusDataset& self) { return poseToNumpy(self.imuTransform()); })
        .def("lidar_transform", [](coloradar::ColoradarPlusDataset& self) { return poseToNumpy(self.lidarTransform()); })
        .def("cascade_transform", [](coloradar::ColoradarPlusDataset& self) { return poseToNumpy(self.cascadeTransform()); })
        .def("cascade_config", &coloradar::ColoradarPlusDataset::cascadeConfig, py::return_value_policy::reference)
        .def("export_to_file", py::overload_cast<const std::string &>(&coloradar::ColoradarPlusDataset::exportToFile), py::arg("config_path"))
    ;

    // ColoradarDataset
    py::class_<coloradar::ColoradarDataset, std::shared_ptr<coloradar::ColoradarDataset>, coloradar::ColoradarPlusDataset>(m, "ColoradarDataset")
        .def(py::init<const std::filesystem::path&>())
        .def(py::init<const std::filesystem::path&, const std::filesystem::path&>(), py::arg("runs_dir"), py::arg("calib_dir"))
        .def("get_run", &coloradar::ColoradarDataset::getRun, py::return_value_policy::reference)
        .def("single_chip_transform", [](coloradar::ColoradarDataset& self) { return poseToNumpy(self.singleChipTransform()); })
        .def("single_chip_config", &coloradar::ColoradarDataset::singleChipConfig, py::return_value_policy::reference);

    // DatasetVisualizer
    py::class_<coloradar::DatasetVisualizer>(m, "DatasetVisualizer")
        .def(py::init([](const coloradar::CascadeConfig cascadeRadarConfig,
                        const py::array_t<float>& baseToLidarArray,
                        const py::array_t<float>& baseToCascadeArray,
                        int frameIncrement,
                        double cascadeRadarIntensityThreshold,
                        const std::string cameraConfigPath) {
                Eigen::Affine3f baseToLidar = numpyToPose(baseToLidarArray);
                Eigen::Affine3f baseToCascade = numpyToPose(baseToCascadeArray);
                return std::make_unique<coloradar::DatasetVisualizer>(cascadeRadarConfig, baseToLidar, baseToCascade, frameIncrement, cascadeRadarIntensityThreshold, cameraConfigPath);
            }),
            py::arg("cascade_radar_config"),
            py::arg("base_to_lidar_transform"),
            py::arg("base_to_cascade_transform"),
            py::arg("frame_increment") = 1,
            py::arg("cascade_radar_intensity_threshold") = 0.0,
            py::arg("camera_config_path") = "camera_config.txt")
           
        .def("visualize", &coloradar::DatasetVisualizer::visualize,
            py::arg("run"),
            py::arg("use_prebuilt_map") = false);

}
