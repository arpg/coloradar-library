#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <cstring>
#include "cuda/coloradar_cuda.h"

namespace py = pybind11;

PYBIND11_MODULE(coloradar_cuda_lib, m) {
    py::module dataset_tools = py::module::import("coloradar_dataset_lib");
    py::object radar_config_class = dataset_tools.attr("RadarConfig");

    py::class_<coloradar::RadarProcessor, std::shared_ptr<coloradar::RadarProcessor>>(m, "RadarProcessor")
        .def(py::init<const coloradar::RadarConfig*, const double&, const double&, const double&>(),
             py::arg("config"),
             py::arg("blackmanParamA0") = 0.42,
             py::arg("blackmanParamA1") = 0.5,
             py::arg("blackmanParamA2") = 0.08)
        .def("cube_to_heatmap", [](coloradar::RadarProcessor &self,
                                   py::array_t<int16_t> datacube,
                                   bool applyCollapseDoppler,
                                   bool removeAntennaCoupling,
                                   bool applyPhaseFrequencyCalib)
        {
            auto buf = datacube.request();
            if (buf.ndim != 1)
                throw std::runtime_error("cube_to_heatmap: input array must be 1-dimensional");
            auto ptr = static_cast<int16_t*>(buf.ptr);
            std::vector<int16_t> cube(ptr, ptr + buf.shape[0]);
            std::vector<float> heatmap = self.cubeToHeatmap(cube, applyCollapseDoppler, removeAntennaCoupling, applyPhaseFrequencyCalib);
            py::array_t<float> result(py::array_t<float>::ShapeContainer({heatmap.size()}));
            std::memcpy(result.mutable_data(), heatmap.data(), heatmap.size() * sizeof(float));
            return result;
        },
             py::arg("datacube"),
             py::arg("apply_collapse_doppler") = true,
             py::arg("remove_antenna_coupling") = true,
             py::arg("apply_phase_frequency_calib") = true)
        .def_readonly("config", &coloradar::RadarProcessor::config);
}
