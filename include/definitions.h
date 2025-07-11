#ifndef INCLUDE_H
#define INCLUDE_H

#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdlib>
#include <cstring>

#include <unistd.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <sys/types.h>

#include <cerrno>
#include <locale>

#include <cctype>
#include <map>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <functional>
#include <fstream>
#include <iostream>
#include <memory>
#include <regex>
#include <span>
#include <sstream>
#include <stdexcept>
#include <typeinfo>
#include <cxxabi.h>  // optional for demangling on GCC/Clang


// GCC 11 + Ubuntu 20 compatibility
#if defined(__GNUC__) && __GNUC__ == 11 && !defined(__clang__)

using std::memcpy;
extern "C" void free(void*) throw();

namespace std {
    inline std::string to_string(float value) {
        std::ostringstream oss;
        oss.precision(8);  // GCC's default precision
        oss << value;
        return oss.str();
    }
    // inline double sqrt(double x) { return ::sqrt(static_cast<double>(x)); }
    // inline double fabs(double x) { return ::fabs(static_cast<double>(x)); }
    // inline double cos(double x)  { return ::cos(static_cast<double>(x)); }
    // inline double sin(double x)  { return ::sin(static_cast<double>(x)); }
    // inline double floor(double x) { return ::floor(static_cast<double>(x)); }
    // inline double log(double x) { return ::log(static_cast<double>(x)); }
    // inline double exp(double x) { return ::exp(static_cast<double>(x)); }
    // inline double acos(double x) { return ::acos(static_cast<double>(x)); }
    // inline double asin(double x) { return ::asin(static_cast<double>(x)); }
}

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


extern "C" {
    #include <sys/types.h>
    #include <sys/syscall.h>
    #include <linux/futex.h>
    #include <errno.h>
}
#undef syscall
inline int syscall_wrapper(long number, ...) {
    return syscall(number);
}
#define syscall syscall_wrapper

#define _GLIBCXX_USE_CXX11_ABI 1
#define _GLIBCXX_ASSERTIONS 1

#endif


#include <json.h>
#include <yaml-cpp/yaml.h>
#include <H5Cpp.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/point_cloud_geometry_handlers.h>

#include <vtkSmartPointer.h>
#include <vtkImageData.h>
#include <vtkImageActor.h>
#include <vtkImageMapper3D.h>
#include <octomap/octomap.h>


#endif
