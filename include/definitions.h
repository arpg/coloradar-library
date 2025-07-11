#ifndef INCLUDE_H
#define INCLUDE_H

#define _USE_MATH_DEFINES

// First include all standard headers correctly and completely.
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <unordered_map>
#include <tuple>
#include <iostream>
#include <sstream>
#include <fstream>
#include <filesystem>
#include <stdexcept>
#include <typeinfo>
#include <locale>
#include <regex>
#include <span>
#include <functional>
#include <chrono>
#include <cctype>

#include <unistd.h>
#include <sys/types.h>
#include <sys/resource.h>
#include <sys/syscall.h>

// Critical compatibility macros for GCC 11 on Ubuntu 20
#if defined(__GNUC__) && __GNUC__ == 11
extern "C" void free(void*) throw();

#define _GLIBCXX_USE_CXX11_ABI 1
#define _GLIBCXX_ASSERTIONS 1
#endif

// Explicitly define M_PI if missing
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 3rd-party headers included after standard headers
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
