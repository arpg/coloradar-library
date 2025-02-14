cmake_minimum_required(VERSION 3.12)
project(coloradar_lib VERSION 0.1 LANGUAGES C CXX)


find_program(GCC_COMPILER NAMES gcc-12 gcc-11 gcc-10 PATHS /usr/bin NO_DEFAULT_PATH)
if (NOT GCC_COMPILER)
    message(FATAL_ERROR "No suitable GCC compiler found! Please ensure GCC version 10, 11, or 12 is installed.")
endif()
string(REGEX REPLACE "gcc-([0-9]+)" "g++-\\1" GPP_COMPILER ${GCC_COMPILER})
set(CMAKE_C_COMPILER ${GCC_COMPILER} CACHE PATH "C compiler" FORCE)
set(CMAKE_CXX_COMPILER ${GPP_COMPILER} CACHE PATH "C++ compiler" FORCE)
message(STATUS "Using GCC compiler: ${GCC_COMPILER}")
message(STATUS "Using G++ compiler: ${GPP_COMPILER}")


set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-fconcepts)
set(BOOST_ROOT "/usr/local")
set(BOOST_INCLUDEDIR "/usr/local/include")
set(BOOST_LIBRARYDIR "/usr/local/lib")
set(PYBIND11_ROOT "/usr/local")
set(PYBIND11_INCLUDEDIR "/usr/local/include")
set(PYBIND11_LIBRARYDIR "/usr/local/lib")


find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
if(DEFINED ENV{VIRTUAL_ENV})
    set(VENV_PYTHON "$ENV{VIRTUAL_ENV}/bin/python3")
    if(EXISTS ${VENV_PYTHON})
        message(STATUS "Virtual environment detected: $ENV{VIRTUAL_ENV}")
        set(Python_EXECUTABLE ${VENV_PYTHON} CACHE FILEPATH "Python interpreter from active virtual environment" FORCE)
    endif()
endif()
message(STATUS "Using Python: ${Python_EXECUTABLE} (${Python_VERSION})")


find_package(Boost 1.78 REQUIRED COMPONENTS system filesystem thread)
find_package(MPI REQUIRED)
find_package(VTK REQUIRED)
find_package(PCL 1.12 REQUIRED)
find_package(octomap REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA)
find_package(HDF5 REQUIRED COMPONENTS C CXX)
find_package(pybind11 2.6)
find_package(yaml-cpp REQUIRED)


set(CUDA_LIBS ${CUDA_LIBRARIES} ${CUDA_CUFFT_LIBRARIES})
include_directories(${PROJECT_SOURCE_DIR}/include ${Boost_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${HDF5_INCLUDE_DIRS} ${JSONCPP_INCLUDE_DIRS})


### LIB

add_library(coloradar_lib SHARED
    src/internal.cpp
    src/radar_configs.cpp
    src/device.cpp
    src/dataset_configs.cpp
    src/pcl_functions.cpp
    src/octo_pointcloud.cpp
    src/coloradar_run.cpp
    src/dataset.cpp
)
target_link_libraries(coloradar_lib ${YAML_CPP_LIBRARIES} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES} ${HDF5_LIBRARIES} ${JSONCPP_LIBRARIES})


if(pybind11_FOUND)
    message(STATUS "Pybind11 found, compiling Python bindings.")
    pybind11_add_module(coloradar_dataset_lib src/python_bindings/main.cpp)
    target_link_libraries(coloradar_dataset_lib PRIVATE coloradar_lib)
else()
    message(WARNING "Pybind11 not found, skipping Python bindings compilation.")
endif()


if(CUDA_FOUND)
    message(STATUS "CUDA found, compiling CUDA code.")
    set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++20")

    cuda_add_library(coloradar_cuda SHARED
        src/cuda_kernels.cu
        src/radar_processing.cu
    )
    target_link_libraries(coloradar_cuda coloradar_lib ${CUDA_LIBS})
    include_directories(${CUDA_INCLUDE_DIRS})

    if(pybind11_FOUND)
        pybind11_add_module(coloradar_cuda_lib src/python_bindings/cuda.cpp)
        target_link_libraries(coloradar_cuda_lib PRIVATE coloradar_cuda)
    endif()
else()
    message(STATUS "CUDA not found, skipping CUDA compilation.")
endif()


### TESTS

add_executable(coloradar_tests tests/test_coloradar_tools.cpp)
target_link_libraries(coloradar_tests coloradar_lib gtest gtest_main pthread)
