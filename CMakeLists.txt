cmake_minimum_required(VERSION 3.12)

project(coloradar_lib VERSION 1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler
if(APPLE)
    find_program(CLANG_COMPILER 
        NAMES clang clang-16 clang-15 clang-14
        PATHS /usr/bin /usr/local/bin /opt/homebrew/bin
        NO_DEFAULT_PATH
    )
    find_program(CLANGXX_COMPILER 
        NAMES clang++ clang++-16 clang++-15 clang++-14
        PATHS /usr/bin /usr/local/bin /opt/homebrew/bin
        NO_DEFAULT_PATH
    )
    if (NOT CLANG_COMPILER OR NOT CLANGXX_COMPILER)
        message(FATAL_ERROR "No suitable Clang compiler found! Please ensure Clang is installed.")
    endif()
    set(CMAKE_C_COMPILER ${CLANG_COMPILER} CACHE PATH "C compiler" FORCE)
    set(CMAKE_CXX_COMPILER ${CLANGXX_COMPILER} CACHE PATH "C++ compiler" FORCE)
    message(STATUS "Using Clang compiler: ${CLANG_COMPILER}")
    message(STATUS "Using Clang++ compiler: ${CLANGXX_COMPILER}")

    add_definitions(-DWITH_SSE=OFF)
else()
    find_program(GCC_COMPILER 
        NAMES gcc-12 gcc-11 gcc-10 
        PATHS /usr/bin /usr/local/bin /opt/homebrew/bin
        NO_DEFAULT_PATH
    )
    find_program(GPP_COMPILER 
        NAMES g++-12 g++-11 g++-10
        PATHS /usr/bin /usr/local/bin /opt/homebrew/bin
        NO_DEFAULT_PATH
    )
    if (NOT GCC_COMPILER OR NOT GPP_COMPILER)
        message(FATAL_ERROR "No suitable GCC compiler found! Please ensure GCC version 10, 11, or 12 is installed.")
    endif()
    string(REGEX REPLACE "gcc-([0-9]+)" "g++-\\1" GPP_COMPILER ${GCC_COMPILER})
    set(CMAKE_C_COMPILER ${GCC_COMPILER} CACHE PATH "C compiler" FORCE)
    set(CMAKE_CXX_COMPILER ${GPP_COMPILER} CACHE PATH "C++ compiler" FORCE)
    message(STATUS "Using GCC compiler: ${GCC_COMPILER}")
    message(STATUS "Using G++ compiler: ${GPP_COMPILER}")
    add_compile_options(-fconcepts)
endif()

# cmake_policy(SET CMP0144 NEW)

# System Paths (macOS & Linux)
if(APPLE)
    list(APPEND CMAKE_PREFIX_PATH /opt/homebrew /opt/homebrew/opt /opt/homebrew/lib /opt/homebrew/include /usr/local /usr)
    list(APPEND CMAKE_SYSTEM_PREFIX_PATH /opt/homebrew /opt/homebrew/opt /opt/homebrew/Cellar /usr/local /usr)
    set(CMAKE_LIBRARY_PATH "/opt/homebrew/lib" ${CMAKE_LIBRARY_PATH})
    set(CMAKE_INCLUDE_PATH "/opt/homebrew/include" ${CMAKE_INCLUDE_PATH})
    set(CMAKE_PROGRAM_PATH "/opt/homebrew/bin" ${CMAKE_PROGRAM_PATH})
    set(CMAKE_PREFIX_PATH "/opt/homebrew/opt/open-mpi" ${CMAKE_PREFIX_PATH})
endif()


# Python
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
find_package(Threads REQUIRED)
find_package(MPI REQUIRED)
find_package(VTK REQUIRED)
find_package(PCL 1.12 REQUIRED)
find_package(octomap REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA)
find_package(HDF5 REQUIRED COMPONENTS C CXX)
find_package(pybind11 2.6)
find_package(jsoncpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(GTest REQUIRED)

if(APPLE)
    set(YAML_CPP_LIBRARIES "/opt/homebrew/lib/libyaml-cpp.dylib")
    set(YAML_CPP_INCLUDE_DIRS "/opt/homebrew/include")
    find_path(YAML_CPP_INCLUDE_DIRS yaml-cpp/yaml.h PATHS /opt/homebrew/include /opt/homebrew/Cellar/yaml-cpp/0.8.0/include/yaml-cpp)
    find_library(YAML_CPP_LIBRARIES NAMES yaml-cpp PATHS /opt/homebrew/lib /opt/homebrew/Cellar/yaml-cpp/0.8.0/lib)
    if (YAML_CPP_INCLUDE_DIRS AND YAML_CPP_LIBRARIES)
        message(STATUS "Found yaml-cpp: ${YAML_CPP_LIBRARIES}")
        message(STATUS "Including yaml-cpp: ${YAML_CPP_INCLUDE_DIRS}")
    else()
        message(FATAL_ERROR "yaml-cpp not found! Install it via Homebrew.")
    endif()

    find_path(JSONCPP_INCLUDE_DIR json/json.h PATHS /opt/homebrew/include /opt/homebrew/Cellar/jsoncpp/1.9.6/include/jsoncpp)
    find_library(JSONCPP_LIBRARIES NAMES jsoncpp PATHS /opt/homebrew/lib /opt/homebrew/Cellar/jsoncpp/1.9.6/lib)
    if (JSONCPP_INCLUDE_DIR AND JSONCPP_LIBRARIES)
        message(STATUS "Found jsoncpp: ${JSONCPP_LIBRARIES}")
    else()
        message(FATAL_ERROR "jsoncpp not found! Install it via Homebrew.")
    endif()
endif()

set(CUDA_LIBS ${CUDA_LIBRARIES} ${CUDA_CUFFT_LIBRARIES})


### LIB

include_directories(${PROJECT_SOURCE_DIR}/include ${Boost_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${HDF5_INCLUDE_DIRS} ${JSONCPP_INCLUDE_DIRS})

add_library(coloradar_lib SHARED
    src/internal.cpp
    src/utils/pcl_functions.cpp
    src/utils/octo_pointcloud.cpp

    src/radar_configs.cpp
    src/device.cpp
    
    src/dataset_configs.cpp
    src/coloradar_run.cpp
    src/dataset.cpp
)
target_link_libraries(coloradar_lib Threads::Threads ${YAML_CPP_LIBRARIES} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES} ${HDF5_LIBRARIES} ${JSONCPP_LIBRARIES})

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
        src/cuda/kernels.cu
        src/cuda/radar_processing.cu
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
target_link_libraries(coloradar_tests PRIVATE coloradar_lib GTest::GTest GTest::Main pthread)

