cmake_minimum_required(VERSION 3.12)

project(coloradar_lib VERSION 1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)

# Compiler
add_definitions(-DNDEBUG)

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
    # List possible GCC versions (from newest to oldest)
    set(POSSIBLE_GCC_VERSIONS 12 11)

    # Try to find the newest available GCC/G++ pair
    foreach(ver IN LISTS POSSIBLE_GCC_VERSIONS)
        find_program(GCC_COMPILER gcc-${ver} PATHS /usr/bin /usr/local/bin NO_DEFAULT_PATH)
        find_program(GPP_COMPILER g++-${ver} PATHS /usr/bin /usr/local/bin NO_DEFAULT_PATH)
        if (GCC_COMPILER AND GPP_COMPILER)
            set(SELECTED_GCC_VERSION ${ver})
            break()
        endif()
    endforeach()

    if (NOT GCC_COMPILER OR NOT GPP_COMPILER)
        message(FATAL_ERROR "No suitable GCC/G++ compiler found! Please ensure GCC version between 11 and 12 is installed.")
    endif()

    set(CMAKE_C_COMPILER ${GCC_COMPILER} CACHE PATH "C compiler" FORCE)
    set(CMAKE_CXX_COMPILER ${GPP_COMPILER} CACHE PATH "C++ compiler" FORCE)
    message(STATUS "Using GCC compiler: ${GCC_COMPILER}")
    message(STATUS "Using G++ compiler: ${GPP_COMPILER}")

    add_compile_options(-fconcepts)
endif()

    # find_program(GCC_COMPILER 
    #     NAMES gcc-12 gcc-11
    #    PATHS /usr/bin /usr/local/bin
    #    NO_DEFAULT_PATH
    #)
    #find_program(GPP_COMPILER 
    #    NAMES g++-12 g++-11
    #    PATHS /usr/bin /usr/local/bin
    #    NO_DEFAULT_PATH
    #)
    #if (NOT GCC_COMPILER OR NOT GPP_COMPILER)
    #    message(FATAL_ERROR "No suitable GCC compiler found! Please ensure GCC version 11, or 12 is installed.")
    #endif()
    #string(REGEX REPLACE "gcc-([0-9]+)" "g++-\\1" GPP_COMPILER ${GCC_COMPILER})



# System Paths (macOS & Linux)
if(APPLE)
    # Extend search paths for Homebrew and system directories
    list(APPEND CMAKE_PREFIX_PATH 
        /opt/homebrew 
        /opt/homebrew/opt 
        /opt/homebrew/lib 
        /opt/homebrew/include 
        /usr/local 
        /usr
    )
    list(APPEND CMAKE_SYSTEM_PREFIX_PATH 
        /opt/homebrew 
        /opt/homebrew/opt 
        /opt/homebrew/Cellar 
        /usr/local 
        /usr
    )
    # Ensure Homebrew libraries and includes are prioritized
    list(APPEND CMAKE_LIBRARY_PATH /opt/homebrew/lib)
    list(APPEND CMAKE_INCLUDE_PATH /opt/homebrew/include)
    list(APPEND CMAKE_PROGRAM_PATH /opt/homebrew/bin)

    # Append Open MPI if it's required
    list(APPEND CMAKE_PREFIX_PATH /opt/homebrew/opt/open-mpi)

    # Set runtime paths to avoid linking issues on macOS
    set(CMAKE_INSTALL_RPATH "${CMAKE_BINARY_DIR};/opt/homebrew/lib")
    set(CMAKE_BUILD_WITH_INSTALL_RPATH ON)
    set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
endif()


# Python
if(DEFINED ENV{VIRTUAL_ENV})
    set(Python3_ROOT_DIR "$ENV{VIRTUAL_ENV}")
    set(Python3_EXECUTABLE "$ENV{VIRTUAL_ENV}/bin/python3")
    message(STATUS "Using Python from virtual environment: ${Python3_EXECUTABLE}")
endif()
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
message(STATUS "Final Python used: ${Python3_EXECUTABLE} (${Python3_VERSION})")

include(GNUInstallDirs)
set(Python3_SITEARCH ${Python3_SITEARCH} CACHE PATH "Python site-packages directory")
execute_process(
    COMMAND ${Python3_EXECUTABLE} -c "import sysconfig; print(sysconfig.get_paths()['purelib'])"
    OUTPUT_VARIABLE Python3_SITEARCH OUTPUT_STRIP_TRAILING_WHITESPACE
)


find_package(Boost 1.78 REQUIRED COMPONENTS system filesystem thread)
find_package(Threads REQUIRED)
find_package(MPI REQUIRED)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(VTK 9 REQUIRED)
find_package(PCL 1.12 REQUIRED COMPONENTS visualization)
find_package(octomap REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA)
find_package(HDF5 REQUIRED COMPONENTS C CXX)
find_package(pybind11 2.6)
find_package(GTest REQUIRED)

if(APPLE)
    find_path(YAML_CPP_INCLUDE_DIRS yaml-cpp/yaml.h PATHS /opt/homebrew/include)
    set(YAML_CPP_LIBRARIES "/opt/homebrew/lib/libyaml-cpp.dylib")
    if (YAML_CPP_INCLUDE_DIRS AND YAML_CPP_LIBRARIES)
        message(STATUS "Found yaml-cpp ${YAML_CPP_LIBRARIES}")
    else()
        message(FATAL_ERROR "yaml-cpp not found!")
    endif()

    find_path(JSONCPP_INCLUDE_DIRS json.h PATHS /opt/homebrew/include/json)
    find_library(JSONCPP_LIBRARIES NAMES jsoncpp PATHS /opt/homebrew/lib)
    if (JSONCPP_INCLUDE_DIRS AND JSONCPP_LIBRARIES)
        message(STATUS "Found jsoncpp")
    else()
        message(FATAL_ERROR "jsoncpp not found!")
    endif()
else()
    # find_package(jsoncpp REQUIRED)
    # find_package(yaml-cpp REQUIRED)
    # find_path(JSONCPP_INCLUDE_DIRS json.h
    # PATHS
    #     /usr/include
    #     /usr/include/jsoncpp/json
    #     NO_DEFAULT_PATH
    # )

    # jsoncpp layout on Ubuntu 22 puts headers in jsoncpp/json/
    find_package(yaml-cpp REQUIRED)

    if(NOT TARGET JsonCpp::JsonCpp)
        find_package(jsoncpp CONFIG QUIET)
    endif()

    if(NOT TARGET JsonCpp::JsonCpp)
        find_package(jsoncpp REQUIRED)
    endif()

    set(JSONCPP_LIBRARIES JsonCpp::JsonCpp)

    # Hack to make <json.h> resolve correctly (Ubuntu 22 jsoncpp layout)
    find_path(JSONCPP_INCLUDE_H_PATH json.h
        PATH_SUFFIXES jsoncpp/json
        PATHS /usr/include
    )
    if (NOT JSONCPP_INCLUDE_H_PATH)
        message(FATAL_ERROR "Could not find json.h in expected location")
    endif()

    set(JSONCPP_INCLUDE_DIRS "${JSONCPP_INCLUDE_H_PATH}")

endif()

set(CUDA_LIBS ${CUDA_LIBRARIES} ${CUDA_CUFFT_LIBRARIES})


### LIB

include_directories(${PROJECT_SOURCE_DIR}/include ${Boost_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${HDF5_INCLUDE_DIRS} ${JSONCPP_INCLUDE_DIRS} ${JSONCPP_INCLUDE_DIRS}/json)

add_library(coloradar_lib SHARED
    src/internal.cpp

    src/utils/basic.cpp
    src/utils/octo_pointcloud.cpp

    src/radar_configs.cpp
    src/device.cpp
    src/dataset_configs.cpp
    src/coloradar_run.cpp

    src/dataset/parent_init.cpp
    src/dataset/parent_data.cpp
    src/dataset/parent_export.cpp
    src/dataset/child_init.cpp
    src/dataset/child_data.cpp

    src/visualizer.cpp
)
target_link_libraries(coloradar_lib Threads::Threads ${YAML_CPP_LIBRARIES} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES} ${HDF5_LIBRARIES} ${JSONCPP_LIBRARIES})
install(TARGETS coloradar_lib LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

if(pybind11_FOUND)
    message(STATUS "Pybind11 found, compiling Python bindings.")
    pybind11_add_module(coloradar_dataset_lib src/python_bindings/main.cpp)
    target_link_libraries(coloradar_dataset_lib PRIVATE coloradar_lib)
    install(TARGETS coloradar_dataset_lib LIBRARY DESTINATION ${Python3_SITEARCH})
endif()

if(CUDA_FOUND AND NOT ("$ENV{CUDA_ENV}" STREQUAL "False" OR "$ENV{CUDA_ENV}" STREQUAL "false"))
    message(STATUS "CUDA found, compiling CUDA code.")
    set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++20")
    cuda_add_library(coloradar_cuda SHARED
        src/cuda/kernels.cu
        src/cuda/radar_processing.cu
    )
    target_link_libraries(coloradar_cuda coloradar_lib ${CUDA_LIBS})
    include_directories(${CUDA_INCLUDE_DIRS})
    install(TARGETS coloradar_cuda LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

    if(pybind11_FOUND)
        pybind11_add_module(coloradar_cuda_lib src/python_bindings/cuda.cpp)
        target_link_libraries(coloradar_cuda_lib PRIVATE coloradar_cuda)
        install(TARGETS coloradar_cuda_lib LIBRARY DESTINATION ${Python3_SITEARCH})
    endif()
else()
    message(STATUS "($ENV{CUDA_ENV}) CUDA not found, skipping CUDA compilation.")
endif()


### TESTS

add_executable(coloradar_tests tests/test_coloradar_tools.cpp)
target_link_libraries(coloradar_tests PRIVATE coloradar_lib GTest::GTest GTest::Main pthread)

