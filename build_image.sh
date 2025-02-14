#!/bin/bash


# Check requirements
if ! command -v yq &> /dev/null; then
    echo "Error: yq is not installed. Please install yq."
    exit 1
fi


# Set ROS version
CONFIG_FILE="docker/ros-versions.yaml"
AVAILABLE_ROS_DISTROS="$(yq '.ros_versions | keys | map(select(. != "default")) | join(" ")' "$CONFIG_FILE")"
ROS_DISTRO=""
if [ $# -gt 0 ]; then
  ROS_DISTRO="$1"
  if ! yq -e ".ros_versions | has(\"$ROS_DISTRO\")" "$CONFIG_FILE" > /dev/null 2>&1; then
    echo "Error: Invalid ROS distribution '${ROS_DISTRO}'."
    echo "Available options: '' ${AVAILABLE_ROS_DISTROS}"
    exit 1
  fi
fi

echo "Using ROS distribution: '${ROS_DISTRO}'"


# Set cuda version
if ! command -v nvcc &> /dev/null; then
    DOCKER_CUDA_VERSION=""
else
    DOCKER_CUDA_VERSION=$(nvcc --version | grep -oP "release \K[0-9]+\.[0-9]+")
    if [ -z "$DOCKER_CUDA_VERSION" ]; then
        DOCKER_CUDA_VERSION=""
    fi
fi
echo "Using CUDA version: $DOCKER_CUDA_VERSION"


# Set base image
OS_IMAGE_NAME="annazabnus/ros-cuda:$DOCKER_CUDA_VERSION-${ROS_DISTRO}"
echo "Using base image: $OS_IMAGE_NAME"


# Build lib image
TAG=$(echo "$OS_IMAGE_NAME" | awk -F: '{print $2}')
LIB_IMAGE_NAME="coloradar-tools:$TAG"
echo "Building lib image: $LIB_IMAGE_NAME"

UBUNTU_VERSION=$(yq -r ".ros_versions.\"$ROS_DISTRO\".ubuntu // \"\"" "$CONFIG_FILE")
DOCKER_GCC_VERSION=$(yq -r ".ubuntu_versions.\"$UBUNTU_VERSION\".gcc // \"\"" "$CONFIG_FILE")
DOCKER_BOOST_VERSION=$(yq -r ".ubuntu_versions.\"$UBUNTU_VERSION\".boost // \"\"" "$CONFIG_FILE")
DOCKER_PCL_VERSION=$(yq -r ".ubuntu_versions.\"$UBUNTU_VERSION\".pcl // \"\"" "$CONFIG_FILE")
DOCKER_PYBIND_VERSION=$(yq -r ".ubuntu_versions.\"$UBUNTU_VERSION\".pybind // \"\"" "$CONFIG_FILE")

docker build \
    --build-arg BASE_IMAGE=$OS_IMAGE_NAME \
    --build-arg DOCKER_GCC_VERSION="$DOCKER_GCC_VERSION"  \
    --build-arg DOCKER_BOOST_VERSION="$DOCKER_BOOST_VERSION" \
    --build-arg DOCKER_PCL_VERSION="$DOCKER_PCL_VERSION" \
    --build-arg DOCKER_PYBIND_VERSION="$DOCKER_PYBIND_VERSION" \
    -f "docker/tools.Dockerfile" -t "$LIB_IMAGE_NAME" .

if [ $? -ne 0 ]; then
    exit 1
fi
