ARG BASE_IMAGE
FROM ${BASE_IMAGE} AS base


ARG CUDA_ENV
ENV CUDA_ENV=${CUDA_ENV}
SHELL ["/bin/bash", "-c"]
WORKDIR /tmp


# Base image check
RUN set -eux; \
    OS_NAME=$(grep '^ID=' /etc/os-release | cut -d= -f2 | tr -d '"'); \
    if [ "$OS_NAME" != "ubuntu" ]; then \
        echo "Error: Expected an Ubuntu base image, found: $OS_NAME"; \
        exit 1; \
    fi


# OS dependencies
RUN ARCH=$(dpkg --print-architecture); \
    if [ "$ARCH" = "arm64" ]; then \
        UBUNTU_MIRROR="http://ports.ubuntu.com/ubuntu-ports"; \
        SECURITY_MIRROR="http://ports.ubuntu.com/ubuntu-ports"; \
    else \
        UBUNTU_MIRROR="http://archive.ubuntu.com/ubuntu"; \
        SECURITY_MIRROR="http://security.ubuntu.com/ubuntu"; \
    fi; \
    UBUNTU_CODENAME=$(grep '^VERSION_CODENAME=' /etc/os-release | cut -d= -f2 || echo "noble"); \
    echo "Using Ubuntu codename: $UBUNTU_CODENAME, Mirror: $UBUNTU_MIRROR, Security Mirror: $SECURITY_MIRROR"; \
    echo "deb $UBUNTU_MIRROR ${UBUNTU_CODENAME} main restricted universe multiverse" > /etc/apt/sources.list; \
    echo "deb $UBUNTU_MIRROR ${UBUNTU_CODENAME}-updates main restricted universe multiverse" >> /etc/apt/sources.list; \
    echo "deb $UBUNTU_MIRROR ${UBUNTU_CODENAME}-backports main restricted universe multiverse" >> /etc/apt/sources.list; \
    echo "deb $SECURITY_MIRROR ${UBUNTU_CODENAME}-security main restricted universe multiverse" >> /etc/apt/sources.list; \
    apt update && apt upgrade -y

RUN apt update && apt install --no-install-recommends -y \
    software-properties-common \
    wget \
    curl \
    build-essential \
    cmake \
    liboctomap-dev \
    libgtest-dev \
    libopencv-dev \
    libopenmpi-dev \
    openmpi-bin \
    libjsoncpp-dev \
    libdbus-1-dev \
    gobject-introspection \
    libgirepository1.0-dev \
    qtbase5-dev qt5-qmake qtbase5-dev-tools libqt5opengl5-dev



# GCC
ARG DOCKER_GCC_VERSION=""
RUN if [ -n "$DOCKER_GCC_VERSION" ]; then \
        add-apt-repository -y ppa:ubuntu-toolchain-r/test && \
        apt update && \
        apt install -y gcc-${DOCKER_GCC_VERSION} g++-${DOCKER_GCC_VERSION} && \
        update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-${DOCKER_GCC_VERSION} 100 && \
        update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-${DOCKER_GCC_VERSION} 100; \
    fi
RUN gcc --version && g++ --version && cmake --version


# BOOST
ARG DOCKER_BOOST_VERSION=""
RUN if [ -n "$DOCKER_BOOST_VERSION" ]; then \
        CONVERTED_VERSION=${DOCKER_BOOST_VERSION//./_} && \
        wget https://sourceforge.net/projects/boost/files/boost/${DOCKER_BOOST_VERSION}/boost_${CONVERTED_VERSION}.tar.gz/download -O boost_${CONVERTED_VERSION}.tar.gz && \
        tar xzf boost_${CONVERTED_VERSION}.tar.gz && \
        cd boost_${CONVERTED_VERSION} && \
        ./bootstrap.sh --prefix=/usr/local && \
        ./b2 install --with=all -j$(nproc); \
    else \
        apt install -y libboost-all-dev; \
    fi


# PCL
ARG DOCKER_PCL_VERSION=""
RUN if [ -n "$DOCKER_PCL_VERSION" ]; then \
        wget https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-${DOCKER_PCL_VERSION}.tar.gz -O pcl-${DOCKER_PCL_VERSION}.tar.gz && \
        tar xzf pcl-${DOCKER_PCL_VERSION}.tar.gz && \
        cd pcl-pcl-${DOCKER_PCL_VERSION} && \
        mkdir build && \
        cd build && \
        cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_SHARED_LIBS=ON -DBUILD_apps=OFF -DBUILD_examples=OFF .. && \
        make -j$(nproc) && \
        make install; \
    else \
        apt install -y libpcl-dev; \
    fi


# Pybind11
ARG DOCKER_PYBIND_VERSION=""
RUN if [ -n "$DOCKER_PYBIND_VERSION" ]; then \
        wget https://github.com/pybind/pybind11/archive/refs/tags/v${DOCKER_PYBIND_VERSION}.tar.gz -O pybind11.tar.gz && \
        tar xzf pybind11.tar.gz && \
        cd pybind11-${DOCKER_PYBIND_VERSION} && \
        mkdir build && \
        cd build && \
        cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local && \
        make -j$(nproc) && \
        make install; \
    else \
        apt install -y pybind11-dev; \
    fi


# Cleanup
RUN rm -rf /tmp/*


# Build Library
WORKDIR /src/coloradar_lib
COPY include include
COPY src src
COPY tests tests
COPY CMakeLists.txt .

RUN mkdir build
RUN cmake -B build
RUN make -C build -j$(nproc)
RUN ./build/coloradar_tests
RUN python3 tests/test_bindings.py

COPY scripts scripts


CMD ["bash"]
