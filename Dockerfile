ARG BASE_IMAGE
FROM ${BASE_IMAGE} AS base

ENV DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC
ARG CUDA_ENV
ENV CUDA_ENV=${CUDA_ENV}
SHELL ["/bin/bash", "-c"]
WORKDIR /tmp

# Lib versions
ARG DOCKER_VTK_VERSION=""
ARG DOCKER_GCC_VERSION=""
ARG DOCKER_BOOST_VERSION=""
ARG DOCKER_PCL_VERSION=""
ARG DOCKER_PYBIND_VERSION=""


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
    tzdata \
    software-properties-common \
    wget \
    curl \
    build-essential \
    cmake \
    manpages-dev \
    libeigen3-dev \
    libflann-dev \
    liboctomap-dev \
    libgtest-dev \
    libopencv-dev \
    libopenmpi-dev \
    openmpi-bin \
    libjsoncpp-dev \
    libyaml-cpp-dev \
    libdbus-1-dev \
    gobject-introspection \
    libgirepository1.0-dev \
    qtbase5-dev qt5-qmake qtbase5-dev-tools libqt5opengl5-dev \
    python3-pip


# VTK
RUN if [ -n "$DOCKER_VTK_VERSION" ]; then \
        apt update && apt install --no-install-recommends -y libvtk${DOCKER_VTK_VERSION}-dev python3-vtk${DOCKER_VTK_VERSION}; \
    fi


# GCC
RUN if [ -n "$DOCKER_GCC_VERSION" ]; then \
        add-apt-repository -y ppa:ubuntu-toolchain-r/test && \
        apt update && \
        apt install -y gcc-${DOCKER_GCC_VERSION} g++-${DOCKER_GCC_VERSION} && \
        update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-${DOCKER_GCC_VERSION} 100 && \
        update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-${DOCKER_GCC_VERSION} 100; \
    fi
RUN gcc --version && g++ --version && cmake --version


# BOOST
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
RUN if [ -n "$DOCKER_PCL_VERSION" ]; then \
        wget https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-${DOCKER_PCL_VERSION}.tar.gz -O pcl-${DOCKER_PCL_VERSION}.tar.gz && \
        tar xzf pcl-${DOCKER_PCL_VERSION}.tar.gz && \
        cd pcl-pcl-${DOCKER_PCL_VERSION} && \
        mkdir build && \
        cd build && \
        if [ "$(uname -m)" != "x86_64" ]; then \
            SSE_FLAG="-DWITH_SSE=OFF -DWITH_SSE2=OFF -DWITH_SSE3=OFF -DWITH_SSE4_1=OFF -DWITH_SSE4_2=OFF -DWITH_SSSE3=OFF -DWITH_AVX=OFF -DWITH_AVX2=OFF"; \
        else \
            SSE_FLAG=""; \
        fi && \
        cmake -DCMAKE_BUILD_TYPE=Release \
              -DCMAKE_INSTALL_PREFIX=/usr/local \
              -DBUILD_SHARED_LIBS=ON \
              -DBUILD_apps=OFF \
              -DBUILD_examples=OFF \
              -DCMAKE_CXX_FLAGS="-march=native -U__SSE2__ -U__SSE3__ -U__SSE4_1__ -U__SSE4_2__" \
              ${SSE_FLAG} .. && \
        make -j2 && \
        make install; \
    else \
        apt install -y libpcl-dev; \
    fi


# Pybind11
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
RUN make -C build
RUN make install -C build
RUN ./build/coloradar_tests
RUN echo "/usr/local/lib" | tee -a /etc/ld.so.conf.d/coloradar.conf && ldconfig

RUN if [ "$(lsb_release -rs | cut -d. -f1)" -ge 24 ]; then \
        pip3 install --break-system-packages numpy; \
    else \
        pip3 install numpy; \
    fi
RUN python3 tests/test_bindings.py

COPY export-config-template.yaml .


CMD ["bash"]
