ARG BASE_IMAGE
FROM ${BASE_IMAGE} AS base

ENV DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC
ARG VERBOSE=true
ARG CUDA_ENV
ENV CUDA_ENV=${CUDA_ENV}
SHELL ["/bin/bash", "-c"]
WORKDIR /tmp
ENV BUILD_VARIABLES="/tmp/build-variables"

# Lib versions
ARG DOCKER_VTK_VERSION=""
ARG DOCKER_GCC_VERSION=""
ARG DOCKER_BOOST_VERSION=""
ARG DOCKER_PCL_VERSION=""
ARG DOCKER_PYBIND_VERSION=""


# Set build variables
RUN set -eu; \
    # Log verbosity
    if [ "$VERBOSE" = "true" ]; then \
        APT_FLAGS=""; \
        OUTPUT_REDIRECT=""; \
        echo "Using verbose mode."; \
    else \
        APT_FLAGS="-qq"; \
        OUTPUT_REDIRECT='" > /dev/null"'; \
        echo "Using quiet mode."; \
    fi; \
    echo "APT_FLAGS=$APT_FLAGS" > $BUILD_VARIABLES; \
    echo "OUTPUT_REDIRECT=$OUTPUT_REDIRECT" >> $BUILD_VARIABLES; \
    \
    # System architecture
    ARCH=$(dpkg --print-architecture | tr '[:upper:]' '[:lower:]'); \
    UBUNTU_CODENAME=$(grep '^VERSION_CODENAME=' /etc/os-release | cut -d= -f2 || echo "noble"); \
    echo "Detected architecture: $ARCH, OS Ubuntu $UBUNTU_CODENAME."; \
    echo "ARCH=$ARCH" >> $BUILD_VARIABLES; \
    echo "UBUNTU_CODENAME=$UBUNTU_CODENAME" >> $BUILD_VARIABLES; \
    \
    # Package sources
    if [ "$ARCH" = "arm64" ]; then \
        UBUNTU_MIRROR="http://ports.ubuntu.com/ubuntu-ports"; \
        SECURITY_MIRROR="http://ports.ubuntu.com/ubuntu-ports"; \
    else \
        UBUNTU_MIRROR="http://archive.ubuntu.com/ubuntu"; \
        SECURITY_MIRROR="http://security.ubuntu.com/ubuntu"; \
    fi; \
    echo "UBUNTU_MIRROR=$UBUNTU_MIRROR" >> $BUILD_VARIABLES; \
    echo "SECURITY_MIRROR=$SECURITY_MIRROR" >> $BUILD_VARIABLES


# Validate base image
RUN set -eu; \
    OS_NAME=$(grep '^ID=' /etc/os-release | cut -d= -f2 | tr -d '"'); \
    if [ "$OS_NAME" != "ubuntu" ]; then \
        echo "Error: expected an Ubuntu base image, found: $OS_NAME"; \
        exit 1; \
    fi


# Install Ubuntu libraries
RUN set -eu; \
    . $BUILD_VARIABLES; \
    if [ "$UBUNTU_CODENAME" = "noble" ]; then \
        for line in \
            "deb $UBUNTU_MIRROR ${UBUNTU_CODENAME} main restricted universe multiverse" \
            "deb $UBUNTU_MIRROR ${UBUNTU_CODENAME}-updates main restricted universe multiverse" \
            "deb $UBUNTU_MIRROR ${UBUNTU_CODENAME}-backports main restricted universe multiverse" \
            "deb $SECURITY_MIRROR ${UBUNTU_CODENAME}-security main restricted universe multiverse"; do \
            grep -qxF "$line" /etc/apt/sources.list || echo "$line" >> /etc/apt/sources.list; \
        done; \
    fi;\
    sh -c "apt update $APT_FLAGS $OUTPUT_REDIRECT"; \
    sh -c "apt upgrade -y $APT_FLAGS $OUTPUT_REDIRECT"; \
    sh -c "apt install --no-install-recommends -y $APT_FLAGS \
        wget \
        lsb-release \
        gnupg \
        software-properties-common \
        build-essential \
        cmake \
        tzdata \
        curl \
        git \
        manpages-dev \
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
        qtbase5-dev qt5-qmake qtbase5-dev-tools \
        libqt5opengl5-dev \
        libglvnd-dev \
        python3-pip \
        $OUTPUT_REDIRECT"

# focal specific
RUN set -eu; \
    . $BUILD_VARIABLES; \
    if [ "$UBUNTU_CODENAME" = "focal" ]; then \
        apt update $APT_FLAGS $OUTPUT_REDIRECT; \
        apt upgrade -y $APT_FLAGS $OUTPUT_REDIRECT; \
        apt install --no-install-recommends -y $APT_FLAGS \
            libhdf5-dev \
            libhdf5-cpp-103 \
            libhdf5-serial-dev \
            $OUTPUT_REDIRECT; \
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


# EIGEN
RUN set -eu; \
    . "$BUILD_VARIABLES"; \
    if [ "$UBUNTU_CODENAME" = "focal" ]; then \
        echo "Ubuntu 20 detected – installing Eigen 3.4.0 …"; \
        wget -qO- https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz | tar xz; \
        cd eigen-3.4.0 && mkdir build && cd build; \
        cmake .. \
          -DCMAKE_INSTALL_PREFIX=/usr/local \
          -DCMAKE_BUILD_TYPE=Release \
          -DCMAKE_CXX_STANDARD=20 \
          -DEIGEN_BUILD_TESTS=OFF \
          -DEIGEN_BUILD_DOC=OFF; \
        make install; \
        cd /tmp && rm -rf eigen-3.4.0; \
    else \
        apt-get update && apt-get install -y --no-install-recommends libeigen3-dev; \
    fi


RUN set -eu; \
    . "$BUILD_VARIABLES"; \
    if [ -n "$DOCKER_VTK_VERSION" ]; then \
        echo "Requested VTK major version: $DOCKER_VTK_VERSION"; \
        if apt-cache show "libvtk${DOCKER_VTK_VERSION}-dev" > /dev/null 2>&1; then \
            echo "Installing libvtk${DOCKER_VTK_VERSION}-dev from APT …"; \
            apt-get update && \
            apt-get install -y --no-install-recommends \
                "libvtk${DOCKER_VTK_VERSION}-dev" \
                "python3-vtk${DOCKER_VTK_VERSION}"; \
        else \
            # ---------- build from source (default: 9.2.6) ----------
            case "$DOCKER_VTK_VERSION" in \
                9) VTK_TAG=v9.2.6 ;; \
                # 8) VTK_TAG=v8.2.0 ;; \
                7) VTK_TAG=v7.1.1 ;; \
                *) echo "Unknown VTK version: $DOCKER_VTK_VERSION" >&2; exit 1 ;; \
            esac; \
            echo "Building VTK $VTK_TAG from source …"; \
            git clone --branch "$VTK_TAG" --depth 1 https://gitlab.kitware.com/vtk/vtk.git; \
            mkdir -p vtk/build && cd vtk/build; \
            cmake .. \
              -DCMAKE_BUILD_TYPE=Release \
              -DCMAKE_INSTALL_PREFIX=/usr/local \
              -DCMAKE_CXX_STANDARD=20 \
              -DBUILD_SHARED_LIBS=ON \
              -DVTK_BUILD_TESTING=OFF \
              -DVTK_WRAP_PYTHON=OFF \
              -DVTK_BUILD_EXAMPLES=OFF \
              -DVTK_GROUP_ENABLE_Qt=NO \
              -DVTK_GROUP_ENABLE_Matlab=NO \
              -DVTK_GROUP_ENABLE_MPI=NO \
              -DVTK_GROUP_ENABLE_Rendering=YES \
              -DVTK_MODULE_ENABLE_VTK_IOParallel=NO \
              -DVTK_MODULE_ENABLE_VTK_RenderingOpenGL2=YES; \
            make -j"$(nproc)"; \
            make install; \
            cd /tmp && rm -rf vtk; \
        fi; \
    else \
        echo "No DOCKER_VTK_VERSION provided – skipping VTK install."; \
    fi


RUN set -eu; \
    if [ -n "$DOCKER_PCL_VERSION" ]; then \
        echo "Building PCL ${DOCKER_PCL_VERSION} …"; \
        wget -qO- "https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-${DOCKER_PCL_VERSION}.tar.gz" \
          | tar xz; \
        mkdir -p "pcl-pcl-${DOCKER_PCL_VERSION}/build"; \
        cd       "pcl-pcl-${DOCKER_PCL_VERSION}/build"; \
        # SIMD only on x86-64
        if [ "$(uname -m)" != "x86_64" ]; then \
            SIMD_OPTS="-DWITH_SSE=OFF -DWITH_AVX=OFF -DWITH_AVX2=OFF"; \
        else \
            SIMD_OPTS=""; \
        fi; \
        cmake .. \
          -DCMAKE_BUILD_TYPE=Release \
          -DCMAKE_INSTALL_PREFIX=/usr/local \
          -DCMAKE_CXX_STANDARD=20 \
          -DCMAKE_CXX_FLAGS="-march=native -O3" \
          -DCMAKE_CXX_FLAGS_RELEASE="-march=native -O3" \
          -DCMAKE_CXX_FLAGS_DEBUG="-g" \
          -DBUILD_SHARED_LIBS=ON \
          -DBUILD_apps=OFF \
          -DBUILD_examples=OFF \
          -DBUILD_tests=OFF \
          ${SIMD_OPTS}; \
        make -j"$(nproc)"; \
        make install; \
        cd /tmp && rm -rf "pcl-pcl-${DOCKER_PCL_VERSION}"; \
    else \
        echo "No DOCKER_PCL_VERSION set – installing libpcl-dev from APT …"; \
        apt-get update && \
        apt-get install -y --no-install-recommends libpcl-dev; \
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
