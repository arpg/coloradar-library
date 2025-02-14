ARG BASE_IMAGE
FROM ${BASE_IMAGE} AS base


SHELL ["/bin/bash", "-c"]
WORKDIR /tmp


# GCC
ARG DOCKER_GCC_VERSION=""
RUN if [ -n "$DOCKER_GCC_VERSION" ]; then \
        add-apt-repository ppa:ubuntu-toolchain-r/test && \
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
        echo ${CONVERTED_VERSION}; \
        wget https://sourceforge.net/projects/boost/files/boost/${DOCKER_BOOST_VERSION}/boost_${CONVERTED_VERSION}.tar.gz/download -O boost_${CONVERTED_VERSION}.tar.gz && \
        tar xzf boost_${CONVERTED_VERSION}.tar.gz && \
        cd boost_${CONVERTED_VERSION} && \
        ./bootstrap.sh --prefix=/usr/local && \
        ./b2 install --with=all -j4; \
    fi


# Other dependencies
RUN apt install -y liboctomap-dev libgtest-dev libopencv-dev libopenmpi-dev openmpi-bin libjsoncpp-dev libdbus-1-dev gobject-introspection libgirepository1.0-dev


# PCL
ARG DOCKER_PCL_VERSION=""
RUN if [ -n "$DOCKER_PCL_VERSION" ]; then \
        wget https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-${DOCKER_PCL_VERSION}.tar.gz -O pcl-${DOCKER_PCL_VERSION}.tar.gz && \
        tar xzf pcl-${DOCKER_PCL_VERSION}.tar.gz && \
        cd pcl-pcl-${DOCKER_PCL_VERSION} && \
        mkdir build && \
        cd build && \
        cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_SHARED_LIBS=ON -DBUILD_apps=OFF -DBUILD_examples=OFF .. && \
        make -j4 && \
        make install; \
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
        make -j4 && \
        make install; \
    fi


# Cleanup
RUN rm -rf tmp/*


# Build Tools Library
WORKDIR /src/coloradar_tools
COPY coloradar_tools/src src
COPY coloradar_tools/CMakeLists.txt .

RUN mkdir build
RUN cmake -B build
RUN make -C build -j2
RUN ./build/coloradar_tests


# Install Python dependencies
COPY requirements.txt /tmp
RUN apt update && apt install -y python3-pip
RUN pip3 install --upgrade --ignore-installed -r /tmp/requirements.txt

COPY coloradar_tools/scripts scripts
COPY coloradar_tools/tests tests
COPY coloradar_tools/__init__.py coloradar_tools/demo.ipynb ./


# Run python tests
ENV PYTHONPATH="${PYTHONPATH}:/src"
WORKDIR /src
RUN python3 coloradar_tools/tests/test_bindings.py


CMD ["bash"]
