# Coloradar Library

## 1. Pre-built Images

Login to the Github container Registry:
```bash
docker login ghcr.io
```

Pull an image:

```bash
docker pull ghcr.io/arpg/coloradar-lib:{<X.Y>}{-<ros_distro>}
```

**Example:**
```bash
docker pull ghcr.io/arpg/coloradar-lib:12.6-jazzy
```

#### Available Image Configurations

| ros_version | OS                 | Python | Cuda             |
|-------------|--------------------|--------|------------------|
| humble      | Ubuntu 22.04 Jammy | 3.10   | 12.6, 12.8, none |
| jazzy       | Ubuntu 24.04 Noble | 3.12   | 12.6, 12.8, none |
| none        | Ubuntu 24.04 Noble | 3.12   | 12.6, 12.8, none |

#### System Requirements for Using Images

- **OS**: Linux/Mac
- **Docker**: Version 20.10+
- **NVIDIA GPU Drivers (Optional)**: Compatible with the chosen CUDA version
- **NVIDIA Container Toolkit (Optional)**: For GPU support in Docker

---

## 2. Building Your Own Images

Run the provided script to build an image for your desired **ROS** and **CUDA** versions:

```bash
python3 build.py --ros <ros_distro> --cuda <cuda_version>
```

- **`<ros_distro>`**: ROS distribution (e.g., `noetic`, `humble`, `jazzy`). Leave empty or use `none` for no ROS.
- **`<cuda_version>`**: CUDA version in `X.Y` format (e.g., `12.4`). Use `none` to skip CUDA.

**Examples:**

- Build with ROS Noetic with CUDA 12.4:
  ```bash
  python3 build.py --ros noetic --cuda 12.4
  ```
  
- Build ROS Humble while **detecting the local CUDA version**:
  ```bash
  python3 build.py --ros humble
  ```
  
- Build without ROS or CUDA:
  ```bash
  python3 build.py --ros none --cuda none
  ```

---

## 3. Local Build

```bash
mkdir build && cd build
cmake ..
make
```

#### Requirements:
- Linux/Mac
- GCC v.10-12
- BOOST 1.78+
- open-mpi
- VTK
- openCV
- PCL v.1.12+
- octomap
- Yaml-cpp
- (optional) PyBind11 v.2.6+

#### Running tests from project root:
```bash
./build/coloradar_tests
python3 tests/test_bindings.py
```
