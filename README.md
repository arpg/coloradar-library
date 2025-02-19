# Coloradar Library

## 1. Docker Build
### 1.1. Requirements
- Linux OS
- Docker 
- Docker Compose
- `yq`

### 2.2. Build
```bash
chmod +x build_image.sh
./build_image.sh <ros_version>
```

#### Available Image Configurations
| ros_version | OS                 | Python |
|-------------|--------------------|--------|
| noetic      | Ubuntu 20.04 Focal | 3.8    |
| humble      | Ubuntu 22.04 Jammy | 3.10   |
| jazzy       | Ubuntu 24.04 Noble | 3.12   |


## 2. Local Build
Requirements:
- GCC v.10-12
- BOOST 1.78+
- open-mpi
- VTK
- openCV
- PCL v.1.12+
- octomap
- Yaml-cpp
- (optional) PyBind11 v.2.6+

```bash
mkdir build && cd build
cmake ..
make
```
Running tests from project root:
```bash
./build/coloradar_tests
python3 tests/test_bindings.py
```