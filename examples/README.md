# Coloradar Library Examples

This directory contains example scripts and configurations for working with the Coloradar library. All examples are designed to run in a Docker container using the provided Docker Compose configuration.

- [1. Initial Setup](#1-initial-setup)
  - [1.1. Download the Coloradar dataset](#11-download-the-coloradar-dataset)
  - [1.2. Detect your CUDA version](#12-detect-your-cuda-version)
  - [1.3. Set the corresponding container tag](#13-set-the-corresponding-container-tag)
  - [1.4. Set the volumes to point to the dataset](#14-set-the-volumes-to-point-to-the-dataset)
- [2. Example Docker Compose Services](#2-example-docker-compose-services)
  - [2.1. Get Export Config Template](#21-get-export-config-template)
  - [2.2. Build Lidar Maps](#22-build-lidar-maps)
  - [2.3. Sample Lidar Maps](#23-sample-lidar-maps)
  - [2.4 Visualize Dataset](#24-visualize-dataset)
  - [2.5. Export Dataset](#25-export-dataset)
  - [2.6. Jupyter Notebook Server](#26-jupyter-notebook-server)
- [3. Using the Demo Notebook](#3-using-the-demo-notebook)
  - [3.1. Run the Jupyter Server](#31-run-the-jupyter-server)
  - [3.2. Example Functions](#32-example-functions)
- [4. Notes and Troubleshooting](#4-notes-and-troubleshooting)


## 1. Initial Setup

#### 1.0. Install Docker and Docker Compose

#### 1.1. Download the Coloradar dataset. Put it in a folder so that the following subdirectories are present:
- `kitti`
- `calib`

#### 1.2. Detect your CUDA version. 
```bash
nvidia-smi
```

#### 1.3. Set the container tag and dataset path in `examples/docker-compose.yaml`:
```
x-base-image: &base_image ghcr.io/arpg/coloradar-lib:12.6-jazzy
x-dataset-volume: &dataset_volume ~/coloradar:/data/coloradar
```

## 2. Example Docker Compose Services

The `docker-compose.yaml` file provides several services for different tasks, including **exporting** the necessary parts of the dataset into a *single **.h5** file*.

### 2.1. Get Export Config Template
```bash
docker compose run get_export_config_template
```
This service copies the export configuration template to your local directory. The template will be saved as `export-config.yaml` in the current directory.

### 2.2. Build Lidar Maps
```bash
docker compose run build_maps
```
Builds maps from the dataset. This service mounts your local data directory to `/data/coloradar` inside the container.

### 2.3. Sample Lidar Maps
```bash
docker compose run sample_maps
```
Samples maps from the dataset. Similar to build_maps, this service uses your local data directory. The sampling FOV is hardcoded in the script `scripts/sample_maps`.


### 2.4. Visualize Dataset
```bash
docker compose up visualize_run
```
Tested on Ubuntu only. If seeing a display error, run
```bash
xhost +local:docker
```

Specify the target run name and optionally the frame step size in `docker-compose.yaml` under `services.visualize_run.command`.


### 2.5. Export Dataset
```bash
docker compose run export_dataset
```
Exports the dataset according to the configuration in `export-config.yaml`. This service mounts both your local data directory and the current directory for access to the configuration file.

### 2.6. Jupyter Notebook Server
```bash
docker compose up jupyter
```
Starts a Jupyter notebook server accessible at http://localhost:8888. The server is configured to:
- Run without authentication (no token or password required)
- Allow access from any origin
- Mount your local data directory and the current directory


## 3. Using the Demo Notebook

### 3.1. Run the Jupyter Server

The `demo.ipynb` notebook is designed to help you select the appropriate radar configuration for your dataset. To use it:

1. Start the Jupyter server:
   ```bash
   docker compose up jupyter
   ```

2. Open http://localhost:8888 in your browser or IDE

3. Navigate to `demo.ipynb` and run the cells to:
   - Visualize the radar's field of view
   - Test different FOV configurations
   - Generate the appropriate FOV parameters for your `export-config.yaml`


### 3.2. Example Functions
TBD

## 4. Notes and Troubleshooting

- The Jupyter server is configured to run without authentication for development purposes. Remote access may be hindered.

- The container paths (`/data/coloradar`, `/app`, `/export`) should not be changed randomly as they are hardcoded in the scripts. The current directory is mounted to `/app` inside the container which allows access to local scripts and configurations.

- If the first import cell in a notebook fails with this:
```bash
ModuleNotFoundError: No module named 'coloradar_cuda_lib'
```
set a CUDA-compatible base image (see **1.3**) OR remove everything that uses the `cu.RadarProcessor` object. Currently, only the datacube-to-heatmap transformation requires that object.