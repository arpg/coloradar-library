# Coloradar Library Examples

This directory contains example scripts and configurations for working with the Coloradar library. All examples are designed to run in a Docker container using the provided Docker Compose configuration.

## Prerequisites

- Docker and Docker Compose installed on your system
- A local directory for storing data (default: `~/data/coloradar`)

## Docker Compose Services

The `docker-compose.yaml` file provides several services for different tasks:

### 1. Get Export Config Template
```bash
docker compose run get_export_config_template
```
This service copies the export configuration template to your local directory. The template will be saved as `export-config.yaml` in the current directory.

### 2. Build Maps
```bash
docker compose run build_maps
```
Builds maps from the dataset. This service mounts your local data directory to `/data/coloradar` inside the container.

### 3. Sample Maps
```bash
docker compose run sample_maps
```
Samples maps from the dataset. Similar to build_maps, this service uses your local data directory. The sampling FOV is hardcoded in the script `scripts/sample_maps`.

The following runs have been processed:
- 2_22_2021_longboard_run0 (1034 frames)
- 2_22_2021_longboard_run7 (1309 frames)
- 12_21_2020_ec_hallways_run3 (1252 frames)
- 12_21_2020_ec_hallways_run4 (447 frames)
- 2_22_2021_longboard_run6 (1141 frames)
- 2_22_2021_longboard_run1 (882 frames)
- 12_21_2020_ec_hallways_run2 (1421 frames)
- 2_23_2021_edgar_classroom_run2 (982 frames)
- 2_23_2021_edgar_classroom_run5 (829 frames)
- 2_23_2021_edgar_classroom_run4 (816 frames)
- 2_23_2021_edgar_classroom_run3 (1380 frames)
- 2_28_2021_outdoors_run5 (619 frames)
- 12_21_2020_arpg_lab_run0 (589 frames)
- 2_28_2021_outdoors_run2 (653 frames)
- 12_21_2020_arpg_lab_run1 (590 frames)
- 2_28_2021_outdoors_run3 (663 frames)
- 2_28_2021_outdoors_run4 (544 frames)
- 2_24_2021_aspen_run1 (456 frames)
- 2_24_2021_aspen_run6 (579 frames)
- 2_23_2021_edgar_army_run2 (720 frames)
- 2_24_2021_aspen_run8 (516 frames)
- 2_23_2021_edgar_army_run5 (567 frames)
- 2_24_2021_aspen_run9 (414 frames)
- 2_23_2021_edgar_army_run4 (1514 frames)
- 2_23_2021_edgar_army_run3 (1226 frames)
- 2_24_2021_aspen_run7 (536 frames)
- 2_24_2021_aspen_run0 (434 frames)
- 12_21_2020_ec_hallways_run0 (520 frames)
- 2_22_2021_longboard_run4 (987 frames)

### 4. Export Dataset
```bash
docker compose run export_dataset
```
Exports the dataset according to the configuration in `export-config.yaml`. This service mounts both your local data directory and the current directory for access to the configuration file.

### 5. Jupyter Notebook Server
```bash
docker compose up jupyter
```
Starts a Jupyter notebook server accessible at http://localhost:8888. The server is configured to:
- Run without authentication (no token or password required)
- Allow access from any origin
- Mount your local data directory and the current directory

## Volume Mounts

The following volume mounts are used across services:

- `~/data/coloradar:/data/coloradar`: This is the main data directory. You can change the local path (`~/data/coloradar`) to match your setup, but the container path (`/data/coloradar`) should remain unchanged.
- `.:/app` or `.:/export`: The current directory is mounted to either `/app` or `/export` depending on the service. This allows access to local scripts and configurations.

## Using the FOV Notebook

The `fov.ipynb` notebook is designed to help you select the appropriate radar configuration for your dataset. To use it:

1. Start the Jupyter server:
   ```bash
   docker compose up jupyter
   ```

2. Open http://localhost:8888 in your browser or IDE

3. Navigate to `fov.ipynb` and run the cells to:
   - Visualize the radar's field of view
   - Test different FOV configurations
   - Generate the appropriate FOV parameters for your `export-config.yaml`

## Notes

- The Jupyter server is configured to run without authentication for development purposes
- The container paths (`/data/coloradar`, `/app`, `/export`) should not be changed as they are hardcoded in the scripts 