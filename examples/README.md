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
Samples maps from the dataset. Similar to build_maps, this service uses your local data directory.

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