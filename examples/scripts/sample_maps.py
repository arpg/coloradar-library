import os
import coloradar_dataset_lib as tools


if __name__ == "__main__":
    dataset = tools.ColoradarPlusDataset(os.getenv('DOCKER_DATASET_VOLUME_PATH'))
    radar_config = dataset.cascade_config()

    # azimuth_max_bin, elevation_max_bin, max_range = 63, 4, 7
    # horizontal_fov = radar_config.azimuth_idx_to_fov_degrees(azimuth_max_bin)
    # vertical_fov = radar_config.elevation_idx_to_fov_degrees(elevation_max_bin)
    horizontal_fov, vertical_fov, max_range = 156.6, 32.2, 7
    print(f'Sampling lidar map in cascade FOV: {horizontal_fov} degrees azimuth, {vertical_fov} degrees elevation, {max_range} meters range.')
    
    for run in dataset.get_runs():
        print(f'Processing {run.name()}...')
        run.create_map_samples(
            sensor_timestamps=run.cascade_timestamps(), base_to_sensor_transform=dataset.cascade_transform(),
            horizontal_fov=horizontal_fov, vertical_fov=vertical_fov, range=max_range
        )
