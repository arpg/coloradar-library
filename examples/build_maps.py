import os
import coloradar_dataset_lib as tools


DOCKER_DATASET_VOLUME_PATH = '/data/coloradar'

if __name__ == "__main__":
    dataset = tools.ColoradarDataset(DOCKER_DATASET_VOLUME_PATH)

    for run in dataset.get_runs():
        if run.name.startswith('2_') or run.name.startswith('12_'):
            print('Building octomap for run:', run.name)
            run.create_lidar_octomap(
                map_resolution=0.25, base_to_lidar_transform=dataset.lidar_transform(),
                lidar_total_horizontal_fov=360, lidar_total_vertical_fov=33.2, lidar_max_range=40
            )
