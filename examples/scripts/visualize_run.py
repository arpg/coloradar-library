import os
import argparse
import coloradar_dataset_lib as tools


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', type=str, required=True, help='Name of the run to visualize')
    parser.add_argument('--step', type=int, default=1, help='Frame increment for visualization')
    args = parser.parse_args()

    dataset_path = os.getenv('DOCKER_DATASET_VOLUME_PATH')

    dataset = tools.ColoradarPlusDataset(dataset_path)
    run = dataset.get_run(args.name)
    radar_config = dataset.cascade_config()
    lidar_transform, radar_transform = dataset.lidar_transform(), dataset.cascade_transform()
    
    visualizer = tools.DatasetVisualizer(radar_config, lidar_transform, radar_transform, frame_increment=args.step, camera_config_path='/export/camera_config.txt')
    visualizer.visualize(run, use_prebuilt_map=False)
