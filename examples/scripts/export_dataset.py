import os
import coloradar_dataset_lib as tools


if __name__ == "__main__":
    dataset = tools.ColoradarDataset(os.getenv('DOCKER_DATASET_VOLUME_PATH'))
    dataset.export_to_file('/export/export-config.yaml')
