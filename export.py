"""
CURRENT LIB/IMAGE ISSUES:
 - EATS 8 GIGS OF RAM???
 - dataset init too demanding in terms of folder structure
 - expected folder structure not documented
 - doesn't build with GCC 11 / Ubuntu 20 / noetic
 - export fails on longboard run 0
 - export config classes not bound
 - some run/dataset methods not bound
 - device-run-dataset interaction need refactoring
 - need to update readme (including login to ghcr, export file format description)
 - need pip package with bindings

 UTILS / PYTHON ISSUES
 - demo notebook not integrated
 - no compose utils (need export, get sample export config)
 - no usage instructions
 -
"""


import os
import sys


if __name__ == "__main__":
    sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
    CUDA_ENV = os.environ.get("CUDA_ENV", "true").lower() == "true"

    cwd = os.getcwd()
    build_dir = os.path.join(cwd, "build")
    sys.path.append(build_dir)
    print(f'Importing bindings from {build_dir}')

    import coloradar_dataset_lib as ct

    COLORADAR_PATH = os.path.join(os.path.expanduser('~'), 'coloradar')
    dataset = ct.ColoradarDataset(COLORADAR_PATH)
    dataset.export_to_file('tmp_export_config.yaml')