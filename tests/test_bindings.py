import os
import sys


if __name__ == "__main__":
    sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
    DOCKER_ENV = os.environ.get("DOCKER_ENV", "false").lower() == "true"
    CUDA_ENV = os.environ.get("CUDA_ENV", "true").lower() == "true"

    if DOCKER_ENV:
        build_dir = '/src/coloradar_library/build'
    else:
        cwd = os.getcwd()
        build_dir = os.path.join(cwd, "build")
    sys.path.append(build_dir)
    print(f'Importing bindings from {build_dir}')

    import coloradar_dataset_lib
    if CUDA_ENV:
        import coloradar_cuda_lib
