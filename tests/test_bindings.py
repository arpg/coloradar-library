import os


if __name__ == "__main__":
    CUDA_ENV = os.environ.get("CUDA_ENV", "true").lower() == "true"
    import coloradar_dataset_lib
    if CUDA_ENV:
        import coloradar_cuda_lib
