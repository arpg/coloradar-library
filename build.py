import argparse
import os
import re
import subprocess
import shutil
from typing import Optional
from utils.version_selector import VersionSelector, ImageNotFoundError


CACHE_FROM_DIR = os.getenv("DOCKER_CACHE_FROM", "/tmp/.buildx-cache")
CACHE_TO_DIR = os.getenv("DOCKER_CACHE_TO", "/tmp/.buildx-cache-new")
PUSH_IMAGES = os.getenv("DOCKER_PUSH_IMAGES", "").lower() == 'true'


class ImageBuild:
    def __init__(self, version_config_file: str = 'ros-versions.yaml'):
        self.default_builder = "ros-cuda-builder"
        self.dependencies = {
            "yq": "Please install with 'apt install yq'",
            "python3": "Please install Python 3.",
            "docker": "Please install Docker.",
        }
        self.image_name_prefix = "ghcr.io/arpg/coloradar-lib"
        self.version_selector = VersionSelector(version_config_file=version_config_file)

    def _check_dependencies(self) -> None:
        """Check if all required dependencies are installed."""
        for command, error_msg in self.dependencies.items():
            if shutil.which(command.split()[0]) is None:
                raise ValueError(f"{command} is not installed. {error_msg}")

    def _check_builder(self) -> None:
        """Ensure that a Buildx builder exists."""
        try:
            subprocess.run(["docker", "buildx", "version"], check=True, stdout=subprocess.DEVNULL)
        except subprocess.CalledProcessError:
            raise ValueError("Docker Buildx is not available. Please install it.")
        result = subprocess.run(["docker", "buildx", "ls"], capture_output=True, text=True)
        if self.default_builder not in result.stdout:
            print(f"No active Buildx builder found. Creating {self.default_builder}.")
            subprocess.run(["docker", "buildx", "create", "--name", self.default_builder, "--use"], check=True)
            subprocess.run(["docker", "buildx", "inspect", "--bootstrap"], check=True)

    def detect_local_cuda(self) -> Optional[str]:
        """Detect the local CUDA version in X.Y format if available."""
        print("Detecting local CUDA version...")
        try:
            cuda_version_output = subprocess.run(["nvcc", "--version"], capture_output=True, text=True, check=True).stdout
            match = re.search(r"release (\d+\.\d+)", cuda_version_output)
            if match:
                cuda_version = match.group(1)
                return cuda_version
        except (FileNotFoundError, subprocess.CalledProcessError):
            pass
        return None

    def setup(self) -> None:
        """Run initial setup checks."""
        self._check_dependencies()
        self._check_builder()

    def get_image_name(self, ros_distro: Optional[str], cuda_version: Optional[str]) -> str:
        """Generate the image name based on ROS and CUDA versions."""
        image_tag = f"{cuda_version or ''}"
        if ros_distro:
            if image_tag:
                image_tag += "-"
            image_tag += ros_distro
        if not image_tag:
            raise ValueError("Neither ROS distribution nor CUDA version specified.")
        return self.image_name_prefix + ":" + image_tag

    def run(self, ros_distro: Optional[str], cuda_version: Optional[str]) -> None:
        """Execute the build process."""
        cuda_version = cuda_version or self.detect_local_cuda()
        if cuda_version and cuda_version.lower() == 'none':
            cuda_version = None
        cuda_version = self.version_selector.validate_cuda_version(cuda_version)
        if cuda_version:
            print(f"Using CUDA version: {cuda_version}")

        ros_distro = self.version_selector.validate_ros_version(ros_distro)
        lib_versions =  self.version_selector.version_config.get(ros_distro, {})
        if ros_distro:
            print(f"Using ROS version: {ros_distro}")

        image_name = self.get_image_name(ros_distro=ros_distro, cuda_version=cuda_version)
        try:  # Not raising error when base image not found
            base_image = self.version_selector.get_base_image(cuda_version, ros_distro)
        except ImageNotFoundError as e:
            print(e)
            return image_name
        print(f'Using base image: {base_image}')

        print(f'Building image: {image_name}')
        build_command = [
            "docker", "buildx", "build",
            "--build-arg", f"BASE_IMAGE={base_image}",
            "--build-arg", f"DOCKER_GCC_VERSION={lib_versions.get('gcc', '')}",
            "--build-arg", f"DOCKER_BOOST_VERSION={lib_versions.get('boost', '')}",
            "--build-arg", f"DOCKER_PCL_VERSION={lib_versions.get('pcl', '')}",
            "--build-arg", f"DOCKER_PYBIND_VERSION={lib_versions.get('pybind', '')}",
            f"--cache-from=type=local,src={CACHE_FROM_DIR}",
            f"--cache-to=type=local,dest={CACHE_TO_DIR},mode=max",
            "-t", image_name,
            "--push" if PUSH_IMAGES else "--load",
            "."
        ]
        subprocess.run(build_command, check=True)


def main() -> None:
    """Run setup steps and build."""
    parser = argparse.ArgumentParser(description="Setup ROS + CUDA Docker image building.")
    parser.add_argument("--ros", required=False, help="ROS distribution (e.g., 'noetic', 'humble', etc.)")
    parser.add_argument("--cuda", required=False, help="CUDA version in X.Y format (e.g., '12.6')")
    parser.add_argument("--config", default="version-config.yaml", help="Path to the ROS version configuration file")
    args = parser.parse_args()

    build = ImageBuild(version_config_file=args.config)
    build.setup()
    build.run(ros_distro=args.ros, cuda_version=args.cuda)


if __name__ == "__main__":
    main()
