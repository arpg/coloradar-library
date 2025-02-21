import argparse
import os
import re
import subprocess
import shutil
from typing import Optional
from utils.version_selector import VersionSelector, CustomImageNotFoundError


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
            image_tag = "latest"
        return self.image_name_prefix + ":" + image_tag

    def run(self, ros_distro: Optional[str], cuda_version: Optional[str]) -> None:
        """Execute the build process."""
        cuda_version = self.version_selector.validate_cuda_version(cuda_version, detect_local=True)
        if cuda_version:
            print(f"Using CUDA version: {cuda_version}")

        ros_distro = self.version_selector.validate_ros_version(ros_distro)
        if ros_distro:
            print(f"Using ROS version: {ros_distro}")

        image_name = self.get_image_name(ros_distro=ros_distro, cuda_version=cuda_version)
        try:
            base_image, ubuntu_version = self.version_selector.get_base_image(cuda_version, ros_distro, detect_local_cuda=False)
        # Not raising error when custom base image not found
        except CustomImageNotFoundError as e:
            print(e)
            print(f'Aborting build of {image_name}')
            return
        print(f'Using base image: {base_image}')

        print(f'Building image: {image_name}')
        build_args = []
        for lib_name in self.version_selector.default_version_settings:
            build_args.append("--build-arg")
            build_args.append(f"DOCKER_{lib_name.upper()}_VERSION={self.version_selector.get_lib_version(lib_name, ubuntu_version)}")
        build_command = [
            "docker", "buildx", "build",
            "--build-arg", f"CUDA_ENV={'true' if cuda_version else 'false'}",
            "--build-arg", f"BASE_IMAGE={base_image}",
            "-t", image_name
        ] + build_args
        if PUSH_IMAGES:
            build_command.append("--push")
            if not cuda_version:
                build_command.extend(["--platform", "linux/amd64,linux/arm64"])
        else:
            build_command.append("--load")
        build_command.append(".")
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
