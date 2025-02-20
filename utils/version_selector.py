import argparse
import os
import re
import subprocess
import yaml
from typing import Optional, Dict


class ImageNotFoundError(Exception):
    pass


class CustomImageNotFoundError(ImageNotFoundError):
    pass


class VersionSelector:
    def __init__(self, version_config_file: str = 'version-config.yaml') -> None:
        """
        Initializes the ImageSelector with a given ROS version configuration file.
        """
        self.default_ros = "jazzy"
        self.version_config = self._load_ros_config(version_config_file)
        self.lib_versions = self.version_config.get(self.default_ros, {})
        self.available_ros_distros = ", ".join(str(v) for v in self.version_config.keys())
        self.mac_os = os.uname().sysname == "Darwin"

    def _load_ros_config(self, version_config_file: str) -> Dict[Optional[str], Dict]:
        """
        Load ROS configuration from a YAML file.
        """
        if not os.path.isfile(version_config_file):
            raise FileNotFoundError(f"Configuration file not found: {version_config_file}")
        with open(version_config_file, 'r') as file:
            data = yaml.safe_load(file)
            return {k: v for k, v in data.get('ros_versions', {}).items()}

    def validate_cuda_version(self, cuda_version: Optional[str], detect_local: bool = True) -> Optional[str]:
        """
        Validate that the input CUDA version is in the correct X.Y format.
        """
        if not cuda_version:
            if detect_local and not self.mac_os:
                return self.detect_local_cuda()
            return None
        if not isinstance(cuda_version, str):
            raise ValueError(f"CUDA version {cuda_version} is not a string.")
        if cuda_version.lower() == 'none':
            return None
        if self.mac_os:
            raise ValueError("Error: CUDA not available on Mac.")
        if not re.match(r"^\d+\.\d+$", cuda_version):
            raise ValueError("Error: CUDA version must be in the format 'X.Y' where X and Y are numeric.")
        return cuda_version

    def validate_ros_version(self, ros_version: Optional[str]) -> Optional[str]:
        """
        Validate that the input ROS version is available.
        """
        if not ros_version:
            return None
        if not isinstance(ros_version, str):
            raise ValueError(f"ROS version {ros_version} is not a string.")
        if ros_version.lower() == 'none':
            return None
        if ros_version not in self.version_config:
            raise ValueError(f"Error: ROS version must be one of the following: {self.available_ros_distros}, not {ros_version}")
        return ros_version

    def get_base_image(self, cuda_version: Optional[str], ros_version: Optional[str]) -> str:
        """
        Determine the base image based on CUDA and ROS versions.
        """
        cuda_version = self.validate_cuda_version(cuda_version, detect_local=False)
        ros_version = self.validate_ros_version(ros_version)

        image_name = "annazabnus/ros-cuda" if cuda_version else "ros"
        tag = f'{cuda_version}-{ros_version}' if cuda_version else ros_version
        base_image = f'{image_name}:{tag}'

        if not self._image_exists(base_image):
            if base_image.startswith("annazabnus"):
                raise CustomImageNotFoundError(f"Custom image '{base_image}' not found.")
            raise ImageNotFoundError(f"Base image '{base_image}' not found.")
        return base_image

    def _image_exists(self, image: str) -> bool:
        """
        Check if a Docker image exists locally or in the registry.
        """
        result = subprocess.run(
            ["docker", "manifest", "inspect", image],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        return result.returncode == 0

    def get_lib_version(self, lib_name: str, ros_distro: Optional[str] = None) -> Optional[str]:
        lib_versions = self.version_config.get(ros_distro, {}) or self.lib_versions
        if not lib_versions:
            return ''
        return lib_versions.get(lib_name, '') or ''

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


def main():
    parser = argparse.ArgumentParser(description="Determine the base Docker image for CUDA and ROS.")
    parser.add_argument("--cuda", required=False, help="CUDA version in X.Y format (e.g., 12.6).")
    parser.add_argument("--ros", required=False, help="ROS version (e.g., noetic).")
    parser.add_argument("--config", default="ros-versions.yaml", help="Path to the ROS version configuration file.")
    args = parser.parse_args()

    selector = VersionSelector(version_config_file=args.config)
    image = selector.determine_base_image(args.cuda, args.ros)
    print(image)


if __name__ == "__main__":
    main()
