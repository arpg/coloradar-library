import argparse
import os
import re
import requests
import subprocess
import yaml
from typing import Optional, Dict


NVIDIA_DOCKERHUB_URL = "https://hub.docker.com/v2/repositories/nvidia/cuda/tags"
ROSCUDA_DOCKERHUB_URL = "https://hub.docker.com/v2/repositories/annazabnus/ros-cuda/tags"
IGNORE_OS_TAGS = {'runtime', 'cudnn'}


class ImageNotFoundError(Exception):
    pass

class CustomImageNotFoundError(ImageNotFoundError):
    pass


class VersionSelector:
    def __init__(self, version_config_file: str = 'version-config.yaml') -> None:
        """
        Initializes the VersionSelector with a given ROS version configuration file.
        """
        self.version_config = self._load_ros_config(version_config_file)
        self.default_version_settings = self.version_config['jazzy']
        self.available_ros_distros = ", ".join(str(v) for v in self.version_config.keys())
        self.mac_os = os.uname().sysname == "Darwin"
        self._cuda_tags_cache: Optional[dict] = None 
        self._roscuda_tags_cache: Optional[dict] = None 

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
        
    def _load_ros_config(self, version_config_file: str) -> Dict[Optional[str], Dict]:
        """
        Load ROS configuration from a YAML file.
        """
        if not os.path.isfile(version_config_file):
            raise FileNotFoundError(f"Configuration file not found: {version_config_file}")
        with open(version_config_file, 'r') as file:
            data = yaml.safe_load(file)
            return {k: v for k, v in data.get('ros_versions', {}).items()}
        
    def get_base_image(self, cuda_version: Optional[str], ros_version: Optional[str], detect_local_cuda: bool = True) -> str:
        """
        Determine the base image based on CUDA and ROS versions.
        """
        cuda_version = self.validate_cuda_version(cuda_version, detect_local=detect_local_cuda)
        ros_version = self.validate_ros_version(ros_version)
        if cuda_version:
            if ros_version:
                tag = f'{cuda_version}-{ros_version}'
                self.check_roscuda_tag(tag)
                return f'annazabnus/ros-cuda:{tag}'
            return f'nvidia/cuda:{self._get_latest_cuda_tag(cuda_version, "-ubuntu")}'
        if ros_version:
            return f'ros:{ros_version}' 
        return f'ubuntu:{self.default_version_settings["ubuntu"]}'

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
        lib_versions = self.version_config.get(ros_distro, {}) if ros_distro else self.default_version_settings
        if not lib_versions:
            return ''
        return lib_versions.get(lib_name, '') or ''

    def _get_latest_cuda_tag(self, cuda_version: str, base_image_postfix: str) -> str:
        """
        Query Docker Hub to find the latest patch version for a given CUDA X.Y version.
        Uses cached results if available.
        """
        if self._cuda_tags_cache is None:  # Fetch only once
            self._cuda_tags_cache = self._fetch_nvidia_tags()

        latest_patch = None
        latest_patch_version = -1
        for tag in self._cuda_tags_cache:
            if tag.startswith(cuda_version) and base_image_postfix in tag and not any(t in tag for t in IGNORE_OS_TAGS):
                match = re.match(rf"{cuda_version}\.(\d+)", tag)
                if match:
                    patch_version = int(match.group(1))
                    if patch_version > latest_patch_version:
                        latest_patch = tag
                        latest_patch_version = patch_version
        if latest_patch:
            return latest_patch
        else:
            raise ImageNotFoundError(f"No matching CUDA image found for version {cuda_version} containing {base_image_postfix}.")

    def _fetch_nvidia_tags(self) -> list:
        """
        Fetch all available CUDA tags from Docker Hub and return them as a list.
        """
        params = {"page_size": 100}
        base_url = NVIDIA_DOCKERHUB_URL
        tags = []
        try:
            while base_url:
                response = requests.get(base_url, params=params)
                response.raise_for_status()
                data = response.json()
                tags.extend(tag.get("name", "") for tag in data.get("results", []))
                base_url = data.get("next")
        except requests.RequestException as e:
            raise ValueError(f"Error fetching CUDA tags: {e}")
        return tags
    
    def check_roscuda_tag(self, tag: str) -> None:
        if not self._roscuda_tags_cache:
            self._roscuda_tags_cache = self._fetch_roscuda_tags()
        if tag not in self._roscuda_tags_cache:
            raise CustomImageNotFoundError(f"ROS-CUDA image '{tag}' not found at {ROSCUDA_DOCKERHUB_URL}.")
    
    def _fetch_roscuda_tags(self) -> list:
        """
        Fetch all available CUDA tags from Docker Hub and return them as a list.
        """
        params = {"page_size": 100}
        base_url = ROSCUDA_DOCKERHUB_URL
        tags = []
        try:
            while base_url:
                response = requests.get(base_url, params=params)
                response.raise_for_status()
                data = response.json()
                tags.extend(tag.get("name", "") for tag in data.get("results", []))
                base_url = data.get("next")
        except requests.RequestException as e:
            raise ValueError(f"Error fetching ros-cuda tags: {e}")
        return tags

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
    parser.add_argument("--config", default="version-config.yaml", help="Path to the ROS version configuration file.")
    args = parser.parse_args()

    selector = VersionSelector(version_config_file=args.config)
    image = selector.get_base_image(args.cuda, args.ros)
    print(image)


if __name__ == "__main__":
    main()
