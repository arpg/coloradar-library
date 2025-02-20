import unittest
from version_selector import VersionSelector, ImageNotFoundError


class TestBaseImage(unittest.TestCase):
    CUDA_INVALID = (1222.1222, 'abd', 12, '12', '12.2.3', ['12.4'])
    CUDA_UNSUPPORTED = ('1222.1222', '10.10')
    CUDA_VALID = ('11.8', '12.6')

    ROS_INVALID = ('ros', 123, ['noetic'], ' humble ')
    ROS_UNSUPPORTED = ('galactic', 'foxy')
    ROS_VALID = ('jazzy', 'humble', 'noetic')

    VALID_KWARGS = (
        {'cuda_version': '11.8', 'ros_version': 'noetic'},
        {'cuda_version': '12.4', 'ros_version': 'humble'},
        {'cuda_version': '12.6', 'ros_version': 'jazzy'}
    )

    EMPTY_VALS = (None, '', 'none', 'None', [], 0)


    def setUp(self):
        self.selector = VersionSelector()

    def test_no_arguments(self):
        for cuda in self.EMPTY_VALS:
            for ros in self.EMPTY_VALS:
                base_image = self.selector.get_base_image(cuda, ros, detect_local_cuda=False)
                self.assertEqual(base_image, "ubuntu:24.04")

    def test_invalid_cuda(self):
        for cuda in self.CUDA_INVALID:
            for ros in self.ROS_VALID + self.ROS_INVALID + self.ROS_UNSUPPORTED + self.EMPTY_VALS:
                try:
                    self.selector.get_base_image(cuda, ros, detect_local_cuda=False)
                except ValueError:
                    pass
                else:
                    print(cuda, ros)
                    raise

    def test_invalid_ros(self):
        for cuda in self.CUDA_VALID + self.CUDA_UNSUPPORTED + self.EMPTY_VALS:
            for ros in self.ROS_INVALID:
                try:
                    self.selector.get_base_image(cuda, ros, detect_local_cuda=False)
                except ValueError:
                    pass
                else:
                    print(cuda, ros)
                    raise

    def test_unsupported_ros(self):
        for cuda in self.CUDA_VALID + self.CUDA_UNSUPPORTED + self.EMPTY_VALS:
            for ros in self.ROS_UNSUPPORTED:
                try:
                    self.selector.get_base_image(cuda, ros, detect_local_cuda=False)
                except ValueError:
                    pass
                else:
                    print(cuda, ros)
                    raise

    def test_unsupported_cuda(self):
        for cuda in self.CUDA_UNSUPPORTED:
            for ros in self.ROS_VALID + self.EMPTY_VALS:
                try:
                    self.selector.get_base_image(cuda, ros, detect_local_cuda=False)
                except ImageNotFoundError:
                    pass
                else:
                    print(cuda, ros)
                    raise

    def test_valid_ros_base(self):
        for cuda in self.EMPTY_VALS:
            for ros in self.ROS_VALID:
                base_image = self.selector.get_base_image(cuda, ros, detect_local_cuda=False)
                assert base_image == f'ros:{ros}'

    def test_valid_nvidia_base(self):
        for kwargs in self.VALID_KWARGS:
            base_image = self.selector.get_base_image(detect_local_cuda=False, **kwargs)
            self.assertEqual(base_image, f'annazabnus/ros-cuda:{kwargs["cuda_version"]}-{kwargs["ros_version"]}')
            for ros in self.EMPTY_VALS:
                base_image = self.selector.get_base_image(kwargs['cuda_version'], ros, detect_local_cuda=False)
                assert base_image.startswith('nvidia/cuda:') and kwargs['cuda_version'] in base_image


if __name__ == "__main__":
    unittest.main()
