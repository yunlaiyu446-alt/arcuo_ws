from pathlib import Path
import unittest


ROOT = Path(__file__).parents[1]


class PackageAssetsTest(unittest.TestCase):
    def test_default_configuration_matches_existing_aruco_topics(self):
        config = (ROOT / 'config' / 'camera.yaml').read_text(encoding='utf-8')

        self.assertIn('image_topic: "/camera/color/image_raw"', config)
        self.assertIn(
            'camera_info_topic: "/camera/color/camera_info"',
            config,
        )
        self.assertIn('frame_id: "camera_color_optical_frame"', config)

    def test_launch_and_readme_are_present(self):
        launch = ROOT / 'launch' / 'single_d435.launch.py'
        readme = ROOT / 'README.md'

        self.assertTrue(launch.is_file())
        self.assertTrue(readme.is_file())
        self.assertIn('d435_python_camera_node', launch.read_text(encoding='utf-8'))
        self.assertIn(
            'colcon build --symlink-install --packages-select d435_python_camera',
            readme.read_text(encoding='utf-8'),
        )


if __name__ == '__main__':
    unittest.main()
