import ast
from pathlib import Path
import unittest


SOURCE_PATH = (
    Path(__file__).parents[1]
    / 'd435_python_camera'
    / 'camera_node.py'
)


class CameraNodeContractTest(unittest.TestCase):
    def test_defines_required_entry_points(self):
        tree = ast.parse(SOURCE_PATH.read_text(encoding='utf-8'))
        class_names = {
            node.name for node in tree.body if isinstance(node, ast.ClassDef)
        }
        function_names = {
            node.name for node in tree.body if isinstance(node, ast.FunctionDef)
        }

        self.assertIn('D435PythonCameraNode', class_names)
        self.assertIn('main', function_names)

    def test_uses_python_sdk_without_cv_bridge(self):
        source = SOURCE_PATH.read_text(encoding='utf-8')

        self.assertNotIn('cv_bridge', source)
        self.assertIn('rs.stream.color', source)
        self.assertIn('rs.format.bgr8', source)
        self.assertIn('pipeline.stop()', source)


if __name__ == '__main__':
    unittest.main()
