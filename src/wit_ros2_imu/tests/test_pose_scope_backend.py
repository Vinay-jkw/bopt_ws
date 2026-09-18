import os
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from wit_ros2_imu import pose_scope


class PoseScopeBackendTest(unittest.TestCase):
    def test_headless_backend_is_agg(self):
        os.environ.pop('DISPLAY', None)
        os.environ.pop('WAYLAND_DISPLAY', None)
        backend = pose_scope.select_matplotlib_backend()
        self.assertEqual(backend, 'Agg')


if __name__ == '__main__':
    unittest.main()
