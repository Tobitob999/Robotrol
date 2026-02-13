import unittest

try:
    import numpy as np
    import cv2
except Exception:
    np = None
    cv2 = None

from perception.tnt_detector import detect_tnt_pose


class TestTntDetector(unittest.TestCase):
    def test_detect_tnt_pose(self):
        if np is None or cv2 is None:
            self.skipTest("numpy/cv2 not available")
        img = np.zeros((240, 320, 3), dtype=np.uint8)
        img[:, :] = (50, 50, 50)
        cv2.rectangle(img, (120, 80), (200, 160), (0, 0, 255), -1)

        camera_cfg = {
            "intrinsics": {"fx": 300.0, "fy": 300.0, "cx": 160.0, "cy": 120.0},
            "distortion": [0, 0, 0, 0, 0],
        }
        tnt_cfg = {
            "edge_length_mm": 50.0,
            "face_offset_mm": 25.0,
            "min_area_px": 500.0,
            "color_order": "bgr",
            "hsv_red_ranges": [
                [0, 80, 50, 10, 255, 255],
                [170, 80, 50, 180, 255, 255]
            ],
            "morph_kernel": 3
        }
        pose = detect_tnt_pose(img, camera_cfg, tnt_cfg)
        self.assertIsNotNone(pose)


if __name__ == "__main__":
    unittest.main()
