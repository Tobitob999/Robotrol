import unittest

try:
    import numpy as np
except Exception:
    np = None

from chess.piece_detector import detect_pieces


class TestPieceDetector(unittest.TestCase):
    def test_detect_piece_on_white_square(self):
        if np is None:
            self.skipTest("numpy not available")
        size = 8
        sq = 10
        img = np.zeros((size * sq, size * sq), dtype=np.uint8)
        for r in range(size):
            for c in range(size):
                val = 200 if (r + c) % 2 == 0 else 50
                img[r * sq:(r + 1) * sq, c * sq:(c + 1) * sq] = val
        img[0 * sq + 2:0 * sq + 8, 0 * sq + 2:0 * sq + 8] = 20

        cfg = {
            "occupancy_threshold": 10.0,
            "min_piece_ratio": 0.05,
            "max_piece_ratio": 0.9,
            "light_piece_char": "P",
            "dark_piece_char": "p",
        }
        det = detect_pieces(img, size, sq, cfg, color_order="rgb")
        self.assertEqual(det.board[0][0], "p")


if __name__ == "__main__":
    unittest.main()
