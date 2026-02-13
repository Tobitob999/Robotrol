import unittest

from control.transforms import make_transform, matmul, invert_transform


class TestTransforms(unittest.TestCase):
    def test_inverse(self):
        R = [
            [1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0],
            [0.0, 1.0, 0.0],
        ]
        t = [10.0, 20.0, 30.0]
        T = make_transform(R, t)
        T_inv = invert_transform(T)
        I = matmul(T, T_inv)
        self.assertAlmostEqual(I[0][0], 1.0, places=6)
        self.assertAlmostEqual(I[1][1], 1.0, places=6)
        self.assertAlmostEqual(I[2][2], 1.0, places=6)
        self.assertAlmostEqual(I[0][3], 0.0, places=6)
        self.assertAlmostEqual(I[1][3], 0.0, places=6)
        self.assertAlmostEqual(I[2][3], 0.0, places=6)


if __name__ == "__main__":
    unittest.main()
