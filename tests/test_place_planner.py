import unittest

from planning.place_planner import PlacePlanner


class TestPlacePlanner(unittest.TestCase):
    def test_grid_position(self):
        grid_cfg = {
            "origin_mm": [100.0, 200.0, 0.0],
            "spacing_mm": [60.0, 70.0],
            "rows": 2,
            "cols": 2,
        }
        cube_cfg = {
            "edge_length_mm": 50.0,
            "approach_distance_mm": 40.0,
            "lift_distance_mm": 60.0,
        }
        planner = PlacePlanner(grid_cfg, cube_cfg)
        plan = planner.plan(3)
        self.assertAlmostEqual(plan.place_pose[0][3], 160.0, places=6)
        self.assertAlmostEqual(plan.place_pose[1][3], 270.0, places=6)


if __name__ == "__main__":
    unittest.main()
