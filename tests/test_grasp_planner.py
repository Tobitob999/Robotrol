import unittest

from planning.grasp_planner import GraspPlanner
from perception.types import ObjectPose
from control.transforms import identity


class TestGraspPlanner(unittest.TestCase):
    def test_grasp_positions(self):
        cube_cfg = {
            "edge_length_mm": 50.0,
            "grasp_offset_mm": 25.0,
            "approach_distance_mm": 40.0,
            "lift_distance_mm": 60.0,
            "grasp_normal_axis": "z",
            "grasp_normal_sign": 1,
        }
        obj_pose = ObjectPose(
            frame_id="obj",
            parent_frame="base",
            matrix=identity(),
            position_mm=(0.0, 0.0, 0.0),
            quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            confidence=1.0,
        )
        planner = GraspPlanner(cube_cfg)
        plan = planner.plan(obj_pose)
        self.assertAlmostEqual(plan.grasp_pose[2][3], 25.0, places=6)
        self.assertAlmostEqual(plan.approach_pose[2][3], 65.0, places=6)
        self.assertAlmostEqual(plan.lift_pose[2][3], 85.0, places=6)


if __name__ == "__main__":
    unittest.main()
