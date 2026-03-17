import os

from robotrol.pickplace.control.config import load_all_configs
from robotrol.pickplace.control.pipeline import PickPlacePipeline
from robotrol.pickplace.control.robot import MockRobot
from robotrol.pickplace.control.gripper import MockGripper
from robotrol.pickplace.control.executor import TrajectoryExecutor
from robotrol.pickplace.perception.camera import build_camera
from robotrol.pickplace.perception.pose_estimator import PoseEstimator
from robotrol.pickplace.planning.grasp_planner import GraspPlanner
from robotrol.pickplace.planning.place_planner import PlacePlanner


def build_pipeline(base_dir: str):
    configs = load_all_configs(base_dir)
    frames_cfg = configs["system"].get("frames", {})

    camera = build_camera(configs["camera"])
    perception = PoseEstimator(
        camera=camera,
        perception_cfg=configs["perception"],
        markers_cfg=configs["markers"],
        transforms_cfg=configs["transforms"],
        frames_cfg=frames_cfg,
        camera_cfg=configs["camera"],
        tnt_cfg=configs.get("tnt", {}),
    )

    robot = MockRobot()
    gripper = MockGripper(configs["robot"].get("gripper", {}))
    executor = TrajectoryExecutor(robot, gripper, configs["robot"])

    grasp_planner = GraspPlanner(configs["cube"])
    place_planner = PlacePlanner(configs["grid"], configs["cube"])

    pipeline = PickPlacePipeline(
        perception=perception,
        grasp_planner=grasp_planner,
        place_planner=place_planner,
        executor=executor,
        robot=robot,
        gripper=gripper,
    )
    return pipeline, configs


def run():
    base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
    pipeline, configs = build_pipeline(base_dir)
    success_target = int(configs["system"].get("state_machine", {}).get("success_target", 1))
    successes = 0
    for _ in range(success_target):
        ok = pipeline.run_cycle()
        if not ok:
            break
        successes += 1
    return successes
