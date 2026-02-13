from simulation.mock_world import MockWorld


class SimulationRunner:
    def __init__(self, pipeline, perception_cfg: dict):
        self.pipeline = pipeline
        self.world = MockWorld(perception_cfg.get("mock_pose", {}))

    def run(self, cycles: int):
        successes = 0
        for i in range(int(cycles)):
            T, pos, quat = self.world.pose_for_cycle(i)
            self.pipeline.context.perception.set_mock_pose(pos, quat, 0.99)
            ok = self.pipeline.run_cycle()
            if ok:
                successes += 1
        return successes
