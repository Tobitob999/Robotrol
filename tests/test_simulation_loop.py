import os
import unittest

from control.app import build_pipeline
from simulation.simulation_loop import SimulationRunner


class TestSimulationLoop(unittest.TestCase):
    def test_simulation(self):
        base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
        pipeline, configs = build_pipeline(base_dir)
        runner = SimulationRunner(pipeline, configs["perception"])
        successes = runner.run(3)
        self.assertEqual(successes, 3)


if __name__ == "__main__":
    unittest.main()
