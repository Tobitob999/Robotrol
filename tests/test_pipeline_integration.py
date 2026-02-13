import os
import unittest

from control.app import build_pipeline


class TestPipelineIntegration(unittest.TestCase):
    def test_single_cycle(self):
        base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
        pipeline, _ = build_pipeline(base_dir)
        ok = pipeline.run_cycle()
        self.assertTrue(ok)


if __name__ == "__main__":
    unittest.main()
