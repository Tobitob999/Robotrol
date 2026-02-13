import unittest

from control.fsm import PickPlaceStateMachine


class DummyContext:
    def __init__(self):
        self.calls = []

    def detect(self):
        self.calls.append("detect")

    def plan_grasp(self):
        self.calls.append("plan_grasp")

    def execute_pick(self):
        self.calls.append("execute_pick")

    def plan_place(self):
        self.calls.append("plan_place")

    def execute_place(self):
        self.calls.append("execute_place")

    def safe_stop(self):
        self.calls.append("safe_stop")


class TestStateMachine(unittest.TestCase):
    def test_cycle(self):
        sm = PickPlaceStateMachine()
        ctx = DummyContext()
        ok = sm.run_cycle(ctx)
        self.assertTrue(ok)
        self.assertEqual(
            ctx.calls,
            ["detect", "plan_grasp", "execute_pick", "plan_place", "execute_place"],
        )


if __name__ == "__main__":
    unittest.main()
