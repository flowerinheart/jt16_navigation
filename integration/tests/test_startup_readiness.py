import ast
from pathlib import Path
import unittest
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from benchmark_geometry import obstacle_observations_fresh, covariance_valid


class StartupReadinessTests(unittest.TestCase):
    def checker(self):
        source = Path(__file__).resolve().parents[1]/'scripts/run_nav2_goal.py'
        tree = ast.parse(source.read_text())
        method = next(x for x in tree.body if isinstance(x, ast.FunctionDef) and x.name == 'navigation_input_failure')
        namespace = dict(obstacle_observations_fresh=obstacle_observations_fresh,covariance_valid=covariance_valid,
                         obstacle_count=10,obstacle_stamps={i:8.12 for i in range(7)},fused=[[0,0,0,8.12]],
                         static_prediction_enabled=True,prediction_stamps=[8.12],timed_prediction_enabled=True,forecast_stamps=[8.12],
                         static_scan_enabled=True,static_scans=[[8.12]],amcl=[[0,0,0,8.12,0,0,0]])
        exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),namespace)
        return namespace

    def test_fast_localizer_does_not_make_missing_callbacks_ready(self):
        data = self.checker()
        self.assertEqual(data['navigation_input_failure'](8.22),'Benchmark obstacle observations stale')
        for i in range(7,10): data['obstacle_stamps'][i] = 8.22
        self.assertIsNone(data['navigation_input_failure'](8.22))

    def test_ready_once_does_not_exempt_later_stale_inputs(self):
        data = self.checker()
        data['obstacle_stamps'].update({i:8.22 for i in range(10)})
        self.assertIsNone(data['navigation_input_failure'](8.22))
        data['forecast_stamps'] = [7.]
        self.assertEqual(data['navigation_input_failure'](8.22),'LiDAR timed obstacle input stale')


if __name__ == '__main__': unittest.main()
