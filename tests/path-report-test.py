import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location("report", Path(__file__).resolve().parents[1] / "tools/path-report.py")
report = importlib.util.module_from_spec(spec)
spec.loader.exec_module(report)


class ReportTests(unittest.TestCase):
    def test_failures_count_without_polluting_success_accuracy(self):
        rows = [dict(result="Completed", elapsed_ms="1000", position_error="1", heading_error="2", truncated="0"),
                dict(result="Completed", elapsed_ms="3000", position_error="3", heading_error="4", truncated="1"),
                dict(result="Timed out", elapsed_ms="30000", position_error="nan", heading_error="nan", truncated="0")]
        value = report.summarize(rows)
        self.assertEqual(value["runs"], 3)
        self.assertEqual(value["completed"], 2)
        self.assertAlmostEqual(value["completion_percent"], 200/3)
        self.assertEqual(value["completed_mean_seconds"], 2)
        self.assertEqual(value["completed_max_position_error_inches"], 3)
        self.assertEqual(value["truncated_logs"], 1)

    def test_no_completed_runs_has_no_invented_accuracy(self):
        value = report.summarize([])
        self.assertIsNone(value["completed_mean_seconds"])
        self.assertEqual(value["runs"], 0)


if __name__ == "__main__":
    unittest.main()
