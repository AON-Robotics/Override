"""Exercise named export discovery and safe incremental generation."""
import importlib.util
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("generator", ROOT / "tools/generate-static-path.py")
generator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(generator)


class MultiPathTests(unittest.TestCase):
    def test_speed_export_is_preserved_and_rounded_to_bytes(self):
        points = generator.convert("0,0,127\n10,0,31.6\n20,0,0\nendData")
        self.assertEqual([p[3] for p in points], [127, 32, 0])
        self.assertEqual(points[-1][:3], (20.0, 0.0, 0.0))

    def test_invalid_motion_rows_are_rejected(self):
        for data in ("0,0,100\n0,0,0", "0,0,100\n1,0,0\n2,0,0",
                     "0,0,128\n1,0,0", "0,0,nan\n1,0,0"):
            with self.subTest(data=data), self.assertRaises(ValueError):
                generator.convert(data + "\nendData")

    def test_add_edit_rename_remove_and_unchanged(self):
        with tempfile.TemporaryDirectory() as folder:
            directory = Path(folder)
            output = directory / "routes.hpp"
            first = directory / "path.jerryio.txt"
            first.write_text("0,0,100\n10,0,0\nendData")
            generator.generate(directory, output)
            before = output.stat().st_mtime_ns
            generator.generate(directory, output)
            self.assertEqual(before, output.stat().st_mtime_ns)
            second = directory / "red-left.jerryio.txt"
            second.write_text("10,20,100\n10,30,100\n0,30,0\nendData\nmetadata")
            generator.generate(directory, output)
            content = output.read_text()
            self.assertIn('name == "path"', content)
            self.assertIn('name == "red-left"', content)
            self.assertIn('-10.000000000f', content)
            self.assertIn('270.000000000', content)
            self.assertNotIn('metadata', content)
            first.write_text("0,0,100\n24,0,0\nendData")
            generator.generate(directory, output)
            self.assertIn('24.000000000', output.read_text())
            second.rename(directory / "blue.jerryio.txt")
            generator.generate(directory, output)
            self.assertNotIn('name == "red-left"', output.read_text())
            self.assertIn('name == "blue"', output.read_text())
            first.unlink()
            generator.generate(directory, output)
            self.assertNotIn('name == "path"', output.read_text())

    def test_bad_export_fails_without_overwriting_previous_header(self):
        with tempfile.TemporaryDirectory() as folder:
            directory = Path(folder)
            output = directory / "routes.hpp"
            output.write_text("previous valid output")
            with self.assertRaisesRegex(ValueError, "no .* exports"):
                generator.generate(directory, output)
            (directory / "bad.jerryio.txt").write_text("0,0,100\n1,0,0")
            with self.assertRaisesRegex(ValueError, "bad.jerryio.txt.*missing endData"):
                generator.generate(directory, output)
            self.assertEqual(output.read_text(), "previous valid output")

    def test_existing_route_geometry(self):
        points = generator.convert((ROOT / "static/path.jerryio.txt").read_text())
        export = (ROOT / "static/path.jerryio.txt").read_text()
        metadata = json.loads(export.split("#PATH.JERRYIO-DATA ", 1)[1])
        start, end = metadata["paths"][0]["segments"][-1]["controls"]
        self.assertAlmostEqual(math.hypot(end["x"]-start["x"], end["y"]-start["y"]), 24)
        rows = export.split("endData", 1)[0].splitlines()
        terminal = tuple(map(float, rows[-3].split(",")[:2]))
        self.assertLess(math.dist(terminal, (end["x"], end["y"])), 0.001)
        self.assertEqual(len(points), 47)
        self.assertAlmostEqual(points[-1][1], 17.12, delta=0.05)
        self.assertAlmostEqual(points[-1][2], 182.1, delta=0.2)


if __name__ == "__main__":
    unittest.main(testRunner=unittest.TextTestRunner(stream=sys.stdout))
