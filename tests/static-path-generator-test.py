"""Exercise named export discovery and safe incremental generation."""
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("generator", ROOT / "tools/generate-static-path.py")
generator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(generator)


class MultiPathTests(unittest.TestCase):
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
            self.assertIn('-10.000000000, 270.000000000', content)
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
        self.assertEqual(len(points), 48)
        self.assertAlmostEqual(points[-1][1], 17.04, delta=0.05)
        self.assertAlmostEqual(points[-1][2], 182.1, delta=0.2)


if __name__ == "__main__":
    unittest.main(testRunner=unittest.TextTestRunner(stream=sys.stdout))
