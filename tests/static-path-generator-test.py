import importlib.util
import math
import sys
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("generator", ROOT / "tools/generate-static-path.py")
generator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(generator)


class StaticPathTests(unittest.TestCase):
    def test_checked_in_route(self):
        points = generator.convert((ROOT / "static/path.jerryio.txt").read_text())
        self.assertEqual(len(points), 48)
        self.assertEqual(points[0][:2], (0.0, 0.0))
        self.assertAlmostEqual(points[-1][1], 17.04, delta=0.05)
        self.assertAlmostEqual(points[-1][2], 182.1, delta=0.2)
        length = sum(math.dist(a[:2], b[:2]) for a, b in zip(points, points[1:]))
        self.assertAlmostEqual(length, 92.85, delta=0.05)

    def test_right_and_left_from_rotated_start(self):
        right = generator.convert("10,20,100\n10,30,100\n20,30,0\nendData\nmetadata")
        self.assertAlmostEqual(right[-1][1], 10)
        self.assertAlmostEqual(right[-1][2], 90)
        left = generator.convert("10,20,100\n10,30,100\n0,30,0\nendData")
        self.assertAlmostEqual(left[-1][1], -10)
        self.assertAlmostEqual(left[-1][2], 270)

    def test_rejects_invalid_or_unsupported_data(self):
        for text in ("", "0,0,100\n1,0,0", "0,0,100\nendData",
                     "0,0,100\n1,0,nan\nendData", "0,0,128\n1,0,0\nendData",
                     "0,0,100\n0,0,0\nendData", "0,0,100\n1,0,0\n2,0,0\nendData",
                     "0,0,100\n1,0,40\nendData", "0,0,100\n1,0,0,5\nendData"):
            with self.subTest(text=text), self.assertRaises(ValueError):
                generator.convert(text)

    def test_ordinary_endpoint_is_not_trimmed(self):
        self.assertEqual(len(generator.convert("0,0,100\n1,0,100\n2,0,0\nendData")), 3)


if __name__ == "__main__":
    unittest.main(testRunner=unittest.TextTestRunner(stream=sys.stdout))
