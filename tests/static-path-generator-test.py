"""Exercise named export discovery and safe incremental generation."""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("generator", ROOT / "tools/generate-static-path.py")
generator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(generator)


def corner_export():
    metadata = {"paths": [{"segments": [
        {"controls": [{"x": 10, "y": 20}, {"x": 10, "y": 24, "heading": 0}]},
        {"controls": [{"x": 10, "y": 24}, {"x": 14, "y": 24, "heading": 90}]},
    ]}]}
    return "10,20,100\n10,22,80\n10,24,60\n14,24,0\nendData\n#PATH.JERRYIO-DATA " + json.dumps(metadata)


class MultiPathTests(unittest.TestCase):
    def test_segment_endpoints_are_emitted_from_editor_metadata(self):
        # North then east in editor coordinates becomes forward then right.
        export = corner_export()
        points, stops = generator.convert_route(export)
        self.assertEqual(stops, [(2, 0.0), (3, 90.0)])
        self.assertAlmostEqual(points[2][0], 4)
        self.assertAlmostEqual(points[2][1], 0)
        self.assertAlmostEqual(points[3][0], 4)
        self.assertAlmostEqual(points[3][1], 4)
        self.assertEqual([p[3] for p in points], [100, 80, 60, 0])

    def test_unsampled_segment_endpoint_is_rejected(self):
        metadata = {"paths": [{"segments": [
            {"controls": [{"x": 0, "y": 0}, {"x": 3, "y": 0, "heading": 90}]},
            {"controls": [{"x": 3, "y": 0}, {"x": 10, "y": 0, "heading": 90}]},
        ]}]}
        export = "0,0,100\n2,0,80\n4,0,60\n10,0,0\nendData\n#PATH.JERRYIO-DATA " + json.dumps(metadata)
        with self.assertRaisesRegex(ValueError, "boundary sample"):
            generator.convert_route(export)

    def test_unsampled_boundary_cannot_bind_to_a_later_revisit(self):
        metadata = {"paths": [{"segments": [
            {"controls": [{"x": 0, "y": 0}, {"x": 3, "y": 0, "heading": 90}]},
            {"controls": [{"x": 3, "y": 0}, {"x": 10, "y": 0, "heading": 90}]},
            {"controls": [{"x": 10, "y": 0}, {"x": 20, "y": 0, "heading": 90}]},
        ]}]}
        samples = "0,0,100\n2,0,100\n4,0,100\n10,0,100\n3,0,100\n20,0,0\nendData\n#PATH.JERRYIO-DATA "
        with self.assertRaisesRegex(ValueError, "boundary sample"):
            generator.convert_route(samples + json.dumps(metadata))
        # The misplaced first boundary must fail even if later stops are present.
        metadata["paths"][0]["segments"] = [
            metadata["paths"][0]["segments"][0],
            {"controls": [{"x": 3, "y": 0}, {"x": 20, "y": 0, "heading": 90}]},
        ]
        with self.assertRaisesRegex(ValueError, "boundary sample"):
            generator.convert_route(samples + json.dumps(metadata))

    def test_final_stop_uses_terminal_occurrence(self):
        metadata = {"paths": [{"segments": [{"controls": [
            {"x": 0, "y": 0}, {"x": 10, "y": 0, "heading": 90}]}]}]}
        export = "0,0,100\n10,0,100\n10,10,100\n10,0,0\nendData\n#PATH.JERRYIO-DATA " + json.dumps(metadata)
        _, stops = generator.convert_route(export)
        self.assertEqual(stops, [(3, 0.0)])

    def test_repeated_boundary_coordinate_is_rejected_as_ambiguous(self):
        metadata = {"paths": [{"segments": [
            {"controls": [{"x": 0, "y": 0}, {"x": 2, "y": 0, "heading": 90}]},
            {"controls": [{"x": 2, "y": 0}, {"x": 4, "y": 0, "heading": 45}]},
        ]}]}
        export = "0,0,100\n2,0,100\n3,0,100\n2,0,100\n4,0,0\nendData\n#PATH.JERRYIO-DATA " + json.dumps(metadata)
        with self.assertRaisesRegex(ValueError, "ambiguous"):
            generator.convert_route(export)

    def test_terminal_heading_is_owned_by_editor_metadata(self):
        metadata = {"paths": [{"segments": [{"controls": [
            {"x": 0, "y": 0}, {"x": 10, "y": 0, "heading": 45}]}]}]}
        export = "0,0,100\n10,0,0\nendData\n#PATH.JERRYIO-DATA " + json.dumps(metadata)
        points, stops = generator.convert_route(export)
        self.assertEqual(stops, [(1, -45.0)])
        self.assertEqual(points[-1][2], -45.0)

    def test_malformed_metadata_fails_generation(self):
        with self.assertRaises(ValueError):
            generator.convert_route("0,0,100\n10,0,0\nendData\n#PATH.JERRYIO-DATA {bad}")

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
            self.assertIn('-10.000000000', content)
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

    def test_inconsistent_editor_segments_are_rejected(self):
        samples, encoded = corner_export().split("#PATH.JERRYIO-DATA ")
        for case in ("start", "join", "units", "format", "controls", "nonfinite"):
            metadata = json.loads(encoded)
            segments = metadata["paths"][0]["segments"]
            if case == "start": segments[0]["controls"][0]["x"] = 99
            if case == "join": segments[1]["controls"][0]["x"] = 99
            if case == "units": metadata["gc"] = {"uol": 1}
            if case == "format": metadata["format"] = "unsupported"
            if case == "controls": segments[0]["controls"] = segments[0]["controls"][-1:]
            if case == "nonfinite": segments[0]["controls"][0]["x"] = float("nan")
            with self.subTest(case=case), self.assertRaises(ValueError):
                generator.convert_route(samples + "#PATH.JERRYIO-DATA " + json.dumps(metadata))

    def test_decimal_precision_and_scientific_notation(self):
        metadata = {"paths": [{"segments": [{"controls": [
            {"x": 0, "y": 0}, {"x": 1.2345674, "y": 0, "heading": 90}]}]}]}
        for endpoint in ("1.234567", "1234567e-6"):
            points, stops = generator.convert_route(
                "0,0,100\n" + endpoint + ",0,0\nendData\n#PATH.JERRYIO-DATA " + json.dumps(metadata))
            self.assertEqual(stops, [(1, 0)])
            self.assertAlmostEqual(points[-1][0], 1.234567, places=9)

    def test_self_crossing_away_from_boundaries_is_supported(self):
        metadata = {"paths": [{"segments": [
            {"controls": [{"x": 0, "y": 0}, {"x": 4, "y": 0, "heading": 90}]},
            {"controls": [{"x": 4, "y": 0}, {"x": 6, "y": 0, "heading": 180}]},
        ]}]}
        samples = "0,0,100\n2,2,80\n0,2,60\n2,0,40\n4,0,20\n6,0,0\nendData\n"
        _, stops = generator.convert_route(samples + "#PATH.JERRYIO-DATA " + json.dumps(metadata))
        self.assertEqual(stops, [(4,45), (5,135)])

    def test_invalid_boundaries_and_metadata_are_rejected(self):
        samples, encoded = corner_export().split("#PATH.JERRYIO-DATA ")
        for case in ("final", "heading", "paths", "empty", "order"):
            metadata = json.loads(encoded)
            segments = metadata["paths"][0]["segments"]
            if case == "final": segments[-1]["controls"][-1]["x"] = 99
            if case == "heading": del segments[0]["controls"][-1]["heading"]
            if case == "paths": metadata["paths"] *= 2
            if case == "empty": metadata["paths"][0]["segments"] = []
            if case == "order": segments.reverse()
            with self.subTest(case=case), self.assertRaises(ValueError):
                generator.convert_route(samples + "#PATH.JERRYIO-DATA " + json.dumps(metadata))

    def test_shipped_exports_generate(self):
        for file in (ROOT / "static").glob("*.jerryio.txt"):
            with self.subTest(file=file.name):
                points, stops = generator.convert_route(file.read_text())
                self.assertGreaterEqual(len(points), 2)
                self.assertEqual(points[-1][3], 0)
                if stops:
                    self.assertEqual(stops[-1][0], len(points)-1)


if __name__ == "__main__":
    unittest.main(testRunner=unittest.TextTestRunner(stream=sys.stdout))
