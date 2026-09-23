"""Paste a complete export into static/<name>.jerryio.txt, then build and upload.

Build-time only: validates the export, removes the known editor trailer, and
converts inches/handedness/final heading to native AON poses. Speeds are stored
as rounded bytes; editor segments supply stop indices and headings. The robot
anchors these poses to its live starting pose and follows them with AON
PurePursuit at the routine's RPM limit.
Do not edit include/aon/generated/static-path.hpp; it is a disposable artifact.
"""
from decimal import Decimal
import hashlib
import argparse
import math
import json
from pathlib import Path


def convert(text):
    rows = []
    for number, line in enumerate(text.splitlines(), 1):
        if line.strip() == "endData":
            break
        fields = line.split(",")
        if len(fields) != 3:
            raise ValueError(f"line {number}: expected x,y,speed")
        row = tuple(float(field) for field in fields)
        if not all(math.isfinite(value) for value in row) or not 0 <= row[2] <= 127:
            raise ValueError(f"line {number}: invalid coordinates or speed")
        rows.append(row)
    else:
        raise ValueError("missing endData")
    # This export variant appends a duplicate endpoint and an editor-only point.
    if len(rows) >= 3 and rows[-3] == rows[-2] and all(r[2] == 0 for r in rows[-3:]):
        rows = rows[:-2]
    if len(rows) < 2 or rows[-1][2] != 0:
        raise ValueError("at least two points and a zero-speed endpoint are required")
    if any(row[2] == 0 for row in rows[:-1]):
        raise ValueError("internal stop markers are not supported by geometry-only following")
    if any(a[:2] == b[:2] for a, b in zip(rows, rows[1:])):
        raise ValueError("consecutive duplicate positions")
    x0, y0, _ = rows[0]
    angle = math.atan2(rows[1][1] - y0, rows[1][0] - x0)
    c, s = math.cos(angle), math.sin(angle)
    points = [((x-x0)*c + (y-y0)*s, (x-x0)*s - (y-y0)*c) for x, y, _ in rows]
    heading = math.degrees(math.atan2(points[-1][1]-points[-2][1],
                                     points[-1][0]-points[-2][0])) % 360
    return [(x, y, heading if i == len(points)-1 else 0.0,
             max(1, round(rows[i][2])) if rows[i][2] else 0)
            for i, (x, y) in enumerate(points)]


def convert_route(text):
    points = convert(text)
    marker = "#PATH.JERRYIO-DATA "
    if marker not in text:
        return points, []
    try:
        metadata = json.loads(text.split(marker, 1)[1])
        if metadata.get("format", "LemLib v0.5") != "LemLib v0.5":
            raise ValueError("only LemLib v0.5 exports are supported")
        if metadata.get("gc", {}).get("uol", 2.54) != 2.54:
            raise ValueError("export coordinates must use inches (uol 2.54)")
        if len(metadata["paths"]) != 1:
            raise ValueError("one editor path per export is required")
        segments = metadata["paths"][0]["segments"]
        if not segments:
            raise ValueError("editor path has no segments")
        for segment in segments:
            if len(segment["controls"]) not in (2, 4):
                raise ValueError("segments require two line controls or four cubic controls")
            for control in segment["controls"]:
                if not all(math.isfinite(float(control[k])) for k in ("x", "y")):
                    raise ValueError("non-finite segment control")
        endpoints = [segment["controls"][-1] for segment in segments]
        rows = [tuple(map(float, line.split(",")))
                for line in text.split("endData", 1)[0].splitlines()][:len(points)]
        precision = max(0, max(-Decimal(field.strip()).as_tuple().exponent
                        for line in text.split("endData", 1)[0].splitlines()
                        for field in line.split(",")[:2]))
        key = lambda x, y: (round(x, precision), round(y, precision))
        if key(endpoints[-1]["x"], endpoints[-1]["y"]) != rows[-1][:2]:
            raise ValueError("editor endpoint disagrees with exported endpoint")
        previous = segments[0]["controls"][0]
        if key(previous["x"], previous["y"]) != rows[0][:2]:
            raise ValueError("editor start disagrees with exported start")
        for segment in segments:
            start = segment["controls"][0]
            if (start["x"], start["y"]) != (previous["x"], previous["y"]):
                raise ValueError("disconnected editor segments")
            previous = segment["controls"][-1]
        x0, y0, _ = rows[0]
        angle = math.atan2(rows[1][1]-y0, rows[1][0]-x0)
        stops, first = [], 0
        for stage, endpoint in enumerate(endpoints):
            x, y, heading = (float(endpoint[k]) for k in ("x", "y", "heading"))
            if not all(math.isfinite(v) for v in (x, y, heading)):
                raise ValueError("non-finite segment endpoint")
            matches = [i for i in range(first+1, len(rows)) if rows[i][:2] == key(x, y)]
            if stage == len(endpoints)-1:
                index = len(rows)-1
                if index <= first:
                    raise ValueError("segment endpoints must advance along the route")
            elif matches:
                if len(matches) > 1:
                    raise ValueError("ambiguous segment boundary sample; repeated endpoint coordinates require an unambiguous export")
                index = matches[0]
                # A later exact revisit cannot establish which editor segment
                # ended if an earlier chord already crosses this boundary.
                target = key(x, y)
                for i in range(first, index-1):
                    ax, ay, _ = rows[i]
                    dx, dy = rows[i+1][0]-ax, rows[i+1][1]-ay
                    t = ((target[0]-ax)*dx+(target[1]-ay)*dy)/(dx*dx+dy*dy)
                    if 0 < t < 1 and key(ax+t*dx, ay+t*dy) == target:
                        raise ValueError("ambiguous segment boundary sample; include its first crossing")
            else:
                raise ValueError("missing ordered segment boundary sample; re-export with segment endpoints included")
            native_heading = math.degrees(angle)+heading-90
            stops.append((index, native_heading))
            first = index
        points[-1] = (*points[-1][:2], stops[-1][1], points[-1][3])
        return points, stops
    except (KeyError, IndexError, TypeError, AttributeError) as error:
        raise ValueError("invalid editor segment metadata") from error


def render(routes):
    branches = []
    for name, (points, stops) in sorted(routes.items()):
        revision = int(hashlib.sha256(json.dumps((points, stops)).encode()).hexdigest()[:8], 16)
        values = ",\n".join("      {" + ", ".join(f"{v:.9f}" for v in row[:2]) + "}" for row in points)
        speeds = ", ".join(str(row[3]) for row in points)
        boundaries = ", ".join(f"{{{index}, {heading:.9f}}}" for index, heading in stops)
        branches.append("  if (name == " + json.dumps(name) + ") {\n"
                        "    static constexpr double xy[][2] = {\n" + values + "\n    };\n"
                        "    static constexpr std::uint8_t speeds[] = {" + speeds + "};\n"
                        "    route.points.reserve(sizeof(speeds));\n"
                        "    for (const auto& point : xy) route.points.emplace_back(point[0], point[1], 0);\n"
                        f"    route.points.back().theta = {points[-1][2]:.9f};\n"
                        "    route.speeds.assign(speeds, speeds+sizeof(speeds));\n"
                        "    route.stops = {" + boundaries + "};\n"
                        f"    route.revision = {revision}u;\n  }}")
    return '''// Generated by tools/generate-static-path.py; edit static/*.jerryio.txt.
#pragma once
#include "aon/controls/path.hpp"
#include <cmath>
#include <string_view>

namespace aon::generated {
struct StaticRoute : PathRoute {
  std::uint32_t revision = 0;
  struct Stop { std::size_t index; double heading; };
  std::vector<Stop> stops;
};
// Unknown names return an invalid empty route. Stops follow editor segment order.
inline StaticRoute staticRouteAt(const Pose& start, std::string_view name = "path") {
  StaticRoute route;
''' + " else\n".join(branches) + '''
  const double radians = start.theta * 3.14159265358979323846 / 180.0;
  const double c = std::cos(radians), s = std::sin(radians);
  for (auto& point : route.points) {
    const double x = point.x, y = point.y;
    point.x = start.x + x*c - y*s;
    point.y = start.y + x*s + y*c;
    point.theta += start.theta;
  }
  for (auto& stop : route.stops) stop.heading += start.theta;
  return route;
}
}  // namespace aon::generated
'''


def generate(source, output):
    files = sorted(source.glob("*.jerryio.txt")) if source.is_dir() else [source]
    routes = {}
    for file in files:
        try:
            text = file.read_text(encoding="utf-8-sig")
            name = file.name.removesuffix(".jerryio.txt")
            routes[name] = convert_route(text)
        except (ValueError, OSError) as error:
            raise ValueError(f"{file}: {error}") from error
    if not routes:
        raise ValueError(f"{source}: no *.jerryio.txt exports found")
    content = render(routes)
    # Make checks the directory every build to catch removed/renamed exports.
    # Keep the header timestamp stable when geometry and route names are unchanged.
    if output.exists() and output.read_text(encoding="utf-8") == content:
        return
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(content, encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    try:
        generate(args.source, args.output)
    except (ValueError, OSError) as error:
        parser.exit(1, f"Static path conversion failed: {error}\n")


if __name__ == "__main__":
    main()
