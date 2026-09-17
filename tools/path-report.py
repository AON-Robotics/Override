"""Summarize SD run logs: python tools/path-report.py <name>-runs.csv [...]."""
import argparse
from collections import Counter
import csv
import json
import math
from pathlib import Path


def summarize(rows):
    outcomes = Counter(row["result"] for row in rows)
    completed = [row for row in rows if row["result"] == "Completed"]

    def values(field):
        return [value for row in completed if math.isfinite(value := float(row[field]))]

    times = values("elapsed_ms")
    positions = values("position_error")
    headings = values("heading_error")
    return {
        "runs": len(rows),
        "completed": len(completed),
        "completion_percent": 100*len(completed)/len(rows) if rows else 0,
        "outcomes": dict(outcomes),
        "completed_mean_seconds": sum(times)/len(times)/1000 if times else None,
        "completed_max_seconds": max(times)/1000 if times else None,
        "completed_mean_position_error_inches": sum(positions)/len(positions) if positions else None,
        "completed_max_position_error_inches": max(positions) if positions else None,
        "completed_max_heading_error_degrees": max(headings) if headings else None,
        "truncated_logs": sum(row["truncated"] == "1" for row in rows),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("files", type=Path, nargs="+")
    args = parser.parse_args()
    try:
        for path in args.files:
            with path.open(newline="", encoding="utf-8-sig") as file:
                result = summarize(list(csv.DictReader(file)))
            print(json.dumps({"file": str(path), **result}, indent=2, allow_nan=False))
    except (OSError, ValueError, KeyError) as error:
        parser.exit(1, f"Cannot read run log: {error}\n")


if __name__ == "__main__":
    main()
