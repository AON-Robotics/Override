"""Guided path calibration, bounded candidate search, and independent validation.
See docs/path-autons.md. Uses only the Python standard library.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics

HARDWARE = ("wheel_diameter", "gear_ratio", "drive_width", "tracking_diameter", "base_accel", "base_decel")
FIELDS = ("maximumRpm", "lookahead", "lookaheadAtSpeed", "positionTolerance", "headingTolerance",
          "lateralAcceleration", "settleMs", "settledRpm", "timeoutMs", "accelerationScale",
          "decelerationScale", "turnAccelerationScale", "turnDecelerationScale")
ROUTES = ("straight", "curve", "uturn")
BOUNDS = dict(zip(FIELDS, ((1,200),(2,18),(2,18),(0.01,2),(0.01,5),(0.01,60),
                         (50,2000),(0,5),(100,30000),(0.1,2),(0.1,2),(0.1,2),(0.1,2))))


def number(value, low=0, high=math.inf):
    result = float(value)
    if not math.isfinite(result) or not low <= result <= high:
        raise ValueError(f"number outside [{low}, {high}]: {value}")
    return result


def read_csv(path):
    with Path(path).open(newline="", encoding="utf-8-sig") as file:
        reader = csv.DictReader(file)
        if not reader.fieldnames or len(set(reader.fieldnames)) != len(reader.fieldnames):
            raise ValueError(f"missing or duplicate CSV columns: {path}")
        rows = list(reader)
        if any(None in row or None in row.values() for row in rows):
            raise ValueError(f"incomplete CSV row: {path}")
        return rows


def save_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False)+"\n", encoding="utf-8")


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"))


def calibrate(rows, tracking_diameter):
    if len(rows) < 3:
        raise ValueError("at least three independently measured distance/turn trials are required")
    diameter = number(tracking_diameter,0.01,20)
    scales = [number(r["actual_distance"],0.01)/number(r["reported_distance"],0.01) for r in rows]
    heading_errors = [abs(number(r["actual_turn"],1,360)-number(r["reported_turn"],1,360)) for r in rows]
    scale = statistics.median(scales)
    if max(scales)-min(scales) > scale*0.05:
        raise ValueError("distance calibration varies by more than 5%; inspect sensing/slip and repeat")
    return dict(distance_scale=scale, tracking_diameter=diameter*scale,
                measured_tracking_diameter=diameter, max_heading_error=max(heading_errors),
                trials=len(rows), ready=abs(scale-1) <= 0.02 and max(heading_errors) <= 2,
                note="Apply geometry corrections, then repeat calibration and collect fresh baseline logs.")


def validate_options(options):
    if set(options) != set(FIELDS):
        raise ValueError("profile fields do not match the tuning schema")
    for field in FIELDS:
        value = number(options[field], *BOUNDS[field])
        if field in ("settleMs", "timeoutMs") and value != int(value):
            raise ValueError(f"{field} must be an integer")


def validate_plan(plan):
    if plan["version"] != 1 or plan["calibration"].get("ready") is not True:
        raise ValueError("unsupported plan or uncalibrated sensing")
    if set(plan["hardware"]) != set(HARDWARE) or set(plan["revisions"]) != set(ROUTES):
        raise ValueError("incomplete robot/route context")
    for value in plan["hardware"].values(): number(value,0.000001)
    for value in plan["revisions"].values():
        if number(value,1,2**32-1) != int(value): raise ValueError("invalid route revision")
    r = plan["requirements"]
    number(r["position"],0.01,2); number(r["heading"],0.01,5); number(r["cross_track"],0.01,20)
    if number(r["repeats"],3,100) != int(r["repeats"]): raise ValueError("invalid repeat count")
    if not plan["profiles"]: raise ValueError("empty candidate plan")
    for identity,options in plan["profiles"].items():
        validate_options(options)
        if number(identity,1,2**32-1) != int(identity): raise ValueError("invalid profile ID")
        if options["positionTolerance"] != r["position"] or options["headingTolerance"] != r["heading"]:
            raise ValueError("candidate must preserve required arrival tolerances")


def write_profile(path, profile, hardware, options):
    validate_options(options)
    identity = number(profile,1,2**32-1)
    if identity != int(identity):
        raise ValueError("profile ID must be an integer")
    values = [identity] + [number(hardware[k],0.000001) for k in HARDWARE] + [options[k] for k in FIELDS]
    # No partial profile is emitted when validation fails.
    Path(path).write_text("AON_PATH_TUNING_V1\n" + ",".join(format(v,".17g") for v in values)+"\n", encoding="ascii")


def load_runs(paths):
    runs, seen = [], set()
    expanded = []
    for path in map(Path,paths):
        if path.is_dir():
            expanded.extend(file for route in ROUTES if (file := path/f"aon-{route}-v2-runs.csv").exists())
        else: expanded.append(path)
    for path in expanded:
        route = next((r for r in ROUTES if path.name == f"aon-{r}-v2-runs.csv"), None)
        if route is None:
            raise ValueError(f"expected an aon-straight/curve/uturn-v2-runs.csv: {path}")
        samples = read_csv(path.with_name(path.name.replace("-runs.csv", ".csv")))
        by_run = {}
        for sample in samples:
            by_run.setdefault(sample["run"], []).append(sample)
        for row in read_csv(path):
            # Byte-offset run IDs remain unique across appends and brain reboots.
            key = (route, row["run"])
            if key in seen:
                raise ValueError(f"duplicate run {key}; do not mix copied or reset log files")
            seen.add(key)
            hardware = {k:number(row[k],0.000001) for k in HARDWARE}
            options = {k:number(row[k], *BOUNDS[k]) for k in FIELDS}
            validate_options(options)
            trace = by_run.pop(row["run"], [])
            count = number(row["samples"],0,320)
            if len(trace) != count or count != int(count) or (not trace and row["result"] == "Completed"):
                raise ValueError(f"missing/incomplete trace for {key}")
            if row["truncated"] not in ("0", "1"):
                raise ValueError("invalid truncated flag")
            last = -1
            for sample in trace:
                ms = number(sample["ms"],0,2**32-1)
                if ms <= last: raise ValueError(f"unordered trace for {key}")
                last = ms
                for field in ("left_cmd","right_cmd","left_rpm","right_rpm","heading","x","y"):
                    number(sample[field],-1e9,1e9)
                number(sample["cross_track"],0)
                if sample["aligning"] not in ("0", "1"): raise ValueError("invalid alignment flag")
            elapsed = number(row["elapsed_ms"],0,2**32-1)
            # A pre-motion cancellation/disable has no observed samples. A
            # stopped run's summary clock may tick just after the final callback.
            if trace and row["truncated"] == "0" and not 0 <= elapsed-last <= 20:
                raise ValueError(f"missing final braking sample for {key}")
            for field, low in (("profile",0),("revision",1),("start_ms",0),("run",0)):
                value = number(row[field],low,2**32-1)
                if value != int(value): raise ValueError(f"non-integer {field}")
            identity = (route, row["run"], row["start_ms"], row["profile"], row["revision"])
            runs.append(dict(route=route, run=row["run"], identity=list(identity),
                             profile=int(number(row["profile"],0,2**32-1)),
                             revision=int(number(row["revision"],1,2**32-1)),
                             hardware=hardware, options=options, trace=trace,
                             result=row["result"], elapsed=elapsed, truncated=row["truncated"] == "1",
                             position=number(row["position_error"]), heading=number(row["heading_error"],0,180)))
        if by_run:
            raise ValueError(f"orphan trace rows in {path}")
    if not runs: raise ValueError("no runs found")
    return runs


def check_context(runs, hardware, revisions):
    for run in runs:
        if run["hardware"] != hardware or run["revision"] != revisions[run["route"]]:
            raise ValueError("robot geometry, motion constants, or route revision changed; create a fresh plan")


def response_estimates(runs):
    """Observed closed-loop response, not an estimate of the traction limit."""
    accel, decel, turn_accel, turn_decel, lateral, widths = [], [], [], [], [], []
    for run in runs:
        h = run["hardware"]
        inches_per_rpm_second = math.pi*h["wheel_diameter"]*h["gear_ratio"]/60
        for a,b in zip(run["trace"],run["trace"][1:]):
            dt = (float(b["ms"])-float(a["ms"]))/1000
            if not 0.05 <= dt <= 0.2: continue
            la,ra,lb,rb = (float(s[k]) for s,k in ((a,"left_rpm"),(a,"right_rpm"),(b,"left_rpm"),(b,"right_rpm")))
            yaw = math.radians(math.remainder(float(b["heading"])-float(a["heading"]),360))/dt
            v = (lb+rb)/2*inches_per_rpm_second
            if b["aligning"] == "0":
                change = (abs((lb+rb)/2)-abs((la+ra)/2))/dt
                if change > 5: accel.append(change)
                if change < -5: decel.append(-change)
                if abs(v) > 2 and abs(yaw) > 0.05: lateral.append(abs(v*yaw))
            else:
                change = (abs((lb-rb)/2)-abs((la-ra)/2))/dt
                if change > 5: turn_accel.append(change)
                if change < -5: turn_decel.append(-change)
            if abs(yaw) > 0.2 and abs(lb-rb) > 10:
                width = ((la-ra+lb-rb)/2)*inches_per_rpm_second/yaw
                if width > 0: widths.append(width)
    def observed(values):
        return sorted(values)[int((len(values)-1)*0.75)] if len(values) >= 5 else None
    return dict(acceleration=observed(accel), deceleration=observed(decel),
                turn_acceleration=observed(turn_accel), turn_deceleration=observed(turn_decel),
                lateral_acceleration=observed(lateral),
                effective_drive_width=statistics.median(widths) if len(widths) >= 5 else None,
                note="Observed response under the current controller; not maximum grip or an automatic geometry correction.")


def make_plan(runs, calibration, position, heading, cross_track, rpm, repeats):
    if calibration.get("ready") is not True:
        raise ValueError("calibration is not ready; correct geometry and repeat measurements first")
    position, heading = number(position,0.01,2), number(heading,0.01,5)
    cross_track, rpm = number(cross_track,0.01,20), number(rpm,1,200)
    if repeats < 3: raise ValueError("at least three runs per route are required")
    hardware = runs[0]["hardware"]
    if hardware["tracking_diameter"] != calibration["measured_tracking_diameter"]:
        raise ValueError("calibration used a different tracking wheel diameter")
    revisions = {}
    for route in ROUTES:
        group = [r for r in runs if r["route"] == route]
        if len(group) < repeats or any(r["result"] != "Completed" or r["truncated"] for r in group):
            raise ValueError(f"need {repeats} complete, untruncated baseline runs for {route}")
        revisions[route] = group[0]["revision"]
    check_context(runs,hardware,revisions)
    baseline = runs[0]["options"]
    if any(r["options"] != baseline for r in runs):
        raise ValueError("baseline runs must all use the same settings")
    base = dict(baseline,maximumRpm=rpm,positionTolerance=position,headingTolerance=heading)
    estimates = response_estimates(runs)
    candidates = [base]
    # Coordinate search around a measured baseline, avoiding a combinatorial sweep.
    for changes in (
        {"lookahead": max(2,base["lookahead"]*0.8)},
        {"lookahead": min(18,base["lookahead"]*1.2)},
        {"lookaheadAtSpeed": min(18,base["lookahead"]*1.5)},
        {"maximumRpm": max(1,rpm*0.8)},
        {"lateralAcceleration": max(1,base["lateralAcceleration"]*0.8)},
        {"accelerationScale": max(0.1,base["accelerationScale"]*0.8)},
        {"decelerationScale": max(0.1,base["decelerationScale"]*0.8)},
        {"settleMs": min(2000,base["settleMs"]+100)},
        {"settledRpm": base["settledRpm"]*0.5},
        {"turnAccelerationScale": max(0.1,base["turnAccelerationScale"]*0.8)},
        {"turnDecelerationScale": max(0.1,base["turnDecelerationScale"]*0.8)},
    ):
        candidates.append(dict(base,**changes))
    measured = dict(base)
    for estimate, field, nominal in (
        ("acceleration","accelerationScale",hardware["base_accel"]),
        ("deceleration","decelerationScale",hardware["base_decel"]),
        ("turn_acceleration","turnAccelerationScale",hardware["base_accel"]*3),
        ("turn_deceleration","turnDecelerationScale",hardware["base_decel"]*0.8)):
        if estimates[estimate] is not None:
            measured[field] = max(0.1,min(base[field],estimates[estimate]*0.8/nominal))
    candidates.append(measured)
    profiles = {}
    for options in candidates:
        validate_options(options)
        encoded = json.dumps([hardware,revisions,options],sort_keys=True).encode()
        identity = str(int(hashlib.sha256(encoded).hexdigest()[:8],16) or 1)
        if identity in profiles and profiles[identity] != options: raise ValueError("profile ID collision")
        profiles[identity] = options
    plan = dict(version=1, hardware=hardware, revisions=revisions, calibration=calibration,
                requirements=dict(position=position,heading=heading,cross_track=cross_track,repeats=repeats),
                estimates=estimates, profiles=profiles)
    validate_plan(plan)
    return plan


def evaluate(plan, runs, only=None):
    validate_plan(plan)
    check_context(runs,plan["hardware"],plan["revisions"])
    requirements = plan["requirements"]
    ranking, rejected = [], {}
    for identity, options in plan["profiles"].items():
        if only is not None and identity != str(only): continue
        group = [r for r in runs if str(r["profile"]) == identity]
        reasons, scores = [], []
        for route in ROUTES:
            route_runs = [r for r in group if r["route"] == route]
            if len(route_runs) < requirements["repeats"]:
                reasons.append(f"{route}: insufficient repeats")
            for run in route_runs:
                if run["options"] != options: raise ValueError("logged settings do not match profile ID")
                peak = max((float(s["cross_track"]) for s in run["trace"]),default=0)
                if (run["result"] != "Completed" or run["truncated"] or
                    run["position"] > requirements["position"] or run["heading"] > requirements["heading"] or
                    peak > requirements["cross_track"] or run["elapsed"] > options["timeoutMs"]+300):
                    reasons.append(f"{route} run {run['run']}: failed accuracy/completion gate")
            if route_runs:
                # Each route has equal weight even if one was repeated more often.
                scores.append(statistics.mean(r["elapsed"] for r in route_runs))
        if reasons: rejected[identity] = reasons
        else:
            ranking.append(dict(profile=identity, score_ms=sum(scores),
                                runs=[r["identity"] for r in group]))
    unknown = {str(r["profile"]) for r in runs} - set(plan["profiles"]) - {"0"}
    if unknown: raise ValueError(f"unknown profile IDs: {sorted(unknown)}")
    ranking.sort(key=lambda r:(r["score_ms"],int(r["profile"])))
    return dict(ranking=ranking,rejected=rejected)


def next_trial(plan, runs):
    result = evaluate(plan,runs)
    for identity in plan["profiles"]:
        reasons = result["rejected"].get(identity,[])
        if any("failed accuracy/completion" in reason for reason in reasons): continue
        group = [r for r in runs if str(r["profile"]) == identity]
        remaining = {route:max(0,plan["requirements"]["repeats"]-sum(r["route"] == route for r in group)) for route in ROUTES}
        if any(remaining.values()): return identity,remaining
    raise ValueError("candidate trials are finished; run select (failed candidates are not retried automatically)")


def approve(plan, selection, runs, measurements):
    if not selection["ranking"]: raise ValueError("selection contains no passing candidate")
    winner = selection["ranking"][0]
    old = {tuple(identity) for candidate in selection["ranking"] for identity in candidate["runs"]}
    new_runs = [r for r in runs if tuple(r["identity"]) not in old]
    result = evaluate(plan,new_runs,winner["profile"])
    if not result["ranking"]: raise ValueError("held-out runs failed validation; collect new runs")
    chosen = [r for r in new_runs if str(r["profile"]) == winner["profile"]]
    physical = {}
    for row in measurements:
        key = (row["route"],row["run"])
        if key in physical: raise ValueError("duplicate physical measurement")
        physical[key] = row
    conditions = set()
    for run in chosen:
        key = (run["route"],run["run"])
        if key not in physical: raise ValueError(f"missing independent endpoint measurement for {key}")
        row = physical[key]
        number(row["position_error"],0,plan["requirements"]["position"])
        number(row["heading_error"],0,plan["requirements"]["heading"])
        if not row["condition"].strip(): raise ValueError("record battery/load conditions")
        conditions.add(row["condition"].strip())
    if len(conditions) < 2: raise ValueError("validate at least two battery/load conditions")
    return winner["profile"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command",required=True)
    calibration = commands.add_parser("calibrate",help="compute sensing corrections from external measurements")
    calibration.add_argument("measurements",type=Path)
    calibration.add_argument("--tracking-diameter",type=float,required=True)
    calibration.add_argument("--output",type=Path,required=True)
    plan = commands.add_parser("plan",help="estimate response and generate bounded trial settings")
    plan.add_argument("logs",nargs="+",type=Path)
    plan.add_argument("--calibration",type=Path,required=True)
    plan.add_argument("--position",type=float,required=True)
    plan.add_argument("--heading",type=float,required=True)
    plan.add_argument("--cross-track",type=float,required=True)
    plan.add_argument("--max-rpm",type=float,default=120)
    plan.add_argument("--repeats",type=int,default=3)
    plan.add_argument("--output",type=Path,required=True)
    trial = commands.add_parser("trial",help="write one candidate for the existing robot diagnostics")
    trial.add_argument("plan",type=Path)
    trial.add_argument("--profile",default="next",help="candidate ID, or next to schedule from recorded attempts")
    trial.add_argument("--logs",nargs="*",type=Path,default=[])
    trial.add_argument("--output",type=Path,required=True)
    select = commands.add_parser("select",help="rank candidates that pass every route and accuracy gate")
    select.add_argument("plan",type=Path)
    select.add_argument("logs",nargs="+",type=Path)
    select.add_argument("--output",type=Path,required=True)
    validation = commands.add_parser("approve",help="export settings after new runs and physical endpoint checks")
    validation.add_argument("plan",type=Path)
    validation.add_argument("selection",type=Path)
    validation.add_argument("logs",nargs="+",type=Path)
    validation.add_argument("--measurements",type=Path,required=True)
    validation.add_argument("--output",type=Path,required=True)
    args = parser.parse_args()
    try:
        if args.command == "calibrate":
            result = calibrate(read_csv(args.measurements),args.tracking_diameter)
            save_json(args.output,result)
            print(json.dumps(result,indent=2))
        elif args.command == "plan":
            result = make_plan(load_runs(args.logs),read_json(args.calibration),args.position,args.heading,
                               args.cross_track,args.max_rpm,args.repeats)
            save_json(args.output,result)
            print("Profiles:",", ".join(result["profiles"]))
            print(json.dumps(result["estimates"],indent=2))
        elif args.command == "trial":
            plan = read_json(args.plan)
            validate_plan(plan)
            profile = args.profile
            if profile == "next":
                runs = load_runs(args.logs) if args.logs else []
                profile, remaining = next_trial(plan,runs)
                print("Profile",profile,"remaining diagnostic runs:",remaining)
            write_profile(args.output,int(profile),plan["hardware"],plan["profiles"][profile])
        elif args.command == "select":
            result = evaluate(read_json(args.plan),load_runs(args.logs))
            save_json(args.output,result)
            print(json.dumps(result,indent=2))
            if not result["ranking"]: parser.exit(1,"No candidate passed; no settings selected.\n")
        else:
            plan = read_json(args.plan)
            profile = approve(plan,read_json(args.selection),load_runs(args.logs),read_csv(args.measurements))
            write_profile(args.output,int(profile),plan["hardware"],plan["profiles"][profile])
            print(f"Validated profile {profile} written to {args.output}")
    except (OSError,ValueError,KeyError,TypeError,IndexError) as error:
        parser.exit(1,f"Path tuning failed: {error}\n")


if __name__ == "__main__":
    main()
