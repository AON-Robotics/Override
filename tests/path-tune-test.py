import math
import csv
import json
import subprocess
import sys
import importlib.util
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location("tune", Path(__file__).resolve().parents[1] / "tools/path-tune.py")
tune = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tune)


def verify_cpp_trace(directory):
    # Actual production C++ serialization consumed by the production PC parser.
    runs = tune.load_runs([Path(directory)/"aon-straight-v2-runs.csv"])
    assert len(runs) == 1
    run = runs[0]
    assert run["profile"] == 7 and run["revision"] == 123
    assert run["heading"] == 60 and run["elapsed"] == 300
    assert run["options"]["lookahead"] == 7/3
    assert run["options"]["decelerationScale"] == 0.8
    assert run["options"]["turnAccelerationScale"] == 0.7
    assert run["options"]["turnDecelerationScale"] == 0.6


class TuneTests(unittest.TestCase):
    def test_distance_calibration_uses_physical_reference(self):
        rows = [dict(reported_distance="24", actual_distance="22", reported_turn="90", actual_turn="90")]*3
        result = tune.calibrate(rows, 2)
        self.assertAlmostEqual(result["tracking_diameter"], 11/6)
        self.assertFalse(result["ready"])

    def test_calibration_rejects_missing_nonfinite_and_inconsistent_evidence(self):
        good = dict(reported_distance="24",actual_distance="24",reported_turn="90",actual_turn="90")
        self.assertTrue(tune.calibrate([good]*3,2)["ready"])
        for bad in ("nan", "0", "-1", "30"):
            with self.subTest(value=bad), self.assertRaises(ValueError):
                tune.calibrate([good,good,dict(good,actual_distance=bad)],2)
        with self.assertRaises(ValueError): tune.calibrate([good],2)

    def test_profile_interchange_bounds(self):
        hardware = dict(zip(tune.HARDWARE, [2.75,0.75,12.5,2,2500,200]))
        options = dict(zip(tune.FIELDS, [120,6,6,2,2,35,150,5,30000,1,1,1,1]))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"profile.csv"
            tune.write_profile(path, 7, hardware, options)
            lines = path.read_text().splitlines()
            self.assertEqual(lines[0], "AON_PATH_TUNING_V1")
            self.assertEqual(len(lines[1].split(",")),20)
            self.assertEqual(float(lines[1].split(",")[8]),6)
            for field, value in (("maximumRpm",201),("decelerationScale",0),("lookahead",float("nan")),("settleMs",1.5)):
                with self.subTest(field=field), self.assertRaises(ValueError):
                    tune.write_profile(path,7,hardware,dict(options,**{field:value}))

    def baseline(self):
        hardware = dict(zip(tune.HARDWARE,[2.75,0.75,12.5,2,2500,200]))
        options = dict(zip(tune.FIELDS,[120,6,6,2,2,35,150,5,30000,1,1,1,1]))
        runs = []
        for route in tune.ROUTES:
            for index in range(3):
                trace = [dict(run=str(index),ms=str(ms),left_cmd="0",right_cmd="0",left_rpm="0",right_rpm="0",
                              heading="0",x="0",y="0",cross_track="0.5",aligning="0") for ms in (0,100,1000)]
                runs.append(dict(route=route,run=str(index),identity=[route,str(index),"0","0","123"],
                                 profile=0,revision=123,hardware=hardware.copy(),options=options.copy(),trace=trace,
                                 result="Completed",elapsed=1000,truncated=False,position=0.5,heading=1))
        return runs

    def plan(self):
        return tune.make_plan(self.baseline(),dict(ready=True,measured_tracking_diameter=2),1,2,1,120,3)

    def trials(self, plan, identity, offset=10, elapsed=1000):
        runs = self.baseline()
        for index,run in enumerate(runs):
            run["profile"] = int(identity)
            run["options"] = plan["profiles"][identity].copy()
            run["run"] = str(index+offset)
            run["identity"] = [run["route"],run["run"],"0",identity,"123"]
            run["elapsed"] = elapsed
            for sample in run["trace"]: sample["run"] = run["run"]
            run["trace"][-1]["ms"] = str(elapsed)
        return runs

    def test_corrected_diameter_survives_profile_roundtrip(self):
        runs = self.baseline()
        diameter = 2*(22/24)
        for run in runs: run["hardware"]["tracking_diameter"] = diameter
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/"profile.csv"
            tune.write_profile(path,1,runs[0]["hardware"],runs[0]["options"])
            serialized = float(path.read_text().splitlines()[1].split(",")[4])
            self.assertEqual(serialized,diameter)
            calibration = dict(ready=True,measured_tracking_diameter=diameter)
            plan = tune.make_plan(runs,calibration,1,2,1,120,3)
            self.assertEqual(plan["hardware"]["tracking_diameter"],diameter)

    def test_plan_does_not_relax_accuracy_or_invent_response_measurements(self):
        plan = self.plan()
        self.assertGreaterEqual(len(plan["profiles"]),10)
        for options in plan["profiles"].values():
            tune.validate_options(options)
            self.assertEqual(options["positionTolerance"],1)
            self.assertEqual(options["headingTolerance"],2)
        self.assertIsNone(plan["estimates"]["acceleration"])
        with self.assertRaisesRegex(ValueError,"calibration"):
            tune.make_plan(self.baseline(),dict(ready=False),1,2,1,120,3)

    def test_fractional_requirements_survive_candidate_planning(self):
        tolerance = 1/3
        plan = tune.make_plan(self.baseline(),dict(ready=True,measured_tracking_diameter=2),tolerance,2,1,120,3)
        tune.validate_plan(plan)
        for options in plan["profiles"].values(): self.assertEqual(options["positionTolerance"],tolerance)
        self.assertTrue(tune.next_trial(plan,[])[0])

    def test_selection_rejects_fast_but_inaccurate_and_incomplete_candidates(self):
        plan = self.plan()
        good, fast, missing = list(plan["profiles"])[:3]
        runs = self.trials(plan,good,10,1500)+self.trials(plan,fast,100,900)
        runs[-1]["trace"][0]["cross_track"] = "4"
        result = tune.evaluate(plan,runs)
        self.assertEqual(result["ranking"][0]["profile"],good)
        self.assertEqual(result["ranking"][0]["score_ms"],4500)
        self.assertIn(fast,result["rejected"])
        self.assertIn(missing,result["rejected"])

    def test_every_failed_trial_counts_and_context_must_match(self):
        plan = self.plan()
        identity = next(iter(plan["profiles"]))
        for field,value in (("result","Cancelled"),("result","Disabled"),("result","Timed out"),
                            ("truncated",True),("position",2),("heading",3)):
            runs = self.trials(plan,identity)
            runs[0][field] = value
            self.assertEqual(tune.evaluate(plan,runs)["ranking"],[])
        for field,value in (("revision",124),("hardware",{}),("options",{})):
            runs = self.trials(plan,identity)
            runs[0][field] = value
            with self.subTest(field=field), self.assertRaises(ValueError): tune.evaluate(plan,runs)

    def test_approval_requires_new_trials_and_independent_physical_accuracy(self):
        plan = self.plan()
        identity = next(iter(plan["profiles"]))
        training = self.trials(plan,identity)
        selected = tune.evaluate(plan,training)
        with self.assertRaisesRegex(ValueError,"held-out"): tune.approve(plan,selected,training,[])
        new = self.trials(plan,identity,100)
        measurements = [dict(route=r["route"],run=r["run"],position_error="0.75",heading_error="1.5",
                             condition="loaded" if i%2 else "unloaded") for i,r in enumerate(new)]
        self.assertEqual(tune.approve(plan,selected,training+new,measurements),identity)
        with self.assertRaisesRegex(ValueError,"missing independent"):
            tune.approve(plan,selected,new,measurements[:-1])
        measurements[0]["position_error"] = "1.5"
        with self.assertRaises(ValueError): tune.approve(plan,selected,new,measurements)

    def write_logs(self, directory, runs):
        paths = []
        for route in tune.ROUTES:
            group = [r for r in runs if r["route"] == route]
            path = directory/f"aon-{route}-v2-runs.csv"
            summaries = [dict(run=r["run"],start_ms="0",result=r["result"],elapsed_ms=r["elapsed"],
                              position_error=r["position"],heading_error=r["heading"],samples=len(r["trace"]),
                              truncated=int(r["truncated"]),profile=r["profile"],revision=r["revision"],
                              **r["hardware"],**r["options"]) for r in group]
            with path.open("w",newline="") as file:
                writer = csv.DictWriter(file,fieldnames=list(summaries[0]))
                writer.writeheader(); writer.writerows(summaries)
            trace_path = path.with_name(path.name.replace("-runs.csv",".csv"))
            traces = [s for r in group for s in r["trace"]]
            with trace_path.open("w",newline="") as file:
                writer = csv.DictWriter(file,fieldnames=list(traces[0]))
                writer.writeheader(); writer.writerows(traces)
            paths.append(path)
        return paths

    def test_next_trial_schedules_missing_runs_and_skips_failed_candidates(self):
        plan = self.plan()
        first,second = list(plan["profiles"])[:2]
        identity,remaining = tune.next_trial(plan,[])
        self.assertEqual(identity,first)
        self.assertEqual(remaining,dict(straight=3,curve=3,uturn=3))
        identity,remaining = tune.next_trial(plan,self.trials(plan,first)[:2])
        self.assertEqual(identity,first)
        self.assertEqual(remaining["straight"],1)
        failed = self.trials(plan,first)[:1]
        failed[0]["result"] = "Cancelled"
        self.assertEqual(tune.next_trial(plan,failed)[0],second)

    def test_log_loading_handles_clock_tick_and_pre_motion_cancellation(self):
        with tempfile.TemporaryDirectory() as folder:
            directory = Path(folder)
            plan = self.plan()
            identity = next(iter(plan["profiles"]))
            runs = self.trials(plan,identity)
            runs[0]["elapsed"] += 1
            paths = self.write_logs(directory,runs)
            self.assertEqual(len(tune.load_runs(paths)),9)
            runs[0]["elapsed"] += 100
            paths = self.write_logs(directory,runs)
            with self.assertRaisesRegex(ValueError,"final braking"): tune.load_runs(paths)
            runs[0].update(trace=[],result="Cancelled",elapsed=300)
            paths = self.write_logs(directory,runs)
            loaded = tune.load_runs(paths)
            self.assertEqual(tune.evaluate(plan,loaded)["ranking"],[])
            self.assertNotEqual(tune.next_trial(plan,loaded)[0],identity)
            runs = self.trials(plan,identity)
            runs[0].update(result="Timed out",elapsed=30301)
            runs[0]["trace"][-1]["ms"] = "30301"
            paths = self.write_logs(directory,runs)
            loaded = tune.load_runs(paths)
            self.assertEqual(tune.evaluate(plan,loaded)["ranking"],[])
            self.assertNotEqual(tune.next_trial(plan,loaded)[0],identity)

    def test_observed_response_has_independent_units_and_requires_samples(self):
        run = self.baseline()[0]
        trace = []
        for i in range(21):
            rpm = i*10 if i <= 10 else (20-i)*10
            trace.append(dict(ms=str(i*100),left_rpm=str(rpm),right_rpm=str(rpm),heading="0",aligning="0"))
        run["trace"] = trace
        estimates = tune.response_estimates([run])
        self.assertAlmostEqual(estimates["acceleration"],100)
        self.assertAlmostEqual(estimates["deceleration"],100)
        self.assertIsNone(estimates["effective_drive_width"])
        yaw = math.pi*2.75*0.75/12.5
        for i,sample in enumerate(trace):
            sample.update(left_rpm="60",right_rpm="0",heading=str(math.degrees(yaw)*i/10))
        estimates = tune.response_estimates([run])
        self.assertAlmostEqual(estimates["effective_drive_width"],12.5)
        self.assertAlmostEqual(estimates["lateral_acceleration"],(math.pi*2.75*0.75)**2/25)

    def test_csv_cli_roundtrip_and_rejection_without_overwriting_profile(self):
        with tempfile.TemporaryDirectory() as folder:
            directory = Path(folder)
            paths = self.write_logs(directory,self.baseline())
            loaded = tune.load_runs(paths)
            self.assertEqual(len(loaded),9)
            with self.assertRaisesRegex(ValueError,"duplicate run"): tune.load_runs(paths+paths)
            calibration = directory/"calibration.json"
            calibration.write_text(json.dumps(dict(ready=True,measured_tracking_diameter=2)))
            plan_file, profile_file = directory/"plan.json",directory/"trial.csv"
            cmd = [sys.executable,str(Path(tune.__file__))]
            subprocess.run(cmd+["plan",*map(str,paths),"--calibration",str(calibration),"--position","1",
                                "--heading","2","--cross-track","1","--output",str(plan_file)],check=True,capture_output=True)
            plan = tune.read_json(plan_file)
            identity = next(iter(plan["profiles"]))
            subprocess.run(cmd+["trial",str(plan_file),"--profile",identity,"--output",str(profile_file)],
                           check=True,capture_output=True)
            self.assertTrue(profile_file.read_text().startswith("AON_PATH_TUNING_V1"))
            before = profile_file.read_text()
            result = subprocess.run(cmd+["trial",str(plan_file),"--profile","bad","--output",str(profile_file)],capture_output=True)
            self.assertNotEqual(result.returncode,0)
            self.assertEqual(profile_file.read_text(),before)
            paths = self.write_logs(directory,self.trials(plan,identity))
            selection = directory/"selection.json"
            subprocess.run(cmd+["select",str(plan_file),*map(str,paths),"--output",str(selection)],check=True,capture_output=True)
            self.assertEqual(tune.read_json(selection)["ranking"][0]["profile"],identity)
            new_runs = self.trials(plan,identity,100)
            paths = self.write_logs(directory,new_runs)
            measurements = directory/"physical.csv"
            with measurements.open("w",newline="") as file:
                writer = csv.writer(file)
                writer.writerow(["route","run","position_error","heading_error","condition"])
                for i,run in enumerate(new_runs): writer.writerow([run["route"],run["run"],0.5,1,"loaded" if i%2 else "unloaded"])
            subprocess.run(cmd+["approve",str(plan_file),str(selection),*map(str,paths),"--measurements",str(measurements),
                                "--output",str(profile_file)],check=True,capture_output=True)
            self.assertEqual(profile_file.read_text(),before)
            trace_path = paths[0].with_name(paths[0].name.replace("-runs.csv",".csv"))
            trace_path.write_text("\n".join(trace_path.read_text().splitlines()[:-1])+"\n")
            with self.assertRaisesRegex(ValueError,"incomplete trace"): tune.load_runs(paths)


if __name__ == "__main__":
    unittest.main()
