# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Consequential measurement/scoring contracts; no engine, ROS or network."""

import copy
import importlib.util
import io
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location("amcl_trial", Path(__file__).with_name("run_trial.py"))
runner = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(runner)
REPEAT_SPEC = importlib.util.spec_from_file_location("amcl_repeat", Path(__file__).with_name("run_repeated.py"))
repeated = importlib.util.module_from_spec(REPEAT_SPEC)
REPEAT_SPEC.loader.exec_module(repeated)


def endpoint():
    return dict(
        amcl=dict(frame="map", x=0., y=0., yaw=0., stamp=10.3, covariance_diagonal=[.1, .1, .1]),
        refresh_request_clock=10.2, matching_scan_stamp=10.3,
        tf_at_scan={edge: dict(stamp=10.3, translation=[0, 0, 0], yaw=0)
                    for edge in ("map->odom", "odom->base_link", "base_link->base_laser")},
        ground_truth=[dict(node={"def": "HUSKY", "position": [0, 0, .13],
                                 "orientation": [1, 0, 0, 0, 1, 0, 0, 0, 1]},
                           requested_monotonic_ns=i*1000, received_monotonic_ns=i*1000+100,
                           clock_before=t, clock_after=t+.1, v_linear=0., v_angular=0.)
                      for i, t in enumerate((10., 10.6, 11.2))])


def series(count=7, interval=.4):
    rows = []
    for i in range(count):
        t, stamp = 100.1+i*interval, 10.+i*interval
        row = endpoint()
        row.update(phase="initial", index=i, valid=True, monotonic_s=t,
                   refresh_request_clock=stamp-.01, matching_scan_stamp=stamp,
                   timing=dict(clock=stamp+.05, odom=stamp+.05, scan=stamp),
                   receipt_ages_wall_s=dict(clock=.1, odom=.1, scan=.1))
        row["amcl"]["stamp"] = stamp
        for tf in row["tf_at_scan"].values():
            tf["stamp"] = stamp
        row["ground_truth"] = row["ground_truth"][:2]
        for gt, delta in zip(row["ground_truth"], (-.05, .02)):
            gt.update(clock_before=stamp+delta, clock_after=stamp+delta+.02,
                      requested_monotonic_ns=round((t+delta*2)*1e9),
                      received_monotonic_ns=round((t+delta*2+.02)*1e9))
        rows.append(row)
    return rows


def observation(rows=None, end=102.6, **kwargs):
    return runner.analyze_observation(series() if rows is None else rows, 100., end, **kwargs)


def repeat_result(identity):
    return dict(schema_version=2, trial_id=identity, cold_start=True,
                verdicts=dict(measurement="PASS", localization="PASS", overall="PASS"),
                endpoints={p: endpoint() for p in ("initial", "post_motion")},
                stationary_observations={p: observation() for p in ("initial", "post_motion")},
                provenance=dict(commit="fixture", sha256={}, machine="fixture", physics_packages={}),
                configuration=dict(acceptance_thresholds=dict(position_m=runner.DEFAULT_POSITION_LIMIT_M,
                                                              absolute_yaw_rad=runner.DEFAULT_YAW_LIMIT_RAD),
                                   observation=dict(position_jitter_limit_m=runner.DEFAULT_POSITION_JITTER_M,
                                                    yaw_jitter_limit_rad=runner.DEFAULT_YAW_JITTER_RAD)),
                process_exit=dict(harness=0), cleanup_listeners={str(p): False for p in runner.PORTS})


class MeasurementTests(unittest.TestCase):
    def setUp(self):
        self.io = [patch.object(runner, "http", side_effect=AssertionError("live HTTP in unit test")),
                   patch.object(runner.subprocess, "Popen", side_effect=AssertionError("process launch in unit test"))]
        for mock in self.io:
            mock.start()
            self.addCleanup(mock.stop)

    def test_position_and_wrapped_yaw(self):
        errors = runner.pose_errors(dict(frame="map", x=1, y=2, yaw=math.radians(179)),
                                    dict(frame="map", x=4, y=6, yaw=math.radians(-179)))
        self.assertEqual(errors["position_error_m"], 5)
        self.assertAlmostEqual(errors["yaw_error_rad"], math.radians(-2))

    def test_world_root_projection(self):
        node = {"def": "HUSKY", "position": [1, -2, .13], "orientation": [0, -1, 0, 1, 0, 0, 0, 0, 1]}
        self.assertEqual(runner.world_to_map(node), dict(frame="map", x=1, y=-2, yaw=math.pi/2))
        node["orientation"] = [0, 1, 0, 1, 0, 0, 0, 0, 1]  # Reflection, not a rotation.
        with self.assertRaises(ValueError):
            runner.world_to_map(node)

    def test_missing_nonfinite_and_wrong_frame(self):
        valid = dict(frame="map", x=0, y=0, yaw=0)
        for change in ({"frame": "odom"}, {"x": math.nan}, {"yaw": None}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                runner.pose_errors(dict(valid, **change), valid)
        self.assertTrue(runner.endpoint_reasons({}))

    def test_stale_or_unmatched_localization(self):
        valid = endpoint()
        self.assertEqual(runner.endpoint_reasons(valid), [])
        for key, value in (("stamp", 9.9), ("stamp", 12.), ("covariance_diagonal", [math.nan, 0, 0])):
            bad = copy.deepcopy(valid)
            bad["amcl"][key] = value
            with self.subTest(key=key, value=value):
                self.assertTrue(runner.endpoint_reasons(bad))
        for key, value in (("matching_scan_stamp", 10.), ("tf_at_scan", {})):
            bad = copy.deepcopy(valid)
            bad[key] = value
            self.assertTrue(runner.endpoint_reasons(bad))

    def test_moving_or_badly_timed_ground_truth(self):
        for change in ("position", "velocity", "request", "clock"):
            bad = endpoint()
            sample = bad["ground_truth"][-1]
            if change == "position":
                sample["node"]["position"][0] = .02
            elif change == "velocity":
                sample["v_linear"] = .1
            elif change == "request":
                sample["received_monotonic_ns"] = -1
            else:
                sample["clock_after"] = 9
            with self.subTest(change=change):
                self.assertTrue(runner.endpoint_reasons(bad))

    def test_invalid_evidence_never_passes_even_with_loose_limits(self):
        for endpoints, errors in (({}, []), ({"initial": endpoint()}, []),
                                  ({"initial": endpoint(), "post_motion": endpoint()}, ["engine died"])):
            verdict = runner.score(endpoints, errors, 1e6, 1e6)
            self.assertEqual(verdict["overall"], "INVALID")
            self.assertEqual(verdict["localization"], "NOT_SCORED")

    def test_valid_measurements_are_unscored_without_limits(self):
        verdict = runner.score({"initial": endpoint(), "post_motion": endpoint()}, [], None, None)
        self.assertEqual(verdict, dict(measurement="PASS", localization="NOT_SCORED", overall="NOT_SCORED", reasons=[]))

    def test_default_gate_boundaries_at_both_endpoints(self):
        self.assertEqual(runner.DEFAULT_POSITION_LIMIT_M, .10)
        self.assertEqual(runner.DEFAULT_YAW_LIMIT_RAD, .05)
        for label in ("initial", "post_motion"):
            for position, angle, expected in ((.10, .05, "PASS"), (.10, -.05, "PASS"),
                                              (.10000001, 0., "FAIL"), (0., .05000001, "FAIL"),
                                              (0., -.05000001, "FAIL")):
                endpoints = dict(initial=endpoint(), post_motion=endpoint())
                endpoints[label]["amcl"].update(x=position, yaw=angle)
                with self.subTest(label=label, position=position, angle=angle):
                    verdict = runner.score(endpoints, [])
                    self.assertEqual(verdict["measurement"], "PASS")
                    self.assertEqual(verdict["localization"], expected)
                    self.assertEqual(verdict["overall"], expected)

    def test_bad_tf_evidence_cannot_pass_default_scoring(self):
        for change in ("nonfinite", "missing_values", "wrong_time"):
            bad = endpoint()
            transform = bad["tf_at_scan"]["odom->base_link"]
            if change == "nonfinite":
                transform["translation"][0] = math.nan
            elif change == "missing_values":
                transform.clear()
            else:
                transform["stamp"] = 9.9
            with self.subTest(change=change):
                verdict = runner.score(dict(initial=endpoint(), post_motion=bad), [])
                self.assertEqual(verdict["measurement"], "INVALID")
                self.assertEqual(verdict["localization"], "NOT_SCORED")
                self.assertEqual(verdict["overall"], "INVALID")

    def test_nonfinite_tf_is_rejected_before_recording(self):
        recorder = runner.Recorder.__new__(runner.Recorder)
        recorder.latest, recorder.edges = {}, {"dynamic": set()}
        message = SimpleNamespace(transforms=[SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=math.nan, y=0., z=0.)))])
        with self.assertRaisesRegex(ValueError, "nonfinite TF"):
            recorder.transforms(message, "dynamic")
        self.assertEqual(recorder.latest, {})
        self.assertEqual(recorder.edges["dynamic"], set())

    def test_cli_defaults_overrides_and_unscored_result(self):
        def measurements(trial):
            trial.data["endpoints"] = dict(initial=endpoint(), post_motion=endpoint())
            trial.data["endpoints"]["post_motion"]["amcl"]["x"] = .02
            trial.data["stationary_observations"] = {
                p: observation(scoring=not trial.args.no_score) for p in ("initial", "post_motion")}

        cases = (([], "PASS", .10), (["--max-position-error-m", ".01"], "FAIL", .01),
                 (["--no-score"], "NOT_SCORED", None))
        with tempfile.TemporaryDirectory(prefix="amcl-scoring-test-") as directory:
            for index, (options, expected, position_limit) in enumerate(cases):
                output = Path(directory) / str(index)
                argv = ["run_trial.py", "--output", str(output), *options]
                with self.subTest(options=options), patch.object(runner.sys, "argv", argv), \
                        patch.dict(runner.os.environ, {"_HUSKY_AMCL_ENV": str(runner.ROOT)}), \
                        patch.object(runner.Trial, "run", measurements), \
                        patch.object(runner.Trial, "cleanup"), patch.object(runner.signal, "signal"), \
                        patch.object(runner.sys, "stdout", io.StringIO()):
                    code = runner.main()
                result = json.loads((output / "result.json").read_text())
                self.assertEqual(code, 1 if expected == "FAIL" else 0)
                self.assertEqual(result["verdicts"]["measurement"], "PASS")
                self.assertEqual(result["verdicts"]["overall"], expected)
                self.assertEqual(result["configuration"]["acceptance_thresholds"]["position_m"], position_limit)

    def test_explicit_limits_recompute_errors(self):
        final = endpoint()
        final["amcl"].update(x=3., y=4., yaw=.4)
        final["errors"] = dict(position_error_m=0, yaw_error_rad=0)  # Never trust precomputed verdict inputs.
        endpoints = dict(initial=endpoint(), post_motion=final)
        self.assertEqual(runner.score(endpoints, [], 6, .5)["overall"], "PASS")
        self.assertEqual(runner.score(endpoints, [], 4, .5)["overall"], "FAIL")
        self.assertEqual(runner.score(endpoints, [], 6, .3)["overall"], "FAIL")

    def test_valid_series_and_convergence_boundary(self):
        rows = series()
        for row in rows:
            row["amcl"].update(x=.10, yaw=-.05)
        result = observation(rows)
        self.assertEqual(result["measurement"], "PASS")
        self.assertTrue(result["convergence"]["converged"])
        self.assertAlmostEqual(result["convergence"]["convergence_time_s"], 2.1)
        self.assertFalse(result["convergence"]["timeout"])

    def test_bad_series_timing_and_nonfinite_poses(self):
        for kind in ("order", "stamp_order", "pose", "gt_time", "gap", "missing", "stale_receipt", "invalid_flag"):
            rows = series()
            if kind == "order":
                rows[3]["monotonic_s"] = rows[2]["monotonic_s"]
            elif kind == "stamp_order":
                rows[3]["amcl"]["stamp"] = rows[2]["amcl"]["stamp"]
            elif kind == "pose":
                rows[3]["amcl"]["x"] = math.nan
            elif kind == "gt_time":
                rows[3]["ground_truth"][-1]["clock_after"] = 1.
            elif kind == "gap":
                del rows[2:5]
            elif kind == "missing":
                rows[3].pop("amcl")
            elif kind == "stale_receipt":
                rows[3]["receipt_ages_wall_s"]["scan"] = 5.
            else:
                rows[3]["valid"] = False
            with self.subTest(kind=kind):
                result = observation(rows)
                self.assertEqual(result["measurement"], "INVALID")
                self.assertEqual(result["convergence"]["outcome"], "INVALID")

    def test_short_convergence_and_insufficient_samples(self):
        for rows in (series(5), [], series(2)):
            self.assertEqual(observation(rows)["measurement"], "INVALID")

    def test_valid_convergence_timeout(self):
        rows = series(38)
        for row in rows:
            row["amcl"]["x"] = .2
        result = observation(rows, end=115.)
        self.assertEqual(result["measurement"], "PASS")
        self.assertEqual(result["convergence"]["outcome"], "FAIL")
        self.assertTrue(result["convergence"]["timeout"])
        self.assertIsNone(result["convergence"]["convergence_time_s"])

    def test_convergence_restarts_after_threshold_excursion(self):
        rows = series(14)
        rows[4]["amcl"]["x"] = .2
        result = observation(rows, end=105.5)
        self.assertEqual(result["convergence"]["outcome"], "PASS")
        self.assertAlmostEqual(result["convergence"]["threshold_entry_time_s"], 2.1)
        self.assertAlmostEqual(result["convergence"]["convergence_time_s"], 4.1)

    def test_zero_small_and_excessive_jitter(self):
        self.assertEqual(observation()["jitter"]["position_std_m"], 0.)
        for amplitude, expected in ((.005, "PASS"), (.03, "PASS"), (.0300001, "FAIL"), (.04, "FAIL")):
            rows = series(6, .41)
            for i, row in enumerate(rows):
                row["amcl"].update(x=amplitude*(-1)**i, yaw=.002*(-1)**i)
            result = observation(rows, end=102.3)
            self.assertAlmostEqual(result["jitter"]["position_std_m"], amplitude)
            self.assertAlmostEqual(result["jitter"]["yaw_std_rad"], .002)
            self.assertEqual(result["jitter"]["outcome"], expected)

    def test_circular_yaw_jitter_near_pi(self):
        rows = series(6, .41)
        for i, row in enumerate(rows):
            row["amcl"]["yaw"] = runner.wrap(math.pi+.01*(-1)**i)
            for gt in row["ground_truth"]:
                gt["node"]["orientation"] = [-1, 0, 0, 0, -1, 0, 0, 0, 1]
        result = observation(rows, end=102.3)
        self.assertEqual(result["measurement"], "PASS")
        self.assertAlmostEqual(result["jitter"]["yaw_std_rad"], .01)
        self.assertEqual(result["jitter"]["outcome"], "PASS")

    def test_moving_samples_and_cumulative_drift_are_invalid(self):
        for mode in ("velocity", "drift"):
            rows = series()
            for i, row in enumerate(rows):
                for gt in row["ground_truth"]:
                    if mode == "velocity":
                        gt["v_linear"] = .2
                    else:
                        gt["node"]["position"][0] = i*.001
            self.assertEqual(observation(rows)["jitter"]["outcome"], "INVALID")

    def test_full_scoring_requires_new_evidence_and_preserves_no_score(self):
        endpoints = dict(initial=endpoint(), post_motion=endpoint())
        self.assertEqual(runner.score_trial(endpoints, [], None)["overall"], "INVALID")
        rows = series(38)
        for row in rows:
            row["amcl"]["x"] = .2
        observations = dict(initial=observation(), post_motion=observation(rows, end=115.))
        self.assertEqual(runner.score_trial(endpoints, [], observations)["overall"], "FAIL")
        observations["post_motion"] = observation(rows, end=115., scoring=False)
        self.assertEqual(runner.score_trial(endpoints, [], observations, None, None)["overall"], "NOT_SCORED")
        observations["post_motion"]["jitter"]["outcome"] = "INVALID"
        self.assertEqual(runner.score_trial(endpoints, [], observations, None, None)["overall"], "INVALID")
        observations["post_motion"] = observation([])
        self.assertEqual(runner.score_trial(endpoints, [], observations, None, None)["overall"], "INVALID")

    def test_repeat_verdicts_missing_results_and_independence(self):
        with tempfile.TemporaryDirectory(prefix="amcl-repeat-test-") as directory:
            paths = []
            for i in range(3):
                folder = Path(directory)/str(i)
                folder.mkdir()
                path = folder/"result.json"
                value = repeat_result(str(i))
                path.write_text(json.dumps(value))
                paths.append(path)
            self.assertEqual(repeated.summarize(paths, 3)["overall"], "PASS")
            originals = [path.read_bytes() for path in paths]
            repeated.summarize(paths, 3)
            self.assertEqual([path.read_bytes() for path in paths], originals)
            value["trial_id"] = "0"
            paths[-1].write_text(json.dumps(value))
            self.assertEqual(repeated.summarize(paths, 3)["overall"], "INVALID")
            value["trial_id"] = "2"
            paths[-1].write_text(json.dumps(value))
            self.assertEqual(repeated.summarize(paths[:2], 2)["overall"], "INVALID")
            value["verdicts"].update(overall="FAIL", localization="FAIL")
            value["endpoints"]["post_motion"]["amcl"]["x"] = .11
            paths[-1].write_text(json.dumps(value))
            result = repeated.summarize(paths, 3)
            self.assertEqual(result["overall"], "FAIL")
            self.assertAlmostEqual(result["pass_rate"], 2/3)
            value["verdicts"].update(overall="INVALID", localization="NOT_SCORED", measurement="INVALID")
            paths[-1].write_text(json.dumps(value))
            self.assertEqual(repeated.summarize(paths, 3)["overall"], "INVALID")
            paths[-1].write_text("corrupt json")
            self.assertEqual(repeated.summarize(paths, 3)["overall"], "INVALID")
            paths[-1].unlink()
            self.assertEqual(repeated.summarize(paths, 3)["overall"], "INVALID")
            self.assertEqual(repeated.summarize([paths[0], paths[1], paths[0]], 3)["overall"], "INVALID")

    def test_nonfinite_ground_truth_acquisition_times(self):
        for key in ("requested_monotonic_ns", "received_monotonic_ns"):
            for value in (math.nan, math.inf):
                bad = endpoint()
                bad["ground_truth"][1][key] = value
                self.assertEqual(runner.score(dict(initial=endpoint(), post_motion=bad), [])["overall"], "INVALID")

    def test_invalid_observation_interval_and_tipped_sample(self):
        for start, end in ((math.nan, 103.), (100., math.inf), (100., 99.), (None, 103.)):
            self.assertEqual(runner.analyze_observation(series(), start, end)["measurement"], "INVALID")
        rows = series()
        rows[2]["ground_truth"][0]["node"]["orientation"] = [0]*9
        self.assertEqual(observation(rows)["measurement"], "INVALID")

    def test_inconsistent_observation_summaries_cannot_pass(self):
        changes = (("convergence", "convergence_time_s", math.nan),
                   ("convergence", "converged", False), ("convergence", "timeout", True),
                   ("convergence", "valid_samples", 0),
                   ("jitter", "position_std_m", .04), ("jitter", "yaw_std_rad", math.inf),
                   ("jitter", "duration_s", 1.), ("jitter", "valid_samples", 1))
        for check, key, value in changes:
            observations = dict(initial=observation(), post_motion=observation())
            observations["post_motion"][check][key] = value
            with self.subTest(check=check, key=key):
                result = runner.score_trial(dict(initial=endpoint(), post_motion=endpoint()), [], observations)
                self.assertEqual(result["overall"], "INVALID")
                self.assertEqual(result["localization"], "NOT_SCORED")
        observations = dict(initial=observation(), post_motion=observation())
        observations["post_motion"]["jitter"].update(position_std_m=.04, outcome="PASS")
        self.assertEqual(runner.score_trial(dict(initial=endpoint(), post_motion=endpoint()), [], observations,
                                            position_jitter_limit=.05)["overall"], "PASS")

    def test_invalid_nonfinite_evidence_is_persisted_as_strict_json(self):
        def malformed(trial):
            trial.data["endpoints"] = dict(initial=endpoint(), post_motion=endpoint())
            trial.data["endpoints"]["post_motion"]["ground_truth"][0]["node"]["position"][0] = math.nan
        with tempfile.TemporaryDirectory(prefix="amcl-invalid-json-") as directory:
            output = Path(directory)/"trial"
            with patch.object(runner.sys, "argv", ["run_trial.py", "--output", str(output)]), \
                    patch.dict(runner.os.environ, {"_HUSKY_AMCL_ENV": str(runner.ROOT)}), \
                    patch.object(runner.Trial, "run", malformed), patch.object(runner.Trial, "cleanup"), \
                    patch.object(runner.signal, "signal"), patch.object(runner.sys, "stdout", io.StringIO()):
                self.assertEqual(runner.main(), 2)
            data = json.loads((output/"result.json").read_text(),
                              parse_constant=lambda value: self.fail(f"nonstandard JSON number: {value}"))
            self.assertEqual(data["verdicts"]["overall"], "INVALID")
            self.assertEqual(data["endpoints"]["post_motion"]["ground_truth"][0]["node"]["position"][0], "nan")
            self.assertFalse(runner.finite(["nan"]))

    def test_repeat_rechecks_metrics_configuration_and_unscored_mode(self):
        with tempfile.TemporaryDirectory(prefix="amcl-repeat-integrity-") as directory:
            paths = []
            values = [repeat_result(str(i)) for i in range(3)]
            for i, value in enumerate(values):
                folder = Path(directory)/str(i)
                folder.mkdir()
                paths.append(folder/"result.json")

            def summary():
                for path, value in zip(paths, values):
                    path.write_text(json.dumps(value))
                return repeated.summarize(paths, 3)
            for defect in ("convergence", "jitter", "endpoint", "configuration", "cleanup", "metrics", "malformed"):
                values[-1] = repeat_result("2")
                last = values[-1]
                if defect == "convergence":
                    last["stationary_observations"]["initial"]["convergence"]["converged"] = False
                elif defect == "jitter":
                    last["stationary_observations"]["initial"]["jitter"].update(position_std_m=.04, outcome="FAIL")
                elif defect == "endpoint":
                    last["endpoints"]["initial"]["amcl"]["stamp"] = 0.
                elif defect == "configuration":
                    last["configuration"]["observation"]["position_jitter_limit_m"] = .05
                elif defect == "cleanup":
                    last["process_exit"]["harness"] = None
                elif defect == "metrics":
                    last["stationary_observations"]["initial"]["jitter"]["position_std_m"] = math.nan
                else:
                    last["stationary_observations"] = ["malformed"]
                with self.subTest(defect=defect):
                    result = summary()
                    self.assertEqual(result["overall"], "INVALID")
                    self.assertEqual(result["trials"][-1]["trial_verdicts"]["overall"], "PASS")
                    json.dumps(runner.json_safe(result), allow_nan=False)
            values = [repeat_result(str(i)) for i in range(3)]
            values[0]["endpoints"]["initial"]["errors"] = dict(position_error_m=99, yaw_error_rad=99)
            self.assertEqual(summary()["statistics"]["initial"]["position_error_m"]["maximum"], 0.)
            for value in values:
                value["verdicts"].update(localization="NOT_SCORED", overall="NOT_SCORED")
                value["configuration"]["acceptance_thresholds"].update(position_m=None, absolute_yaw_rad=None)
                for ob in value["stationary_observations"].values():
                    for check in ("convergence", "jitter"):
                        ob[check]["outcome"] = "NOT_SCORED"
            self.assertEqual(summary()["overall"], "NOT_SCORED")

    def test_jitter_yaw_and_sampling_gap_boundaries(self):
        for angle, expected in ((.02, "PASS"), (.0200001, "FAIL")):
            rows = series(6, .41)
            for i, row in enumerate(rows):
                row["amcl"]["yaw"] = angle*(-1)**i
            self.assertEqual(observation(rows, end=102.3)["jitter"]["outcome"], expected)
        self.assertEqual(observation(series(6, 1.), end=105.3)["measurement"], "PASS")
        self.assertEqual(observation(series(6, 1.000001), end=105.3)["measurement"], "INVALID")

    def test_repeat_does_not_start_next_trial_after_incomplete_cleanup(self):
        with tempfile.TemporaryDirectory(prefix="amcl-repeat-cleanup-") as directory:
            output = Path(directory)/"repeat"

            def child(command, **kwargs):
                folder = Path(command[command.index("--output")+1])
                folder.mkdir()
                data = repeat_result("0")
                data["process_exit"]["harness"] = None
                (folder/"result.json").write_text(json.dumps(data))
            with patch.object(repeated.sys, "argv", ["run_repeated.py", "--runs", "3", "--output", str(output)]), \
                    patch.object(repeated.subprocess, "run", side_effect=child) as launch, \
                    patch.object(repeated.sys, "stdout", io.StringIO()):
                self.assertEqual(repeated.main(), 2)
            self.assertEqual(launch.call_count, 1)
            self.assertEqual(json.loads((output/"repeat_summary.json").read_text())["actual_runs"], 1)


if __name__ == "__main__":
    unittest.main()
