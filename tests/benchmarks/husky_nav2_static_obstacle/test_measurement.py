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

"""Synthetic contracts for the actual benchmark; no ROS, HTTP, or engine.

Geometry expectations are hand-computed. Event tests run the production polling
loop synchronously against finite response batches; verdict tests use synthetic
measurements, never a second scoring implementation.
"""

import ast
import importlib.util
import json
import math
import threading
from pathlib import Path

import pytest

SPEC = importlib.util.spec_from_file_location('husky_obstacle_trial', Path(__file__).with_name('run_trial.py'))
runner = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(runner)


@pytest.fixture(autouse=True)
def forbid_live_io(monkeypatch):
    """Fail immediately if a test accidentally attempts HTTP or process launch."""
    def forbidden(*args, **kwargs):
        pytest.fail('unit test attempted live HTTP or process launch')

    monkeypatch.setattr(runner, 'http', forbidden)
    monkeypatch.setattr(runner.subprocess, 'Popen', forbidden)


@pytest.fixture
def trial(tmp_path):
    """Minimal valid measurements; bypass constructor Git/machine discovery."""
    value = runner.Trial.__new__(runner.Trial)
    value.out = tmp_path
    value.control = False
    value.robot_ids = {'HUSKY', '#wheel'}
    value.scene_ids = {'HUSKY', '#wheel', 'NAV_OBSTACLE', 'FLOOR'}
    value.data = {
        'invalid_reasons': [], 'failure_reasons': [], 'event_errors': [],
        'sequence_errors': [], 'event_drops': 0, 'ring_overwrites': 0,
        'event_polls': 1, 'robot_contact_readbacks': 0,
        'observed_contact_episodes': 0, 'contact_events': [],
        'contact_scope': {'contacts': [{'a_def': 'HUSKY', 'b_def': 'FLOOR'}]},
        'robot_start_node': {'orientation': [1, 0, 0, 0, 1, 0, 0, 0, 1]},
        'robot_end_node': {'orientation': [1, 0, 0, 0, 1, 0, 0, 0, 1]},
        'max_pose_gap_sim_s': 0.1, 'local_marking_during_approach': {'cells': 1},
        'detour': {'crossing_samples': 2}, 'action_status': 4, 'action_error': 0,
        'position_error_m': 0.1, 'yaw_error_rad': 0.1, 'sampled_overlap_count': 0,
        'min_sampled_hull_clearance_m': 0.4, 'traveled_distance_m': 5.0,
        'sim_navigation_s': 20.0, 'wall_navigation_s': 40.0,
    }
    return value


def assess(trial):
    """Invoke production verdict and verify the persisted result matches it."""
    trial.verdict()
    assert json.loads((trial.out / 'result.json').read_text()) == trial.data
    return (trial.data['navigation_outcome'], trial.data['measurement_outcome'], trial.data['outcome'])


@pytest.mark.parametrize(('pose', 'expected'), [
    ((0, 0, 0), 1.2063),             # 1.7 - 0.4937 metres.
    ((2, 1, 0), 0.16535),           # 1 - 0.33465 - 0.5 metres.
    ((2, 1, math.pi / 2), 0.0063),  # Rotating swaps the hull half-extents.
    ((0, 2, 0), math.hypot(1.2063, 1.16535)),  # Two separated axis-aligned corners.
])
def test_clearance_in_metres(pose, expected):
    """Check face, rotated face, and diagonal Euclidean separations."""
    distance, overlap = runner.clearance(runner.hull(*pose))
    assert distance == pytest.approx(expected, abs=1e-12)
    assert not overlap


def test_hull_translation_and_radians():
    """The envelope is in metres and yaw is in radians, including wheels."""
    points = runner.hull(10, -3, math.pi / 2)
    assert points[0] == pytest.approx((10.33465, -3.4937))
    assert points[2] == pytest.approx((9.66535, -2.5063))
    assert runner.BOX == [(1.7, -0.5), (2.3, -0.5), (2.3, 0.5), (1.7, 0.5)]


@pytest.mark.parametrize(('right_edge', 'expected', 'overlap'), [
    (1.7 - 1e-9, 1e-9, False), (1.7, 0.0, True), (1.7 + 1e-9, 0.0, True),
])
def test_touch_and_near_boundary(right_edge, expected, overlap):
    """Exact touch counts as overlap; a tiny positive separation does not."""
    poly = [(right_edge - 1, -0.25), (right_edge, -0.25),
            (right_edge, 0.25), (right_edge - 1, 0.25)]
    distance, actual_overlap = runner.clearance(poly)
    assert distance == pytest.approx(expected, abs=1e-15)
    assert actual_overlap is overlap


def test_hull_inside_obstacle():
    """A penetrating robot envelope has zero clearance and overlap=True."""
    assert runner.clearance(runner.hull(2, 0, 0)) == (0.0, True)


def event(seq, kind, robot='HUSKY', other='NAV_OBSTACLE'):
    """Build the collector's existing supervisor contact record."""
    return {'source': 'sup', 'seq': seq, 'type': kind,
            'a_def': robot, 'b_def': other, 't_sim_ms': seq * 16}


def batch(events, cursor, dropped=0, log_cursor=0):
    """Build the fields actually read from /sim/events."""
    return {'events': events, 'next_since': cursor, 'next_log_since': log_cursor, 'dropped_sup': dropped}


class PollBudget:
    """Bound the existing synchronous loop without threads or real waiting."""

    def __init__(self, count):
        """Allow exactly count HTTP attempts."""
        self.remaining = count

    def is_set(self):
        """Stop after the final attempt."""
        return self.remaining == 0

    def wait(self, seconds):
        """Consume one attempt without sleeping."""
        assert seconds == 0.25
        self.remaining -= 1


def collect(trial, monkeypatch, responses):
    """Run Trial.events against finite batches and retain raw production output."""
    trial.stop = PollBudget(len(responses))
    trial.data['event_polls'] = 0
    pending = iter(responses)
    requests = []

    def reply(port, path):
        assert port == 6789
        requests.append(path)
        response = next(pending)
        if isinstance(response, Exception):
            raise response
        return response

    monkeypatch.setattr(runner, 'http', reply)
    trial.events()
    rows = [json.loads(line) for line in (trial.out / 'events.jsonl').read_text().splitlines()]
    return requests, rows


def test_single_batch_began_ended(trial, monkeypatch):
    """Attributed raw records and one observed episode agree, and imply FAIL."""
    records = [event(1, 'contact.began'), event(2, 'contact.ended')]
    _, rows = collect(trial, monkeypatch, [batch(records, 2)])
    assert trial.data['contact_events'] == records
    assert trial.data['observed_contact_episodes'] == 1
    assert trial.data['robot_contact_readbacks'] == 2
    assert rows[0]['response']['events'] == records
    assert assess(trial) == ('FAIL', 'PASS', 'FAIL')


def test_cursor_empty_repoll_and_ring_evictions(trial, monkeypatch):
    """Empty re-polls add no episodes; evicting already-read events is not loss."""
    responses = [batch([event(1, 'contact.began')], 1, log_cursor=7),
                 batch([], 1, dropped=1, log_cursor=7),
                 batch([event(2, 'contact.ended')], 2, dropped=1, log_cursor=8)]
    requests, rows = collect(trial, monkeypatch, responses)
    assert requests == ['/sim/events?since=0&log_since=0&limit=1000',
                        '/sim/events?since=1&log_since=7&limit=1000',
                        '/sim/events?since=1&log_since=7&limit=1000']
    assert [row['requested_since'] for row in rows] == [0, 1, 1]
    assert [row['response'] for row in rows] == responses
    assert all(row['received_monotonic_ns'] >= row['requested_monotonic_ns'] for row in rows)
    assert trial.data['event_polls'] == 3
    assert trial.data['observed_contact_episodes'] == 1
    assert len(trial.data['contact_events']) == 2
    assert trial.data['ring_overwrites'] == 1
    assert trial.data['event_drops'] == 0
    assert trial.data['sequence_errors'] == []
    assert trial.data['event_errors'] == []
    assert assess(trial) == ('FAIL', 'PASS', 'FAIL')


def test_overlapping_pairs_are_one_episode_until_separation(trial, monkeypatch):
    """Multiple touching robot links share one episode; separation permits another."""
    records = [event(1, 'contact.began'), event(2, 'contact.began', '#wheel'),
               event(3, 'contact.ended'), event(4, 'contact.ended', '#wheel'),
               event(5, 'contact.began'), event(6, 'contact.ended')]
    collect(trial, monkeypatch, [batch(records[:3], 3), batch(records[3:], 6)])
    assert trial.data['observed_contact_episodes'] == 2
    assert trial.data['contact_events'] == records


@pytest.mark.parametrize(('responses', 'diagnostic'), [
    ([batch([event(2, 'contact.began')], 2)], 'sequence_errors'),
    ([batch([event(1, 'contact.began')], 2)], 'sequence_errors'),
    ([batch([], 0, dropped=1)], 'event_drops'),
    ([{'events': [], 'dropped_sup': 0}], 'event_errors'),
    ([batch([{'source': 'sup', 'seq': 1}], 1)], 'event_errors'),
    ([OSError('observer unavailable')], 'event_errors'),
    ([batch([event(1, 'contact.began', 'UNKNOWN')], 1)], 'event_errors'),
])
def test_broken_contact_evidence_is_invalid(trial, monkeypatch, responses, diagnostic):
    """Gaps, malformed records, unknown attribution and observer failure cannot pass."""
    collect(trial, monkeypatch, responses)
    assert trial.data[diagnostic]
    assert assess(trial)[1:] == ('INVALID', 'INVALID')


@pytest.mark.parametrize(('kind', 'episodes', 'expected'), [
    ('contact.began', 1, ('FAIL', 'PASS', 'FAIL')),
    ('contact.ended', 0, ('PASS', 'INVALID', 'INVALID')),
])
def test_unpaired_contact_record(trial, monkeypatch, kind, episodes, expected):
    """Unmatched ends invalidate evidence without inventing a contact episode."""
    record = event(1, kind)
    _, rows = collect(trial, monkeypatch, [batch([record], 1)])
    assert trial.data['contact_events'] == [record]
    assert rows[0]['response']['events'] == [record]
    assert trial.data['observed_contact_episodes'] == episodes
    assert trial.data['sequence_errors'] == []
    if kind == 'contact.ended':
        assert trial.data['event_errors'] == [{'unpaired_contact_ended': record}]
    else:
        assert trial.data['event_errors'] == []
    assert assess(trial) == expected


def test_server_replay_is_invalid_not_silently_deduplicated(trial, monkeypatch):
    """Old sequence replay violates the API contract, even if contacts look complete."""
    records = [event(1, 'contact.began'), event(2, 'contact.ended')]
    _, rows = collect(trial, monkeypatch, [batch(records, 2), batch(records, 2)])
    assert len(rows) == 2
    assert trial.data['sequence_errors']
    assert trial.data['event_errors'] == []
    assert assess(trial)[1:] == ('INVALID', 'INVALID')


def test_empty_complete_poll_has_no_contact_or_pair_error(trial, monkeypatch):
    """An empty continuous batch plus existing readback proof can still PASS."""
    collect(trial, monkeypatch, [batch([], 0)])
    assert trial.data['contact_events'] == []
    assert trial.data['observed_contact_episodes'] == 0
    assert trial.data['event_errors'] == []
    assert assess(trial) == ('PASS', 'PASS', 'PASS')


def test_replayed_end_is_only_a_continuity_error(trial, monkeypatch):
    """An already matched old end must not create a new unmatched-end error."""
    records = [event(1, 'contact.began'), event(2, 'contact.ended')]
    collect(trial, monkeypatch, [batch(records, 2), batch([records[1]], 2)])
    assert trial.data['sequence_errors']
    assert trial.data['event_errors'] == []
    assert trial.data['observed_contact_episodes'] == 1
    assert assess(trial) == ('FAIL', 'INVALID', 'INVALID')


def test_replayed_end_within_batch_is_only_a_continuity_error(trial, monkeypatch):
    """A repeated sequence in one response must not invent a pairing anomaly."""
    records = [event(1, 'contact.began'), event(2, 'contact.ended')]
    collect(trial, monkeypatch, [batch(records + [records[1]], 2)])
    assert trial.data['sequence_errors']
    assert trial.data['event_errors'] == []
    assert len(trial.data['contact_events']) == 3
    assert trial.data['observed_contact_episodes'] == 1
    assert assess(trial) == ('FAIL', 'INVALID', 'INVALID')


def test_reversed_pair_identifiers_match_across_batches(trial, monkeypatch):
    """Pair identity is unordered, and active state survives HTTP boundaries."""
    began = event(1, 'contact.began')
    ended = event(2, 'contact.ended', robot='NAV_OBSTACLE', other='HUSKY')
    collect(trial, monkeypatch, [batch([began], 1), batch([ended], 2)])
    assert trial.data['event_errors'] == []
    assert trial.data['sequence_errors'] == []
    assert trial.data['observed_contact_episodes'] == 1
    assert assess(trial) == ('FAIL', 'PASS', 'FAIL')


@pytest.mark.parametrize('scope', ['floor', 'other_robot', 'log'])
def test_unmatched_end_outside_trial_contact_scope(trial, monkeypatch, scope):
    """Pair validation must not broaden the existing attribution rules."""
    if scope == 'floor':
        record = event(1, 'contact.ended', other='FLOOR')
    elif scope == 'other_robot':
        trial.scene_ids.add('OTHER_ROBOT')
        record = event(1, 'contact.ended', robot='OTHER_ROBOT')
    else:
        record = dict(event(1, 'contact.ended'), source='log')
    cursor = 0 if scope == 'log' else 1
    _, rows = collect(trial, monkeypatch, [batch([record], cursor)])
    assert rows[0]['response']['events'] == [record]
    assert trial.data['contact_events'] == []
    assert trial.data['event_errors'] == []
    assert trial.data['observed_contact_episodes'] == 0
    assert assess(trial) == ('PASS', 'PASS', 'PASS')


def test_non_obstacle_contacts_and_log_events_are_retained_raw(trial, monkeypatch):
    """Floor contacts prove readback; log records do not enter supervisor sequencing."""
    records = [event(1, 'contact.began', other='FLOOR'),
               {'source': 'log', 'seq': 10, 'type': 'controller.log'}]
    _, rows = collect(trial, monkeypatch, [batch(records, 1, log_cursor=10)])
    trial.data['contact_scope'] = {}
    assert rows[0]['response']['events'] == records
    assert trial.data['robot_contact_readbacks'] == 1
    assert not trial.data['contact_events']
    assert trial.data['observed_contact_episodes'] == 0
    assert assess(trial) == ('PASS', 'PASS', 'PASS')


def test_valid_measurements_pass(trial):
    """Successful behavior plus credible evidence produces all three PASS values."""
    assert assess(trial) == ('PASS', 'PASS', 'PASS')


@pytest.mark.parametrize(('key', 'limit', 'direction'), [
    ('position_error_m', 0.25, math.inf), ('yaw_error_rad', 0.25, math.inf),
    ('min_sampled_hull_clearance_m', 0.10, -math.inf),
    ('traveled_distance_m', 8.0, math.inf), ('sim_navigation_s', 60.0, math.inf),
    ('wall_navigation_s', 120.0, math.inf),
])
@pytest.mark.parametrize('outside', [False, True])
def test_behavior_threshold_boundaries(trial, key, limit, direction, outside):
    """Exact limits pass; the next representable float beyond each limit fails."""
    trial.data[key] = math.nextafter(limit, direction) if outside else limit
    assert assess(trial) == (('FAIL', 'PASS', 'FAIL') if outside else ('PASS', 'PASS', 'PASS'))


@pytest.mark.parametrize(('key', 'value'), [
    ('action_status', 6), ('action_error', 1), ('observed_contact_episodes', 1),
    ('sampled_overlap_count', 1),
])
def test_behavior_failure_with_valid_evidence(trial, key, value):
    """Action and observed safety failures are FAIL, not measurement INVALID."""
    trial.data[key] = value
    assert assess(trial) == ('FAIL', 'PASS', 'FAIL')


@pytest.mark.parametrize('missing', [
    'position_error_m', 'yaw_error_rad', 'min_sampled_hull_clearance_m',
    'traveled_distance_m', 'sim_navigation_s', 'wall_navigation_s',
])
def test_missing_behavior_metric_never_passes(trial, missing):
    """Existing conservative defaults classify missing behavior metrics as FAIL."""
    del trial.data[missing]
    assert assess(trial) == ('FAIL', 'PASS', 'FAIL')


@pytest.mark.parametrize('defect', [
    'no_polls', 'no_readback', 'tf_expiry', 'local_marking',
    'no_crossing', 'start_orientation', 'end_orientation', 'preflight',
])
def test_measurement_failures_preserve_navigation_result(trial, defect):
    """Exercise verdict's evidence gates, including propagated preflight failure."""
    if defect == 'no_polls':
        trial.data['event_polls'] = 0
    elif defect == 'no_readback':
        trial.data['contact_scope'] = {}
    elif defect == 'tf_expiry':
        trial.data['scan_tf_expired'] = 1
    elif defect == 'local_marking':
        trial.data['local_marking_during_approach'] = None
    elif defect == 'no_crossing':
        trial.data['detour'] = {}
    elif defect == 'start_orientation':
        trial.data['robot_start_node'] = {}
    elif defect == 'end_orientation':
        trial.data['robot_end_node'] = {'orientation': [1]}
    else:
        # run() raises on scan/global preflight failure; main() retains it here.
        trial.data['invalid_reasons'].append('preflight scan/global obstacle marking missing')
    assert assess(trial) == ('PASS', 'INVALID', 'INVALID')


@pytest.mark.parametrize('outside', [False, True])
def test_merged_pose_gap_boundary(trial, outside):
    """The 0.20 simulation-second validity limit is inclusive."""
    trial.data['max_pose_gap_sim_s'] = math.nextafter(0.2, math.inf) if outside else 0.2
    expected = ('PASS', 'INVALID', 'INVALID') if outside else ('PASS', 'PASS', 'PASS')
    assert assess(trial) == expected


@pytest.mark.parametrize('angle', [0.0, 0.03, 0.03001])
def test_endpoint_flatness(trial, angle):
    """Verify the actual acos(matrix[8]) endpoint gate, in radians."""
    trial.data['robot_end_node']['orientation'][8] = math.cos(angle)
    expected = 'INVALID' if angle > 0.03 else 'PASS'
    assert assess(trial) == ('PASS', expected, expected)


def test_invalid_takes_precedence_over_behavior_failure(trial):
    """Keep FAIL navigation visible even when INVALID wins the combined verdict."""
    trial.data['position_error_m'] = 1.0
    trial.data['event_errors'].append('observer failed')
    assert assess(trial) == ('FAIL', 'INVALID', 'INVALID')
    assert trial.data['failure_reasons'] and trial.data['invalid_reasons']


@pytest.mark.parametrize(('filename', 'text'), [
    ('sidecar.log', 'no cmd_vel for 0.5 s; stopping the robot\n'),
    ('nav.log', 'Message Filter dropping message: earlier than all the data\n'),
])
def test_existing_log_failure_signatures(trial, filename, text):
    """The production log counters feed the unchanged behavior gate."""
    (trial.out / filename).write_text(text)
    assert assess(trial) == ('FAIL', 'PASS', 'FAIL')
    assert trial.data['watchdog_stops'] + trial.data['stale_scans'] == 1


@pytest.mark.parametrize('missing', [None, 'matched_began_ended_pairs', 'native_contact_event_points'])
def test_contact_positive_control_contract(trial, missing):
    """Collector-only validity needs both pairing and physical contact evidence."""
    trial.control = True
    proof = {'matched_began_ended_pairs': [['began', 'ended']], 'native_contact_event_points': [[0, 0, 0]]}
    if missing:
        proof.pop(missing)
    trial.data['collector_control'] = proof
    expected = 'INVALID' if missing else 'PASS'
    assert assess(trial) == ('NOT_RUN', expected, expected)


@pytest.mark.parametrize('native_missing', [False, True])
def test_native_and_merged_pose_coverage_remain_distinct(trial, native_missing):
    """Bridge coverage does not fabricate native odometry or interpolate poses."""
    recorder = runner.PoseRecorder.__new__(runner.PoseRecorder)
    recorder.lock = threading.Lock()
    recorder.errors = []
    recorder.clocks = []
    recorder.rows = []
    for stamp in (0.0, 0.125, 0.25, 0.375, 0.5):
        recorder.rows.append({'channel': 'bridge', 'stamp': stamp, 'pose': [stamp, 0, 0],
                              'request_latency_s': 0.001})
    if not native_missing:
        for stamp in (0.0, 0.5):
            ns = int((1 + stamp) * 1e9)
            recorder.rows.append({'channel': 'odom', 'stamp': stamp, 'pose': [stamp, 0, 0],
                                  'dds_source_ns': ns, 'dds_received_ns': ns,
                                  'callback_wall_ns': ns, 'callback_monotonic_ns': ns})
    poses = recorder.poses(0, 0.5)
    assert poses == [(stamp, stamp, 0, 0) for stamp in (0.0, 0.125, 0.25, 0.375, 0.5)]
    summary = recorder.summary(0, 0.5)
    assert summary['odom']['samples'] == (0 if native_missing else 2)
    assert summary['odom']['max_sim_gap_s'] == (None if native_missing else 0.5)
    assert summary['bridge']['max_sim_gap_s'] == 0.125
    trial.data['timing'] = summary
    trial.data['max_pose_gap_sim_s'] = 0.125
    assert assess(trial) == ('PASS', 'PASS', 'PASS')


def test_runner_module_scope_has_no_ros_or_launch_dependency():
    """Import executes stdlib definitions only; ROS imports stay inside methods."""
    tree = ast.parse(Path(runner.__file__).read_text())
    roots = {node.names[0].name.split('.')[0] if isinstance(node, ast.Import) else node.module.split('.')[0]
             for node in tree.body if isinstance(node, (ast.Import, ast.ImportFrom))}
    assert roots <= {'argparse', 'json', 'math', 'os', 'pathlib', 'signal', 'socket',
                     'subprocess', 'sys', 'threading', 'time', 'urllib'}


def test_startup_scan_clock_serialization(tmp_path, monkeypatch):
    """Serialize exact stamps, frames, DDS metadata and receipt clocks."""
    from types import SimpleNamespace as NS

    monkeypatch.setattr(runner.time, 'monotonic_ns', lambda: 123)
    monkeypatch.setattr(runner.time, 'time_ns', lambda: 456)
    diagnostic = runner.StartupTiming(tmp_path)
    stamp = NS(sec=2, nanosec=336000000)
    diagnostic.message('scan', NS(header=NS(stamp=stamp, frame_id='base_laser')),
                       NS(source_timestamp=100, received_timestamp=110))
    diagnostic.message('clock', NS(clock=stamp))
    diagnostic.close()
    rows = [json.loads(line) for line in (tmp_path / 'startup_timing.jsonl').read_text().splitlines()]
    assert rows[0] == {'channel': 'scan', 'stamp_ns': 2336000000, 'frame_id': 'base_laser',
                       'received_monotonic_ns': 123, 'received_wall_ns': 456,
                       'dds_source_ns': 100, 'dds_received_ns': 110}
    assert rows[1]['channel'] == 'clock' and rows[1]['stamp_ns'] == 2336000000
    assert rows[-1]['errors'] == []


def test_startup_tf_batch_filters_and_preserves_frames(tmp_path):
    """Retain every relevant transform in a mixed batch."""
    from types import SimpleNamespace as NS

    def transform(parent, child):
        return NS(header=NS(stamp=NS(sec=3, nanosec=16), frame_id=parent), child_frame_id=child)

    diagnostic = runner.StartupTiming(tmp_path)
    message = NS(transforms=[transform('odom', 'base_link'), transform('world', 'unrelated'),
                             transform('base_link', 'base_laser'), transform('map', 'odom')])
    diagnostic.message('tf', message)
    diagnostic.message('tf_static', message)
    diagnostic.close()
    rows = [json.loads(line) for line in (tmp_path / 'startup_timing.jsonl').read_text().splitlines()]
    expected = [{'parent': p, 'child': c, 'stamp_ns': 3000000016}
                for p, c in [('odom', 'base_link'), ('base_link', 'base_laser'), ('map', 'odom')]]
    assert rows[0]['transforms'] == rows[1]['transforms'] == expected
    assert [r['channel'] for r in rows[:2]] == ['tf', 'tf_static']


def test_startup_goal_and_log_boundaries(tmp_path):
    """Keep live goal boundaries distinct from imported Nav2 log time."""
    diagnostic = runner.StartupTiming(tmp_path)
    diagnostic.boundary('goal_send', clock_sim_s=3.92)
    diagnostic.nav_log('[bt_navigator-5] [INFO] [1791799873.847191498] [bt_navigator]: Begin navigating\n')
    diagnostic.close()
    rows = [json.loads(line) for line in (tmp_path / 'startup_timing.jsonl').read_text().splitlines()]
    assert rows[0]['event'] == 'goal_send' and rows[0]['clock_sim_s'] == 3.92
    assert rows[1]['event_wall_ns'] == 1791799873847191498
    assert rows[1]['channel'] == 'nav_log'
    source = Path(runner.__file__).read_text()
    assert source.index('self.recorder = PoseRecorder(self)') < source.index('self.launch("nav",')
    assert source.index('self.startup.boundary("goal_send"') < source.index('sent = client.send_goal_async(')


@pytest.mark.parametrize('stale_scan', [False, True])
def test_startup_diagnostics_do_not_change_verdict(trial, stale_scan, capsys):
    """Diagnostic success or failure cannot change strict scoring."""
    if stale_scan:
        (trial.out / 'nav.log').write_text('Message Filter dropping message: earlier than all the data\n')
    before = assess(trial)
    trial.startup = runner.StartupTiming(trial.out)
    trial.startup.boundary('goal_send', clock_sim_s=3.92)
    trial.startup.message('scan', object())  # Diagnostic failure is explicit, not scoring input.
    trial.startup.close()
    assert trial.startup.errors
    assert 'startup timing diagnostic error:' in capsys.readouterr().err
    trial.data['failure_reasons'] = []
    trial.data['invalid_reasons'] = []
    assert assess(trial) == before
    assert before == (('FAIL', 'PASS', 'FAIL') if stale_scan else ('PASS', 'PASS', 'PASS'))


@pytest.fixture
def diagnostic_close_failure(tmp_path):
    """Use a real diagnostic file with an injected final-close I/O error."""
    diagnostic = runner.StartupTiming(tmp_path)

    class CloseFailureStream:
        def __init__(self, stream):
            self.stream = stream
            self.close_calls = 0
            self.failure = OSError('synthetic diagnostic flush failure')

        def write(self, value):
            return self.stream.write(value)

        def close(self):
            self.close_calls += 1
            self.stream.close()
            raise self.failure

    diagnostic.stream = CloseFailureStream(diagnostic.stream)
    return diagnostic


def test_startup_close_normal(tmp_path):
    """Normal close persists the final record and releases the file."""
    diagnostic = runner.StartupTiming(tmp_path)
    diagnostic.close()
    assert diagnostic.stream.closed
    assert not diagnostic.errors
    rows = [json.loads(line) for line in (tmp_path / 'startup_timing.jsonl').read_text().splitlines()]
    assert rows[-1]['channel'] == 'diagnostic_end'
    assert rows[-1]['errors'] == []


def test_startup_close_oserror_visible(diagnostic_close_failure, capsys):
    """A final-close OSError is reported without escaping the diagnostic."""
    diagnostic = diagnostic_close_failure
    diagnostic.close()
    assert diagnostic.errors == ['synthetic diagnostic flush failure']
    assert 'startup timing diagnostic error: synthetic diagnostic flush failure' in capsys.readouterr().err
    assert diagnostic.stream.close_calls == 1
    assert diagnostic.stream.stream.closed


def test_startup_close_non_io_error_propagates(diagnostic_close_failure):
    """The local handler must not swallow unrelated implementation errors."""
    diagnostic_close_failure.stream.failure = RuntimeError('unrelated close defect')
    with pytest.raises(RuntimeError, match='unrelated close defect'):
        diagnostic_close_failure.close()
    assert diagnostic_close_failure.stream.close_calls == 1
    assert diagnostic_close_failure.stream.stream.closed


@pytest.mark.parametrize(('navigation_failed', 'measurement_invalid', 'expected'), [
    (False, False, ('PASS', 'PASS', 'PASS')),
    (True, False, ('FAIL', 'PASS', 'FAIL')),
    (False, True, ('PASS', 'INVALID', 'INVALID')),
    (True, True, ('FAIL', 'INVALID', 'INVALID')),
])
def test_startup_close_failure_cleanup_reaches_verdict(
        trial, diagnostic_close_failure, capsys, navigation_failed, measurement_invalid, expected):
    """Actual cleanup continues and production verdict writes every classification."""
    import io

    if navigation_failed:
        trial.data['position_error_m'] = 0.3
    if measurement_invalid:
        trial.data['max_pose_gap_sim_s'] = 0.3
    trial.stop = threading.Event()
    trial.processes = []  # No engine/ROS process needed to exercise diagnostic cleanup.
    trial.files = [io.StringIO()]
    trial.startup = diagnostic_close_failure
    trial.cleanup()
    assert trial.stop.is_set()
    assert trial.files[0].closed  # Cleanup after the failing diagnostic still executes.
    assert trial.startup.stream.close_calls == 1
    assert trial.startup.stream.stream.closed
    assert 'synthetic diagnostic flush failure' in capsys.readouterr().err
    assert assess(trial) == expected  # Calls Trial.verdict and reads its result.json.
    assert not any('diagnostic' in reason for key in ('invalid_reasons', 'failure_reasons')
                   for reason in trial.data[key])


@pytest.mark.parametrize('metadata', [
    {'source_timestamp': 100, 'received_timestamp': 110},
    {'source_timestamp': 100}, {}, None,
])
def test_startup_dict_metadata(tmp_path, metadata):
    """Dictionary metadata preserves supplied fields without inventing timestamps."""
    from types import SimpleNamespace as NS

    diagnostic = runner.StartupTiming(tmp_path)
    diagnostic.message('scan', NS(header=NS(stamp=NS(sec=1, nanosec=2), frame_id='base_laser')), metadata)
    diagnostic.close()
    row = json.loads((tmp_path / 'startup_timing.jsonl').read_text().splitlines()[0])
    supplied = metadata or {}
    assert row['dds_source_ns'] == supplied.get('source_timestamp')
    assert row['dds_received_ns'] == supplied.get('received_timestamp')
    assert row['stamp_ns'] == 1000000002


@pytest.mark.parametrize('control', [False, True])
@pytest.mark.parametrize('mode', ['resolved', 'expired', 'navigation_failed', 'outside', 'incoming', 'interrupted'])
def test_final_scan_retry_drain_and_verdict(trial, monkeypatch, mode, control):
    """Production drain settles the frozen batch before production verdict writes results."""
    import io
    from types import SimpleNamespace as NS

    trial.control = control
    if control:
        trial.data['collector_control'] = {
            'matched_began_ended_pairs': [['HUSKY', 'NAV_OBSTACLE']],
            'native_contact_event_points': [[1.7, 0.0, 0.3]],
        }
    now = [10.0]
    monkeypatch.setattr(runner.time, 'monotonic', lambda: now[0])
    trial.tf_resolved = 0
    trial.tf_expired = []
    trial.tf_startup_expired = []
    scan = NS(header=NS(stamp=NS(sec=3, nanosec=16000000)))
    outside = NS(header=NS(stamp=NS(sec=1, nanosec=0)))
    pending = [(outside, 9.9, 'startup-only', False)]
    if mode != 'outside':
        pending.append((scan, 9.5, 'TF not ready', True))
    raw = io.StringIO()
    timeouts = []
    callbacks = []

    def project(message):
        callbacks.append(message)
        if mode == 'resolved' and now[0] >= 10.5:
            return
        raise RuntimeError('TF still unavailable')

    def spin_until(predicate, timeout):
        timeouts.append(timeout)
        if mode == 'interrupted':
            return False
        for _ in range(10):
            if predicate():
                return True
            if now[0] >= 10.0 + timeout:
                return False
            # Model TF delivery while the real executor stays available.
            now[0] += 0.5
            if mode == 'incoming':
                pending.append((outside, now[0], 'post-window', False))
        pytest.fail('drain followed newly arriving scans instead of its frozen batch')

    if mode == 'navigation_failed':
        trial.data['position_error_m'] = 0.3
    trial.drain_scan_retries(pending, project, raw, spin_until)
    trial.data['scan_tf_expired'] = trial.tf_expired
    expected_measurement = 'PASS' if mode in ('resolved', 'outside') else 'INVALID'
    expected_navigation = 'NOT_RUN' if control else 'FAIL' if mode == 'navigation_failed' else 'PASS'
    expected_overall = 'INVALID' if expected_measurement == 'INVALID' else 'PASS' if control else expected_navigation
    assert assess(trial) == (expected_navigation, expected_measurement, expected_overall)
    assert outside not in callbacks
    assert all(not item[3] for item in pending)
    assert trial.tf_startup_expired == []
    assert now[0] <= 11.5  # Original begin=9.5 + existing two-second budget.
    if mode == 'outside':
        assert timeouts == [] and callbacks == [] and raw.getvalue() == ''
    else:
        assert timeouts == [1.5]
        rows = [json.loads(line) for line in raw.getvalue().splitlines()]
        assert len(rows) == 1
        if mode == 'resolved':
            assert trial.tf_resolved == 1 and trial.tf_expired == []
            assert rows[0]['scan_tf_resolved']['stamp'] == pytest.approx(3.016)
        else:
            assert trial.tf_resolved == 0 and len(trial.tf_expired) == 1
            assert rows[0]['scan_tf_expired']['stamp'] == pytest.approx(3.016)
            assert 'scan transform unavailable after bounded observer retry' in trial.data['invalid_reasons']


def test_scan_measurement_closes_before_drain_and_node_cleanup():
    """The live callback excludes new scans after the action result, retaining TF during drain."""
    source = Path(runner.__file__).read_text()
    assert 'if self.measurement_finished:\n                return' in source
    for end in (source.index('self.measurement_finished = True'), source.rindex('self.measurement_finished = True')):
        drain = source.index('self.drain_scan_retries(pending_scans, project_scan, raw, spin_until)', end)
        cleanup = source.index('self.recorder.close()', drain)
        destroy = source.index('node.destroy_node()', cleanup)
        assert end < drain < cleanup < destroy
