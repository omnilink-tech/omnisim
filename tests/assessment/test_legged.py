"""Independent outcome checks for physical legged assessments."""
from omnisim.assessment.legged import LeggedSpec,grade


def trace(robot):
    z=.3 if robot=='go2' else .76
    return [dict(t_s=i*.016,position_m=[i*.001 if robot=='go2' else .01,0,z],
                 tilt_rad=.02,roll_rad=0.,pitch_rad=.02,contacts=4,
                 contact_nodes=['left_ankle_roll_link','right_ankle_roll_link']) for i in range(2001)]


def test_progress_and_policy_pose_scores_cannot_hide_a_fall():
    rows=trace('go2')
    rows[1000]['roll_rad']=3.1
    result=grade(LeggedSpec('go2'),rows,{'complete':True},{'solver':'mujoco'},
                 'ONNX loaded:\nBATON switch #1\nBATON switch #2')
    assert result['status']=='FAIL'
    assert 'fell_or_lost_upright_posture' in result['reasons']


def test_missing_policy_or_runtime_is_invalid_not_robot_failure():
    assert grade(LeggedSpec('go2'),trace('go2'),{'complete':True},{},'')['status']=='ERROR'


def test_incomplete_handover_is_not_successful_even_if_upright():
    r=grade(LeggedSpec('go2'),trace('go2'),{'complete':True},{'solver':'mujoco'},'ONNX loaded:')
    assert r['status']=='FAIL' and 'walk_stand_walk_not_completed' in r['reasons']


def test_g1_stand_requires_both_feet_and_rejects_body_support():
    rows=trace('g1')
    spec=LeggedSpec('g1',g1_task='stand')
    assert grade(spec,rows,{'complete':True},{'solver':'mujoco'},'')['status']=='PASS'
    for r in rows[-200:]: r['contact_nodes']=['pelvis','left_ankle_roll_link','right_ankle_roll_link']
    r=grade(spec,rows,{'complete':True},{'solver':'mujoco'},'')
    assert r['status']=='FAIL' and 'two_foot_support_not_maintained' in r['reasons']


def test_truncated_or_discontinuous_trace_is_invalid():
    spec=LeggedSpec('g1',g1_task='stand')
    assert grade(spec,trace('g1')[:-2],{'complete':True},{'solver':'mujoco'},'')['status']=='ERROR'
    rows=trace('g1');del rows[60]
    assert grade(spec,rows,{'complete':True},{'solver':'mujoco'},'')['status']=='ERROR'


def test_level_guard_requires_timestamped_handover_evidence():
    result=grade(LeggedSpec('go2',gate_mode='level'),trace('go2'),{'complete':True},{'solver':'mujoco'},
                 'ONNX loaded:\nBATON switch #1\nBATON switch #2')
    assert result['status']=='ERROR' and 'handover_timing_evidence_missing' in result['reasons']


def test_level_guard_cannot_pass_by_skipping_the_stand_window():
    log='ONNX loaded:\nBATON switch #1\nBATON switch #2\nHANDOVER mode=stand sim_time_s=10.000\nHANDOVER mode=walk sim_time_s=11.000'
    result=grade(LeggedSpec('go2',gate_mode='level'),trace('go2'),{'complete':True},{'solver':'mujoco'},log)
    assert result['status']=='FAIL' and 'standing_window_under_5s' in result['reasons']


def test_level_guard_requires_motion_after_the_return_handover():
    rows=trace('go2')
    for row in rows[1000:]:row['position_m'][0]=1.
    log='ONNX loaded:\nBATON switch #1\nBATON switch #2\nHANDOVER mode=stand sim_time_s=10.000\nHANDOVER mode=walk sim_time_s=16.000'
    result=grade(LeggedSpec('go2',gate_mode='level'),rows,{'complete':True},{'solver':'mujoco'},log)
    assert result['status']=='FAIL' and 'return_walk_not_demonstrated' in result['reasons']
