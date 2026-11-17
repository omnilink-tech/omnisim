"""Behavioral and invalid-evidence checks for the mixed-surface assessment."""
from dataclasses import replace
import math
import pytest
from omnisim.assessment.mixed import MixedSpec, grade


def evidence(spec=MixedSpec()):
    rows=[]
    for i in range(1500):
        t=(i+1)*.016
        started=t>=4
        vx=spec.cruise_m_s if not started else 0
        rows.append(dict(t_s=t,x_m=spec.trigger_x_m if started else spec.trigger_x_m-.8*(4-t),
                         y_m=0.,z_m=.132,vx_m_s=vx,vy_m_s=0.,wz_rad_s=0.,yaw_rad=0.,tilt_rad=0.,
                         path_error_m=0.,event_started=started,left_command_rad_s=0. if started else 4.,right_command_rad_s=0. if started else 4.))
    return rows,dict(complete=True),dict(backend='newton',finalised=True,degraded=False),dict(valid=True)


def test_stationary_after_qualified_approach_passes():
    s=MixedSpec()
    rows,c,r,a=evidence(s)
    # The trigger sample records pre-command measured speed, as the live controller does.
    next(v for v in rows if v['event_started'])['vx_m_s']=s.cruise_m_s
    assert grade(s,rows,c,r,a)['status']=='PASS'


def test_excessive_excursion_fails_even_if_final_robot_is_stopped():
    s=MixedSpec();rows,c,r,a=evidence(s)
    next(v for v in rows if v['event_started'])['vx_m_s']=s.cruise_m_s
    for row in rows:
        if row['t_s']>4.2:row['x_m']+=.4
    result=grade(s,rows,c,r,a)
    assert result['status']=='FAIL'
    assert 'stopping_excursion_exceeded' in result['reasons']


def test_no_drive_cannot_pass_the_stop_test():
    s=MixedSpec();rows,c,r,a=evidence(s)
    for row in rows:row.update(x_m=0,event_started=False,vx_m_s=0)
    assert 'task_trigger_not_reached' in grade(s,rows,c,r,a)['reasons']


@pytest.mark.parametrize('fault',['runtime','adoption','completion','nan','gap'])
def test_invalid_evidence_is_error(fault):
    s=MixedSpec();rows,c,r,a=evidence(s)
    if fault=='runtime':r['finalised']=False
    if fault=='adoption':a['valid']=False
    if fault=='completion':c['complete']=False
    if fault=='nan':rows[40]['x_m']=math.nan
    if fault=='gap':rows.pop(40)
    assert grade(s,rows,c,r,a)['status']=='ERROR'


def test_speed_cap_does_not_change_the_task_requirements_or_environment():
    from dataclasses import asdict
    before=asdict(MixedSpec());after=asdict(replace(MixedSpec(),policy='surface-cap'))
    assert {k:v for k,v in before.items() if k!='policy'}=={k:v for k,v in after.items() if k!='policy'}


def test_turn_oracle_recomputes_path_error_instead_of_trusting_controller():
    s=MixedSpec(task='turn');rows,c,r,a=evidence(s)
    c['turn_finished_t_s']=9.
    for row in rows:
        if row['event_started']:
            angle=min(math.pi/2,max(0,(row['t_s']-4)*math.pi/8))
            row.update(x_m=s.trigger_x_m+math.sin(angle),y_m=1-math.cos(angle),yaw_rad=angle)
    next(v for v in rows if v['event_started'])['vx_m_s']=s.cruise_m_s
    assert grade(s,rows,c,r,a)['status']=='PASS'
    rows[400]['x_m']+=.8
    rows[400]['path_error_m']=0. # A false driver readout cannot conceal the deviation.
    assert 'turn_corridor_exceeded' in grade(s,rows,c,r,a)['reasons']


def test_leaving_fixture_is_invalid_not_a_material_failure():
    s=MixedSpec();rows,c,r,a=evidence(s)
    rows[-1]['z_m']=-1
    assert grade(s,rows,c,r,a)==dict(status='ERROR',reasons=['left_physical_fixture'],metrics={})


@pytest.mark.parametrize('values',[{'low_mu':math.nan},{'cap_m_s':0},{'payload_kg':True},{'task':'unknown'},{'timestep_ms':8}])
def test_invalid_specs_refused(values):
    with pytest.raises(ValueError):MixedSpec(**values)


def test_turn_approach_cannot_cut_outside_the_added_corridor():
    s=MixedSpec(task='turn');rows,c,r,a=evidence(s)
    rows[150]['y_m']=.31
    assert 'approach_corridor_exceeded' in grade(s,rows,c,r,a)['reasons']


def test_repair_preserves_longitudinal_braking_under_yaw_saturation():
    import runpy
    from pathlib import Path
    allocate=runpy.run_path(str(Path(__file__).resolve().parents[2]/'projects/assessments/controllers/husky_mixed/drive_allocator.py'))['allocate']
    cfg=dict(yaw_rate_gain=.475,half_track_m=.2854,wheel_radius_m=.1651,max_wheel_speed_radps=6.)
    # Recorded failure signature: already too fast, yaw integral saturated.
    # The old common-mode clamp kept the high-grip right wheels driving forward.
    left,right,_,_=allocate(.18,1.2,.89,.02,-.6,3.3,cfg)
    assert -6 <= left <= 6 and -6 <= right <= 6
    assert right < 0
    with pytest.raises(ValueError):allocate(.18,1.2,math.nan,.02,-.6,3.3,cfg)


def test_development_modes_are_not_deliverable_controller_choices():
    for mode in ('balanced-pi','yaw-pulse','unknown'):
        with pytest.raises(ValueError):MixedSpec(steering_mode=mode)
    with pytest.raises(ValueError):MixedSpec(steering_mode='traction-route',policy='cruise')
