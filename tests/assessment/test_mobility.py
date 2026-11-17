"""Geometry, supported coverage, physical adoption and slope-frame oracles."""
from dataclasses import replace
import json
import math
import xml.etree.ElementTree as ET

import pytest

from omnisim.assessment.schema import BrakingSpec
from omnisim.assessment.registry import profiles, get_profile, inventory
from omnisim.assessment.scenarios import make_world
from omnisim.assessment.oracles import grade
from omnisim.assessment.runner import verify_fixture
from omnisim.assessment.campaign import boundaries
from .test_braking import analytic_case


def slope_trace():
    _, rows, completion, physics = analytic_case()
    angle=math.radians(20)
    distance=0.
    for r in rows:
        v=r['vx_m_s'] if r['t_s'] >= 1. else 0.
        distance+=v*.016
        s=distance
        r.update(x_m=s*math.cos(angle)+.132*math.sin(angle),
                 z_m=-s*math.sin(angle)+.132*math.cos(angle),
                 vx_m_s=v*math.cos(angle),vz_m_s=-v*math.sin(angle),
                 along_m=s,forward_m_s=v,normal_height_m=.132,ground_tilt_rad=0,
                 wheel_position_rad=[s/.1651]*4)
    completion.update(wheel_count=4,terminal='observation_complete')
    return BrakingSpec(schema_version=2,slope_deg=20),rows,completion,physics


def test_slope_projection_does_not_mistake_downhill_height_for_falling():
    spec,rows,completion,physics=slope_trace()
    assert rows[-1]['z_m']<0
    assert grade(spec,rows,completion,physics)['status']=='PASS'


def test_real_tip_is_a_failure_not_an_invalid_flat_world():
    spec,rows,completion,physics=slope_trace()
    rows[-1]['ground_tilt_rad']=.9
    v=grade(spec,rows,completion,physics)
    assert v['status']=='FAIL'
    assert 'tipped_relative_to_ground' in v['reasons']


def test_approach_failure_is_not_called_a_braking_failure():
    spec,rows,completion,physics=slope_trace()
    for r in rows: r['stop_requested']=False
    completion['terminal']='approach_timeout'
    v=grade(spec,rows,completion,physics)
    assert v['status']=='FAIL'
    assert not v['metrics']['braking_qualified']
    assert v['reasons']==['approach_speed_not_reached']


@pytest.mark.parametrize('robot',list(profiles()))
def test_every_physical_mobile_profile_can_build_its_own_model(robot,tmp_path):
    p=get_profile(robot)
    speed=min(.6,.7*p.config['wheel_radius_m']*p.config['max_wheel_speed_radps'])
    spec=BrakingSpec(schema_version=2,robot=robot,target_speed_m_s=speed,
                     slope_deg=10,ground_mu=.2,payload_kg=2)
    world,dependencies,fixture=make_world(spec,tmp_path)
    assert all(p.is_file() for p in dependencies)
    assert fixture['wheel_count'] in (2,4)
    model=ET.parse(tmp_path/'model.urdf').getroot()
    assert model.find("./link[@name='assessment_payload']/inertial/mass").get('value')=='2'
    assert model.find("./joint[@name='assessment_payload_mount']").get('type')=='fixed'
    assert all(float(x.get('value'))==.2 for x in model.findall('./gazebo/mu1'))
    assert 'newtonGroundMu 0.2' in world
    assert 'rotation 0 1 0' in world


def test_model_inventory_does_not_claim_all_urdfs_have_braking_adapters():
    rows=inventory()
    assert len({r['robot'] for r in rows if r['supported_tasks']})==7
    assert any(r['state']=='asset_only' and 'unitree/go2' in r['model'] for r in rows)
    with pytest.raises(ValueError,match='UNSUPPORTED'): get_profile('go2')
    with pytest.raises(ValueError,match='UNSUPPORTED'): get_profile('omnitug500')


def test_adoption_rejects_hidden_tyre_friction_override(tmp_path):
    fixture={'expected_total_mass_kg':66.682,'ground_mu':.1,'slope_deg':0}
    (tmp_path/'compiled-model.txt').write_text(
        'body 0 world mass=0\nbody 1 robot mass=66.682\n'
        'geom 0 floor body=0 fric=[0.1, 0, 0] quat=[1, 0, 0, 0]\n'
        'geom 1 tyre body=1 fric=[1.0, 0, 0] quat=[1, 0, 0, 0]\n')
    assert not verify_fixture(tmp_path,fixture)['valid']


def test_brackets_keep_acquisition_failures_distinct():
    spec=BrakingSpec(schema_version=2,slope_deg=5).to_dict()
    low={'kind':'grid','case':'low','spec':dict(spec,ground_mu=.1),'status':'FAIL',
         'metrics':{'braking_qualified':False},'reasons':['approach_speed_not_reached']}
    high={'kind':'grid','case':'high','spec':dict(spec,ground_mu=.4),'status':'PASS',
          'metrics':{'braking_qualified':True},'reasons':[]}
    b=boundaries([low,high])[0]
    assert not b['both_braking_qualified']
    assert b['failure_reasons']==['approach_speed_not_reached']


@pytest.mark.parametrize('change',[{'ground_mu':0},{'slope_deg':math.nan},
    {'payload_kg':-1},{'timestep_ms':16.0}])
def test_invalid_physical_specs_are_refused(change):
    with pytest.raises(ValueError): replace(BrakingSpec(schema_version=2),**change)
