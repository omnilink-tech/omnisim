"""Task support, frozen robot scope and package identity stay separate."""
import json
from pathlib import Path

import pytest

from omnisim.assessment import delivery
from omnisim.assessment.package_profiles import DEFINITIONS, get_package_profile, render_document
from omnisim.assessment.registry import ROOT, profiles
from omnisim.assessment.runner import save


def test_each_supported_wheel_adapter_has_a_package_definition():
    assert set(DEFINITIONS) == set(profiles())


@pytest.mark.parametrize("robot",list(DEFINITIONS))
def test_robot_profile_respects_its_command_envelope_and_asset_notices(robot):
    p=get_package_profile(robot)
    adapter=profiles()[robot]
    assert p.speed <= adapter.config["wheel_radius_m"]*adapter.config["max_wheel_speed_radps"]
    assert p.spec().robot == robot and p.spec().brake_delay_s == 0
    assert p.stop_distance == pytest.approx(p.speed**2/(2*.72))
    assert all((ROOT/n).is_file() for n in p.notices)
    assert p.to_dict()["total_episodes"] == 64
    assert "Definition only" in p.to_dict()["qualification"]


def test_husky_profile_preserves_the_previous_physical_scope():
    p=get_package_profile("husky")
    assert p.speed == .6 and p.stop_distance == .25 and p.height == .35
    assert p.payloads == (0,10,20)


def test_jackal_does_not_inherit_husky_speed_or_loads():
    p=get_package_profile("jackal")
    assert p.speed == pytest.approx(.4116)
    assert p.stop_distance == pytest.approx(.117649)
    assert p.height == pytest.approx(.2254)
    assert p.payloads == (0,2,5)


@pytest.mark.parametrize("robot",["g1","omnitug","unknown"])
def test_asset_or_kinematic_robot_is_not_a_supported_package(robot):
    with pytest.raises(ValueError,match="UNSUPPORTED"):
        get_package_profile(robot)


@pytest.mark.parametrize("robot",list(DEFINITIONS))
def test_all_robot_customer_documents_render_without_husky_scope_leaks(robot):
    p=get_package_profile(robot)
    for name in ("SCOPE.md","QUICKSTART.md","CUSTOMER-ACCEPTANCE.md","RUNTIME-NOTICES.md"):
        template=(ROOT/"projects/assessments/packages/templates"/name).read_text(encoding="utf-8")
        rendered=render_document(template,p)
        assert "{{" not in rendered
        if robot != "husky": assert "Husky" not in rendered and "husky_description" not in rendered
        if name == "SCOPE.md":
            assert p.sku in rendered and p.label in rendered and f"{p.speed:.6g}" in rendered
            assert f"{p.stop_distance:.6g}" in rendered


def test_cross_robot_and_mixed_campaigns_are_rejected_before_copying(tmp_path):
    save(tmp_path/"campaign.json",{"complete":True,"all_evidence_valid":True,
                                  "axes":{"robots":["jackal"]}})
    with pytest.raises(ValueError,match="matching robot"):
        delivery.validated_campaign(tmp_path,robot="husky")
    state=json.loads((tmp_path/"campaign.json").read_text())
    state["axes"]["robots"]=["husky","jackal"]
    save(tmp_path/"campaign.json",state)
    with pytest.raises(ValueError,match="mixed-robot"):
        delivery.validated_campaign(tmp_path)


def test_manifest_cannot_claim_a_different_robot_sku(tmp_path):
    save(tmp_path/"delivery-manifest.json",{"sku":"HUSKY-BRAKING-100","robot":"jackal","files_sha256":{}})
    with pytest.raises(ValueError,match="SKU"):
        delivery.verify_delivery(tmp_path)


def test_report_uses_jackal_data_instead_of_husky_examples(tmp_path):
    p=get_package_profile("jackal")
    cases=[]
    for name,mu,status,distance in (("case-a",.1,"FAIL",.6502),("case-b",.2,"PASS",.07839)):
        cases.append({"case":name,"kind":"grid","spec":p.spec(slope_deg=5,ground_mu=mu,payload_kg=2).to_dict(),
                      "status":status,"reasons":[],"metrics":{"max_forward_excursion_m":distance,"braking_qualified":True}})
    state={"axes":{"robots":["jackal"]},"cases":cases,"confirmations":[],"boundaries":[
        {"both_braking_qualified":True,"low_case":"case-a","high_case":"case-b","slope_deg":5,"payload_kg":2}]}
    delivery.write_customer_report(tmp_path,state)
    text=(tmp_path/"REPORT.html").read_text(encoding="utf-8")
    assert "Jackal braking assessment" in text and "Husky" not in text
    assert "0.4116" in text and "0.117649" in text and "0.6502" in text


def test_acceptance_selects_the_actually_confirmed_pair():
    b1={"both_braking_qualified":True,"low_case":"a","high_case":"b"}
    b2={"both_braking_qualified":True,"low_case":"c","high_case":"d"}
    s={"boundaries":[b1,b2],"confirmations":[{"original":"c"},{"original":"d"}]}
    assert delivery.acceptance_pair(s)==b2
