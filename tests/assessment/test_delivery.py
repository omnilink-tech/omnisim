"""Customer delivery rejects changed evidence and failed reproduction checks."""
import copy
import json
from pathlib import Path

import pytest

from omnisim.assessment import delivery
from omnisim.assessment.runner import save


def test_delivery_inventory_detects_changed_and_missing_files(tmp_path, monkeypatch):
    (tmp_path/"source.py").write_text("original")
    save(tmp_path/"delivery-manifest.json",{"sku":"HUSKY-BRAKING-100","robot":"husky","files_sha256":delivery.inventory(tmp_path)})
    monkeypatch.setattr(delivery,"validated_campaign",lambda p,**kw: ({"axes":{"robots":["husky"]},"cases":[{}]*64},{}))
    assert delivery.verify_delivery(tmp_path)["episodes_regraded"] == 64
    (tmp_path/"source.py").write_text("changed")
    with pytest.raises(ValueError,match="modified"):
        delivery.verify_delivery(tmp_path)
    (tmp_path/"source.py").unlink()
    with pytest.raises(ValueError,match="Missing"):
        delivery.verify_delivery(tmp_path)


@pytest.mark.parametrize("path",["../outside", "C:/outside", "/outside", "a\\b"])
def test_manifest_cannot_read_outside_delivery(tmp_path,path):
    save(tmp_path/"delivery-manifest.json",{"sku":"HUSKY-BRAKING-100","robot":"husky","files_sha256":{path:"0"*64}})
    with pytest.raises(ValueError,match="Invalid manifest path"):
        delivery.verify_delivery(tmp_path)


def acceptance_fixture(tmp_path,monkeypatch):
    monkeypatch.setattr(delivery,"ROOT",tmp_path)
    monkeypatch.setattr(delivery,"MAX_ROOT_CHARS",10000)
    monkeypatch.setattr(delivery,"verify_delivery",lambda p:{"verified":True})
    evidence=tmp_path/"evidence"
    evidence.mkdir()
    save(evidence/"campaign.json",{"cases":[{"case":"case-001"},{"case":"case-002"}],
         "boundaries":[{"both_braking_qualified":True,
         "low_status":"FAIL","high_status":"PASS","low_case":"case-001","high_case":"case-002"}]})
    for name,status,distance in (("case-001","FAIL",1.),("case-002","PASS",.1)):
        (evidence/name).mkdir()
        save(evidence/name/"verdict.json",{"status":status,"metrics":{"max_forward_excursion_m":distance}})
    def rerun(source,dest):
        dest.mkdir()
        v=json.loads((source/"verdict.json").read_text())
        save(dest/"verdict.json",v)
        return v
    monkeypatch.setattr(delivery,"rerun",rerun)
    monkeypatch.setattr(delivery,"regrade",lambda p:json.loads((p/"verdict.json").read_text()))
    return rerun


def test_known_robot_failure_is_successful_customer_acceptance(tmp_path,monkeypatch):
    acceptance_fixture(tmp_path,monkeypatch)
    out=tmp_path/"runs"
    assert delivery.customer_main(["accept","--out",str(out)]) == 0
    result=json.loads((out/"acceptance.json").read_text())
    assert result["verified"] and len(result["results"]) == 4
    assert [r["observed"] for r in result["results"]] == ["FAIL","FAIL","PASS","PASS"]


@pytest.mark.parametrize("case,expected",[("case-001",1),("case-002",0),("unknown",2)])
def test_customer_single_case_exit_codes(tmp_path,monkeypatch,case,expected):
    acceptance_fixture(tmp_path,monkeypatch)
    assert delivery.customer_main(["rerun",case,"--out",str(tmp_path/"new-run")]) == expected


def test_customer_cannot_write_runs_into_delivered_evidence(tmp_path,monkeypatch):
    acceptance_fixture(tmp_path,monkeypatch)
    assert delivery.customer_main(["accept","--out",str(tmp_path/"evidence/new-run")]) == 2


@pytest.mark.parametrize("defect",["outcome", "distance", "invalid"])
def test_acceptance_rejects_reproduction_mismatch(tmp_path,monkeypatch,defect):
    original=acceptance_fixture(tmp_path,monkeypatch)
    def changed(source,dest):
        v=original(source,dest)
        if defect == "outcome": v["status"]="PASS"
        if defect == "distance": v["metrics"]["max_forward_excursion_m"]+=.006
        if defect == "invalid": v["status"]="INCONCLUSIVE"
        save(dest/"verdict.json",v)
        return v
    monkeypatch.setattr(delivery,"rerun",changed)
    out=tmp_path/"runs"
    assert delivery.customer_main(["accept","--out",str(out)]) == 2
    assert not json.loads((out/"acceptance.json").read_text())["verified"]


def test_scope_rejects_smaller_grid_before_packaging(tmp_path):
    save(tmp_path/"campaign.json",{"complete":True,"all_evidence_valid":True,
         "axes":{"robots":["husky"],"slopes_deg":[0],"ground_mu":[.8],"payload_kg":[0]}})
    with pytest.raises(ValueError,match="axes"):
        delivery.validated_campaign(tmp_path)


def test_native_dependencies_include_embedded_python():
    engine=delivery.ROOT/"msys64/mingw64/bin/omnisim-bin.exe"
    if not engine.exists(): pytest.skip("Windows engine not installed")
    assert "python312.dll" in delivery.pe_imports(engine)


def test_windows_capsule_contains_activation_and_python_path_files():
    engine=delivery.ROOT/"msys64/mingw64/bin/omnisim-bin.exe"
    if not engine.exists(): pytest.skip("Windows engine not installed")
    selected=delivery.runtime_files()
    for rel in ("msys64/mingw64/bin/OmniLink.OmniSim.Mingw64.Libraries.manifest",
                "msys64/mingw64/bin/omnisim-bin.exe.config",
                "msys64/mingw64/bin/python312._pth"):
        assert delivery.ROOT/rel in selected


def test_cuda_capsule_has_loader_libraries_and_license(tmp_path,monkeypatch):
    toolkit=tmp_path/"cuda"
    bindir=toolkit/"bin"
    bindir.mkdir(parents=True)
    for name in ("cudart64_12.dll","nvrtc64_120_0.dll","nvrtc-builtins64_121.dll"):
        (bindir/name).write_bytes(b"fixture")
    (toolkit/"EULA.txt").write_text("fixture license")
    (toolkit/"version.json").write_text('{}')
    monkeypatch.setenv("CUDA_PATH",str(toolkit))
    monkeypatch.setattr(delivery,"pe_imports",lambda p:{"cudart64_12.dll","nvrtc64_120_0.dll"})
    selected=delivery.cuda_redistributables()
    assert "licenses/CUDA-EULA.txt" in selected
    assert any("cudart64_" in p for p in selected)
    assert any("nvrtc64_" in p for p in selected)
    assert any("nvrtc-builtins64_" in p for p in selected)
    assert not any(Path(p).name.lower() == "nvcuda.dll" for p in selected)


def test_runtime_fingerprint_normalizes_binary_path_aliases(tmp_path,monkeypatch):
    from omnisim.assessment import runner
    monkeypatch.setattr(runner,"REPO_ROOT",tmp_path)
    monkeypatch.chdir(tmp_path)
    for rel in ("lib/controller/Controller.dll","bin/engine.exe",
                "bin/newton-runtime/site-packages/omnisim_newton_runtime.py"):
        p=tmp_path/rel;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(rel.encode())
    absolute=runner.runtime_hashes(tmp_path/"bin/engine.exe")
    assert len(absolute) == 2
    assert runner.runtime_hashes(Path("bin/engine.exe")) == absolute


def test_customer_rejects_unsupported_deep_install_before_launch(monkeypatch):
    monkeypatch.setattr(delivery,"ROOT",Path("C:/"+"nested/"*25))
    monkeypatch.setattr(delivery,"verify_delivery",lambda p:pytest.fail("must reject path before verification"))
    assert delivery.customer_main(["verify"]) == 2
