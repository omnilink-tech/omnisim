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

"""`omnisim doctor` -- the Tier 0 install-coherence gate.

The cross-machine determinism tiers all ask whether a run BEHAVES the same.
None can catch a stale libController, because that failure produces NO tick to
compare: the world finalises and then every controller blocks forever in
Robot() while the run still exits 0 (commit 6eea9d76). Coherence is the layer
below them, and since 2026-08-28 PLAIN `doctor` is the gate: it always prints
a VERDICT line and its exit code follows it (a broken install reported at exit
0 was read as a pass by every script, CI lane and agent branching on `$?`).
`--strict` survives only as a deprecated alias -- it must run, and it must
exit and report exactly like plain `doctor`. These tests pin both directions
of the gate in both spellings, the alias equivalence, and the one hard
constraint that outlives every contract change: every line doctor can print
is safe on a cp1252 Windows console.
"""

import io
import sys
from contextlib import redirect_stdout
from pathlib import Path

import pytest

_REPO = Path(__file__).resolve().parents[1]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))

from omnisim import doctor  # noqa: E402


def _ok_build() -> dict:
    return {
        "verdict": "ok",
        "engine": "engine-bin",
        "controller_lib": "Controller.dll",
        "engine_ipc_nonce": True,
        "controller_lib_ipc_nonce": True,
        "detail": "",
    }


def _incompatible_build() -> dict:
    return {
        "verdict": "incompatible",
        "engine": "engine-bin",
        "controller_lib": "Controller.dll",
        "engine_ipc_nonce": True,
        "controller_lib_ipc_nonce": False,
        "detail": (
            "libController predates the engine's IPC nonce: controllers will hang "
            "forever in Robot() and the sim will never step (the run still exits 0). "
            "Rebuild: make -C src/controller/c release"
        ),
    }


# --- _coherence classification (fatal vs advisory) --------------------------

def test_coherence_ok_on_clean_build():
    c = doctor._coherence(_ok_build())
    assert c["ok"] is True
    assert c["fatal"] == []


def test_coherence_fatal_on_abi_mismatch():
    c = doctor._coherence(_incompatible_build())
    assert c["ok"] is False
    assert any("ABI mismatch" in f for f in c["fatal"])


def test_coherence_fatal_on_missing_engine():
    build = {"verdict": "unknown", "engine": None, "controller_lib": None, "detail": "x"}
    c = doctor._coherence(build)
    assert c["ok"] is False
    assert any("engine binary not found" in f for f in c["fatal"])


def test_coherence_fatal_on_missing_lib_only():
    build = {"verdict": "unknown", "engine": "engine-bin", "controller_lib": None, "detail": "x"}
    c = doctor._coherence(build)
    assert c["ok"] is False
    assert any("libController not found" in f for f in c["fatal"])


def test_unreadable_binary_is_advisory_not_fatal():
    """A binary we cannot READ leaves coherence unproven -- but unproven is not
    broken, so a gate must never fail on it (false-firing is worse than none)."""
    build = {
        "verdict": "unknown", "engine": "e", "controller_lib": "l",
        "engine_ipc_nonce": None, "controller_lib_ipc_nonce": True, "detail": "x",
    }
    c = doctor._coherence(build)
    assert c["ok"] is True
    assert any("unproven" in a for a in c["advisory"])


# --- env-landmine advisory is conservative ----------------------------------

def test_webots_home_missing_is_advised(monkeypatch, tmp_path):
    monkeypatch.setenv("WEBOTS_HOME", str(tmp_path / "does-not-exist"))
    notes = doctor._env_landmines()
    assert any("WEBOTS_HOME" in n and "does not exist" in n for n in notes)


def test_webots_home_valid_is_silent(monkeypatch, tmp_path):
    """A WEBOTS_HOME that points at a real directory is NOT flagged -- the check
    only fires on a path that does not exist, so it cannot false-fire."""
    monkeypatch.setenv("WEBOTS_HOME", str(tmp_path))
    assert doctor._env_landmines() == []


def test_webots_home_unset_is_silent(monkeypatch):
    monkeypatch.delenv("WEBOTS_HOME", raising=False)
    assert doctor._env_landmines() == []


# --- end-to-end run() exit-code behaviour -----------------------------------

def _run(argv):
    buf = io.StringIO()
    with redirect_stdout(buf):
        rc = doctor.run(argv)
    return rc, buf.getvalue()


def _coherent(monkeypatch):
    """Pin every gating input to its healthy value, so the test measures the
    exit-code contract and not this clone's state: a vendored runtime older
    than its source is itself a FAIL row (3de513525), so another lane editing
    the runtime source would otherwise turn this test red."""
    monkeypatch.setattr(doctor, "_build_provenance", _ok_build)
    monkeypatch.setattr(doctor, "_env_landmines", lambda: [])
    monkeypatch.setattr(doctor, "_physics_runtime",
                        lambda _binary: {"status": "present", "source": "stub",
                                         "detail": "stubbed", "fix": None})
    monkeypatch.setattr(doctor, "_runtime_bundle_status",
                        lambda _source, _bundle: {"status": "ok", "source": "s",
                                                  "bundle": "b", "detail": "stubbed"})


def _verdict_lines(text: str) -> list[str]:
    return [ln for ln in text.splitlines() if ln.startswith("VERDICT")]


@pytest.mark.parametrize("argv", [[], ["--strict"]])
def test_doctor_exits_zero_and_says_ready_when_coherent(monkeypatch, argv):
    _coherent(monkeypatch)
    rc, out = _run(argv)
    assert rc == 0
    assert "VERDICT     READY" in out


@pytest.mark.parametrize("argv", [[], ["--strict"]])
def test_doctor_exits_nonzero_and_says_not_ready_on_abi_mismatch(monkeypatch, argv):
    """The gate is plain `doctor` now: an incoherent install exits 1 with or
    without the deprecated alias. (The old "plain doctor never gates" contract
    was retired on 2026-08-28: a broken install reported at exit 0 read as a
    pass to every caller branching on `$?`.)"""
    monkeypatch.setattr(doctor, "_build_provenance", _incompatible_build)
    rc, out = _run(argv)
    assert rc == 1
    assert "VERDICT     NOT READY" in out
    assert "ABI mismatch" in out


def test_json_exits_nonzero_on_abi_mismatch_and_records_the_alias(monkeypatch):
    import json
    monkeypatch.setattr(doctor, "_build_provenance", _incompatible_build)
    rc, out = _run(["--json"])
    payload = json.loads(out)
    assert rc == 1
    assert payload["strict"] is False
    assert payload["coherence"]["ok"] is False
    rc, out = _run(["--strict", "--json"])
    payload = json.loads(out)
    assert rc == 1
    assert payload["strict"] is True  # still recorded, for callers that read it
    assert payload["coherence"]["ok"] is False


def test_strict_is_an_alias_that_exits_like_plain_doctor(monkeypatch):
    """`--strict` is kept for .githooks/pre-push and existing CI. It must keep
    parsing, and it must change nothing: the same exit code and the same
    VERDICT line as plain `doctor`, on a coherent install AND on a broken one."""
    for build, expected_rc in ((_ok_build, 0), (_incompatible_build, 1)):
        _coherent(monkeypatch)
        monkeypatch.setattr(doctor, "_build_provenance", build)
        rc_plain, out_plain = _run([])
        rc_alias, out_alias = _run(["--strict"])
        assert rc_plain == rc_alias == expected_rc
        assert len(_verdict_lines(out_plain)) == 1
        assert _verdict_lines(out_plain) == _verdict_lines(out_alias)


# --- cp1252 safety (Windows console must never UnicodeEncodeError) ----------

def test_all_output_paths_are_cp1252_safe(monkeypatch):
    for build in (_ok_build, _incompatible_build):
        monkeypatch.setattr(doctor, "_build_provenance", build)
        for argv in ([], ["--strict"], ["--strict", "--json"]):
            _, out = _run(argv)
            out.encode("cp1252")  # raises UnicodeEncodeError on any non-cp1252 char


def test_missing_binary_real_path_is_fatal_and_cp1252_safe(monkeypatch):
    """Exercise the REAL _build_provenance detail strings (not a synthetic dict):
    with no engine resolvable, a fresh clone hits `build ? <detail>` and, under
    --strict, a fatal verdict. This is the exact first-turn/fresh-clone path, so
    its output MUST be cp1252-safe (the detail strings were fixed from em-dash to
    ASCII '--' for this reason)."""
    monkeypatch.setattr(doctor, "resolve_omnisim_binary", lambda: None)
    monkeypatch.setattr(doctor, "_env_landmines", lambda: [])
    rc_plain, out_plain = _run([])
    assert rc_plain == 1  # no binary is a blocking problem; plain doctor gates on it
    assert "engine binary not found" in out_plain
    out_plain.encode("cp1252")
    rc_strict, out_strict = _run(["--strict"])
    assert rc_strict == 1
    assert "engine binary not found" in out_strict
    out_strict.encode("cp1252")


# --- the real thing: synthesize a stale lib from the actual built binary -----

def test_strict_fires_on_synthesized_stale_lib(tmp_path, monkeypatch):
    """Prove the gate on REAL binaries, not a hand-built dict: take this clone's
    actual Controller.dll, strip the IPC-nonce token (as an older lib lacks it),
    point _controller_lib at the copy, and confirm _build_provenance reads it as
    incompatible and --strict exits non-zero. This is the exact technique used to
    verify 6eea9d76."""
    real = doctor._controller_lib()
    engine = doctor.resolve_omnisim_binary()
    if real is None or engine is None:
        pytest.skip("no built engine + libController in this clone")
    data = real.read_bytes()
    if doctor._IPC_NONCE_TOKEN not in data:
        pytest.skip("this clone's libController predates the nonce -- nothing to strip")
    stale = tmp_path / real.name
    # Same replacement used to verify the original gate; length change is
    # irrelevant -- the probe only checks token PRESENCE, never executes the dll.
    stale.write_bytes(data.replace(doctor._IPC_NONCE_TOKEN, b"OMNISIM_IPC_LEGACYX"))
    monkeypatch.setattr(doctor, "_controller_lib", lambda: stale)

    build = doctor._build_provenance()
    assert build["verdict"] == "incompatible"
    assert build["engine_ipc_nonce"] is True
    assert build["controller_lib_ipc_nonce"] is False

    rc, out = _run(["--strict"])
    assert rc != 0
    assert "FAIL" in out
    out.encode("cp1252")
