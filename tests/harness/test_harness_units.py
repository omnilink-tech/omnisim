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



# --- a screenshot that is not a picture (2026-08-03) ------------------------


class _StaleState:
    """Minimal stand-in carrying just what note_render touches."""

    def __init__(self):
        import threading
        self.lock = threading.Lock()


def _note(state, digest, sim_ms):
    import sys as _s
    from pathlib import Path as _P
    _s.path.insert(0, str(_P(__file__).resolve().parents[2] / "scripts" / "harness"))
    import omnisim_harness as oh
    return oh.HarnessState.note_render(state, digest, sim_ms)


def test_identical_frames_while_the_clock_advances_are_flagged_stale():
    """MEASURED: four screenshots of a falling cube came back byte-identical
    -- same sha256, same 6,342,147 bytes -- while its pose changed at every
    step. The agent that hit this nearly shipped them as proof of a grasp."""
    st = _StaleState()
    assert _note(st, "aaaa", 24.0).get("warning") is None      # first frame
    out = _note(st, "aaaa", 376.0)
    assert out["warning"] and "STALE RENDER" in out["warning"]
    assert "24.0" in out["warning"] and "376.0" in out["warning"]


def test_identical_frames_with_a_STOPPED_clock_are_not_flagged():
    """The falsifier. A paused simulation SHOULD render the same picture, and
    calling that a defect would cry wolf on every legitimate repeat shot."""
    st = _StaleState()
    _note(st, "bbbb", 100.0)
    assert _note(st, "bbbb", 100.0).get("warning") is None


def test_a_changing_frame_is_never_flagged():
    st = _StaleState()
    _note(st, "cccc", 10.0)
    out = _note(st, "dddd", 20.0)
    assert out.get("warning") is None
    assert out["identical_to_previous"] is False


def test_an_unknown_clock_is_not_treated_as_evidence():
    """No sim time means we cannot say the world moved, so we must not claim
    the render is stale -- absence of a clock is not proof of anything."""
    st = _StaleState()
    _note(st, "eeee", None)
    assert _note(st, "eeee", None).get("warning") is None
