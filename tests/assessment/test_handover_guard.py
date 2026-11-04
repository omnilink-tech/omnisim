"""Guard regressions from the measured 15-second handover failure."""
import math
import pytest
from projects.assessments.controllers.legged_go2.handover_guard import FEET,support_ready


def test_rejects_the_measured_failure_handover_orientation():
    # Old case-017, scene t=16.496 s: all four calf/foot links contacted.
    tilt=.20470966636223936
    assert support_ready(FEET,tilt,.25)
    assert not support_ready(FEET,tilt,.12)


def test_level_body_still_requires_every_foot():
    assert support_ready(FEET,.05,.12)
    assert not support_ready(FEET-{'FR_calf'},.05,.12)


@pytest.mark.parametrize('tilt',[math.nan,math.inf,-.1,.12])
def test_invalid_or_boundary_orientation_fails_closed(tilt):
    assert not support_ready(FEET,tilt,.12)
