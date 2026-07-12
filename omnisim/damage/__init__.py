"""omnisim.damage — damage-system test harness and regression suite.

`omnisim.damage.headless_test` is the iterative dev harness — spawn
headless OmniSim, connect to the harness_supervisor wire protocol on
:6790, run named scenarios, report stats. Wired up as
`python -m omnisim damage`.

`omnisim.damage.regression_suite` is the numerical regression suite —
deterministic scenarios with stat-range assertions. Wired up as
`python -m omnisim damage-regression`.
"""
