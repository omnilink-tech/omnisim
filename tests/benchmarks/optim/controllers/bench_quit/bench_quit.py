"""Supervisor controller used by optim_bench.py worlds.

Steps the simulation a fixed number of times, then calls
`simulationQuit(0)` so the simulator exits cleanly and writes the
performance log.

Step count is read from `--steps N` in `controllerArgs`. Default 1000.
"""

from __future__ import annotations

import sys

from controller import Supervisor


def parse_steps(argv: list[str], default: int = 1000) -> int:
    for i, arg in enumerate(argv):
        if arg == "--steps" and i + 1 < len(argv):
            try:
                return int(argv[i + 1])
            except ValueError:
                pass
        if arg.startswith("--steps="):
            try:
                return int(arg.split("=", 1)[1])
            except ValueError:
                pass
    return default


def main() -> None:
    sup = Supervisor()
    time_step = int(sup.getBasicTimeStep())
    target = parse_steps(sys.argv)

    steps = 0
    while sup.step(time_step) != -1:
        steps += 1
        if steps >= target:
            sup.simulationQuit(0)
            break


if __name__ == "__main__":
    main()
