"""Print the tool calls in a cell whose input matches a regex, with results."""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from defect_scan import load_pairs  # noqa: E402

cell, rx = sys.argv[1], sys.argv[2]
nres = int(sys.argv[3]) if len(sys.argv) > 3 else 400
pairs, _t0, _src = load_pairs(cell)
for (i, ts, tool, inp, res, err) in pairs:
    if re.search(rx, "%s %s" % (tool or "", inp), re.I):
        print("=" * 78)
        print("[#%d %s %s err=%s]" % (i, ts, tool, err))
        print("IN : %s" % inp[:600].replace("\\n", "\n"))
        print("OUT: %s" % res[:nres].replace("\\n", "\n"))
