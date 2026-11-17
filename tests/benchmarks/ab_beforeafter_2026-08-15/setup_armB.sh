#!/usr/bin/env bash
# Bring arm B to the current product state and freeze its engine.
#
# Arm B is "the product at HEAD with the newly rebuilt engine". HEAD moved
# while arm A ran (the engine agent landed the /sim/reset fix), so this is run
# immediately before arm B, and it RECORDS what it pinned rather than assuming.
set -eu
cd /o/omnisim
HEAD_SHA=$(git rev-parse HEAD)
DIRTY=$(git status --short src/omnisim/ | wc -l)
echo "real repo HEAD      : $HEAD_SHA"
echo "src/omnisim dirty   : $DIRTY file(s)  <-- recorded, not assumed clean"
git log --oneline -3 -- src/omnisim/

cd /o/omnisim-ab/armB
MSYS_NO_PATHCONV=1 git checkout --detach "$HEAD_SHA" 2>&1 | tail -2
MSYS_NO_PATHCONV=1 git sparse-checkout set --no-cone '/*' '!/projects/' >/dev/null
MSYS_NO_PATHCONV=1 git sparse-checkout reapply >/dev/null
echo "armB now at         : $(git rev-parse --short HEAD)"

# instrument identity: HEAD's cc_lane on BOTH arms, always
rm -rf /o/omnisim-ab/armB/tests/benchmarks/agentbench
cp -a /o/omnisim/tests/benchmarks/agentbench \
      /o/omnisim-ab/armB/tests/benchmarks/agentbench
find /o/omnisim-ab/armB/tests/benchmarks/agentbench -name __pycache__ -type d \
     -exec rm -rf {} + 2>/dev/null || true

# projects/ + msys64/ + lib/ are recreated by freeze_engine (msys64, lib) and
# must exist as junctions (projects)
if [ ! -e /o/omnisim-ab/armB/projects ]; then
  cmd //c mklink //J "O:\\omnisim-ab\\armB\\projects" "O:\\omnisim\\projects" >/dev/null
fi

cp /o/omnisim/msys64/mingw64/bin/omnisim-bin.exe \
   /o/omnisim/_scratch/ab_beforeafter/omnisim-bin-armB.exe
python /o/omnisim/_scratch/ab_beforeafter/freeze_engine.py \
   "O:\\omnisim-ab\\armB" "O:\\omnisim\\_scratch\\ab_beforeafter\\omnisim-bin-armB.exe"

echo
echo "instrument digests (must match):"
python /o/omnisim/_scratch/ab_beforeafter/tree_digest.py \
   /o/omnisim-ab/armA/tests/benchmarks/agentbench/cc_lane \
   /o/omnisim-ab/armB/tests/benchmarks/agentbench/cc_lane
echo
echo "arm engines:"
sha256sum /o/omnisim-ab/armA/msys64/mingw64/bin/omnisim-bin.exe \
          /o/omnisim-ab/armB/msys64/mingw64/bin/omnisim-bin.exe
