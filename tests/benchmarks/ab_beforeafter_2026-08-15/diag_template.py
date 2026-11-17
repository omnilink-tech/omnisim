import json
import os
import sys

sys.path.insert(0, "tests/benchmarks/agentbench/cc_lane")
import stage_workspaces as st  # noqa: E402

root = sys.argv[1]
man = json.load(open(os.path.join(root, "templates", "omnisim.manifest.json"),
                     encoding="utf-8"))
files = st.git_tracked_files(st.REPO)
copied, excluded = st.select_omnisim_files(files)
print("REPO                :", st.REPO)
print("manifest filelist   :", man.get("filelist_sha256"))
print("now (pre-missing)   :", st.filelist_sha256(copied))
missing = [r for r in copied if not os.path.exists(os.path.join(st.REPO, r))]
kept = [r for r in copied if r not in set(missing)]
print("now (post-missing)  :", st.filelist_sha256(kept))
print("tracked-but-absent  :", len(missing))
for m in missing[:15]:
    print("   ", m)
print("manifest included   :", man.get("included_file_count"))
on_disk = sum(1 for p in __import__("pathlib").Path(
    os.path.join(root, "templates", "omnisim")).rglob("*") if p.is_file())
print("template on disk    :", on_disk)
