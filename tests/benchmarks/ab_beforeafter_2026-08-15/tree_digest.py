import hashlib
import os
import sys


def tree_digest(root, skip=("__pycache__",)):
    h = hashlib.sha256()
    n = 0
    for dp, dn, fn in os.walk(root):
        dn[:] = [d for d in dn if d not in skip]
        for f in sorted(fn):
            p = os.path.join(dp, f)
            rel = os.path.relpath(p, root).replace(os.sep, "/")
            h.update(rel.encode("utf-8"))
            with open(p, "rb") as fh:
                h.update(fh.read())
            n += 1
    return h.hexdigest(), n


if __name__ == "__main__":
    for root in sys.argv[1:]:
        d, n = tree_digest(root)
        print("%s  %6d files  %s" % (d, n, root))
