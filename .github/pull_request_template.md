**Description**
Describe the bug fix, enhancement or new feature you are proposing.

**Related Issues**
This pull-request fixes issue #

**Tasks**
Add the list of tasks of this PR.
  - [ ] Update the documentation (if needed)
  - [ ] Task 1
  - [ ] Task 2
  - [ ] ...

**Screenshots**
If this pull-request includes any interesting visible result, add one or more screenshots.

**Additional context**
Add any other context about the pull-request here.

---

**Provenance and licensing** — please confirm before merge:

  - [ ] **Every commit is signed off** (`git commit -s`, or `git rebase --signoff <base>` to
        fix a branch). That is the Developer Certificate of Origin: it certifies you wrote
        this or have the right to submit it. There is no CLA. See
        [CONTRIBUTING.md](../CONTRIBUTING.md#licensing-of-your-contribution).
  - [ ] **New source files carry the Apache-2.0 header** used throughout the tree.
  - [ ] **Any new binary asset** — mesh, texture, font, dataset, model weights — comes with
        a licence or provenance record, and I have checked that **the licensor is the design
        owner**. A vendor's permissively-licensed package does *not* cover another
        manufacturer's CAD sitting inside it; that single question is what forced several
        removals from this tree. See
        [docs/developer/asset-provenance.md](../docs/developer/asset-provenance.md).
  - [ ] **No brand artwork** from `resources/branding/` is copied or altered — it is
        reserved by copyright as well as trademark
        ([resources/branding/LICENSE](../resources/branding/LICENSE)).
  - [ ] The three licence gates pass locally:
        `python -m pytest tests/sources/test_license.py tests/sources/test_asset_provenance.py tests/sources/test_licence_pointers.py`
        (⚠️ do **not** run a bare `pytest` at the repo root — it rewrites every world file).

If any box does not apply, say so rather than ticking it. An honest "N/A, no new assets"
is more useful to a reviewer than a tick.
