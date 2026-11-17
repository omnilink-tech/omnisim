#!/usr/bin/env python

# Copyright 1996-2024 Cyberbotics Ltd.
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
#
# Modifications copyright 2026 OmniLink, licensed under the Apache License, Version 2.0.

"""Test project_structure.

WHAT THIS FILE GUARDS, AND WHAT IT DELIBERATELY NO LONGER GUARDS
===============================================================

This gate was red for a long time, and a permanently-red gate catches nothing --
it is read as "that one always fails" and stops being evidence of anything. It
was red because two of its assertions encoded rules this project does not follow
and has not followed for a long time. Both are now gone, and the reasoning is
recorded here rather than in a commit message so the next person to ask "why is
there no perspective check?" finds the answer next to the code.

Every count below was measured against `projects/` at the time of writing:
366 worlds, 69 tracked perspective files.

DELETED (1/2): "every world has a perspective file".
    A perspective file is a GUI SAVE ARTIFACT. It is written only when a human
    opens a world in the desktop GUI and saves it, and it holds that human's
    viewpoint, dock layout and open editor tabs. The overwhelming majority of
    this repo's worlds are authored programmatically -- by omniworld recipes, by
    generators like `scripts/dev/gen_city_traffic.py`, by agents driving the
    harness -- and are never opened in a GUI at all.

    This is not merely an empirical drift; it is settled PROJECT POLICY, and the
    policy is written down where it is enforced: `.gitignore:204` carries
    `.*.wbproj`. The repo has explicitly decided perspectives are local
    artifacts that do not get committed. MEASURED: of 428 perspective files on
    this working copy, 359 are gitignored -- only 69 are tracked, all of them
    grandfathered in from upstream Webots before that ignore rule existed. So
    303 of the 366 worlds have no tracked perspective, and asserting otherwise
    asked the tree to contradict its own `.gitignore`.

    The old assertion also had a defect worth naming, because it is easy to
    reintroduce: it enumerated worlds from git but resolved the pairing with
    `os.path.isfile` against the FILESYSTEM. Gitignored local residue therefore
    satisfied it. The test's result depended on which worlds the developer
    happened to have opened in the GUI on that machine -- it would have reported
    119 missing here and 303 missing in a clean clone, from identical committed
    content. Everything below pairs git-set against git-set for that reason.

    What is no longer guarded, stated plainly: nothing now notices if a world
    ships without a saved viewpoint. That was never a defect, and the
    viewpoint-quality question has a real owner elsewhere -- worlds are supposed
    to open looking at their subject, which is `scripts/dev/set_viewpoint.py`
    and docs/developer/viewpoint-convention.md, a property of the world's own
    `Viewpoint` node and not of a hidden sibling file.

DELETED (2/2): "no 'worlds' directory may exist above another one".
    MEASURED: 242 of 366 worlds violated this. It was not measuring what its
    message claimed. For a world at `.../demos/worlds/showcase/w.omniworld` it
    walked up past the world's OWN `worlds/` directory and then flagged that
    directory as an offending ancestor -- so the categorised-subdirectory layout
    tripped it, not any actual nesting. Genuinely doubled paths (two `worlds`
    components in one path) number ZERO, so the rule it was reaching for is not
    the rule it implemented, and the thing it implemented is false.

    Large workstreams also organise deliberately: `projects/policies/worlds/`
    and `projects/policies/research/worlds/` are both intentional. That is
    theirs to decide.

KEPT, in a form that is actually true (all four measured at 0 violations, so
each is green today and each can be driven red by a real mistake):
    * a world lives somewhere under a `worlds/` directory        (0/366 bad)
    * a world is not buried more than one level below it         (0/366 bad)
    * a tracked perspective is a HIDDEN sibling                   (0/69 bad)
    * a tracked perspective is not an ORPHAN                      (6/69 bad,
      allowlisted below as pre-existing debt -- see the comment there)

The orphan direction is the half of the old pairing check that was always the
valuable one. A world with no perspective is normal. A perspective with no world
is litter: either a world was renamed or moved and its sibling was left behind,
or a world was deleted and its sibling was not. That is a mistake a human makes,
which is exactly the property worth a gate.
"""
import unittest
import os
import posixpath
import subprocess

# DUAL-READ / SINGLE-WRITE, mirroring src/omnisim/core/OmWorldFileFormat.hpp and
# src/omnisim/core/OmPerspectiveFileFormat.hpp: OmniSim WRITES only `.omniworld` /
# `.omniperspective` and still READS the legacy `.wbt` / `.wbproj` forever. A world
# therefore pairs with a perspective sibling under EITHER extension, and asserting
# specifically on `.wbproj` goes structurally wrong the first time a world is saved.
WORLD_EXTENSIONS = ('.omniworld', '.wbt')
PERSPECTIVE_EXTENSIONS = ('.omniperspective', '.wbproj')

# How far below its `worlds/` directory a world may sit.
#
# 0 would mean "directly in worlds/", which is false here: the demo tree sorts
# its worlds into `worlds/showcase/`, `worlds/flagship/`, `worlds/chat/`,
# `worlds/physics/` and so on, and AGENTS.md cites those paths directly. 1
# admits that and nothing more.
#
# MEASURED across the 366 worlds: 241 sit at depth 0 and 125 at depth 1, and
# NOTHING is deeper -- across four independently-owned subtrees (samples,
# policies, robot_combat, _archive). A bound that four workstreams landed on
# without coordinating is a convention, not a coincidence, so it is worth
# holding. It is still only a convention: if a subtree ever has a real reason to
# nest further, raise this constant deliberately rather than deleting the check.
MAX_WORLD_SUBDIR_DEPTH = 1

# Perspective files that are committed but pair with no world -- pre-existing
# debt, NOT permission to add more.
#
# All six are tracked, so they are red in every clone, not local residue on one
# machine. Cleaning them up means deleting files under `projects/`, which is
# outside this test's ownership; they are listed here so the rule can be enforced
# for everything else instead of being switched off entirely.
#
# What each one is, measured by searching the whole repo for a world of the same
# stem:
#   .cylinder_stack.wbproj  -- the world MOVED. It is now at
#                              projects/samples/demos/worlds/misc/cylinder_stack.omniworld
#                              and the sibling stayed behind in the parent dir.
#                              This is precisely the defect this test exists to
#                              catch, caught after the fact.
#   .gantry / .hexapod / .soccer / .stewart_platform / .floating_geometries
#                           -- no world of that stem exists ANYWHERE in the repo.
#                              Deleted worlds whose siblings outlived them.
#
# Paths are repo-relative POSIX so the list is stable across platforms.
KNOWN_ORPHAN_PERSPECTIVES = frozenset((
    'projects/samples/demos/worlds/.cylinder_stack.wbproj',
    'projects/samples/demos/worlds/.gantry.wbproj',
    'projects/samples/demos/worlds/.hexapod.wbproj',
    'projects/samples/demos/worlds/.soccer.wbproj',
    'projects/samples/demos/worlds/.stewart_platform.wbproj',
    'projects/samples/geometries/worlds/.floating_geometries.wbproj',
))


def repository_files(root):
    """Return every file the repository actually carries under `root`.

    That is: git-tracked files, plus untracked ones that are not gitignored (a
    freshly authored world is expected to satisfy the structure rules before it is
    committed). Walking the filesystem instead sweeps up gitignored run residue --
    `.harness_*` world copies written by the harness, `projects/_scratch/`, agent
    worktrees -- none of which is part of the project structure under test, and all
    of which made this test fail on any machine that had ever run a demo.

    Returns None when `root` is not inside a usable git checkout (a distribution
    tarball, say), so the caller can fall back to walking the filesystem.
    """
    listings = (['git', 'ls-files', '-z', '--', '.'],
                ['git', 'ls-files', '-z', '--others', '--exclude-standard', '--', '.'])
    paths = []
    for listing in listings:
        try:
            completed = subprocess.run(listing, cwd=root, stdout=subprocess.PIPE,
                                       stderr=subprocess.DEVNULL, check=True)
        except (OSError, subprocess.SubprocessError):
            return None
        # -z output is NUL-separated and unquoted; paths are git-relative to `root`.
        paths += [os.path.join(root, os.fsdecode(entry).replace('/', os.sep))
                  for entry in completed.stdout.split(b'\0') if entry]
    return [path for path in paths if os.path.isfile(path)]


def walked_files(root):
    """Return every file under `root`, gitignored residue included."""
    paths = []
    for rootPath, dirNames, fileNames in os.walk(root):
        paths += [os.path.join(rootPath, fileName) for fileName in fileNames]
    return paths


def perspective_stem(fileName):
    """Return the world stem a perspective file names, or None if it is malformed.

    `.my_world.omniperspective` -> `my_world`.

    The extension is stripped FIRST and the leading dot is then required exactly
    once, because the obvious spellings both have holes worth avoiding.
    `os.path.splitext('.omniperspective')` returns `('.omniperspective', '')` --
    Python reads a leading-dot name as a hidden file with no extension -- so a
    stemless perspective looks like a perfectly good stem. And `lstrip('.')`
    strips ANY number of dots, quietly accepting `..my_world.wbproj` and mapping
    it onto the same stem as the well-formed name, which is how two files end up
    silently claiming one world.

    Single-sourced so the hidden-sibling check and the orphan check can never
    disagree about what a given file name means.
    """
    for extension in PERSPECTIVE_EXTENSIONS:
        if fileName.endswith(extension):
            stem = fileName[:-len(extension)]
            # Exactly one leading dot, and something after it.
            if len(stem) > 1 and stem[0] == '.' and stem[1] != '.':
                return stem[1:]
            return None
    return None


class TestProjectStructure(unittest.TestCase):
    """Unit test of the project structure: where worlds live, and what pairs with them.

    (Named `TestTextures` until 2026-08-16, copied from test_textures.py in the
    same directory -- which still defines a class of that name. Two modules
    exporting the same test-class name is a collision waiting for the first
    runner that flattens them into one namespace.)
    """

    def setUp(self):
        """Enumerate the worlds and perspective files the repository carries."""
        OMNISIM_HOME = os.path.normpath(os.environ['OMNISIM_HOME'])
        self.home = OMNISIM_HOME
        self.projects = os.path.join(OMNISIM_HOME, 'projects')

        files = repository_files(self.projects)
        # Recorded so the orphan check can decline to run rather than answer a
        # question a filesystem walk cannot answer -- see test_no_orphan_perspectives.
        self.fromGit = files is not None
        if files is None:
            files = walked_files(self.projects)

        self.worlds = [f for f in files if f.endswith(WORLD_EXTENSIONS)]
        self.perspectives = [f for f in files if f.endswith(PERSPECTIVE_EXTENSIONS)]

    def relative(self, path):
        """Repo-relative POSIX form -- stable across platforms, and what the allowlist holds."""
        return os.path.relpath(path, self.home).replace(os.sep, posixpath.sep)

    def worldsDirectoryOf(self, world):
        """Return the NEAREST ancestor directory named 'worlds', or None.

        Nearest rather than outermost so the depth measured below is the depth
        that the world's own container implies, which is what the convention is
        about. The search stops at `projects/` so it can never escape the tree
        under test.
        """
        directory = os.path.dirname(world)
        while len(directory) > len(self.projects):
            if os.path.basename(directory) == 'worlds':
                return directory
            directory = os.path.dirname(directory)
        return None

    def test_worlds_live_under_a_worlds_directory(self):
        """Every world sits somewhere beneath a 'worlds' directory.

        This replaces the old "the world's PARENT is named worlds", which 125 of
        366 worlds violated. That stricter form was not a rule the project
        follows: `worlds/showcase/`, `worlds/flagship/`, `worlds/chat/` and
        `worlds/repro/` are all deliberate.

        What survives is the part that is both true and load-bearing. `worlds/`
        is how the demo launcher's catalogue, DEMOS.md, the recipes and every
        `--glob` sweep find a world at all, so a world dropped into a
        `controllers/` or `protos/` directory is genuinely lost -- not broken at
        load time, just invisible to everything that enumerates worlds.
        """
        for world in self.worlds:
            self.assertIsNotNone(
                self.worldsDirectoryOf(world),
                msg='This world is not under any "worlds" directory: "%s"' % self.relative(world)
            )

    def test_worlds_are_not_buried(self):
        """A world sits at most MAX_WORLD_SUBDIR_DEPTH levels below its 'worlds' directory."""
        for world in self.worlds:
            worldsDirectory = self.worldsDirectoryOf(world)
            if worldsDirectory is None:
                continue  # already reported by test_worlds_live_under_a_worlds_directory
            below = os.path.relpath(os.path.dirname(world), worldsDirectory)
            depth = 0 if below == os.curdir else len(below.split(os.sep))
            self.assertLessEqual(
                depth, MAX_WORLD_SUBDIR_DEPTH,
                msg='This world is buried %d levels below "%s", the limit is %d: "%s"\n'
                    'If the extra nesting is deliberate, raise MAX_WORLD_SUBDIR_DEPTH in '
                    '%s rather than deleting this check.'
                    % (depth, self.relative(worldsDirectory), MAX_WORLD_SUBDIR_DEPTH,
                       self.relative(world), os.path.basename(__file__))
            )

    def test_perspective_files_are_hidden_siblings(self):
        """Every perspective file is a HIDDEN sibling, i.e. named '.<stem><extension>'.

        Straight from src/omnisim/core/OmPerspectiveFileFormat.hpp: the engine
        writes `worlds/my_world.omniworld -> worlds/.my_world.omniperspective`.
        A perspective without the leading dot is dead weight -- the engine will
        never look for it under that name, so the saved layout is silently lost.

        The previous version of this file FILTERED to hidden files when building
        the list, which meant a wrongly-named one could never be reported at all.
        Enumerating all of them and asserting the property is what makes it a
        check rather than a definition.

        REACH, measured, so nobody later mistakes the rest for dead code: the
        enumeration only sees what git carries, and `.gitignore:204-205` ignore
        `.*.wbproj` and `.*.omniperspective` -- i.e. EVERY dot-leading name. So
        the case this catches in practice is the non-hidden one, which is the
        only malformed perspective git picks up on its own, and it was driven red
        by probe. The malformed-but-hidden spellings (`..stem.wbproj`, a bare
        `.omniperspective`) are only reachable if someone force-adds one; those
        branches of perspective_stem() are covered by reasoning about the name,
        not by a corpus probe, and they are cheap insurance rather than a claim
        that the corpus can currently exercise them.
        """
        for perspective in self.perspectives:
            self.assertIsNotNone(
                perspective_stem(os.path.basename(perspective)),
                msg='Perspective file is not a well-formed hidden sibling: "%s"\n'
                    'Expected exactly ".<world stem>%s" -- one leading dot, a non-empty '
                    'stem, and one of %s.'
                    % (self.relative(perspective), PERSPECTIVE_EXTENSIONS[0],
                       ' / '.join(PERSPECTIVE_EXTENSIONS))
            )

    def test_no_orphan_perspectives(self):
        """Every perspective file pairs with a world of the same stem in the same directory.

        This is the direction of the old pairing check that is worth keeping. It
        fails when someone renames, moves or deletes a world and leaves the
        hidden sibling behind -- which has already happened six times here, hence
        KNOWN_ORPHAN_PERSPECTIVES.

        Pairing is git-set against git-set, never `os.path.isfile`. `.gitignore`
        ignores `.*.wbproj`, so a filesystem probe answers with whatever the
        developer's own GUI sessions left lying about and the verdict stops being
        a property of the repository.
        """
        if not self.fromGit:
            self.skipTest('needs a git checkout: "%s" is not one, and `.gitignore` ignores '
                          '`.*.wbproj`, so a filesystem walk cannot tell committed litter '
                          'from a developer\'s own local GUI state' % self.projects)

        worldStems = {(os.path.dirname(world), os.path.splitext(os.path.basename(world))[0])
                      for world in self.worlds}

        def isOrphan(perspective):
            stem = perspective_stem(os.path.basename(perspective))
            if stem is None:
                # Malformed names are test_perspective_files_are_hidden_siblings'
                # business; reporting them here as well would just double the noise.
                return False
            return (os.path.dirname(perspective), stem) not in worldStems

        for perspective in self.perspectives:
            relative = self.relative(perspective)
            if not isOrphan(perspective):
                continue
            self.assertIn(
                relative, KNOWN_ORPHAN_PERSPECTIVES,
                msg='Perspective file is not associated to any world: "%s"\n'
                    'A world was probably renamed, moved or deleted without its hidden '
                    'sibling. Delete the perspective (it is a regenerable GUI artifact), '
                    'or move it next to the world it belongs to.' % relative
            )

        # Ratchet: an allowlist nobody prunes rots into a silent exemption, and would
        # then re-permit a NEW orphan that happened to land on one of these six exact
        # paths. So paying the debt is required to also record it. This is the one way
        # this test goes red on a GOOD change -- the fix is one deleted line, and the
        # message says so.
        stale = sorted(entry for entry in KNOWN_ORPHAN_PERSPECTIVES
                       if not any(self.relative(p) == entry and isOrphan(p) for p in self.perspectives))
        self.assertEqual(
            stale, [],
            msg='KNOWN_ORPHAN_PERSPECTIVES lists %d entr%s that %s no longer an orphan -- the '
                'litter was cleaned up. Delete these lines from the allowlist in %s so it '
                'cannot silently re-permit a future orphan at the same path:\n  %s'
                % (len(stale), 'y' if len(stale) == 1 else 'ies',
                   'is' if len(stale) == 1 else 'are',
                   os.path.basename(__file__), '\n  '.join(stale))
        )


if __name__ == '__main__':
    unittest.main()
