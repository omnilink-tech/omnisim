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

"""Sets the environment so that the cache tests can be performed in any condition.

The cache tests use a ``web://`` URL placeholder inside their fixture
``.proto`` / ``.wbt`` files; ``update_cache_urls`` rewrites those to
real ``https://raw.githubusercontent.com/cyberbotics/webots/<sha>/...``
URLs so the engine's web cache fetches assets from upstream. That only
works when this clone has a remote pointing at ``cyberbotics/webots``.

The OmniSim fork does NOT track upstream Webots (no live upstream
merge target — ``git remote -v`` carries no ``cyberbotics/webots``
remote). So the upstream-fetch setup is now best-effort: if discovery
fails, ``CACHE_TESTS_ENABLED`` flips to False and ``update_cache_urls``
becomes a no-op. The rest of ``test_suite.py`` (api / physics / parser
/ rendering / protos / other_api) can then run uninterrupted.
"""

import os
import subprocess
import sys
import urllib.error
import urllib.request
from pathlib import Path

if 'OMNISIM_HOME' in os.environ:
    OMNISIM_HOME = os.environ['OMNISIM_HOME'].replace('\\', '/')
elif 'OMNISIM_HOME' in os.environ:
    OMNISIM_HOME = os.environ['OMNISIM_HOME'].replace('\\', '/')
else:
    raise RuntimeError('Neither OMNISIM_HOME nor OMNISIM_HOME is set.')

# necessary in order to be able to run the cache-related tests in the test suite
if 'TESTS_HOME' in os.environ:
    ROOT_FOLDER = os.environ['TESTS_HOME']
else:
    ROOT_FOLDER = OMNISIM_HOME


CACHE_TESTS_ENABLED = True
BRANCH = None

branch_file_path = os.path.join(OMNISIM_HOME, 'resources', 'branch.txt')
if os.path.exists(branch_file_path):
    with open(branch_file_path, 'r') as file:
        BRANCH = file.read().strip()
elif 'BRANCH_HASH' in os.environ:  # fall-back mechanism for CI built image used by the test_suite
    BRANCH = os.environ['BRANCH_HASH']
else:
    # No branch.txt and no BRANCH_HASH -- the cache group cannot resolve its
    # upstream-fetch URLs. Disable the group rather than crash module import.
    print('cache_environment: no branch.txt / BRANCH_HASH; cache tests disabled.', file=sys.stderr)
    CACHE_TESTS_ENABLED = False


if CACHE_TESTS_ENABLED:
    test_url = f'https://raw.githubusercontent.com/cyberbotics/webots/{BRANCH}/README.md'
    try:
        with urllib.request.urlopen(test_url, timeout=5) as response:
            pass
    except (urllib.error.HTTPError, urllib.error.URLError, TimeoutError, OSError):
        # Either the branch isn't pushed to cyberbotics/webots, or the
        # network is unavailable. Fall back to merge-base discovery against
        # a local remote pointing at cyberbotics/webots.git, if one exists.
        print(f'cache_environment: unable to access {test_url}.', file=sys.stderr)
        repo = "cyberbotics/webots.git"
        remotes = subprocess.run(["git", "remote", "-v"], capture_output=True, text=True, check=False).stdout
        remote_line = next(
            (line for line in remotes.splitlines() if repo in line),
            None,
        )
        if remote_line is None:
            print(
                f"cache_environment: no remote pointing at {repo} in this clone; "
                "the cache test group will be skipped. Add the remote manually "
                "if you need to run it.",
                file=sys.stderr,
            )
            CACHE_TESTS_ENABLED = False
        else:
            remote = remote_line.split('\t')[0]
            print(f'cache_environment: resolving {BRANCH} via merge-base against '
                  f'{remote}/develop or {remote}/master.', file=sys.stderr)
            try:
                BRANCH = subprocess.run(
                    ["git", "merge-base", "HEAD", f'{remote}/develop', f'{remote}/master'],
                    capture_output=True, text=True, check=True,
                ).stdout.strip()
                print(f'cache_environment: using commit {BRANCH}', file=sys.stderr)
            except subprocess.CalledProcessError as e:
                print(f'cache_environment: merge-base failed ({e}); cache tests disabled.',
                      file=sys.stderr)
                CACHE_TESTS_ENABLED = False


def update_cache_urls(revert=False):
    if not CACHE_TESTS_ENABLED:
        # No upstream URL to rewrite to -- skip the rewrite entirely. The
        # cache-group worlds will fall through with their unresolved
        # `web://` placeholders, which is the right state for a fork
        # that isn't fetching from cyberbotics/webots.
        return
    paths = []
    paths.extend((Path(ROOT_FOLDER) / 'tests' / 'cache').rglob('*.proto'))
    paths.extend((Path(ROOT_FOLDER) / 'tests' / 'cache').rglob('*.wbt'))

    for path in paths:
        with open(path, 'r') as fd:
            content = fd.read()

        if revert:
            content = content.replace(ROOT_FOLDER + '/', 'absolute://')
            content = content.replace(f'https://raw.githubusercontent.com/cyberbotics/webots/{BRANCH}/', 'web://')
        else:
            content = content.replace('absolute://', ROOT_FOLDER + '/')
            content = content.replace('web://', f'https://raw.githubusercontent.com/cyberbotics/webots/{BRANCH}/')

        with open(path, 'w', newline='\n') as fd:
            fd.write(content)


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('Action not provided, options: "setup", "reset"')
    else:
        if sys.argv[1] == "setup":
            update_cache_urls()
        if sys.argv[1] == "reset":
            update_cache_urls(True)
