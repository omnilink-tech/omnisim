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

"""Replace the omnisim:// URLs with https://raw.githubusercontent.com/omnilink-tech/omnisim/<version>/
   in world, proto and controller files. Replace the omnisim:// URLs with
   https://cdn.jsdelivr.net/gh/omnilink-tech/omnisim@<version> in robot windows html files.

   These MUST point at the OmniSim repository. Pointing them at cyberbotics/webots (as the
   inherited script did) bakes upstream URLs into every shipped world/proto -- and since
   OmniSim's tags do not exist in cyberbotics/webots, every one of those URLs 404s."""


import os
import re
import sys
from pathlib import Path


def replace_url(file, tag, github, only_extern_proto=False, revert=False):
    if only_extern_proto:
        pre_condition = '(?<=EXTERNPROTO \\")'
    else:
        pre_condition = ''

    if github:
        url = 'https://raw.githubusercontent.com/omnilink-tech/omnisim/'
    else:
        url = 'https://cdn.jsdelivr.net/gh/omnilink-tech/omnisim@'
    with open(file, 'r') as fd:
        content = fd.read()
    if revert:
        # revert any tag
        content = re.sub(pre_condition + url + '[^/]+/', 'omnisim://', content)
    else:
        content = re.sub(pre_condition + 'omnisim://', url + tag + '/', content)
    with open(file, 'w', newline='\n') as fd:
        fd.write(content)


def replace_projects_urls(tag, revert=False):
    if 'OMNISIM_HOME' in os.environ:
        OMNISIM_HOME = os.path.normpath(os.environ['OMNISIM_HOME'])
    else:
        OMNISIM_HOME = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

    only_replace_extern_proto = [
        '/tests/api/worlds/camera_color.omniworld'
    ]
    only_replace_extern_proto = [(Path(OMNISIM_HOME + file)).resolve() for file in only_replace_extern_proto]

    paths = []
    paths.extend(Path(OMNISIM_HOME + '/projects').rglob('*.proto'))
    paths.extend([*Path(OMNISIM_HOME + '/projects').rglob('*.omniworld'),
                  *Path(OMNISIM_HOME + '/projects').rglob('*.wbt')])
    paths.extend([*Path(OMNISIM_HOME + '/tests').rglob('*.omniworld'),
                  *Path(OMNISIM_HOME + '/tests').rglob('*.wbt')])
    paths.extend(Path(OMNISIM_HOME + '/resources/nodes').rglob('*.wrl'))

    with open(OMNISIM_HOME + '/scripts/packaging/controllers_with_urls.txt', 'r') as files:
        paths.extend(list(map(lambda path: Path(OMNISIM_HOME + path), files.read().splitlines())))

    for path in paths:
        if '/tests/cache/' not in str(path.resolve()):  # only cache_evironment.py should take care of cache urls
            replace_url(path, tag, True, path.resolve() in only_replace_extern_proto, revert)

    paths = []
    paths.extend(Path(OMNISIM_HOME + '/projects').rglob("*/plugins/robot_windows/*/*.html"))

    for path in paths:
        replace_url(path, tag, False, False, revert)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit('Missing argument: commit sha or tag.')

    if sys.argv[1] == 'webots':
        replace_projects_urls(None, True)
    else:
        replace_projects_urls(sys.argv[1])
