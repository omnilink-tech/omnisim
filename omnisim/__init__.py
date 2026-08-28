# Copyright 2026 OmniLink
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

"""OmniSim — agent-driven robot simulation built on Webots.

The canonical Python entry point. Run `python -m omnisim --help` for the CLI.
"""

# Tracks the OmniSim release. Kept in sync with omniSimVersionString in
# src/omnisim/core/OmApplicationInfo.cpp by publish_snapshot.sh, which now
# rewrites BOTH. (It used to bump only the C++ string, and this one silently
# went stale -- `omnisim doctor` reported 5.1.1 during the v5.3.0 prep.)
__version__ = "8.1.9"
