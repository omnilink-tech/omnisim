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

"""The two ways the core says no."""


class CoreUnsupported(Exception):
    """A controller asked the core for something it does not model.

    Raised inside a run. The run stops: a call the core would have to guess at
    is never answered with a guess.
    """


class CoreDeclined(Exception):
    """The core will not run this world or job; the engine should.

    Raised before or after a run, with the reason in plain words. A caller in
    `auto` mode catches it and runs the job on the engine.
    """
