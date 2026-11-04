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

"""Measured support/orientation predicate; no actuator or body mutations."""
import math

FEET=frozenset(('FL_calf','FR_calf','RL_calf','RR_calf'))


def support_ready(contacts,tilt_rad,max_tilt_rad):
    return math.isfinite(tilt_rad) and 0<=tilt_rad<max_tilt_rad and FEET<=set(contacts)
