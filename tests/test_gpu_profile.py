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
from scripts.dev.render_ab import summarize_gpu_profile


def test_targets_warmup_duplicates_and_missing_passes():
    rows = '\n'.join([
        'target=1 frame=1 width=800 height=600 gpuSpanUs=999 sceneUs=999',
        'target=1 frame=60 width=800 height=600 gpuSpanUs=10 sceneUs=5 taaUs=1',
        'target=1 frame=60 width=800 height=600 gpuSpanUs=10 sceneUs=5 taaUs=1',
        'target=1 frame=61 width=800 height=600 gpuSpanUs=20 sceneUs=9',
        'target=2 frame=60 width=320 height=200 gpuSpanUs=3 sceneUs=2',
        'target=3 status=unavailable',
        'target=1 frame=62 width=800 height=600 gpuSpanUs=nan',
        'target=1 frame=63 width=800 height=600 gpuSpanUs=-1',
    ])
    a, b, c = summarize_gpu_profile(rows)
    assert a['samples'] == 2
    assert a['gpuSpanUs'] == {'samples': 2, 'p50': 10, 'p95': 20}
    assert a['taaUs']['samples'] == 1
    assert b['gpuSpanUs']['p50'] == 3
    assert c['status'] == 'unavailable' and 'gpuSpanUs' not in c
    assert summarize_gpu_profile('') == []
