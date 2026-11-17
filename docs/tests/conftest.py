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

"""Make this suite's flat helper imports work under pytest.

These modules predate packages and `import books` absolutely. pytest.ini sets
`--import-mode=importlib`, which does NOT put a test's own directory on
sys.path, so all 9 modules died at COLLECTION with
`ModuleNotFoundError: No module named 'books'` -- before books.py's own stale
book list could even be reached. That collection error is the first of the
three independent reasons this suite was dead.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
