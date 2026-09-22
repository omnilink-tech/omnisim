# Copyright 2026 OmniLink
# SPDX-License-Identifier: Apache-2.0
"""Protect clean installations from accidentally borrowing developer packages."""
import json
from pathlib import Path
from types import SimpleNamespace

from omnisim import omnilink_runtime
from omnisim.dev import runner
from scripts.packaging import newton_runtime_pins as pins


def test_controller_lock_is_complete_and_used_by_bundle():
    specs = pins.controller_requirements()
    assert all('==' in spec for spec in specs)
    names = {spec.split('==')[0].lower() for spec in specs}
    assert {'omnilink', 'requests', 'truststore', 'websocket-client', 'pillow',
            'certifi', 'urllib3', 'charset-normalizer', 'idna'} <= names
    assert set(specs) <= set(pins.bundle_requirements())


def test_probe_ignores_developer_python_paths(monkeypatch, tmp_path):
    monkeypatch.setenv('PYTHONPATH', '/private/omnilink/src')
    monkeypatch.setenv('PYTHONHOME', '/private/python')
    monkeypatch.setenv('PYTHONUSERBASE', '/private/site')
    def run(args, **kwargs):
        assert args[1:3] == ['-I', '-c']
        assert not {'PYTHONPATH', 'PYTHONHOME', 'PYTHONUSERBASE'} & kwargs['env'].keys()
        return SimpleNamespace(returncode=1, stderr='ModuleNotFoundError: omnilink', stdout='')
    monkeypatch.setattr(omnilink_runtime.subprocess, 'run', run)
    assert omnilink_runtime.probe('python', tmp_path)['status'] == 'missing'


def test_probe_rejects_sdk_outside_bundle(monkeypatch, tmp_path):
    (tmp_path / 'site-packages').mkdir()
    payload = {'version': '0.6.3', 'sdk_path': str(tmp_path / 'private/client.py'), 'requests': 'x'}
    monkeypatch.setattr(omnilink_runtime.subprocess, 'run', lambda *a, **kw:
                        SimpleNamespace(returncode=0, stdout=json.dumps(payload), stderr=''))
    assert omnilink_runtime.probe(str(tmp_path / 'python.exe'), tmp_path)['status'] == 'missing'


def test_windows_gui_prefers_bundled_controller(monkeypatch, tmp_path):
    bundle = tmp_path / 'msys64/mingw64/bin/newton-runtime'
    bundle.mkdir(parents=True)
    (bundle / 'python.exe').touch()
    monkeypatch.setattr(runner, 'REPO_ROOT', tmp_path)
    monkeypatch.setattr(runner.sys, 'platform', 'win32')
    monkeypatch.setenv('PYTHON_HOME', str(tmp_path))
    monkeypatch.setenv('PATH', 'unrelated-system-python')
    path = runner.omnisim_env()['PATH']
    assert path.index(str(bundle)) < path.index('unrelated-system-python')
