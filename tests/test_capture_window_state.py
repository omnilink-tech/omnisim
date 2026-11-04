"""Live regression for focus-stealing Supervisor captures (Windows)."""
import importlib.util
import os
from pathlib import Path
import tempfile
import unittest

path=Path(__file__).resolve().parent/'rendering/capture_window_probe.py'
spec=importlib.util.spec_from_file_location('capture_window_probe',path)
probe=importlib.util.module_from_spec(spec);spec.loader.exec_module(probe)

@unittest.skipUnless(os.name=='nt','Win32 window-state regression')
class CaptureWindowState(unittest.TestCase):
    def capture(self,kind='images',no_rendering=False):
        binary=os.environ.get('OMNISIM_BINARY') or probe.resolve_omnisim_binary()
        if not binary:self.skipTest('OmniSim engine is not built')
        with tempfile.TemporaryDirectory(prefix='omnisim-capture-window-') as temporary:
            r=probe.run(Path(temporary)/'run',binary,kind,no_rendering,count=12)
            self.assertIsNone(r['error'],r)
            self.assertEqual(r['cleanup_errors'],[],r)
            self.assertGreater(r['samples'],0,r)
            self.assertEqual(r['restores_during_capture'],0,r)
            self.assertEqual(r['foreground_samples'],0,r)
            self.assertEqual(r['nonminimized_samples'],0,r)
            self.assertTrue(r['passed'],r)
    def test_repeated_exports_keep_window_minimized(self):self.capture()
    def test_exports_with_rendering_off_keep_window_minimized(self):self.capture(no_rendering=True)
    def test_native_movie_keeps_window_minimized(self):self.capture(kind='movie')

if __name__=='__main__':unittest.main()
