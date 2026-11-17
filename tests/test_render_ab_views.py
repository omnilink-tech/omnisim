# Copyright 2026 OmniLink. Licensed under the Apache License, Version 2.0.
import json
from pathlib import Path
import tempfile
import unittest

from scripts.dev.render_ab import comparison_worlds, summarize_profile


class ComparisonViews(unittest.TestCase):
    def test_profile_excludes_warmup_and_preserves_missing_data(self):
        self.assertIsNone(summarize_profile('frame=100 calls renderMs=3'))
        def row(frame, time, reused):
            return (f'frame={frame} profile collectUs=10 renderUs={time} localShadowUs=5 '
                    f'localCandidates=600 localDraws=0 localFaces=0 localReused={reused}\n')
        report = row(0, 90000, 0) + row(60, 120, 1) + row(61, 140, 1) + row(62, 300, 0)
        # A duplicate frame must not overweight a sample.
        result = summarize_profile(report + row(60, 120, 1) + 'frame=63 profile broken\n')
        self.assertEqual(result['samples'], 3)
        self.assertEqual(result['renderUs'], {'p50': 140, 'p95': 300})
        self.assertAlmostEqual(result['local_cache_hit_fraction'], 2/3)

    def test_preserves_source_materials_and_relative_assets(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            world = root / "house.omniworld"
            source = ('#OMNISIM R2025a utf8\nViewpoint { position 0 0 1 exposure 1.25 }\n'
                      'Shape { appearance PBRAppearance { roughness 0.8 '
                      'baseColorMap ImageTexture { url [ "textures/brick.png" ] } } }\n')
            world.write_text(source, encoding="utf-8")
            manifest = root / "views.json"
            view = {"name": "inside", "position": [1, 2, 3], "orientation": [0, 0, 1, 0.5]}
            manifest.write_text(json.dumps({"views": [view]}), encoding="utf-8")
            with comparison_worlds(world, str(manifest)) as variants:
                path, name, _ = variants[0]
                self.assertEqual(path.parent, world.parent)
                result = path.read_text(encoding="utf-8")
                self.assertIn("position 1.000 2.000 3.000", result)
                self.assertIn("exposure 1.25", result)
                self.assertEqual(result[result.index("Shape"):], source[source.index("Shape"):])
                self.assertEqual(world.read_text(encoding="utf-8"), source)
                self.assertEqual(name, "house__inside")
            self.assertFalse(path.exists())

    def test_duplicate_views_fail_and_remove_temporary_worlds(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            world = root / "house.omniworld"
            world.write_text("Viewpoint { }", encoding="utf-8")
            view = {"name": "same", "position": [1, 2, 3], "orientation": [0, 0, 1, 0]}
            manifest = root / "views.json"
            manifest.write_text(json.dumps({"views": [view, view]}), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "duplicate"):
                with comparison_worlds(world, str(manifest)):
                    pass
            self.assertEqual(list(root.glob(".render_view_*.omniworld")), [])


if __name__ == "__main__":
    unittest.main()
