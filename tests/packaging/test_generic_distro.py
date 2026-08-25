import sys
import tempfile
import unittest
from pathlib import Path


PACKAGING_DIR = Path(__file__).resolve().parents[2] / "scripts" / "packaging"
sys.path.insert(0, str(PACKAGING_DIR))

from generic_distro import OmniSimPackage, perspective_file_for_world  # noqa: E402


class DummyPackage(OmniSimPackage):
    def make_dir(self, directory):
        pass

    def compute_name_with_prefix_and_extension(self, basename, options):
        return basename


class GenericDistroTest(unittest.TestCase):
    def test_world_without_perspective_is_packaged_by_itself(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            worlds = root / "worlds"
            worlds.mkdir()
            world = worlds / "empty.omniworld"
            world.write_text("#VRML_SIM R2025a utf8\n", encoding="utf-8")

            package = DummyPackage.__new__(DummyPackage)
            package.omnisim_home = str(root)
            package.package_files = []
            package.add_file(str(world))

            self.assertEqual(package.package_files, [str(Path("worlds") / world.name)])
            self.assertIsNone(perspective_file_for_world(str(worlds), world.name))

    def test_written_perspective_is_preferred_to_legacy_file(self):
        with tempfile.TemporaryDirectory() as directory:
            worlds = Path(directory)
            new = worlds / ".demo.omniperspective"
            legacy = worlds / ".demo.wbproj"
            new.write_text("new", encoding="utf-8")
            legacy.write_text("legacy", encoding="utf-8")

            self.assertEqual(
                perspective_file_for_world(str(worlds), "demo.omniworld"),
                str(new),
            )


if __name__ == "__main__":
    unittest.main()
