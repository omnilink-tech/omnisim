"""Regression tests for the Windows MSYS2 dependency manifest."""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[2]
INSTALLER = ROOT / "scripts" / "install" / "msys64_installer.sh"


class Msys64InstallerTest(unittest.TestCase):
    def test_qt_translations_are_an_unconditional_base_dependency(self):
        script = INSTALLER.read_text(encoding="utf-8")
        base_packages = script.split("declare -a BASE_PACKAGES=(", 1)[1].split(
            "declare -a OPTIONAL_PACKAGES=(", 1
        )[0]

        self.assertIn('"mingw-w64-x86_64-qt6-translations"', base_packages)


if __name__ == "__main__":
    unittest.main()
