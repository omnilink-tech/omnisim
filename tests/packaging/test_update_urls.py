import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from scripts.packaging.update_urls import replace_url


class UpdateUrlsTest(unittest.TestCase):
    def test_replace_url_preserves_non_ascii_bytes(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "non_ascii.omniworld"
            source.write_bytes(
                b"# legacy byte: \x90; utf8: \xe2\x80\x94; "
                b"omnisim://projects/demo.proto\r\n"
            )

            real_open = open
            calls = []

            def recording_open(*args, **kwargs):
                calls.append((args, kwargs))
                return real_open(*args, **kwargs)

            with patch("builtins.open", recording_open):
                replace_url(source, "v8.1.3", True)

            self.assertEqual(
                source.read_bytes(),
                b"# legacy byte: \x90; utf8: \xe2\x80\x94; "
                b"https://raw.githubusercontent.com/omnilink-tech/omnisim/"
                b"v8.1.3/projects/demo.proto\r\n",
            )
            self.assertEqual(
                [call[0][1] for call in calls],
                ["rb", "wb"],
            )


if __name__ == "__main__":
    unittest.main()
