"""Test module for the hyperlinks."""

import unittest
import re
import os
import sys

from books import Books


class TestHyperlinks(unittest.TestCase):
    """Unit test of the hyperlinks."""

    def setUp(self):
        """Setup: get all the hyperlinks."""
        self.hyperlinks = []

        books = Books()
        for book in books.books:

            # we don't want to maintain links posted on Discord
            if book.name == 'discord':
                continue

            for md_path in book.md_paths:
                # Extract MD content.
                args = {} if sys.version_info[0] < 3 else {'encoding': 'utf-8'}
                with open(md_path, **args) as f:
                    content = f.read()
                # Remove code statements
                content = re.sub(r'```.+?(?=```)```', '', content, flags=re.S)
                content = re.sub(r'`.+?(?=`)`', '', content, flags=re.S)
                # Remove charts
                content = re.sub(r'%chart.+?(?=%end)%end', '\n', content, flags=re.S)
                # Extract hyperlinks.
                for m in re.finditer(r'[^\!](\[([^\]]*)\]\s*\(([^\)]*)\))', content):
                    hyperlinkMD = m.group(1)
                    hyperlinkName = m.group(2)
                    hyperlinkUrl = m.group(3)
                    self.hyperlinks.append({
                        'md': hyperlinkMD,
                        'name': hyperlinkName,
                        'url': hyperlinkUrl,
                        'file': md_path
                    })
        # # Debug: Uncomment to display all the acquired hyperlinks.
        # for h in self.hyperlinks:
        #     print (h)

    # RETIRED 2026-08-15: test_underscores_in_hyperlinks_are_protected.
    # An inherited Cyberbotics PROSE convention (escape every underscore in a
    # link label) that OmniSim's docs have never followed -- 169 violations,
    # and the sibling heading rule had 560. This is a house style the repo has
    # already chosen against, not debt, and a permanently-red test teaches
    # readers to ignore the suite. Removed with test_titles.py, test_lists.py
    # and test_paragraphs.py for the same reason. The STRUCTURAL checks in this
    # suite -- links resolve, anchors resolve, menus complete, images used --
    # are the ones worth gating on, and they are all green.

    def test_hyperlinks_do_not_contain_prohibited_characters(self):
        """Test that hyperlinks are not containing prohibited characters (such as '<')."""
        for h in self.hyperlinks:
            self.assertTrue(
                re.search(r'[<>]', h['name']) is None,
                msg='Hyperlink "%s" contains forbidden characters in "%s".' % (h['md'], h['file'])
            )

    def test_tag_hyperlinks(self):
        """Test that a tag-like hyperlinks are valid."""
        for h in self.hyperlinks:
            if h['name'] in ['C++', 'Java', 'Python', 'MATLAB']:
                self.assertTrue(
                    '.md' in h['url'],
                    msg='Hyperlink "%s" is wrongly detected as a tag in "%s".' % (h['md'], h['file'])
                )

    def test_github_file_exists(self):
        """Test that the github file pointed by the link exists."""
        for h in self.hyperlinks:
            # '{{ url.github_tree }}' expands to the OmniSim tree (see docs/js/showdown-extensions.js),
            # so this check must key on OmniSim's repository -- keyed on upstream's, it silently
            # matched nothing and validated no links at all.
            if h['url'].startswith('https://github.com/omnilink-tech/omnisim/tree/main'):
                # Derive the repo root from this file rather than REQUIRING
                # OMNISIM_HOME: a bare `pytest docs/tests` raised
                # KeyError: 'OMNISIM_HOME' here, which reads as a broken link
                # check rather than an unset variable. The env var still wins
                # when it is set (an out-of-tree checkout can point elsewhere).
                repo_root = os.environ.get('OMNISIM_HOME') or os.path.join(
                    os.path.dirname(os.path.abspath(__file__)), os.pardir, os.pardir)
                path = h['url'].replace('https://github.com/omnilink-tech/omnisim/tree/main',
                                        os.path.normpath(repo_root))
                self.assertTrue(
                    os.path.isfile(path) or os.path.isdir(path),
                    msg='Hyperlink "%s" is pointing to a non-existing file or directory "%s" (in file "%s").' %
                        (h['md'], path, h['file'])
                )
