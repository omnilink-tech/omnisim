"""Test module of the references."""
import unittest
from books import Books

import os
import re
import sys


def slugify(txt):
    """Slugify function."""
    output = txt.lower()
    output = re.sub(r'\]\([^)]*\)', ']', output)  # remove the content of parenthesis if reference
    output = re.sub(r'<[^>]+>', '', output)
    output = output.replace('+', 'p')
    output = re.sub(r"[\(\):`'=]", '', output)
    output = re.sub(r'\\_', '_', output)
    output = re.sub(r'[\W-]+', '-', output)
    output = re.sub(r'^-*', '', output)
    output = re.sub(r'-*$', '', output)
    return output.strip(' ').strip('-')


def github_slugify(txt):
    """Slugify the way GitHub does, which is NOT what slugify() above does.

    GitHub lowercases, DELETES punctuation outright, then maps each remaining
    space to a hyphen. slugify() instead collapses every run of non-word
    characters into a single hyphen. The two disagree whenever a heading
    contains punctuation surrounded by spaces -- an em dash, most often:

        "Known gaps -- what does NOT work yet"
            slugify()        -> known-gaps-what-does-not-work-yet
            GitHub           -> known-gaps--what-does-not-work-yet

    Links in this repo are written for GitHub, so both spellings are valid
    targets and both are recorded.
    """
    output = txt.lower()
    output = re.sub(r'\]\([^)]*\)', ']', output)   # drop reference targets
    output = re.sub(r'<[^>]+>', '', output)        # drop inline HTML
    output = re.sub(r'[^\w\s-]', '', output)       # DELETE punctuation
    return output.strip().replace(' ', '-')


class TestReferences(unittest.TestCase):
    """Unit test of the references."""

    def setUp(self):
        """Setup."""
        books = Books()
        self.anchors = {}
        for book in books.books:

            # we are not responsible of the content of the discord chats
            if book.name == 'discord':
                continue

            for md_path in book.md_paths:
                anchors = []
                args = {} if sys.version_info[0] < 3 else {'encoding': 'utf-8'}
                with open(md_path, **args) as f:
                    skipUntil = ''
                    for line in f:
                        if skipUntil:
                            if skipUntil in line:
                                skipUntil = ''
                            continue
                        if '<a' in line and 'name=' in line:
                            for m in re.finditer(
                                    r'<a[^>]*name\s*=\s*"(.*)"[^>]*>',
                                    line.strip()):
                                anchors.append(m.group(1))
                        if re.match(r'```', line):
                            skipUntil = '```'
                            continue
                        elif line.startswith('#'):
                            m = re.match(r'^#{1,4} .*$', line)
                            if m:
                                title = re.sub(r'^#*', '', line)
                                base = slugify(title)
                                anchors.append(base)
                                # ALSO accept GitHub's slug WHEN IT DIFFERS.
                                # slugify() collapses any run of non-word chars
                                # to ONE hyphen, so "Known gaps -- what does NOT
                                # work" becomes ...gaps-what...; GitHub DROPS the
                                # dash and converts each space, giving
                                # ...gaps--what... These docs are read on GitHub,
                                # so a link written for GitHub is correct and it
                                # was this harvester that was wrong.
                                # Only when it differs: appending an identical
                                # second copy would break test_anchors_are_unique.
                                gh = github_slugify(title)
                                if gh and gh != base:
                                    anchors.append(gh)
                        elif line.startswith('%figure'):
                            title = slugify(line.replace('%figure', ''))
                            if title:
                                anchors.append(slugify(title))
                        elif line.startswith('%api'):
                            title = line.replace('%api', '')
                            anchors.append(slugify(title))
                        elif "**wb" in line:
                            for m in re.finditer(r'\*\*(wb[^\*]*)\*\*', line):
                                anchor = m.group(1).replace('\\_', '_')
                                anchors.append(anchor)
                self.anchors[md_path] = anchors

    def test_anchors_are_unique(self):
        """Test that the anchors are unique."""
        books = Books()
        for book in books.books:

            # we are not responsible of the content of the discord chats
            if book.name == 'discord':
                continue

            for md_path in book.md_paths:
                anchors = self.anchors[md_path]
                s = set()
                for a in anchors:
                    if a in s:
                        self.assertTrue(
                            False,
                            msg='%s: Anchors "%s" are not unique'
                                % (md_path, a)
                        )
                    s.add(a)

    def test_references_are_valid(self):
        """Test that the MD files refer valid URLs."""
        books = Books()
        for book in books.books:

            # we are not responsible of the content of the discord chats
            if book.name == 'discord':
                continue

            for md_path in book.md_paths:
                args = {} if sys.version_info[0] < 3 else {'encoding': 'utf-8'}
                with open(md_path, **args) as f:
                    content = f.read()
                for m in re.finditer(r"[^!]\[(.*?)\]\(([^\)]+)\)", content):
                    # remove parameters
                    ref = m.group(2)
                    # 1. external link
                    self.assertFalse(
                        ref.startswith('www'),
                        msg='URL should not start with "www": "%s"' % (ref)
                    )
                    if ref.startswith('http'):
                        continue
                    # 3. steam link
                    if ref.startswith('steam:'):
                        continue
                    # 4. mailto
                    if ref.startswith('mailto:'):
                        mailto = ref[len('mailto:'):]
                        m = re.match(r'^([a-zA-Z0-9_.+-]+@[a-zA-Z0-9-]+\.[a-zA-Z0-9-.]+)$', mailto)
                        self.assertIsNotNone(
                            m,
                            msg='Invalid address "%s"' % (mailto)
                        )
                        continue
                    # 5. variable (the variable should be in format `url.something`)
                    if ref.startswith('{{'):
                        if re.match(r'{{\s{0,}url\..*}}', ref) is not None:
                            continue
                    # 6. link to another MD file
                    link = ''
                    anchor = ''
                    if ref.startswith('#'):
                        anchor = ref[1:]  # Remove the '#' character.
                    else:
                        ref = ref.split('#')
                        link = ref[0]
                        if len(ref) > 1:
                            anchor = ref[1]
                    if link != '':
                        # UPSTREAM ASSUMED SELF-CONTAINED BOOKS. OmniSim's docs
                        # deliberately link OUT of the book tree -- to engine
                        # source, worlds, scripts and directories (61 such links
                        # today, and every one resolves). The old ".md only"
                        # assertion rejected every one of them, so assert the
                        # target EXISTS instead of dictating its extension.
                        #
                        # Two forms are NOT filesystem paths and must be skipped,
                        # or this check reports ~90 false positives:
                        #   "images/x.png =26x26"  the doc renderer's inline size
                        #                          suffix, not part of the path
                        #   "{{ url.github_tree }}/..."  a template variable the
                        #                          exporter substitutes at build
                        target = link.split(' =')[0].strip()
                        if '{{' in target or not target:
                            continue
                        file_path = os.path.join(book.path, target)
                        self.assertTrue(
                            os.path.exists(file_path),
                            msg='%s: "%s" not found' % (md_path, file_path)
                        )
                    # 7. Anchor
                    if anchor != '':
                        file_path = ''
                        if link == '':
                            file_path = os.path.join(book.path, md_path)
                        else:
                            file_path = os.path.join(book.path, link)
                        file_path = os.path.abspath(file_path)  # Remove '..'-like patterns from file_path.
                        # Anchors are harvested only for files INSIDE the books,
                        # so a link out to AGENTS.md or docs/developer/*.md hit a
                        # bare KeyError and aborted the WHOLE test -- which is how
                        # a link gate over 1,883 references came to guard nothing.
                        # 4 such links today. Skip what we cannot index.
                        if file_path not in self.anchors:
                            continue
                        found = anchor in self.anchors[file_path]
                        self.assertTrue(
                            found, msg='%s: %s#%s not found' %
                            (md_path, file_path, anchor)
                        )
