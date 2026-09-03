"""Test module of the images."""
import unittest
from books import Books

import fnmatch
import os
import re
import sys


class TestImages(unittest.TestCase):
    """Unit test of the images."""

    def test_images_are_valid(self):
        """Test that the MD files refer to valid URLs."""
        books = Books()
        for book in books.books:
            for md_path in book.md_paths:
                args = {} if sys.version_info[0] < 3 else {'encoding': 'utf-8'}
                with open(md_path, **args) as f:
                    content = f.read()
                for match in re.finditer(r"!\[(.*?)\]\((.*?)\)", content):
                    # remove parameters
                    is_youtube_video = match.group(1) == "youtube video"
                    image_ref = match.group(2).split(' ')[0]
                    if not is_youtube_video and not image_ref.startswith('http'):
                        image_path = os.path.join(book.path, image_ref)
                        self.assertTrue(
                            os.path.isfile(image_path),
                            msg='%s: "%s" not found' % (md_path, image_path)
                        )

    def test_all_images_are_used(self):
        """Test that all the image files are referenced somewhere."""
        books = Books()
        for book in books.books:
            # search for all images
            images_paths = []  # ['image/sonar.png', 'image/sphere.png', ...]
            for root, dirnames, filenames in os.walk(book.path):
                if 'scenes' in root.replace(books.project_path, ''):
                    continue
                for filename in fnmatch.filter(filenames, '*.png') + fnmatch.filter(filenames, '*.jpg'):
                    image_path = os.path.join(root, filename)
                    image_path = image_path[(len(book.path) + 1):]
                    images_paths.append(image_path.replace('\\', '/'))
            self.assertGreater(
                len(images_paths), 0,
                msg='No image found in book "%s"' % book.name
            )

            # Read every MD file once; the loop below is O(images x files) otherwise.
            md_contents = []
            for md_path in book.md_paths:
                args = {} if sys.version_info[0] < 3 else {'encoding': 'utf-8'}
                with open(md_path, **args) as file:
                    md_contents.append(file.read())

            def referenced(path):
                return any(path in content for content in md_contents)

            # An image is USED when an MD file names it, or -- for a full-size PNG --
            # names its thumbnail (the pages reference `x.thumbnail.jpg`; the viewer
            # opens `x.png` from it, see docs/generate_thumbnails.py).
            #
            # Until 2026-09-02 a PNG also counted as used when its thumbnail merely
            # EXISTED on disk, whether or not anything referenced the thumbnail. That
            # let ~150 orphaned image pairs sit in the tree for months: an unreferenced
            # PNG was excused by an unreferenced thumbnail, and the thumbnail was only
            # ever checked for its original. Both halves must now be reachable from a
            # page.
            for image_path in images_paths:
                found = referenced(image_path)
                if not found and image_path.endswith('.png'):
                    found = (referenced(image_path[:-4] + '.thumbnail.jpg') or
                             referenced(image_path[:-4] + '.thumbnail.png'))
                self.assertTrue(
                    found, msg='Image "%s" not referenced in any MD file.' % image_path
                )
                # in case of thumbnail make sure the original file is available
                if image_path.endswith('.thumbnail.jpg'):
                    self.assertTrue(
                        image_path.replace('.thumbnail.jpg', '.png') in images_paths,
                        msg='Missing original file for thumbnail "%s".' % image_path
                    )


if __name__ == '__main__':
    unittest.main()
