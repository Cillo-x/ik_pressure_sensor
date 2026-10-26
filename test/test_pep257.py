"""
Lint test: docstring style, run through ament_pep257.

The default ``ament`` convention ignores D100-D107, so docstrings are checked
for correctness where they exist but are not demanded everywhere.

The test skips itself when the ROS 2 lint tooling is absent; CI installs it
through the ``test_depend`` entries in package.xml.
"""

import pytest

pytest.importorskip('ament_pep257', reason='ROS 2 lint tooling is not installed')

from ament_pep257.main import main  # noqa: E402,I100


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    rc = main(argv=['.', 'test'])
    assert rc == 0, 'Found docstring style errors'
