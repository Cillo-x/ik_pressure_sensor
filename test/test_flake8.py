"""
Lint test: PEP 8 and pyflakes checks, run through ament_flake8.

ament_flake8 already defaults to ``max-line-length = 99`` and ignores the
missing-docstring rules (D1xx), so this checks real style problems only.

The test skips itself when the ROS 2 lint tooling is absent, which keeps a
plain ``pytest`` run usable on machines without ROS.  CI gets the tooling
through the ``test_depend`` entries in package.xml.
"""

import pytest

pytest.importorskip('ament_flake8', reason='ROS 2 lint tooling is not installed')

from ament_flake8.main import main_with_errors  # noqa: E402,I100


@pytest.mark.linter
@pytest.mark.flake8
def test_flake8():
    rc, errors = main_with_errors(argv=[])
    assert rc == 0, 'Found %d code style error(s):\n%s' % (
        len(errors), '\n'.join(errors))
