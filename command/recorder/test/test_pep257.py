"""Run the ROS 2 Python docstring checker."""

from ament_pep257.main import main
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    """Check Python docstrings."""
    assert main(argv=['.', 'test']) == 0
