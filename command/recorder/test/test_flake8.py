"""Run the ROS 2 Python style checker."""

from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    """Check Python source formatting."""
    return_code, errors = main_with_errors(argv=[])
    assert return_code == 0, (
        f'Found {len(errors)} code style errors or warnings:\n' +
        '\n'.join(errors)
    )
