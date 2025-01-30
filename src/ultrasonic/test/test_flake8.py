# Copyright 2017 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
Unit tests for flake8 compliance in the ultrasonic_sensor package.

This module contains tests to verify that the code in the ultrasonic_sensor package
complies with flake8 style guidelines.
"""

from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    """
    Test the code for flake8 compliance.

    This function runs flake8 on the codebase and asserts that there are no
    code style errors or warnings. If any errors or warnings are found, the
    test will fail and display the errors.

    Raises:
        AssertionError: If any code style errors or warnings are found.
    """
    rc, errors = main_with_errors(argv=[])
    assert rc == 0, (
        f'Found {len(errors)} code style errors / warnings:\n' +
        '\n'.join(errors)
    )
