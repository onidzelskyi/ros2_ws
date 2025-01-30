# Copyright 2015 Open Source Robotics Foundation, Inc.
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
Unit tests for PEP 257 compliance in the ultrasonic_sensor package.

This module contains tests to verify that the code in the ultrasonic_sensor package
complies with PEP 257 docstring style guidelines.
"""

from ament_pep257.main import main
import pytest

@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    """
    Test the code for PEP 257 compliance.

    This function runs PEP 257 on the codebase and asserts that there are no
    docstring style errors or warnings. If any errors or warnings are found, the
    test will fail and display the errors.

    Raises:
        AssertionError: If any docstring style errors or warnings are found.
    """
    rc = main(argv=['.', 'test'])
    assert rc == 0, 'Found code style errors / warnings'
