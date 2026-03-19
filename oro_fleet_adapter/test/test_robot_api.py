# Copyright 2021 Open Source Robotics Foundation, Inc.
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

"""Unit tests for RobotAPI."""

from unittest.mock import Mock

from oro_fleet_adapter.RobotClientAPI import RobotAPI, RobotUpdateData
import pytest

EXPECTED_SOC = 0.85
EXPECTED_SOC_ALT = 0.9
EXPECTED_SOC_LOW = 0.42


class FakeResponse:
    """Simple fake HTTP response object for tests."""

    def __init__(self, status_code=200, payload=None):
        self.status_code = status_code
        self._payload = payload or {}

    def json(self):
        return self._payload


@pytest.fixture
def robot_api():
    api = RobotAPI(
        prefix='http://localhost:3001',
        timeout=5.0,
        api_key='test-api-key',
        robot_id='robot_id',
        battery_attribute_id='battery',
        map_attribute_id='map',
    )
    api.requester = Mock()
    api.requester.HTTP_OK = 200
    return api


def test_check_connection_success(robot_api):
    robot_api.requester.get_request.return_value = FakeResponse()
    assert robot_api.check_connection() is True
    robot_api.requester.get_request.assert_called_once_with(endpoint='robots')


def test_check_connection_failure(robot_api):
    robot_api.requester.get_request.return_value = None
    assert robot_api.check_connection() is False


def test_is_command_completed_without_last_activity(robot_api):
    robot_api.last_activity_id = None
    assert robot_api.is_command_completed() is True


def test_is_command_completed_finished(robot_api):
    robot_api.last_activity_id = 'exec-001'
    robot_api.requester.get_request.return_value = FakeResponse(payload={'status': 'finished'})

    assert robot_api.is_command_completed() is True
    assert robot_api.last_activity_id is None
    robot_api.requester.get_request.assert_called_once_with(
        endpoint='robots/robot_id/actions/exec-001'
    )


def test_is_command_completed_not_finished(robot_api):
    robot_api.last_activity_id = 'exec-001'
    robot_api.requester.get_request.return_value = FakeResponse(payload={'status': 'running'})

    assert robot_api.is_command_completed() is False
    assert robot_api.last_activity_id == 'exec-001'


def test_is_command_completed_none_response(robot_api):
    robot_api.last_activity_id = 'exec-001'
    robot_api.requester.get_request.return_value = None

    assert robot_api.is_command_completed() is False


def test_navigate_success(robot_api):
    robot_api.requester.post_request.return_value = FakeResponse(status_code=200)

    result = robot_api.navigate(
        pose=[1.0, 2.0, 3.14],
        map_name='L1',
        speed_limit=0.5,
    )

    assert result is True
    robot_api.requester.post_request.assert_called_once_with(
        endpoint='robots/robot_id/navigation/waypoints',
        json={
            'waypoints': [
                {
                    'frameId': 'L1',
                    'x': 1.0,
                    'y': 2.0,
                    'theta': 3.14,
                }
            ]
        },
    )


def test_navigate_failure_on_none_response(robot_api):
    robot_api.requester.post_request.return_value = None

    result = robot_api.navigate(
        pose=[1.0, 2.0, 3.14],
        map_name='L1',
    )

    assert result is False


def test_localize_success(robot_api):
    robot_api.requester.post_request.return_value = FakeResponse(status_code=200)

    result = robot_api.localize(
        pose=[1.0, -2.0, 0.5],
        map_name='L2',
    )

    assert result is True
    robot_api.requester.post_request.assert_called_once_with(
        endpoint='robots/robot_id/actions',
        json={
            'actionId': 'Relocalize-000000',
            'parameters': {
                'deltaPose': {
                    'x': -1.0,
                    'y': 2.0,
                    'theta': -0.5,
                    'frameId': 'L2',
                }
            },
        },
    )


def test_start_activity_success(robot_api):
    robot_api.requester.post_request.return_value = FakeResponse(
        status_code=200,
        payload={'executionId': 'activity-123'},
    )

    result = robot_api.start_activity(
        activity='dock',
        label='Dock robot',
        activity_args={'dock_name': 'charger_1'},
    )

    assert result is True
    assert robot_api.last_activity_id == 'activity-123'
    assert robot_api.current_action == 'activity-123'
    robot_api.requester.post_request.assert_called_once_with(
        endpoint='robots/robot_id/actions',
        json={
            'actionId': 'dock',
            'parameters': {'dock_name': 'charger_1'},
        },
    )


def test_start_activity_failure_status_code(robot_api):
    robot_api.requester.post_request.return_value = FakeResponse(
        status_code=400,
        payload={'executionId': 'activity-123'},
    )

    result = robot_api.start_activity(
        activity='dock',
        label='Dock robot',
        activity_args={'dock_name': 'charger_1'},
    )

    assert result is False
    assert robot_api.last_activity_id == 'activity-123'
    assert robot_api.current_action is None


def test_stop_without_last_activity(robot_api):
    robot_api.last_activity_id = None
    assert robot_api.stop() is True


def test_stop_success(robot_api):
    robot_api.last_activity_id = 'activity-999'
    robot_api.requester.post_request.return_value = FakeResponse(status_code=200)

    result = robot_api.stop()

    assert result is True
    assert robot_api.last_activity_id is None
    robot_api.requester.post_request.assert_called_once_with(
        endpoint='robots/robot_id/actions',
        json={'actionId': 'activity-999', 'parameters': {}},
    )


def test_position_success(robot_api):
    robot_api.requester.get_request.return_value = FakeResponse(
        payload={'x': 1, 'y': 2.5, 'theta': -0.75}
    )

    result = robot_api.position()

    assert result == [1.0, 2.5, -0.75]
    robot_api.requester.get_request.assert_called_once_with(
        endpoint='robots/robot_id/localization/pose'
    )


def test_position_missing_keys(robot_api):
    robot_api.requester.get_request.return_value = FakeResponse(payload={'x': 1, 'y': 2.5})

    assert robot_api.position() is None


def test_position_none_response(robot_api):
    robot_api.requester.get_request.return_value = None
    assert robot_api.position() is None


def test_battery_soc_success(robot_api):
    robot_api.requester.get_request.return_value = FakeResponse(payload={'value': EXPECTED_SOC})

    result = robot_api.battery_soc()

    assert result == EXPECTED_SOC
    robot_api.requester.get_request.assert_called_once_with(
        endpoint='robots/robot_id/attributes/battery'
    )


def test_battery_soc_missing_value(robot_api):
    robot_api.requester.get_request.return_value = FakeResponse(payload={})
    assert robot_api.battery_soc() is None


def test_battery_soc_empty_string(robot_api):
    robot_api.requester.get_request.return_value = FakeResponse(payload={'value': ''})
    assert robot_api.battery_soc() is None


def test_battery_soc_out_of_range(robot_api):
    robot_api.requester.get_request.return_value = FakeResponse(payload={'value': 1.5})
    assert robot_api.battery_soc() is None


def test_map_success(robot_api):
    robot_api.requester.get_request.return_value = FakeResponse(payload={'value': 'L1'})

    result = robot_api.current_map()

    assert result == 'L1'
    robot_api.requester.get_request.assert_called_once_with(
        endpoint='robots/robot_id/attributes/map'
    )


def test_map_missing_value(robot_api):
    robot_api.requester.get_request.return_value = FakeResponse(payload={})
    assert robot_api.current_map() is None


def test_get_data_success(robot_api):
    robot_api.current_map = Mock(return_value='L1')
    robot_api.position = Mock(return_value=[1.0, 2.0, 3.0])
    robot_api.battery_soc = Mock(return_value=EXPECTED_SOC_ALT)

    result = robot_api.get_data('andino_8')

    assert isinstance(result, RobotUpdateData)
    assert result.robot_name == 'andino_8'
    assert result.current_map == 'L1'
    assert result.position == [1.0, 2.0, 3.0]
    assert result.battery_soc == EXPECTED_SOC_ALT
    assert result.requires_replan is None


def test_get_data_returns_none_when_map_missing(robot_api):
    robot_api.current_map = Mock(return_value=None)
    robot_api.position = Mock(return_value=[1.0, 2.0, 3.0])
    robot_api.battery_soc = Mock(return_value=EXPECTED_SOC_ALT)

    assert robot_api.get_data('andino_8') is None


def test_robot_update_data_init():
    data = RobotUpdateData(
        robot_name='andino_10',
        current_map='L3',
        position=[4.0, 5.0, 6.0],
        battery_soc=EXPECTED_SOC_LOW,
        requires_replan=True,
    )

    assert data.robot_name == 'andino_10'
    assert data.current_map == 'L3'
    assert data.position == [4.0, 5.0, 6.0]
    assert data.battery_soc == EXPECTED_SOC_LOW
    assert data.requires_replan is True
