# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0

"""RobotAPI class wrapper for robot API calls."""

from rclpy.impl.rcutils_logger import RcutilsLogger

from .Requester import Requester


class RobotAPI:
    def __init__(
        self,
        prefix: str,
        timeout: float,
        api_key: str,
        robot_id: str,
        battery_attribute_id: str,
        map_attribute_id: str,
    ):
        self.prefix = prefix
        self.timeout = timeout
        self.logger = RcutilsLogger(f'RobotAPI ({robot_id})')
        self.headers = {
            'Content-Type': 'application/json',
            'Accept': 'application/json',
            'x-auth-inorbit-app-key': api_key,
        }
        self.robot_id = robot_id
        self.battery_attribute_id = battery_attribute_id
        self.map_attribute_id = map_attribute_id
        self.requester = Requester(
            base_url=self.prefix,
            headers=self.headers,
            timeout=self.timeout,
            logger=self.logger,
        )
        self.last_activity_id = None
        self.current_action = None

    def check_connection(self):
        """Return True if connection to the robot API server is successful."""
        response = self.requester.get_request(endpoint='robots')
        if response is None:
            self.logger.error('No response received from robot API server')
            return False
        return True

    def get_online_robot_configs(self):
        """
        Get the list of robots and their RMF/ORO configuration.

        Expected response:
        [
            {
                "robot_id": "andino1",
                "max_delay": 15.0,
                "battery_attribute_id": "battery_soc",
                "map_attribute_id": "current_map",
                "last_online": "2026-05-04T12:30:45Z"
            }
        ]
        """
        response = self.requester.get_request(endpoint='robots/online/configuration')

        if response is None:
            self.logger.error('No response received from robot discovery endpoint')
            return []

        if response.status_code != self.requester.HTTP_OK:
            self.logger.error(
                f'Failed to get online robot configs: '
                f'status={response.status_code}, response={response.text}'
            )
            return []

        response_json = response.json()

        if not isinstance(response_json, list):
            self.logger.error(
                f'Expected online robot config response to be a list, got: ' f'{response_json}'
            )
            return []

        required_fields = {
            'robot_id',
            'max_delay',
            'battery_attribute_id',
            'map_attribute_id',
            'last_online',
        }

        valid_robots = []
        for robot_config in response_json:
            if not isinstance(robot_config, dict):
                self.logger.error(f'Invalid robot config entry: {robot_config}')
                continue

            missing_fields = required_fields - set(robot_config.keys())
            if missing_fields:
                self.logger.error(
                    f'Robot config is missing fields {missing_fields}: ' f'{robot_config}'
                )
                continue

            valid_robots.append(robot_config)

        return valid_robots

    def is_command_completed(self):
        """
        Check if the robot has completed its last command.

        Return True if the robot has completed its last command,
        else return False.
        """
        if self.last_activity_id is None:
            self.logger.info(f"No last activity recorded for robot '{self.robot_id}'")
            self.current_action = None
            return True

        response = self.requester.get_request(
            endpoint=f'robots/{self.robot_id}/actions/{self.last_activity_id}'
        )

        if response is None:
            self.logger.error('No response received from robot API server')
            return False

        response_json = response.json()
        if response_json.get('status', None) == 'finished':
            self.logger.info(
                f"Activity '{self.last_activity_id}' for robot " f"'{self.robot_id}' has completed"
            )
            self.last_activity_id = None
            self.current_action = None
            return True

        return False

    def navigate(
        self,
        pose,
        map_name: str,
        speed_limit=0.0,
    ):
        """
        Request the robot to navigate to a target pose.

        Request the robot to navigate to pose:[x,y,theta].
        Where x, y and theta are in the robot's coordinate convention.
        """
        self.logger.info(
            f'Received navigation request for {self.robot_id} to pose '
            f'{pose} on map {map_name} with speed limit {speed_limit}'
        )

        request_body = {
            'waypoints': [
                {
                    'frameId': map_name,
                    'x': pose[0],
                    'y': pose[1],
                    'theta': pose[2],
                }
            ]
        }

        response = self.requester.post_request(
            endpoint=f'robots/{self.robot_id}/navigation/waypoints',
            json=request_body,
        )

        if response is None:
            self.logger.error('No response received from robot API server')
            return False

        return response.status_code == self.requester.HTTP_OK

    def localize(
        self,
        pose,
        map_name: str,
    ):
        """Request the robot to localize on a target map."""
        action_body = {
            'actionId': 'Relocalize-000000',
            'parameters': {
                'deltaPose': {
                    'x': -pose[0],
                    'y': -pose[1],
                    'theta': -pose[2],
                    'frameId': map_name,
                }
            },
        }

        response = self.requester.post_request(
            endpoint=f'robots/{self.robot_id}/actions',
            json=action_body,
        )

        if response is None:
            self.logger.error('No response received from robot API server')
            return False

        return response.status_code == self.requester.HTTP_OK

    def start_activity(
        self,
        activity: str,
        label: str | None,
        activity_args: dict | None = None,
    ):
        """Request the robot to begin a specific process or activity."""
        if activity is None:
            self.logger.error('Cannot start activity because action_id is None')
            return False

        action_body = {
            'actionId': f'{activity}',
            'parameters': activity_args or {},
        }

        response = self.requester.post_request_exception(
            endpoint=f'robots/{self.robot_id}/actions',
            json=action_body,
        )

        if response is None:
            self.logger.error('No response received from robot API server')
            return False

        response_json = response.json()

        if response.status_code != self.requester.HTTP_OK:
            self.logger.error(
                f'Failed to start activity {activity} for robot '
                f'{self.robot_id}: status={response.status_code}, '
                f'response={response_json}'
            )
            self.last_activity_id = None
            self.current_action = None
            return False

        execution_id = response_json.get('executionId', None)
        if execution_id is None:
            self.logger.error(
                f'Activity {activity} started but response did not include '
                f'executionId: {response_json}'
            )
            self.last_activity_id = None
            self.current_action = None
            return False

        self.logger.info(
            f'Activity started for robot {self.robot_id}: '
            f'{activity}, label={label}, response={response_json}'
        )

        self.last_activity_id = execution_id
        self.current_action = execution_id
        return True

    def stop(self):
        """
        Command the robot to stop its current activity.

        Return True if robot has successfully stopped. Else False.
        """
        if self.last_activity_id is None:
            self.logger.error(
                f"No last activity recorded for robot '{self.robot_id}'. " 'Cannot stop.'
            )
            return True

        action_body = {'actionId': self.last_activity_id, 'parameters': {}}

        response = self.requester.post_request(
            endpoint=f'robots/{self.robot_id}/actions',
            json=action_body,
        )

        if response is None:
            self.logger.error('No response received from robot API server')
            return False

        self.last_activity_id = None
        self.current_action = None
        return response.status_code == self.requester.HTTP_OK

    def position(self):
        """Get the robot's current position as [x, y, theta]."""
        response = self.requester.get_request(endpoint=f'robots/{self.robot_id}/localization/pose')

        if response is None:
            self.logger.error('No response received from robot API server')
            return None

        response_json = response.json()

        if not all(k in response_json for k in ('x', 'y', 'theta')):
            self.logger.error(f'Response JSON missing expected keys: {response_json}')
            return None

        return [
            float(response_json['x']),
            float(response_json['y']),
            float(response_json['theta']),
        ]

    def battery_soc(self):
        """Get the robot's state of charge as a value between 0.0 and 1.0."""
        attribute_id = self.battery_attribute_id

        response = self.requester.get_request(
            endpoint=f'robots/{self.robot_id}/attributes/{attribute_id}'
        )

        if response is None:
            self.logger.error('No response received from robot API server')
            return None

        response_json = response.json()

        if 'value' not in response_json:
            self.logger.error(f"Response JSON missing 'value' key: {response_json}")
            return None

        if response_json['value'] == '':
            self.logger.error(f'Battery SoC value is empty string: {response_json}')
            return None

        if not (0.0 <= float(response_json['value']) <= 1.0):
            self.logger.error(
                f'Battery SoC value out of expected range [0.0, 1.0]: {response_json["value"]}'
            )
            return None

        return float(response_json['value'])

    def current_map(self):
        """Get the name of the map the robot is currently on."""
        response = self.requester.get_request(
            endpoint=f'robots/{self.robot_id}/attributes/{self.map_attribute_id}'
        )

        if response is None:
            self.logger.error('No response received from robot API server')
            return None

        response_json = response.json()

        if 'value' not in response_json:
            self.logger.error(f"Response JSON missing 'value' key: {response_json}")
            return None

        return response_json['value']

    def get_data(self, robot_name: str | None = None):
        """Get update data for a robot."""
        current_map = self.current_map()
        position = self.position()
        battery_soc = self.battery_soc()

        if not (current_map is None or position is None or battery_soc is None):
            return RobotUpdateData(robot_name, current_map, position, battery_soc)

        return None


class RobotUpdateData:
    """Update data for a single robot."""

    def __init__(
        self,
        robot_name: str,
        current_map: str,
        position: list[float],
        battery_soc: float,
        requires_replan: bool | None = None,
    ):
        self.robot_name = robot_name
        x = position[0]
        y = position[1]
        yaw = position[2]
        self.position = [x, y, yaw]
        self.current_map = current_map
        self.battery_soc = battery_soc
        self.requires_replan = requires_replan
