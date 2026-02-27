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


'''
    The RobotAPI class is a wrapper for API calls to the robot. Here users
    are expected to fill up the implementations of functions which will be used
    by the RobotCommandHandle. For example, if your robot has a REST API, you
    will need to make http request calls to the appropriate endpoints within
    these functions.
'''
from .Requester import Requester
from rclpy.impl.rcutils_logger import RcutilsLogger

class RobotAPI:
    # The constructor below accepts parameters typically required to submit
    # http requests. Users should modify the constructor as per the
    # requirements of their robot's API
    def __init__(self, config_yaml: dict, prefix: str, timeout: float) -> None:
        self.prefix = prefix
        self.timeout = timeout
        self.logger = RcutilsLogger(f"RobotAPI ({prefix})")
        self.config_yaml = config_yaml
        
        self.headers = {
            'Content-Type': 'application/json',
            'Accept': 'application/json',
            'x-auth-inorbit-app-key': config_yaml['api_key']
        }
        self.requester = Requester(
            base_url=self.prefix,
            headers=self.headers,
            timeout=self.timeout,
            logger=self.logger
        )

    def check_connection(self):
        ''' Return True if connection to the robot API server is successful '''
        response = self.requester.get_request(endpoint="robots")
        if not response:
            self.logger.error("No response received from robot API server")
            return False
        return response.status_code == 200

    def localize(
        self,
        robot_name: str,
        pose,
        map_name: str,
    ):
        ''' Request the robot to localize on target map. This 
            function should return True if the robot has accepted the 
            request, else False '''
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        #
        # TODO: this is not implemented on inorbit api
        return False
    
    def navigate(
        self,
        robot_name: str,
        pose,
        map_name: str,
        speed_limit=0.0
    ):
        ''' Request the robot to navigate to pose:[x,y,theta] where x, y and
            and theta are in the robot's coordinate convention. This function
            should return True if the robot has accepted the request,
            else False '''
        
        request_body = {
            "waypoints": [
                {
                    "frameId": map_name,
                    "x": pose[0],
                    "y": pose[1],
                    "theta": pose[2]
                }
            ]
        }
        response = self.requester.post_request(
            endpoint=f"robots/{robot_name}/navigation/waypoints",
            json=request_body
        )
        if not response:
            self.logger.error("No response received from robot API server")
            return False
        return response.status_code == 200

    def start_activity(
        self,
        robot_name: str,
        activity: str,
        label: str
    ):
        ''' Request the robot to begin a process. This is specific to the robot
        and the use case. For example, load/unload a cart for Deliverybot
        or begin cleaning a zone for a cleaning robot.
        Return True if process has started/is queued successfully, else
        return False '''
        action_body = {
            "actionId": "string",
            "parameters": {}
        }
        response = self.requester.post_request(
            endpoint=f"robots/{robot_name}/actions",
            json=action_body
        )
        if not response:
            self.logger.error("No response received from robot API server")
            return False
        return response.status_code == 200


    def stop(self, robot_name: str):
        ''' Command the robot to stop.
            Return True if robot has successfully stopped. Else False. '''
        # TODO: this is not implemented on inorbit api, check if for oro will change
        action_body = {'actionId': 'CancelNavGoal-000000'}
        response = self.requester.post_request(
            endpoint=f"robots/{robot_name}/actions",
            json=action_body
        )
        if not response:
            self.logger.error("No response received from robot API server")
            return False
        return response.status_code == 200

    def position(self, robot_name: str):
        ''' Return [x, y, theta] expressed in the robot's coordinate frame or
        None if any errors are encountered '''
        response = self.requester.get_request(
            endpoint=f"robots/{robot_name}/localization/pose"
        )
        if not response:
            self.logger.error("No response received from robot API server")
            return None
        response_json = response.json()
        
        # check if response_json has the expected keys x, y, theta
        if not all(k in response_json for k in ("x", "y", "theta")):
            self.logger.error(f"Response JSON missing expected keys: {response_json}")
            return None
        return response_json['x'], response_json['y'], response_json['theta']

    def battery_soc(self, robot_name: str):
        ''' Return the state of charge of the robot as a value between 0.0
        and 1.0. Else return None if any errors are encountered. '''
        attribute_id = self.config_yaml['battery_attribute_id']
        response = self.requester.get_request(
            endpoint=f"robots/{robot_name}/attributes/{attribute_id}"
        )
        if not response:
            self.logger.error("No response received from robot API server")
            return None
        response_json = response.json()
        # check if response_json has the expected key 'value'
        if 'value' not in response_json:
            self.logger.error(f"Response JSON missing 'value' key: {response_json}")
            return None
        # check that the battery soc value is between 0.0 and 1.0
        try:
            if not (0.0 <= float(response_json['value']) <= 1.0):
                self.logger.error(
                    f"Battery SoC value out of expected range [0.0, 1.0]: {response_json['value']}"
                )
                return None
        except (ValueError, TypeError) as e:
            return 0.5
        return float(response_json['value'])

    def map(self, robot_name: str):
        ''' Return the name of the map that the robot is currently on or
        None if any errors are encountered. '''
        return "L1"
        # TODO: check that inorbit does not return anything just internal error.
        response = self.requester.get_request(
            endpoint=f"robots/{robot_name}/maps/current"
        )
        if not response:
            self.logger.error("No response received from robot API server")
            return None
        if response.status_code != 200:
            self.logger.error(f"Unexpected status code {response.status_code} from robot API server")
            return None
        response_json = response.json()
        # check if response_json has the expected key 'label'
        if isinstance(response_json, list):
            if not response_json:
                self.logger.error("Response JSON is an empty list")
                return None
            response_json = response_json[0]
        if 'label' not in response_json:
            self.logger.error(f"Response JSON missing 'label' key: {response_json}")
            return None
            
        return response_json['label']

    def is_command_completed(self):
        ''' Return True if the robot has completed its last command, else
        return False. '''
        # TODO: launch custom actions and see if the id is returned in the response, then check status of that id to determine if command is completed
        return False

    def get_data(self, robot_name: str):
        ''' Returns a RobotUpdateData for one robot if a name is given. Otherwise
        return a list of RobotUpdateData for all robots. '''
        map = self.map(robot_name)
        position = self.position(robot_name)
        battery_soc = self.battery_soc(robot_name)
        if not (map is None or position is None or battery_soc is None):
            return RobotUpdateData(robot_name, map, position, battery_soc)
        return None


class RobotUpdateData:
    ''' Update data for a single robot. '''
    def __init__(self,
                 robot_name: str,
                 map: str,
                 position: list[float],
                 battery_soc: float,
                 requires_replan: bool | None = None):
        self.robot_name = robot_name
        self.position = position
        self.map = map
        self.battery_soc = battery_soc
        self.requires_replan = requires_replan