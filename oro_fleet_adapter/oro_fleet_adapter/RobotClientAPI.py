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


"""
The RobotAPI class is a wrapper for API calls to the robot.

Here users are expected to fill up the implementations of functions which will
be used by the RobotCommandHandle. For example, if your robot has a REST API,
you will need to make http request calls to the appropriate endpoints within
these functions.
"""
import enum
from urllib.error import HTTPError

import requests
from .Requester import Requester
from rclpy.impl.rcutils_logger import RcutilsLogger

import time
from controller_action_msg.msg import RobotPose
from controller_action_msg.action import AndinoController
from controller_action_msg.msg import RobotPose
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Quaternion
from rclpy.action import ActionClient
from rclpy.task import Future


from collections import deque
from tf_transformations import quaternion_from_euler

import math

S  = 1.0113511241464097
TH = -0.019629783198388227  # radians
TX = 17.10965533
TY = -12.94124537

_C = math.cos(TH)
_S = math.sin(TH)

def robot_to_rmf(x: float, y: float, yaw: float):
    xr = S * (_C * x - _S * y) + TX
    yr = S * (_S * x + _C * y) + TY
    yawr = yaw + TH
    return [xr, yr, yawr]

def rmf_to_robot(x: float, y: float, yaw: float):
    # inverse of similarity transform
    dx = x - TX
    dy = y - TY
    invS = 1.0 / S
    # R(-TH) applied to (dx,dy)
    xr = invS * (_C * dx + _S * dy)
    yr = invS * (-_S * dx + _C * dy)
    yawr = yaw - TH
    return [xr, yr, yawr]


class RobotAPIResult(enum.IntEnum):
    SUCCESS = 0
    """The request was successful"""

    RETRY = 1
    """The client failed to connect but might succeed if you try again"""

    IMPOSSIBLE = 2
    """The client connected but something about the request is impossible"""


class RobotAPI:
    # The constructor below accepts parameters typically required to submit
    # http requests. Users should modify the constructor as per the
    # requirements of their robot's API
    def __init__(self, node, prefix: str, timeout: float, api_key: str, battery_attribute_id: str):
        self.node = node
        self._pose_cache = {}   # robot_name -> [x,y,theta]
        self._pose_time = {}    # robot_name -> time.time()
        self._pose_subs = {}    # robot_name -> subscription
        
        action_name = f'/andino_controller'
        
        self._group1 = MutuallyExclusiveCallbackGroup()
        self.controller_client = ActionClient(self.node, AndinoController, action_name, callback_group=self._group1)

        self.prefix = prefix
        self.timeout = timeout
        self.logger = RcutilsLogger(f"RobotAPI ({prefix})")
        self.headers = {
            'Content-Type': 'application/json',
            'Accept': 'application/json',
            'x-auth-inorbit-app-key': api_key
        }
        self.battery_attribute_id = battery_attribute_id
        self.requester = Requester(
            base_url=self.prefix,
            headers=self.headers,
            timeout=self.timeout,
            logger=self.logger
        )
        self.debug = False
    
    def get_robot_id(self, robot_name: str) -> str:
        """
        Extracts the last part of a robot name split by underscore.
        Example: 'andino_123' -> '123'
        """
        return robot_name.split('_')[-1]
        
    def send_goal(self, robot_name: str, goal):
       # get a goal value
       goal = goal
       self.node.get_logger().debug(f'Goal to send: [{goal[0]}, {goal[1]}, {goal[2]}]\n')
       # create goal msg
       goal_msg = AndinoController.Goal()
       goal_msg.goal_pose.pose.position.x = goal[0]
       goal_msg.goal_pose.pose.position.y = goal[1]
       quaternion = quaternion_from_euler(0, 0, goal[2])

       orientation = Quaternion()
       orientation.x = quaternion[0]
       orientation.y = quaternion[1]
       orientation.z = quaternion[2]
       orientation.w = quaternion[3]
       goal_msg.goal_pose.pose.orientation = orientation
       # send goal async
       if not self.controller_client.server_is_ready():
           self.node.get_logger().info(f'{robot_name} controller server is not ready!')
           return
       
       self._send_goal_future = self.controller_client.send_goal_async(goal_msg)
       self._send_goal_future.add_done_callback(lambda future: self._goal_response_callback(robot_name, future))
    
    def _goal_response_callback(self, robot_name: str, future: Future):
       goal_handle = future.result()
       if not goal_handle.accepted:
           self.node.get_logger().info('Goal rejected :(')
           return
       self.node.get_logger().info('Goal accepted :)')
       self._get_result_future = goal_handle.get_result_async()
       self._get_result_future.add_done_callback(lambda future: self._get_result_callback(robot_name, future))

    def _get_result_callback(self, robot_name: str, future: Future):
       result = future.result().result
       # navigation completed successfully
       self.node.get_logger().info('Result: {0}'.format(result.success))
    
    def ensure_pose_subscription(self, robot_name: str):
        if robot_name in self._pose_subs:
            return

        topic = f"/current_pose"

        def cb(msg: RobotPose):
            if msg.current_pose is None or len(msg.current_pose) < 3:
                self.node.get_logger().warn(
                    f"Invalid current_pose from {topic}: {msg.current_pose}"
                )
                return
            self._pose_cache[robot_name] = [
                float(msg.current_pose[0]),
                float(msg.current_pose[1]),
                float(msg.current_pose[2]),
            ]
            self._pose_time[robot_name] = time.time()

        self.node.get_logger().info(f"Subscribing to pose: {topic}")
        self._pose_subs[robot_name] = self.node.create_subscription(
            RobotPose, topic, cb, 10
        )

    def check_connection(self):
        ''' Return True if connection to the robot API server is successful '''
        response = self.requester.get_request(endpoint="robots")
        if not response:
            self.logger.error("No response received from robot API server")
            return False
        return True

    def is_command_completed(self):
        ''' Return True if the robot has completed its last command, else
        return False. '''
        # TODO: launch custom actions and see if the id is returned in the response, then check status of that id to determine if command is completed
        return False
    
    def navigate(
        self,
        robot_name: str,
        cmd_id: int,
        pose,
        map_name: str,
        speed_limit=0.0,
    ):
        """
        Request the robot to navigate to pose:[x,y,theta].

        Where x, y and theta are in the robot's coordinate convention.
        This function should return True if the robot has accepted the request,
        else False.
        """
        
        ''' Request the robot to navigate to pose:[x,y,theta] where x, y and
            and theta are in the robot's coordinate convention. This function
            should return True if the robot has accepted the request,
            else False '''
        
        robot_name = self.get_robot_id(robot_name)
        robot_goal = rmf_to_robot(pose[0], pose[1], pose[2])
        print(f"Received navigation request for {robot_name} to pose {pose} on map {map_name} with speed limit {speed_limit}")
        self.send_goal(robot_name, robot_goal)
        return True
    
        request_body = {
            "waypoints": [{
                "frameId": map_name,
                "x": pose[0],
                "y": pose[1],
                "theta": pose[2],
            }]
        }
        response = self.requester.post_request(
            endpoint=f"robots/{robot_name}/navigation/waypoints",
            json=request_body
        )
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
        robot_name = self.get_robot_id(robot_name)
        # TODO: this is not implemented on inorbit api
        return False

    def start_activity(
        self, robot_name: str, cmd_id: int, activity: str, label: str
    ):
        """
        Request the robot to begin a process.

        This is specific to the robot and the use case.
        For example, load/unload a cart for Deliverybot
        or begin cleaning a zone for a cleaning robot.
        """
        robot_name = self.get_robot_id(robot_name)
        action_body = {
            "actionId": activity,
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

    def stop(self, robot_name: str, running_cmd_id: int, stop_cmd_id: int):
        ''' Command the robot to stop.
            Return True if robot has successfully stopped. Else False. '''
        robot_name = self.get_robot_id(robot_name)
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
        robot_name = self.get_robot_id(robot_name)
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
        return [float(response_json['x']), float(response_json['y']), float(response_json['theta'])]

    def battery_soc(self, robot_name: str):
        ''' Return the state of charge of the robot as a value between 0.0
        and 1.0. Else return None if any errors are encountered. '''
        robot_name = self.get_robot_id(robot_name)
        attribute_id = self.battery_attribute_id
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
        if response_json['value'] == '':
            return 0.5
        if not (0.0 <= float(response_json['value']) <= 1.0):
            self.logger.error(
                f"Battery SoC value out of expected range [0.0, 1.0]: {response_json['value']}"
            )
            return None
        return float(response_json['value'])

    def map(self, robot_name: str):
        ''' Return the name of the map that the robot is currently on or
        None if any errors are encountered. '''
        robot_name = self.get_robot_id(robot_name)
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

    def toggle_teleop(self, robot_name: str, toggle: bool):
        """
        Request to toggle the robot's mode_teleop parameter.

        Return True if the toggle request is successful
        """
        url = (
            self.prefix
            + f'/open-rmf/rmf_demos_fm/toggle_teleop?robot_name={robot_name}'
        )
        data = {'toggle': toggle}
        try:
            response = requests.post(url, timeout=self.timeout, json=data)
            response.raise_for_status()
            if self.debug:
                print(f'Response: {response.json()}')
            return response.json()['success']
        except HTTPError as http_err:
            print(f'HTTP error for {robot_name} in toggle_teleop: {http_err}')
        except Exception as err:
            print(f'Other error {robot_name} in toggle_teleop: {err}')
        return False

    def toggle_attach(self, robot_name: str, attach: bool, cmd_id: int):
        """
        Request to attach or detach robot to/from cart.

        Return True if the attach request is successful
        """
        url = (
            self.prefix
            + f'/open-rmf/rmf_demos_fm/toggle_attach?robot_name={robot_name}'
            f'&cmd_id={cmd_id}'
        )
        data = {'toggle': attach}
        try:
            response = requests.post(url, timeout=self.timeout, json=data)
            response.raise_for_status()
            if self.debug:
                print(f'Response: {response.json()}')
            return response.json()['success']
        except HTTPError as http_err:
            print(f'HTTP error for {robot_name} in toggle_attach: {http_err}')
        except Exception as err:
            print(f'Other error {robot_name} in toggle_attach: {err}')
        return False

    def get_data(self, robot_name: str | None = None):
        """
        Return a RobotUpdateData for one robot if a name is given.

        Otherwise return a list of RobotUpdateData for all robots.
        """
        map = self.map(robot_name)
        position = self.position(robot_name)
        battery_soc = self.battery_soc(robot_name)
        if not (map is None or position is None or battery_soc is None):
            return RobotUpdateData(robot_name, map, position, battery_soc)
        return None

class RobotUpdateData:
    """Update data for a single robot."""

    def __init__(self, 
                 robot_name: str,
                 map: str,
                 position: list[float],
                 battery_soc: float,
                 requires_replan: bool = False,
                 last_completed_request: int = 0,
                 ):
        self.robot_name = robot_name
        x = position[0]
        y = position[1]
        yaw = position[2]
        self.position = [x, y, yaw]
        self.map = map
        self.battery_soc = battery_soc
        self.requires_replan = requires_replan
        self.last_request_completed = last_completed_request

    def is_command_completed(self, cmd_id):
        return self.last_request_completed == cmd_id