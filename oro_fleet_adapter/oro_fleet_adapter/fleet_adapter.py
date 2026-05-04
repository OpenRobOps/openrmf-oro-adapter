# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0

import argparse
import asyncio
from datetime import datetime, timezone
import faulthandler
import math
import sys
import threading
import time

import nudged
import rclpy
from rclpy.duration import Duration
import rclpy.node
from rclpy.parameter import Parameter
import rmf_adapter
from rmf_adapter import Adapter, Transformation
import rmf_adapter.easy_full_control as rmf_easy
import yaml

from .RobotClientAPI import RobotAPI


# ------------------------------------------------------------------------------
# Helper functions
# ------------------------------------------------------------------------------
def compute_transforms(level, coords, node=None):
    """Get transforms between RMF and robot coordinates."""
    rmf_coords = coords['rmf']
    robot_coords = coords['robot']
    tf = nudged.estimate(rmf_coords, robot_coords)

    if node:
        mse = nudged.estimate_error(tf, rmf_coords, robot_coords)
        node.get_logger().info(f'Transformation error estimate for {level}: {mse}')

    return Transformation(
        tf.get_rotation(),
        tf.get_scale(),
        tf.get_translation(),
    )


def parse_last_online(last_online: str, node=None) -> datetime | None:
    """Parse the last_online timestamp returned by the robot discovery API."""
    try:
        if last_online.endswith('Z'):
            last_online = last_online.replace('Z', '+00:00')

        parsed = datetime.fromisoformat(last_online)

        if parsed.tzinfo is None:
            parsed = parsed.replace(tzinfo=timezone.utc)

        return parsed.astimezone(timezone.utc)

    except Exception as e:
        if node:
            node.get_logger().error(f'Failed to parse last_online [{last_online}]: {e}')
        return None


def is_robot_stale(
    robot_config: dict,
    offline_timeout_sec: float,
    node=None,
) -> bool:
    """Return True if the robot last_online value is older than the timeout."""
    last_online = parse_last_online(robot_config['last_online'], node)

    if last_online is None:
        return True

    now = datetime.now(timezone.utc)
    offline_duration = (now - last_online).total_seconds()

    return offline_duration > offline_timeout_sec


# ------------------------------------------------------------------------------
# Main
# ------------------------------------------------------------------------------
def main(argv=sys.argv):
    faulthandler.enable()

    rclpy.init(args=argv)
    rmf_adapter.init_rclcpp()
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(
        prog='fleet_adapter',
        description='Configure and spin up the fleet adapter',
    )
    parser.add_argument(
        '-c',
        '--config_file',
        type=str,
        required=True,
        help='Path to the config.yaml file',
    )
    parser.add_argument(
        '-n',
        '--nav_graph',
        type=str,
        required=True,
        help='Path to the nav_graph for this fleet adapter',
    )
    parser.add_argument(
        '-sim',
        '--use_sim_time',
        action='store_true',
        help='Use sim time, default: false',
    )

    args = parser.parse_args(args_without_ros[1:])
    print('Starting fleet adapter...')

    config_path = args.config_file
    nav_graph_path = args.nav_graph

    fleet_config = rmf_easy.FleetConfiguration.from_config_files(
        config_path,
        nav_graph_path,
    )
    assert fleet_config, f'Failed to parse config file [{config_path}]'

    with open(config_path, 'r') as f:
        config_yaml = yaml.safe_load(f)

    fleet_name = fleet_config.fleet_name
    node = rclpy.node.Node(f'{fleet_name}_command_handle')

    adapter = Adapter.make(f'{fleet_name}_fleet_adapter')
    assert adapter, (
        'Unable to initialize fleet adapter. ' 'Please ensure RMF Schedule Node is running'
    )

    if args.use_sim_time:
        param = Parameter('use_sim_time', Parameter.Type.BOOL, True)
        node.set_parameters([param])
        adapter.node.use_sim_time()

    adapter.start()
    time.sleep(1.0)

    node.declare_parameter('server_uri', '')
    server_uri = node.get_parameter('server_uri').get_parameter_value().string_value
    if server_uri == '':
        server_uri = None

    fleet_config.server_uri = server_uri

    for level, coords in config_yaml['reference_coordinates'].items():
        tf = compute_transforms(level, coords, node)
        fleet_config.add_robot_coordinates_transformation(level, tf)

    fleet_handle = adapter.add_easy_fleet(fleet_config)
    fleet_handle.more().set_planner_cache_reset_size(2500)

    fleet_manager_yaml = config_yaml['fleet_manager']

    update_period = 1.0 / fleet_manager_yaml.get(
        'robot_state_update_frequency',
        10.0,
    )
    robot_discovery_period = fleet_manager_yaml.get(
        'robot_discovery_period',
        5.0,
    )
    robot_offline_timeout_sec = fleet_manager_yaml.get(
        'robot_offline_timeout_sec',
        60.0,
    )

    robots = {}

    robot_discovery_api = RobotAPI(
        prefix=fleet_manager_yaml['prefix'],
        timeout=fleet_manager_yaml['timeout'],
        api_key=fleet_manager_yaml['api_key'],
        robot_id='fleet_discovery',
        battery_attribute_id='',
        map_attribute_id='',
    )

    def get_template_robot_configuration(robot_name: str):
        if robot_name in fleet_config.known_robots:
            return fleet_config.get_known_robot_configuration(robot_name)

        if not fleet_config.known_robots:
            node.get_logger().error(
                f'Cannot dynamically add robot [{robot_name}] because there '
                'are no template robots in the RMF fleet config.'
            )
            return None

        template_robot_name = next(iter(fleet_config.known_robots))
        node.get_logger().warn(
            f'Robot [{robot_name}] is not declared in the RMF config. '
            f'Using [{template_robot_name}] as the RMF robot configuration '
            'template.'
        )

        return fleet_config.get_known_robot_configuration(template_robot_name)

    def add_robot_from_config(robot_config: dict):
        robot_id = robot_config['robot_id']
        robot_name = robot_config.get('name', robot_id)

        last_online = parse_last_online(robot_config['last_online'], node)
        robot_is_stale = is_robot_stale(
            robot_config,
            robot_offline_timeout_sec,
            node,
        )

        if robot_name in robots:
            robot = robots[robot_name]
            robot.last_online = last_online
            robot.max_delay = robot_config.get('max_delay', robot.max_delay)
            robot.offline_timeout_sec = robot_offline_timeout_sec

            if robot_is_stale:
                robot.decommission(
                    f'last_online is older than ' f'{robot_offline_timeout_sec} seconds'
                )
            else:
                robot.recommission()

            return

        rmf_robot_configuration = get_template_robot_configuration(robot_name)
        if rmf_robot_configuration is None:
            return

        robot_api = RobotAPI(
            prefix=fleet_manager_yaml['prefix'],
            timeout=fleet_manager_yaml['timeout'],
            api_key=fleet_manager_yaml['api_key'],
            robot_id=robot_id,
            battery_attribute_id=robot_config['battery_attribute_id'],
            map_attribute_id=robot_config['map_attribute_id'],
        )

        robot = RobotAdapter(
            name=robot_name,
            configuration=rmf_robot_configuration,
            node=node,
            api=robot_api,
            fleet_handle=fleet_handle,
            localization_tolerance=robot_config.get('localization_tolerance', 0.3),
            max_delay=robot_config.get('max_delay', None),
            offline_timeout_sec=robot_offline_timeout_sec,
        )

        robot.last_online = last_online
        robots[robot_name] = robot

        node.get_logger().info(
            f'Dynamically registered robot [{robot_name}] ' f'with robot_id [{robot_id}]'
        )

        if robot_is_stale:
            node.get_logger().warn(
                f'Robot [{robot_name}] was discovered but is already stale. '
                'It will be decommissioned after RMF creates its update handle.'
            )

    def discover_and_add_robots():
        robot_configs = robot_discovery_api.get_online_robot_configs()
        discovered_robot_names = set()

        for robot_config in robot_configs:
            robot_name = robot_config.get('name', robot_config['robot_id'])
            discovered_robot_names.add(robot_name)
            add_robot_from_config(robot_config)

        for robot_name, robot in list(robots.items()):
            if robot_name not in discovered_robot_names:
                robot.decommission('robot is missing from discovery endpoint response')

    discover_and_add_robots()
    node.get_logger().info(f'Initialized APIs for robots: {list(robots.keys())}')

    def update_loop():
        reassign_task_interval = config_yaml['rmf_fleet'].get(
            'reassign_task_interval',
            60,
        )

        last_task_replan = node.get_clock().now()
        last_robot_discovery = node.get_clock().now()

        asyncio.set_event_loop(asyncio.new_event_loop())

        while rclpy.ok():
            now = node.get_clock().now()

            discovery_interval_sec = (now.nanoseconds - last_robot_discovery.nanoseconds) / 1e9

            if discovery_interval_sec > robot_discovery_period:
                discover_and_add_robots()
                last_robot_discovery = now

            update_jobs = []
            for robot in list(robots.values()):
                update_jobs.append(update_robot(robot))

            if update_jobs:
                done, _pending = asyncio.get_event_loop().run_until_complete(
                    asyncio.wait(update_jobs)
                )

                for task in done:
                    exc = task.exception()
                    if exc is not None:
                        node.get_logger().error(f'Update task failed: {exc}')

            interval_sec = (now.nanoseconds - last_task_replan.nanoseconds) / 1e9

            if interval_sec > reassign_task_interval:
                fleet_handle.more().reassign_dispatched_tasks()
                last_task_replan = now

            next_wakeup = now + Duration(nanoseconds=update_period * 1e9)
            while node.get_clock().now() < next_wakeup:
                time.sleep(0.001)

    update_thread = threading.Thread(target=update_loop, args=())
    update_thread.start()

    rclpy_executor = rclpy.executors.SingleThreadedExecutor()
    rclpy_executor.add_node(node)

    rclpy_executor.spin()

    node.destroy_node()
    rclpy_executor.shutdown()
    rclpy.shutdown()


class RobotAdapter:
    def __init__(
        self,
        name: str,
        configuration,
        node,
        api: RobotAPI,
        fleet_handle,
        localization_tolerance=0.3,
        max_delay: float | None = None,
        offline_timeout_sec: float = 60.0,
    ):
        self.name = name
        self.execution = None
        self.update_handle = None
        self.configuration = configuration
        self.node = node
        self.api = api
        self.fleet_handle = fleet_handle
        self.override = None
        self.issue_cmd_thread = None
        self.cancel_cmd_event = threading.Event()
        self.target_position = None
        self.localization_tolerance = localization_tolerance

        self.last_online = None
        self.max_delay = max_delay
        self.offline_timeout_sec = offline_timeout_sec
        self.is_decommissioned = False

    def is_navigation_within_tolerance(self, state):
        if self.target_position is None:
            return False

        dx = state.position[0] - self.target_position[0]
        dy = state.position[1] - self.target_position[1]
        dist = math.sqrt(dx**2 + dy**2)

        return dist < self.localization_tolerance

    def update(self, state, robot_name):
        activity_identifier = None

        if self.execution:
            is_finished = False

            if self.target_position is not None and self.is_navigation_within_tolerance(state):
                is_finished = True
                self.target_position = None

            elif self.api.current_action is not None and self.api.is_command_completed():
                is_finished = True

            if is_finished:
                self.execution.finished()
                self.execution = None
            else:
                activity_identifier = self.execution.identifier

        self.update_handle.update(state, activity_identifier)

    def make_callbacks(self):
        callbacks = rmf_easy.RobotCallbacks(
            lambda destination, execution: self.navigate(destination, execution),
            lambda activity: self.stop(activity),
            lambda category, description, execution: self.execute_action(
                category,
                description,
                execution,
            ),
        )

        callbacks.localize = lambda estimate, execution: self.localize(
            estimate,
            execution,
        )

        return callbacks

    def localize(self, estimate, execution):
        self.node.get_logger().info(f'Commanding [{self.name}] to change map to [{estimate.map}]')

        if self.api.localize(estimate.position, estimate.map):
            self.node.get_logger().info(
                f'Localized [{self.name}] on {estimate.map} ' f'at position [{estimate.position}]'
            )
            execution.finished()
        else:
            self.node.get_logger().warn(
                f'Failed to localize [{self.name}] on {estimate.map} '
                f'at position [{estimate.position}]. Requesting replanning...'
            )
            if self.update_handle is not None and self.update_handle.more() is not None:
                self.update_handle.more().replan()

    def navigate(self, destination, execution):
        if self.is_decommissioned:
            error_msg = f'Robot [{self.name}] is decommissioned and cannot navigate'
            self.node.get_logger().error(error_msg)
            execution.error(error_msg)
            return

        self.execution = execution
        self.target_position = destination.position

        self.node.get_logger().info(
            f'Commanding [{self.name}] to navigate to {destination.position} '
            f'on map [{destination.map}]'
        )

        accepted = self.api.navigate(
            destination.position,
            destination.map,
            destination.speed_limit,
        )

        if not accepted:
            error_msg = (
                f'Robot [{self.name}] rejected navigation command to '
                f'{destination.position} on map [{destination.map}]'
            )
            self.node.get_logger().error(error_msg)
            execution.error(error_msg)
            self.execution = None
            self.target_position = None

    def stop(self, activity):
        if self.execution is not None:
            if self.execution.identifier.is_same(activity):
                self.execution = None
                self.target_position = None
                self.api.stop()

    def execute_action(self, category: str, description: dict, execution):
        if self.is_decommissioned:
            error_msg = (
                f'Robot [{self.name}] is decommissioned and cannot execute action ' f'[{category}]'
            )
            self.node.get_logger().error(error_msg)
            execution.error(error_msg)
            return

        self.execution = execution

        match category:
            case 'inorbit':
                self.node.get_logger().info(
                    f"Executing 'inorbit' action for robot '{self.name}' "
                    f'with description: {description}'
                )

                try:
                    success = self.api.start_activity(
                        activity=description.get('action_id', None),
                        label=description.get('label', None),
                        activity_args=description.get('action_args', None),
                    )

                    if not success:
                        error_msg = (
                            f'Failed to start InOrbit action for robot '
                            f"'{self.name}'. Description: {description}"
                        )
                        self.node.get_logger().error(error_msg)
                        execution.error(error_msg)
                        self.execution = None
                        return

                except Exception as e:
                    error_msg = f'Exception during InOrbit start_activity: {e!s}'
                    self.node.get_logger().error(error_msg)
                    execution.error(error_msg)
                    self.execution = None
                    return

            case _:
                error_msg = (
                    f"Unsupported action category '{category}' for robot "
                    f"'{self.name}'. Description: {description}"
                )
                self.node.get_logger().error(error_msg)
                execution.error(error_msg)
                self.execution = None
                return

    def finish_action(self):
        if self.execution is not None:
            self.execution.finished()
            self.execution = None

    def apply_maximum_delay(self):
        if self.max_delay is None:
            return

        if self.update_handle is None:
            return

        try:
            self.update_handle.more().maximum_delay(Duration(seconds=float(self.max_delay)))
        except Exception as e:
            self.node.get_logger().warn(
                f'Could not apply maximum_delay={self.max_delay} ' f'to robot [{self.name}]: {e}'
            )

    def decommission(self, reason: str):
        if self.is_decommissioned:
            return

        if self.update_handle is None:
            self.node.get_logger().warn(
                f'Cannot decommission [{self.name}] yet because it has no ' 'update_handle'
            )
            return

        self.node.get_logger().warn(f'Decommissioning robot [{self.name}]: {reason}')

        try:
            more = self.update_handle.more()

            if hasattr(more, 'set_commission'):
                commission_type = type(more.commission())
                more.set_commission(commission_type.decommission())
            elif hasattr(more, 'unstable'):
                more.unstable().decommission()
            else:
                self.node.get_logger().error(
                    f'Robot [{self.name}] update handle does not expose a '
                    'commission/decommission API'
                )
                return

            more.reassign_dispatched_tasks()

            if self.execution is not None:
                self.execution.error(f'Robot [{self.name}] was decommissioned: {reason}')
                self.execution = None
                self.target_position = None

            self.api.stop()
            self.is_decommissioned = True

        except Exception as e:
            self.node.get_logger().error(f'Failed to decommission robot [{self.name}]: {e}')

    def recommission(self):
        if not self.is_decommissioned:
            return

        if self.update_handle is None:
            self.node.get_logger().warn(
                f'Cannot recommission [{self.name}] yet because it has no ' 'update_handle'
            )
            return

        self.node.get_logger().info(f'Recommissioning robot [{self.name}]')

        try:
            more = self.update_handle.more()

            if hasattr(more, 'set_commission'):
                commission_type = type(more.commission())
                more.set_commission(commission_type())
            elif hasattr(more, 'unstable'):
                more.unstable().recommission()
            else:
                self.node.get_logger().error(
                    f'Robot [{self.name}] update handle does not expose a '
                    'commission/recommission API'
                )
                return

            self.is_decommissioned = False

        except Exception as e:
            self.node.get_logger().error(f'Failed to recommission robot [{self.name}]: {e}')


# Parallel processing solution derived from
# https://stackoverflow.com/a/59385935
def parallel(f):
    def run_in_parallel(*args, **kwargs):
        return asyncio.get_event_loop().run_in_executor(None, f, *args, **kwargs)

    return run_in_parallel


@parallel
def update_robot(robot: RobotAdapter):
    try:
        data = robot.api.get_data(robot.name)

        if data is None:
            robot.node.get_logger().warn(f'No data received for robot [{robot.name}]')
            return

        state = rmf_easy.RobotState(
            data.current_map,
            data.position,
            data.battery_soc,
        )

        if robot.update_handle is None:
            robot.update_handle = robot.fleet_handle.add_robot(
                robot.name,
                state,
                robot.configuration,
                robot.make_callbacks(),
            )

            robot.apply_maximum_delay()

            if robot.last_online is not None:
                offline_duration = (datetime.now(timezone.utc) - robot.last_online).total_seconds()

                if offline_duration > robot.offline_timeout_sec:
                    robot.decommission('robot was stale when first added to RMF')

            return

        robot.update(state, robot.name)

    except Exception as e:
        robot.node.get_logger().error(f'Failed updating robot [{robot.name}]: {e}')


if __name__ == '__main__':
    main(sys.argv)
