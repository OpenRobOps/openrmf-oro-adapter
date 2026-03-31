# oro_fleet_adapter

The objective of this package is to serve as a reference or template for writing a python based `full_control` RMF fleet adapter.

> Note: The implementation in this package is not the only way to write a `full_control` fleet adapter. It is only one such example that may be helpful for users to quickly integrate their fleets with RMF.

## Step 1: Update config.yaml
The `config.yaml` file contains important parameters for setting up the fleet adapter. There are three broad sections to this file:

1. **rmf_fleet** : containing parameters that describe the robots in this fleet
2. **robots** : containing configurations for each robot that will be controlled by this fleet adapter
3. **reference_coordinates**: containing two sets of [x,y] coordinates that correspond to the same locations but recorded in RMF (`traffic_editor`) and robot specific coordinates frames respectively. These are required to estimate coordinate transformations from one frame to another. A minimum of 4 matching waypoints is recommended.

> Note: This fleet adapter uses the `nudged` python library to compute transformations from RMF to Robot frame and vice versa. If the user is aware of the `scale`, `rotation` and `translation` values for each transform, they may modify the code in `fleet_adapter.py` to directly create the `nudged` transform objects from these values.

> Note: There is robot specific keys on the `config.yaml` file that are required for the robot. These keys can be removed and the code can be modified to work with other types of robots, this are the `robot_id`, `battery_attribute_id` and `map_attribute_id` keys this may have to be configured manually in the ORO GUI.

## Step 2: Build the package
Use the command below to build the package after filling in the code and updating the configuration file.
```bash
colcon build
source install/setup.bash
```

## Step 3: Run the fleet adapter:

Run the command below while passing the paths to the configuration file and navigation graph that this fleet operates on.

The websocket server URI should also be passed as a parameter in this command inorder to publish task statuses to the rest of the RMF entities.

```bash
#minimal required parameters
ros2 run oro_fleet_adapter fleet_adapter building_map_path:=CONFIG_FILE nav_graph_path:=NAV_GRAPH viz_config_file:=RVIZ_CONFIG_FILE

#example
ros2 launch oro_fleet_adapter fleet.andino.launch.xml building_map_path:="/home/developer/ws/install/andino_rmf_maps/share/andino_rmf_maps/maps/andino_office/andino_office.building.yaml"   nav_graph_path:="/home/developer/ws/install/andino_rmf_maps/share/andino_rmf_maps/maps/andino_office/nav_graph/0.yaml"    viz_config_file:="/home/developer/ws/install/andino_rmf_maps/share/andino_rmf_maps/rviz_config/office.rviz"

```

## Pre-commit hooks


Pre-commit is a tool that allows git's pre-commit hook integrate with various code linters and formatters.

To install `pre-commit`, run
```sh
pip install pre-commit
```

To automatically run it on each commit, from repository's root:
```sh
pre-commit install
```

And that's it! Every time you commit, `pre-commit` will trigger and let you know if everything goes well.
If the checks fail, the commit won't be created, and you'll have to fix the issue (some of them are automatically fixed by `pre-commit`), STAGE the changes, and try again.

To manually run `pre-commit` on the staged changes, one can run:
```sh
pre-commit run
```

Or to change the whole codebase
```sh
pre-commit run --all-files
```

**Note**: `pre-commit` only runs on staged changes by default.

**Note2**: To bypass `pre-commit`, use `git commit --no-verify`.

# Test

To run the tests for this package, use the command below from the root of the repository.
```bash
colcon test --packages-select oro_fleet_adapter
```

To check the test results, use the command below.
```bash
colcon test-result --verbose
```
