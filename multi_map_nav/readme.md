# Multi Map Navigation

This package implements a multi-map navigation system in ROS, enabling a robot to navigate between separately mapped rooms using a wormhole mechanism. Each wormhole defines a logical exit from one map to another, and a local SQLite database stores their positions. The system uses move_base with the NavfnROS global planner for path planning and navigation.

- Each map in the maps/ directory corresponds to a different physical room (map1, map2, etc.).

- Wormholes are logical transitions between maps (e.g., doorways), stored in database/wormholes.db.

- The robot can be given a goal defined by a target map and coordinates.

- If the goal lies in another map, the robot first navigates to the wormhole and then switches maps to continue navigation.

## Usage
1. Start the robot: Launch you robot and its navigation stack.
2. Launch the multi-map system. This starts the action server, loads the wormholes database and available maps.
```bash
roslaunch multi_map_nav multi_map_nav.launch
```
3. Send a goal using `set_action`. Provide the target map name, target x, and target y coordinates.
```bash
rosrun multi_map_nav set_action map3 -4 6
```

## Code Documentation
`NavigatetoGoal.action`: 
Defines the action interface used by the client and server for sending goals across maps.
```
# action/NavigateToGoal.action
float64 target_x
float64 target_y
string target_map
---
bool success
string message
---
string feedback_msg

```
`Wormhole_Manager.cpp` : 
- Handles interaction with the wormhole database.
- Opens the SQLite database located at database/wormholes.db.
- Provides the method getWormholeToMap(map_name), which:
    - Executes an SQL query.
    - Returns the coordinates of the wormhole that leads to the specified target map.

`Map_Switcher.cpp` :
Handles switching between maps by launching a new map_server.
    - Loads the map_folder.
    - switchToMap(map_name) checks if the YAML file exists and is valid.
    - If valid, runs rosrun map_server in the background.
    - Logs success or error based on the result.

`NavigationServer.cpp` :
Implements the action server that handles multi-map navigation.
- Starts an action server for the navigate_to_goal action.
- If the goal is on the current map, the robot navigates directly using move_base.
- If on a different map, the server:
    - Finds a wormhole via WormholeManager.
    - Navigates to the wormhole, switches maps, and continues to the goal.
    - Falls back to a two-step transition via map1 if no direct wormhole exists.
- Uses MapSwitcher to change maps and manages navigation flow and result feedback.

`set_Action.cpp` :
A command-line ROS action client for sending navigation goals.
- Takes `<map_name> <target_x> <target_y>` as arguments.
- Sends a goal to the navigate_to_goal action server.
- Prints the result or failure message based on the action outcome.



