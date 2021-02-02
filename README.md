# The Hanoying Robot

This ROS project presents 2 packages (`hanoying_back`, `hanoying_front`).

The setup as-is was intended for the game "Towers of Hanoi", but should be adaptable (see [here](#Changing%20the%20game))

## roslaunch _that_

To run the setup:
  1. you must have a ROS master running at all time (`roscore`) and source your `/opt/ros/${version}/setup.bash` from every terminal
  0. start coppeliaSim and ensure both ROSInterface and RemoteApi plugin are loaded successfully
  0. add the Python binding of the RemoteApi `sim.py` to $PYTHONPATH (it should be in `${coppeliaSim root dir}/programming/remoteApiBindings/python`)
  0. (make sure to have the `remoteApi.so` in the same folder as `sim.py`, if it's not, copy it over from the `/remoteApiBindings/lib/lib`)
  0. build your catkin_workspace and source everything you can to save you sanity, or just:
    * `cd ~/catkin_ws`
    * `catkin_make`
    * `source devel/setup.bash`
  0. start the nodes from the `hanoying_back` package first: `roslaunch hanoying_back back.launch`
  0. start the nodes from the `hanoying_front` package with `roslaunch hanoying_font font.launch`

> Note: it should behave fine even if `hanoying_back`'s nodes are started after, or restarted at anytime.

## ROS objects list

The parameters, topics, services and action servers are all provided by the hanoying_back package, or coppeliaSim.

### Nodes (see dependencies [here](#Dependencies))
  - `hanoying_back`
    - `game_state.py`
    - `game_solve.py`
    - `game_move_server.py`
    - `decision_system.py`
  - `hanoying_front`
    - `ctrl.py`
    - `gui.py`

### Parameters
  - `/game/allowed_disk_distance: double`
  - `/game/towers: string`
  - `/game/disks: string`
  - `/game/tower{name}/{x|y|z}: double`

### Topics
  - `/game/raw/disk{name}: geometry_msgs/Point` (published from coppeliaSim for message compatibility reasons)
  - `/game/state: hanoying_back/GameState`
  - `/decisys/decision: hanoying_back/GameMoveGoal`
  - `/decisys/solution: hanoying_back/GameSolveResponse`

### Services
  - `/game/solve: hanoying_back/GameSolve`
  - `/game/solve/valid: hanoying_back/GameSolveValidate`
  - `/game/solve/possible: hanoying_back/GameSolveList`

### Action servers
  - `/game/move: hanoying_back/GameMoveAction`

## Dependencies

### `game_state.py`
  - `/game/allowed_disk_distance`
  - `/game/towers`
  - `/game/disks`
  - `/game/tower{name}/{x|y|z}`
  - `/game/raw/disk{name}`

### `game_solve.py`
  - &dash;

### `game_move_server.py`
  - `/game/towers`
  - `/game/disks`

### `decision_system.py`
  - `/game/state`
  - `/game/solve`

### `ctrl.py`
  - `/game/move`

### `gui.py`
  - `/game/state`
  - `/decisys/decision`
  - `/decisys/solution`

## Changing the game

The setup should allow to implement some other games. To do exactly that, the following must be updated accordingly;

In the `hanoying_back` package:
  - `msg/GameState.msg` must be able to reflect any state of the game
  - `action/GameMove.action` must be able to reflect any move in the game [^1]
  - if the previous points require adding custom messages, update `CMakeLists.txt` to include them
  - `src/game_state.py` must be able to provide the current state of the game (publish through `/game/state` as a `GameState`) from raw sensors
  - `src/game_move_server/do_move.py` must enable to execute any given `GameMove` [^2]
  - `src/game_solve/solver.py` must be able to solve the game from any `GameState`, resulting in a (potentially empty) `GameMoveGoal[]`
  - `src/game_solve/rules.py` should be able to validate a move and list every possible moves from a `GameState`

In the `hanoying_front` package:
  - `src/ctrl/ctrl.py` (see comments in file, mainly used for debugging RN...)
  - `src/gui/gui.py` sub to `/game/state`, `/decisys/*` and whatever else you want, make yourself at home

[^1]: In the action file, only need to be updated the goal (first part) and feedback (last part) as needed; `reason` should carry custom flags to interpret why a move failed (only the bit 8 is reserved: -128 aborted/preempted).

[^2]: This node is only expected to do the move, regardless of validity regarding game rules (only failing when the move is not possible); for move validity check against game rules, see `game_solve/rules.py`.
