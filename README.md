# The Hanoying Robot

This ROS project presents 2 packages (`hanoying_back`, `hanoying_front`).

The setup as-is was intended for the game "Towers of Hanoi", but should be adaptable (see [there](#Changing%20the%20game))

This version relies on a simulator (CoppeliaSim) but can be adapted to work with a real setup (see [here](#Adapting%20to%20a%20real%20setup))

## roslaunch _that_

To run the setup:
  1. you must have a ROS master running at all time (`roscore`) and source your `/opt/ros/${version}/setup.bash` from every terminal
  0. start CoppeliaSim and ensure both ROSInterface and RemoteApi plugin are loaded successfully
  0. add the Python binding of the RemoteApi `sim.py` to $PYTHONPATH (it should be in `${CoppeliaSim root dir}/programming/remoteApiBindings/python`)
  0. (make sure to have the `remoteApi.so` in the same folder as `sim.py`, if it's not, copy it over from the `/remoteApiBindings/lib/lib`)
  0. build your catkin_workspace and source everything you can to save you sanity, or just:
    * `cd ~/catkin_ws`
    * `catkin_make`
    * `source devel/setup.bash`
  0. start the nodes from the `hanoying_back` package first (or use `roslaunch hanoying_back back.launch`)
  0. then start the nodes from the `hanoying_front` package (or use `roslaunch hanoying_front front.launch`)

> Note: it should behave fine even if `hanoying_back`'s nodes are started after, or restarted at anytime.

**TODO** a "healthy system" graph (ros_graph).

## ROS objects list

The parameters, topics, services and action servers are all provided by the `hanoying_back` package, or CoppeliaSim. So the assiciated types (msg, srv and action) are also part of `hanoying_back`.

> List was getting pretty long, so I moved it to [another file](OBJ_LIST.md).

## Making the robot smart

The behavior of the robot should be implemented in the `decision_system` node (for now, it just finds a solution from the current state and intend to do the first move).

## Adapting to a real setup

Because this relies directly on the simulator, a few changes will need to be made before using on a real (physical) setup;

The `sim_to_raw` node have for objective to fake the simulation's capture as raw input to the hanoying_back (this hack was made to go around recompiling the ROSInterface to account for new message types). This is the node to replace with a 'real' one that would publish to `/game/raw`, as well as updating the `GameRaw.msg` with whatever the sensors being used deliver.

Example:
```GameRaw.msg
sensor_msgs/Image cam1
sensor_msgs/Image cam2
double weights[]
```

Another point will be to update the `game_state/process.py` which must process a raw `GameRaw` message into a usable `GameState` message.

Finally, the control of the robot needs to be adapted in the `game_move/do_move.py`.

## Changing the game

The setup should allow to implement some other games. To do exactly that, the following must be updated accordingly;

In the `hanoying_back` package:
  - `msg/GameRaw.msg` must be able to carry every sensory input you need to evaluate game state
  - `msg/GameState.msg` must be able to reflect any state of the game
  - `action/GameMove.action` must be able to reflect any move in the game [^1]
  - if the previous points require adding custom messages, update `CMakeLists.txt` to include them
  - `src/game_state/process.py` must be able to provide game state  as a `GameState` from a `GameRaw`
  - `src/game_move/do_move.py` must enable to execute any given `GameMove` [^2]
  - `src/game_solve/solver.py` must be able to solve the game from any `GameState`, resulting in a (potentially empty) `GameMoveGoal[]` [^3]
  - `src/game_solve/rules.py` should be able to validate a move (results in `bool`) and list every possible moves from a `GameState` (results in `GameMoveGoal[]`)

In the `hanoying_front` package:
  - `src/ctrl/ctrl.py` (see comments in file, mainly used for debugging RN...)
  - `src/gui/gui.py` sub to `/game/state`, `/decisys/*` and whatever else you want, make yourself at home

[^1]: In the action file, only needs to be updated the goal (first part) and feedback (last part) as needed; `reason` should carry custom flags to interpret why a move failed (only the bit 8 is reserved: -128 aborted/preempted).

[^2]: This node is only expected to do the move, regardless of validity regarding game rules (only failing when the move is not possible); for move validity check against game rules, see `game_solve/rules.py`.

[^3]: `GameMoveGoal` correspond to the first part of `GameMove.action`, see [this list](OBJ_LIST.md#Types) for more details list of the added ROS types.
