Reminder: the parameters, topics, services and action servers are all provided by the `hanoying_back` package, or CoppeliaSim; so the associated types (msg, srv and action) are also part of `hanoying_back`.

**TODO** some readable dependency graph.

## Types

### Messages (from hanoying_back.msg)
  - [`hanoying_back/GameRaw`](../hanoying_back/msg/GameRaw.msg)
  - [`hanoying_back/Disk`](../hanoying_back/msg/Disk.msg)
  - [`hanoying_back/Tower`](../hanoying_back/msg/Tower.msg)
  - [`hanoying_back/GameState`](../hanoying_back/msg/GameState.msg)

### Services (from hanoying_back.srv)
  - [`hanoying_back/GameSolve`](../hanoying_back/srv/GameSolve.srv)
    - `hanoying_back/GameSolve`
    - `hanoying_back/GameSolveResponse`
  - [`hanoying_back/GameSolveValidate`](../hanoying_back/srv/GameSolveValidate.srv)
    - `hanoying_back/GameSolveValidate`
    - `hanoying_back/GameSolveValidateResponse`
  - [`hanoying_back/GameSolveList`](../hanoying_back/srv/GameSolveList.srv)
    - `hanoying_back/GameSolveList`
    - `hanoying_back/GameSolveListResponse`

### Action servers
  - [`hanoying_back/GameMoveAction`](../hanoying_back/action/GameMove.action)
    - `hanoying_back/GameMoveGoal`
    - `hanoying_back/GameMoveResult`
    - `hanoying_back/GameMoveFeedback`

## Objects

### Nodes (see dependencies [here](#Dependencies))
  - `hanoying_back`
    - [`sim_to_raw.py`](../hanoying_back/src/sim_to_raw/sim_to_raw.py) (to be removed)
    - [`game_state.py`](../hanoying_back/src/game_state/game_state.py)
    - [`game_solve.py`](../hanoying_back/src/game_solve/game_solve.py)
    - [`game_move.py`](../hanoying_back/src/game_move/game_move.py)
    - [`decision_system.py`](../hanoying_back/src/decision_system/decision_system.py)
  - `hanoying_front`
    - [`ctrl.py`](../hanoying_front/src/ctrl/ctrl.py)
    - [`gui.py`](../hanoying_front/src/gui/gui.py)

### Parameters
  - `/decisys/rate: double`
  - `/game/move/skip_move_validation: bool`

  - `/game/allowed_disk_distance: double`
  - `/game/towers: string`
  - `/game/disks: string`
  - `/game/tower{name}/{x|y|z}: double`

### Topics
  - `/game/sim/disk{name}: geometry_msgs/Point` (published from CoppeliaSim for compatibility reasons)

  - `/game/raw: hanoying_back/GameRaw`
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

### `sim_to_raw.py`
  - `/game/disks`
  - `/game/sim/disk{name}`

### `game_state.py`
  - `/game/allowed_disk_distance` (defaults to 0.1m)
  - `/game/towers`
  - `/game/tower{name}/{x|y|z}`
  - `/game/raw`

### `game_solve.py`
  - &dash;

### `game_move.py`
  - `/game/move/skip_move_validation` (default to false)
  - `/game/tower{name}/{x|y|z}`
  - `/game/state`

### `decision_system.py`
  - `/decisys/rate` (defaults to 3hz)
  - `/game/state`
  - `/game/solve`

### `ctrl.py`
  - `/game/move`

### `gui.py`
  - `/game/state`
  - `/decisys/decision`
  - `/decisys/solution`
