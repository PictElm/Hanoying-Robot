**TODO** some readable dependency graph.

### Nodes (see dependencies [here](#Dependencies))
  - `hanoying_back`
    - `sim_to_raw.py` (to be removed)
    - `game_state.py`
    - `game_solve.py`
    - `game_move.py`
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
  - `/game/sim/disk{name}: geometry_msgs/Point` (published from CoppeliaSim for message compatibility reasons)
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
  - `/game/allowed_disk_distance`
  - `/game/towers`
  - `/game/tower{name}/{x|y|z}`
  - `/game/raw`

### `game_solve.py`
  - &dash;

### `game_move.py`
  - `/game/towers`
  - `/game/disks`
  - `/game/state`

### `decision_system.py`
  - `/game/state`
  - `/game/solve`

### `ctrl.py`
  - `/game/move`

### `gui.py`
  - `/game/state`
  - `/decisys/decision`
  - `/decisys/solution`
