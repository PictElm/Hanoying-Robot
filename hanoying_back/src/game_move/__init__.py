"""
inits node 'game_move'

note: stub, does not check for unreachable, relies on simulation
through `setStringSignal("move", -)` then wait for 2s

@ros_param
    /game/move/skip_move_validation: bool
        should goal move be checked for validity?
        default: false (do NOT skip validation)
    /game/tower{name}/{x|y|z}: double
        [do_move.py] to get the destination tower's position

@ros_sub
    /game/state: hanoying_back/GameState
        used when validating a move (needed for /game/solve/valid)
        [do_move.py] to get the source disk's position

@ros_call
    /game/solve/valid: hanoying_back/GameSolveValidate
        use to check move validity (if not set to skip)

@ros_action_server
    /game/move: hanoying_back/GameMoveAction
        processes a game move and reacts accordingly by
        moving the robot arm
        ```GameMove.action'[towers of Hanoi]
        string disk # move disk..
        string tower # ..to tower
        geometry_msgs/Point floating_point # if tower=="floating",
                                           # tries to reach this position
        ---
        bool success # if success==False, refer to reason
        int8 reason # -128: aborted/preempted, 0: none, 1: goal not valid,
                    # 2: disk unreachable, 4: tower unreachable,
                    # 8: point unreachable
        ---
        float32 percent
        geometry_msgs/Transform wrist # maybe?
        ```
"""
