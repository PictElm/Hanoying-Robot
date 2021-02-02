"""
inits node 'game_move'

note: stub, does not check for unreachable, relies on simulation
through `setStringSignal("move", -)` then wait for 2s

@ros_param
    /game/towers: string
        ;-separated list of tower names
        (used to check is move is physically possible BUT PROBABLY IT SHOULDN'T)
    /game/disks: string
        ;-separated list of disk names
        (used to check is move is physically possible BUT PROBABLY IT SHOULDN'T)
    /game/tower{name}/{x|y|z}: double
        to get the destination tower's position

@ros_sub
    /game/state: hanoying_back/GameState
        to get the source disk's position

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
        int8 reason # -128: aborted/preempted, 0: none,
                    # 1: unknown disk, 2: unknown tower,
                    # 4: unreachable
        ---
        float32 percent
        geometry_msgs/Transform wrist # maybe?
        ```
"""
