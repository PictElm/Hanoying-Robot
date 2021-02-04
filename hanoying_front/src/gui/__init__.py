"""
inits node 'gui'

@ros_param
    /game/tower{name}{x|y|z}: double
        to get the destination tower's position

@ros_sub
    /game/state: hanoying_back/GameState
        to get the source disk's position
    /decisys/decision: hanoying_back/GameMoveGoal
        next intended move
    /decisys/solution: hanoying_back/GameSolveResponse
        computed solution of the game
"""
