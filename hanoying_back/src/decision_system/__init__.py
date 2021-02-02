"""
inits node 'decision_system'

@ros_sub
    /game/state: hanoying_back/GameState
        use the published game state to compute a solution
        and update the decision

@ros_pub
    /decisys/decision: hanoying_back/GameMoveGoal
        next intended move
        (note: for now solution[0] if any, else nothing)
    /decisys/solution: hanoying_back/GameSolveResponse
        computed solution of the game

@ros_call
    /game/solve: hanoying_back/GameSolve
        called to compute the solution from current state
"""
