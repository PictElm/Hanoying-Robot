"""
inits node 'game_solve'

@ros_service
    /game/solve: hanoying_back/GameSolve
        exposes a service that solve the game
        from the given state

    /game/solve/valid: hanoying_back/GameSolveValidate
        validate a move, from a state, against the rules

    /game/solve/possible: hanoying_back/GameSolveList
        list the possible valid moves from the given state
"""
