"""
inits node 'game_state'

note: stub relying on direct xyz coordinates from simulation
rather than sensors of any kind (see also sim_to_raw)
if GameRaw is updated to transmit real-world inputs, process.py
needs to be updated to process it to a usable GameState

@ros_param
    /game/allowed_disk_distance: double
        [process.py] max distance between a disk and a tower's center
                     for the disk to be considered on the tower
    /game/towers: string
        [process.py] ;-separated list of tower names
    /game/tower{name}/{x|y|z}: double
        [process.py] position of the tower {name}, note: z is supposed
                     to be irrelevant (vertical)

@ros_sub
    /game/raw: hanoying_back/GameRaw
        raw game state, see message type for structure

@ros_pub
    /game/state: hanoying_back/GameState
        game state, see message type for structure
"""
