"""
inits node 'sim_to_raw'

note: this nodes exist solely to translate messages from simulation
(/game/sim/*) into a valable GameRaw message; no processing is applied
(see game_state for that). also note that the GameRaw message type should
be updated for really 'raw' (currently cheat using simulation's position
transform)

@ros_param
    /game/disks: string
        ;-separated list of disk names

@ros_sub
    /game/sim/disk{name}: geometry_msgs/Point
        position of disk {name}, note: z is used to determine
        position in the tower (if any)

@ros_pub
    /game/raw: hanoying_back/GameRaw
        raw (faked) capture of the game
"""
