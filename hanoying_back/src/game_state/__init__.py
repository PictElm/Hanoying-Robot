"""
inits node 'game_state'

note: stub relying on direct xyz coordinates from simulation
rather than sensors of any kind (camera / weights ...)

@ros_param
    /game/allowed_disk_distance: double
        max distance between a disk and a tower's center
        for the disk to be considered on the tower
    /game/towers: string
        ;-separated list of tower names
    /game/disks: string
        ;-separated list of disk names
    /game/tower{name}/{x|y|z}: double
        position of the tower {name}, note: z is supposed
        to be irrelevant (vertical)

@ros_sub
    /game/raw/disk{name}: geometry_msgs/Point
        position of disk {name}, note: z is used to determine
        position in the tower (if any)

@ros_pub
    /game/state: hanoying_back/GameState
        game state, see message type for structure
"""
