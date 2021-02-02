#! /usr/bin/python3
import rospy
from hanoying_back.msg import GameState

rospy.init_node("gui")

def show_game_state(msg: GameState):
    r = "\n"

    if msg.towers:
        for tower in msg.towers:
            disks = " ".join(tower.disks)
            r+= f"{tower.name}: {disks}\n"
    else:
        r+= "no tower registered (is hanoying_back running?)"

    r+= f"\nfloating:\n"
    if msg.floating_disks:
        for disk, pos in zip(msg.floating_disks, msg.floating_points):
            r+= f"\t{disk} ({pos.x}, {pos.y}, {pos.z})\n"
    else:
        r+= "\tnone"

    r+= "\n"
    rospy.logout(r) # TODO: open some kind of R-Viz window

sub_game_state = rospy.Subscriber("/game/state", GameState, show_game_state)

# sub4 = rospy.Subscriber("/decisys/decision", Int64, cb)
# sub5 = rospy.Subscriber("/decisys/solution", Int64, cb)

rospy.spin()
