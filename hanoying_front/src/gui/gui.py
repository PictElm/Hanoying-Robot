#! /usr/bin/python3
import rospy

from hanoying_back.msg import GameState, GameMoveGoal
from hanoying_back.srv import GameSolveResponse

rospy.init_node("gui")

def show_game_state(msg: GameState):
    r = "\n"

    if msg.towers:
        for tower in msg.towers:
            disks = " ".join(tower.disks)
            r+= f"{tower.name}: {disks}\n"
    else:
        r+= "no tower registered (is hanoying_back running?)"

    r+= "\nfloating:\n"
    if msg.floating.disks:
        r+= "\t" + " ".join(msg.floating.disks)
    else:
        r+= "\tnone"
    r+= "\n"

    rospy.logout(r + "\n\n") # TODO: open some kind of R-Viz window

sub_game_state = rospy.Subscriber("/game/state", GameState, show_game_state)

def plho_cb(msg):
    pass #print("received", msg)

sub_decision = rospy.Subscriber("/decisys/decision", GameMoveGoal, plho_cb)
sub_solution = rospy.Subscriber("/decisys/solution", GameSolveResponse, plho_cb)

rospy.spin()
