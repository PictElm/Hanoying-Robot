#! /usr/bin/python3
import rospy

from hanoying_back.msg import GameState, GameMoveGoal
from hanoying_back.srv import GameSolveResponse

rospy.init_node("gui")

# TODO: open some kind of R-Viz window

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

    print(r + "\n\n")

def show_decision(msg: GameMoveGoal):
    r = f"\ndecision: {msg.disk}-{msg.tower}"
    print(r + "\n\n")

def show_solution(msg: GameSolveResponse):
    plural = "s" if len(msg.moves) else ""
    r = f"\nsolution: ({len(msg.moves)} move{plural})\n\t"
    r+= "  ".join([f"{move.disk}-{move.tower}" for move in msg.moves]) or "none"
    print(r + "\n\n")

sub_game_state = rospy.Subscriber("/game/state", GameState, show_game_state)
sub_decision = rospy.Subscriber("/decisys/decision", GameMoveGoal, show_decision)
sub_solution = rospy.Subscriber("/decisys/solution", GameSolveResponse, show_solution)

rospy.spin()
