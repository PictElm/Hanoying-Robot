#! /usr/bin/python3
from typing import List
import rospy
from hanoying_back.msg import GameState, GameMoveGoal
GameMoveList = List[GameMoveGoal]

def validate(state: GameState, move: GameMoveGoal) -> bool:
    # TODO: rule lol

    can_do = True

    if not move.disk in rospy.get_param("/game/disks", "").split(";"):
        can_do = False

    if not move.tower in rospy.get_param("/game/towers", "").split(";"):
        can_do = False

    return can_do

def list_possible(game_state: GameState) -> GameMoveList:
    # TODO: list lol
    return []
