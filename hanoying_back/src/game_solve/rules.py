#! /usr/bin/python3
from typing import List
import rospy
from hanoying_back.msg import GameState, GameMoveGoal
GameMoveList = List[GameMoveGoal]

def validate(state: GameState, move: GameMoveGoal) -> bool:
    # this version of the rules allow to move a disk on the tower
    # "floating" which is essentially "out of the game" and
    # probably shouldn't be allowed; thoses moves
    # won't appear when listing possible moves

    # disk exists
    if not move.disk in rospy.get_param("/game/disks", "").split(";"):
        return False

    # tower exists
    if not move.tower in rospy.get_param("/game/towers", "").split(";"):
        return False

    # if not moving on a tower
    if move.tower != "floating":
        # tower valid (top-most larger)
        for tower in state.towers:
            if tower.name == move.tower:
                if len(tower.disks) and int(move.disk) < int(tower.disks[-1]):
                    return False
                break

    # find tower disk is on
    for tower in state.towers:
        # if last disk on tower is disk
        if len(tower.disks) and tower.disks[-1] == move.disk:
            return True
        # if has disks above it
        if move.disk in tower.disks:
            return False

    # disk not on a tower
    return True

def list_possible(state: GameState) -> GameMoveList:
    # lazy solution: filter valid moves from all possible combinations
    all_disks = rospy.get_param("/game/disks", "").split(';')
    all_towers = rospy.get_param("/game/towers", "").split(';')
    all_moves = ((GameMoveGoal(disk=a, tower=b) for b in all_towers) for a in all_disks)
    return [it for sublist in all_moves for it in sublist if validate(state, it)]
