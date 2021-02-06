#! /usr/bin/python3
from typing import List
from hanoying_back.msg import GameState, GameMoveGoal
GameSolution = List[GameMoveGoal]

from rules import list_possible
from random import randrange

def solve(state: GameState) -> GameSolution:
    # TODO
    all_moves = list_possible(state)
    return [all_moves[randrange(len(all_moves))]]
