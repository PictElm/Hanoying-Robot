#! /usr/bin/python3
from typing import List
from hanoying_back.msg import GameState, GameMoveGoal as GameMove
GameMoveList = List[GameMove]

def validate(state: GameState, move: GameMove) -> bool:
    # TODO: rule lol
    return False

def list_possible(game_state: GameState) -> GameMoveList:
    # TODO: list lol
    return []
