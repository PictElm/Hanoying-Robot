#! /usr/bin/python3
import rospy

from .solver import solve
from .rules import validate, list_possible

from hanoying_back.msg import GameSolve, GameSolveResponse, GameSolveValidate, GameSolveValidateResponse, GameSolveList, GameSolveListResponse

rospy.init_node("game_solve")

def callback_solve(req):
    res = GameSolveResponse()
    res.moves = solve(req.state)
    return res

def callback_validate(req):
    res = GameSolveValidateResponse()
    res.valid = validate(req.state, req.move)
    return res

def callback_possible(req):
    res = GameSolveListResponse()
    res.moves = list_possible(req.state)
    return res

srv_solve = rospy.Service('/game/solve', GameSolve, callback_solve)
srv_valid = rospy.Service('/game/solve/valid', GameSolveValidate, callback_validate)
srv_possib = rospy.Service('/game/solve/possible', GameSolveList, callback_possible)
print("Solver is ready.")

rospy.spin()
