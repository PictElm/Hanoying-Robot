#! /usr/bin/python3
from typing import List

import rospy

from hanoying_back.msg import GameState, GameMoveGoal
from hanoying_back.srv import GameSolve, GameSolveResponse

rospy.init_node("decision_system")

decision: GameMoveGoal = GameMoveGoal()
solution: List[GameMoveGoal] = []

solve = rospy.ServiceProxy("/game/solve", GameSolve)

def update_decision():
    global decision
    decision = solution[0] if 0 < len(solution) else GameMoveGoal()

def update_solution(state):
    global solution
    try:
        rospy.wait_for_service("/game/solve", 10)
    except rospy.exceptions.ROSException as e:
        solution = []
        print("No response from solver service (" + str(e) + ")")
        return
    try:
        solution = solve(state).moves
    except rospy.ServiceException as e:
        solution = []
        print("Solver service call failed (" + str(e) + ").")

callback = lambda msg: [update_solution(msg), update_decision()]
sub_game_state = rospy.Subscriber("/game/state", GameState, callback)

pub_decision = rospy.Publisher("/decisys/decision", GameMoveGoal, queue_size=10)
pub_solution = rospy.Publisher("/decisys/solution", GameSolveResponse, queue_size=10)
rate = rospy.Rate(rospy.get_param("/decisys/rate", 3))

while not rospy.is_shutdown():
    pub_decision.publish(decision)
    pub_solution.publish(solution)
    rate.sleep()
