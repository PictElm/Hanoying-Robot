#! /usr/bin/python3
import rospy

from hanoying_back.msg import GameState, GameMoveGoal, GameSolve, GameSolveResponse

rospy.init_node("decision_system")

decision: GameMoveGoal = None
solution: list[GameMoveGoal] = []

def update_decision():
    global decision, solution
    decision = solution[0] if 0 < len(solution) else GameMoveGoal()

def update_solution(game_state):
    global decision, solution
    rospy.wait_for_service("/game/solve")
    try:
        # TODO: trynsee if I can.. keep the same proxy for each calls?
        solve = rospy.ServiceProxy("/game/solve", GameSolve)
        solution = solve(game_state)
    except rospy.ServiceException as e:
        print("Solver service call failed (" + e + ").")

callback = lambda game_state: [update_solution(game_state), update_decision()]
sub_game_state = rospy.Subscriber("/game/state", GameState, callback)

pub_decision = rospy.Publisher("/decisys/decision", GameMoveGoal, queue_size=10)
pub_solution = rospy.Publisher("/decisys/solution", GameSolveResponse, queue_size=10)
rate = rospy.Rate(3)

while not rospy.is_shutdown():
    pub_decision.publish(decision)
    pub_solution.publish(solution)
    rate.sleep()
