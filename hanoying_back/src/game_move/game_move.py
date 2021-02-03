#! /usr/bin/python3
import rospy
import actionlib

from do_move import move

from hanoying_back.msg import GameState, GameMoveAction, GameMoveResult
from hanoying_back.srv import GameSolveValidate

rospy.init_node("game_move")

def do_feedback(feedback):
    if act.is_preempt_requested():
        result = GameMoveResult()
        result.success = False
        result.reason = -128 # reason < 0 => action aborted/preempted
        act.set_preempted(result)
        return
    act.publish_feedback(feedback)

def do_success():
    result = GameMoveResult()
    result.success = True
    result.reason = 0
    act.set_succeeded(result)

def do_failure(reason):
    result = GameMoveResult()
    result.success = False
    result.reason = reason or 0
    act.set_aborted(result)

validate = rospy.ServiceProxy("/game/solve/valid", GameSolveValidate)

def do_validation(game_move) -> bool:
    try:
        print(" waiting 10s for validation service..")
        rospy.wait_for_service("/game/solve/valid", 10)
    except rospy.exceptions.ROSException as e:
        print(" no response from validation service (" + str(e) + ")")
        return False

    try:
        print(" retreaving game state and calling validation..")
        game_state = rospy.wait_for_message("/game/state", GameState)
        return validate(state=game_state, move=game_move).valid
    except rospy.ServiceException as e:
        print(" validation service call failed (" + str(e) + ")")

    return False

def callback(game_move):
    check_valid = not rospy.get_param("/game/move/skip_move_validation", False)

    if check_valid:
        if not do_validation(game_move):
            print(" move is not valid")
            do_failure(1) # reason & 1 => move not valid
            return
        print(" move is valid")
    else:
        print(" passing move validation")

    move(game_move, do_feedback, do_success, do_failure)

act = actionlib.SimpleActionServer("/game/move", GameMoveAction, execute_cb=callback, auto_start=False)
act.start()

rospy.spin()
