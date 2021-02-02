#! /usr/bin/python3
import rospy
import actionlib

from do_move import move

from hanoying_back.msg import GameMoveAction, GameMoveResult

rospy.init_node("game_move")

def do_feedback(feedback):
    if act.is_preempt_requested():
        result = GameMoveResult()
        result.success = False
        result.reason = -128
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

callback = lambda game_move: move(game_move, do_feedback, do_success, do_failure)
act = actionlib.SimpleActionServer("/game/move", GameMoveAction, execute_cb=callback, auto_start=False)
act.start()

rospy.spin()
