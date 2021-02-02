#! /usr/bin/python3
import rospy
import actionlib

from hanoying_back.msg import GameMoveAction, GameMoveGoal

rospy.init_node("ctrl")

cli = actionlib.SimpleActionClient("/game/move", GameMoveAction)

def on_feedback(feedback):
    print(str(feedback.percent*100) + "%")

def on_result(status, result):
    # may want to update that if changing game:
    #   process `result.reason` accordingly
    if result.success:
        print("move done successfully")
    else:
        print("could not do the move; reason:")
        if result.reason == 0:
            print("\tno reason specified...")
        if result.reason < 0:
            print("\tmove aborted")
        if result.reason & 1:
            print("\tno such disk")
        if result.reason & 2:
            print("\tno such tower")
        if result.reason & 4:
            print("\tpoint unreachable")

while not rospy.is_shutdown():
    move = input("waiting for input: ")

    # may want to update that if changing game:
    #   currently set to take raw input string,
    #   look for pattern /(disk)-(tower)/
    #   and send goal to the action server
    #   with the move "take {disk} to {tower}"
    dash = move.find('-')
    if -1 < dash:
        goal_move = GameMoveGoal(disk=move[:dash], tower=move[dash+1:])

        cli.wait_for_server()
        cli.send_goal(goal_move, done_cb=on_result)#, feedback_cb=on_feedback)
        cli.wait_for_result(rospy.Duration(10))
    else:
        print("please use the '{disk}-{tower}' move syntax")
