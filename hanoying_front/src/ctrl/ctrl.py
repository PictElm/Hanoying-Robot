#! /usr/bin/python3
import rospy
import actionlib

from hanoying_back.msg import GameState, GameMoveAction, GameMoveGoal

rospy.init_node("ctrl")

def on_feedback(feedback):
    print(str(feedback.percent*100) + "%")

def on_result(status, result):
    # may want to update that if changing game:
    #   process `result.reason` accordingly
    if result.success:
        print(" move done successfully :P")
    else:
        print(" could not do the move - reason(s):")
        if result.reason == 0:
            print("\tno reason specified...")
        else:
            if result.reason < 0:
                print("\tmove aborted")
            if result.reason & 1:
                print("\tmove not valid")

            # game-specific reason flags
            if result.reason & 2:
                print("\tdisk unreachable")
            if result.reason & 4:
                print("\ttower unreachable")
            if result.reason & 8:
                print("\tpoint unreachable")

cli = actionlib.SimpleActionClient("/game/move", GameMoveAction)

while not rospy.is_shutdown():
    move = input("Waiting for input: ")

    # may want to update that if changing game:
    #   currently set to take raw input string,
    #   look for pattern /(disk)-(tower)/
    #   and send goal to the action server
    dash = move.find('-')
    if -1 < dash:
        game_move = GameMoveGoal(disk=move[:dash], tower=move[dash+1:])

        try:
            print(" waiting 10s for move server..")
            cli.wait_for_server(rospy.Duration(10))
            print(" sending move, waiting 10s for result..")
            cli.send_goal(game_move, done_cb=on_result)#, feedback_cb=on_feedback)
            cli.wait_for_result(rospy.Duration(10))
        except Exception as e:
            print(" something went wrong with the action server (" + str(e) + ").")
    else:
        print(" please use the '{disk}-{tower}' move syntax.")
