#! /usr/bin/python3
import rospy
import actionlib

from hanoying_back.msg import GameState, GameMoveAction, GameMoveGoal
from hanoying_back.srv import GameSolveList

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
possible = rospy.ServiceProxy("/game/solve/possible", GameSolveList)

print("""\nUse the '{disk}-{tower}' move syntax (example '3-B')
or 'list' to print all possible moves.""")
while not rospy.is_shutdown():
    move = input("\nWaiting for input: ")

    if move == "list":
        try:
            print(" waiting 10s for possible move service..")
            rospy.wait_for_service("/game/solve/possible", 10)
        except rospy.exceptions.ROSException as e:
            print(" no response from service (" + str(e) + ")")
            continue

        try:
            print(" retreaving game state and calling service..")
            game_state = rospy.wait_for_message("/game/state", GameState)
            for move in possible(state=game_state).moves:
                print(f"\t{move.disk}-{move.tower}")
        except rospy.ServiceException as e:
            print(" service call failed (" + str(e) + ")")
        continue

    # may want to update that if changing game:
    #   currently set to take raw input string,
    #   look for pattern /(disk)-(tower)/
    #   and send goal to the action server
    if -1 < (dash := move.find('-')):
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
        print(" please use the '{disk}-{tower}' move syntax or 'list'.")
