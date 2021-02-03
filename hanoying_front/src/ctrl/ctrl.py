#! /usr/bin/python3
import rospy
import actionlib

from hanoying_back.msg import GameState, GameMoveAction, GameMoveGoal
from hanoying_back.srv import GameSolveValidate

rospy.init_node("ctrl")

cli = actionlib.SimpleActionClient("/game/move", GameMoveAction)
validate = rospy.ServiceProxy("/game/solve/valid", GameSolveValidate)

def on_feedback(feedback):
    print(str(feedback.percent*100) + "%")

def on_result(status, result):
    # may want to update that if changing game:
    #   process `result.reason` accordingly
    if result.success:
        print(" move done successfully :P")
    else:
        print(" could not do the move; reason:")
        if result.reason == 0:
            print("\tno reason specified...")
        else:
            if result.reason < 0:
                print("\tmove aborted")
            if result.reason & 1:
                print("\tdisk unreachable")
            if result.reason & 2:
                print("\ttower unreachable")
            if result.reason & 4:
                print("\tpoint unreachable")

while not rospy.is_shutdown():
    move = input("Waiting for input: ")

    # may want to update that if changing game:
    #   currently set to take raw input string,
    #   look for pattern /(disk)-(tower)/
    #   and validates it with the `/solve/valid`
    #   before send goal to the action server
    dash = move.find('-')
    if -1 < dash:
        goal_move = GameMoveGoal(disk=move[:dash], tower=move[dash+1:])
        can_do = True

        print(" waiting 10s for validation service..")
        try:
            rospy.wait_for_service("/game/solve/valid", 10)
        except (rospy.ROSInterruptException, rospy.exceptions.ROSInterruptException, rospy.exceptions.ROSException) as e:
            can_do = False
            print(" no response from validation service (" + str(e) + ").")

        if can_do:
            try:
                can_do = validate(state=rospy.wait_for_message("/game/state", GameState), move=goal_move).valid
            except rospy.ServiceException as e:
                can_do = False
                print(" validation service call failed (" + str(e) + ").")

            if can_do:
                try:
                    print(" waiting 10s for move server..")
                    cli.wait_for_server(rospy.Duration(10))
                    print(" sending move, waiting 10s for result..")
                    cli.send_goal(goal_move, done_cb=on_result)#, feedback_cb=on_feedback)
                    cli.wait_for_result(rospy.Duration(10))
                except Exception as e:
                    print("Something went wrong with the action server (" + str(e) + ").")
            else:
                print(" move is not valid.")
    else:
        print(" please use the '{disk}-{tower}' move syntax.")
