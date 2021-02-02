from typing import Callable, Any

import rospy

from hanoying_back.msg import GameMoveGoal, GameMoveFeedback

try:
    import sim
    remoteSocketPort = 20202
    simxClient = sim.simxStart("127.0.0.1", remoteSocketPort, True, False, 3000, 5)
    if simxClient < 0:
        exit("Could not join remote API server (have you started simulation?)")
    else:
        rospy.on_shutdown(lambda: sim.simxFinish(simxClient))
except Exception as e:
    exit("Could not setup remote API client (" + e + ")")


def move(game_move: GameMoveGoal, on_feedback: Callable[[GameMoveFeedback], Any], on_success: Callable[[], Any], on_failure: Callable[[int], Any]):
    can_do = True
    reason = 0

    if not (disk := game_move.disk) in rospy.get_param("/game/disks", "").split(";"):
        can_do = False
        reason|= 1

    if not (tower := game_move.tower) in rospy.get_param("/game/towers", "").split(";"):
        can_do = False
        reason|= 2

    # disk and tower locations can be obtained using:
    # ```
    # src = rospy.wait_for_message(f"/game/raw/disk{disk}", Point)
    # dst = rospy.get_param(f"/game/tower{tower}/{x|y|z}")
    # ```
    # 
    # `src` and `dst` should be used to determine if both are reachable
    # (we assume tower is reachable, but disk is not necessarily on a tower)
    # and compute trajectory
    # 
    # if `tower=="floating"`, the position in `game_move.floating_point`
    # should be checked for reachability
    # 
    # if any position is unreachable, the appropriate flags should be set:
    # ```
    # can_do = False  # signals to abort
    # reason|= 4      # signals reason is 'unreachable'
    # ```

    if not can_do:
        on_failure(reason)
        return

    else:
        feedback = GameMoveFeedback()

        # move piece in coppeliaSim with setStringSignal("move", -)
        sim.simxSetStringSignal(simxClient, "move", f"{disk}-{tower}", sim.simx_opmode_oneshot)

        for k in range(101): # let's just pretend the robot is moving for 2s...
            rospy.sleep(.02)

            feedback.percent = k/100.
            #feedback.wrist = "somewhere between {disk} and {tower}"

            on_feedback(feedback)

        on_success()
        return
