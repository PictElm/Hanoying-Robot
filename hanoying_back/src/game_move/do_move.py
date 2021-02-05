#! /usr/bin/python3
from typing import Callable, Any
import rospy
from hanoying_back.msg import GameMoveGoal, GameMoveFeedback
FeedbackCb = Callable[[GameMoveFeedback], Any]
SuccessCb = Callable[[], Any]
FailureCb = Callable[[int], Any]

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

def move(move: GameMoveGoal, on_feedback: FeedbackCb, on_success: SuccessCb, on_failure: FailureCb):
    can_do = True
    reason = 0

    # disk and tower locations can be obtained using:
    # ```
    # src = SELECT FROM rospy.wait_for_message("/game/state", GameState).disk WHERE name="{disk}"
    # dst = rospy.get_param(f"/game/tower{tower}/{x|y|z}")
    # ```
    # 
    # `src` and `dst` should be used to determine if both are reachable
    # (we assume tower is reachable, but disk is not necessarily on a tower)
    # and compute trajectory
    # 
    # if `tower=="floating"`, the position in `move.floating_point`
    # should be checked for reachability
    # 
    # if any position is unreachable, the appropriate flag(s) should be set:
    # ```
    # reason|= 1      # signals reason 'disk unreachable'
    # reason|= 2      # signals reason 'tower unreachable'
    # reason|= 4      # signals reason 'point unreachable'
    # can_do = False  # signals to abort (call on_failure and abort)
    # ```

    if not can_do:
        on_failure(reason)
        return

    else:
        feedback = GameMoveFeedback()

        # move piece in CoppeliaSim with setStringSignal("move", -)
        simMove = f"{move.disk}-{move.tower}"
        sim.simxSetStringSignal(simxClient, "move", simMove, sim.simx_opmode_oneshot)

        for k in range(101): # let's just pretend the robot is moving for 2s...
            rospy.sleep(.02)

            feedback.percent = k/100.
            #feedback.wrist = "somewhere between {disk} and {tower}"

            on_feedback(feedback)

        on_success()
        return
