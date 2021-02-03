#! /usr/bin/python3
import rospy

import tf
import roslaunch

from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, Quaternion, Vector3, Point
from visualization_msgs.msg import Marker

from hanoying_back.msg import GameState, GameMoveGoal
from hanoying_back.srv import GameSolveResponse

rospy.init_node("gui")

launcher = roslaunch.ROSLaunch()
rospy.on_shutdown(launcher.stop)
launcher.start()

print("Starting RViz node..")
rviz = launcher.launch(roslaunch.core.Node("rviz", "rviz"))

br = tf.TransformBroadcaster()
pub_markers = rospy.Publisher("/visualization_marker", Marker, queue_size=5)

from random import random
def show_game_state(msg: GameState):
    for disk in msg.disks:
        pos = disk.point
        name = disk.name
        frame = f"Disk{name}"

        br.sendTransform((pos.x,pos.y,pos.z), (0,0,0,1), rospy.Time.now(), frame, "map")

        pub_markers.publish(Marker(
            header=Header(frame_id=frame),
            lifetime=rospy.Duration(),

            ns="/game/disks",
            id=int(name),

            pose=Pose(
                position=Point(0, 0, 0),
                orientation=Quaternion(0, 0, 0, 1),
            ),
            type=Marker.CYLINDER,
            scale=Vector3(1, 1, 1),

            color=ColorRGBA(random(), random(), random(), 1),
        ))

def show_decision(msg: GameMoveGoal):
    return
    r = f"\ndecision: {msg.disk}-{msg.tower}"
    print(r + "\n\n")

def show_solution(msg: GameSolveResponse):
    return
    plural = "s" if len(msg.moves) else ""
    r = f"\nsolution: ({len(msg.moves)} move{plural})\n\t"
    r+= "  ".join([f"{move.disk}-{move.tower}" for move in msg.moves]) or "none"
    print(r + "\n\n")

sub_game_state = rospy.Subscriber("/game/state", GameState, show_game_state)
sub_decision = rospy.Subscriber("/decisys/decision", GameMoveGoal, show_decision)
sub_solution = rospy.Subscriber("/decisys/solution", GameSolveResponse, show_solution)

while not rospy.is_shutdown() and rviz.is_alive():
    for name in rospy.get_param("/game/towers", "").split(';'):
        x = rospy.get_param(f"/game/tower{name}/x", 0)
        y = rospy.get_param(f"/game/tower{name}/y", 0)
        z = rospy.get_param(f"/game/tower{name}/z", 0)
        br.sendTransform((x,y,z), (0,0,0,1), rospy.Time.now(), f"Tower{name}", "Hanoy")
    br.sendTransform((0,0,0), (0,0,0,1), rospy.Time.now(), "Hanoy", "map")
    rospy.sleep(1)
#rospy.spin()
