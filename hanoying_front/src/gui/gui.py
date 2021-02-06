#! /usr/bin/python3
import rospy
import roslaunch

import tf
import math

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

DISK_COLORS = {
    '1': ColorRGBA(1,0,0,1),
    '2': ColorRGBA(0,1,0,1),
    '3': ColorRGBA(0,0,1,1),
}
DISK_SHAPE = { # (radius, height)
    '1': (1, 1),
    '2': (.66, 1),
    '3': (.33, 1),
}
TOWER_COLORS = {
    'A': ColorRGBA(0,1,1,1),
    'B': ColorRGBA(0,1,1,1),
    'C': ColorRGBA(0,1,1,1),
}
DECISION_COLOR = {
    'path': ColorRGBA(1,0,1,1),
    'arrow': ColorRGBA(1,0,1,1),
}

from random import random
def show_game_state(msg: GameState):
    for disk in msg.disks:
        pos = disk.point
        name = disk.name
        frame = f"Disk{name}"

        br.sendTransform((pos.x,pos.y,pos.z), (0,0,0,1), rospy.Time.now(), frame, "map")

        r = DISK_SHAPE[name][0]*.1
        h = DISK_SHAPE[name][1]*.1

        pub_markers.publish(Marker(
            header=Header(frame_id=frame),
            lifetime=rospy.Duration(),

            ns="/game/disks",
            id=int(name),

            type=Marker.CYLINDER,
            scale=Vector3(r, r, h),
            pose=Pose(
                position=Point(0, 0, 0),
                orientation=Quaternion(0, 0, 0, 1),
            ),

            color=DISK_COLORS[name],
        ))

def show_decision(msg: GameMoveGoal):
    disk_x = 0
    disk_y = 0
    disk_z = 0
    for disk in rospy.wait_for_message("/game/state", GameState).disks:
        if disk.name == msg.disk:
            disk_x = disk.point.x
            disk_y = disk.point.y
            disk_z = disk.point.z
            break
    else:
        return

    tower_x = rospy.get_param(f"/game/tower{msg.tower}/x", 0)
    tower_y = rospy.get_param(f"/game/tower{msg.tower}/y", 0)
    tower_z = rospy.get_param(f"/game/tower{msg.tower}/z", 0)
    if msg.tower == "floating":
        tower_x = msg.floating_point.x
        tower_y = msg.floating_point.y
        tower_z = msg.floating_point.z

    count = 20
    f = lambda a, b, t: (1-t)*a + t*b
    curve = [Point(
        x=f(disk_x, tower_x, k/count),
        y=f(disk_y, tower_y, k/count),
        z=f(disk_z, tower_z, k/count) + ( (count-k)*k * 4*.2 )/count/count,
    ) for k in range(count+1)]

    direction_x = curve[-1].x - curve[0].x
    direction_y = curve[-1].y - curve[0].y
    direction_z = curve[-1].z - curve[0].z
    rotate = math.atan2(direction_y, direction_x)
    slop = math.atan2(direction_z, math.hypot(direction_y, direction_x))
    xyzw = tf.transformations.quaternion_from_euler(0, -slop, rotate)

    pub_markers.publish(Marker(
        header=Header(frame_id="map"),
        lifetime=rospy.Duration(),

        ns="/decisys/decision",
        id=0,

        type=Marker.LINE_STRIP,
        scale=Vector3(x=.012),
        pose=Pose(position=Point(0,0,0), orientation=Quaternion(0,0,0,1)),
        points=curve,

        color=DECISION_COLOR['path'],
    ))

    pub_markers.publish(Marker(
        header=Header(frame_id="map"),
        lifetime=rospy.Duration(),

        ns="/decisys/decision",
        id=1,

        type=Marker.ARROW,
        scale=Vector3(.2, .042, .042),
        pose=Pose(
            position=curve[count//2],
            orientation=Quaternion(xyzw[0], xyzw[1], xyzw[2], xyzw[3]),
        ),

        color=DECISION_COLOR['arrow'],
    ))

def show_solution(msg: GameSolveResponse):
    # TODO
    return

sub_game_state = rospy.Subscriber("/game/state", GameState, show_game_state)
sub_decision = rospy.Subscriber("/decisys/decision", GameMoveGoal, show_decision)
sub_solution = rospy.Subscriber("/decisys/solution", GameSolveResponse, show_solution)

while not rospy.is_shutdown() and rviz.is_alive():
    for k,name in enumerate(rospy.get_param("/game/towers", "").split(';')):
        x = rospy.get_param(f"/game/tower{name}/x", 0)
        y = rospy.get_param(f"/game/tower{name}/y", 0)
        z = rospy.get_param(f"/game/tower{name}/z", 0)
        br.sendTransform((x,y,z), (0,0,0,1), rospy.Time.now(), f"Tower{name}", "Hanoy")

        pub_markers.publish(Marker(
            header=Header(frame_id=f"Tower{name}"),
            lifetime=rospy.Duration(),

            ns="/game/towers",
            id=k,

            type=Marker.LINE_STRIP,
            scale=Vector3(x=.012),
            pose=Pose(position=Point(0,0,0), orientation=Quaternion(0,0,0,1)),
            points=[Point(0,0,0), Point(0,0,10)],

            color=TOWER_COLORS[name],
        ))

    br.sendTransform((0,0,0), (0,0,0,1), rospy.Time.now(), "Hanoy", "map")
    rospy.sleep(1)
