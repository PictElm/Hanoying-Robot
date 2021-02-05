#! /usr/bin/python3
from typing import List

import rospy

from geometry_msgs.msg import Point
from hanoying_back.msg import GameRaw

rospy.init_node("sim_to_raw")

class DiskObject:
    def __init__(self, name: str):
        self.name = name
        self.x = 0
        self.y = 0
        self.z = 0

        def _cb(msg):
            self.x = msg.x
            self.y = msg.y
            self.z = msg.z
        self._sub = rospy.Subscriber(f"/game/sim/disk{name}", Point, _cb)

    def get_point_msg(self) -> Point:
        return Point(x=self.x, y=self.y, z=self.z)

    def unsubscribe(self):
        self._sub.unregister()

disks: List[DiskObject] = []

def load_game_param(disk_names_param: str):
    global disks

    for it in disks:
        it.unsubscribe()

    disk_names = disk_names_param.split(';')
    disks = [DiskObject(name) for name in disk_names if name]

disk_names_param = ""
pub = rospy.Publisher("/game/raw", GameRaw, queue_size=10)
rate = rospy.Rate(10)

# CoppeliaSim scene (coppeliasim_scene.ttt) is set to use the Newton physics engine
# with a dt=50ms which is equivalent to a publication rate of 20hz on every
# `/game/sim/disk{name}`; sim_to_raw halves the rate to around 10hz

while not rospy.is_shutdown():
    if disk_names_param != (disk_names_param := rospy.get_param("/game/disks", "")):
        load_game_param(disk_names_param)

    msg_names = []
    msg_points = []
    for it in disks:
        msg_names.append(it.name)
        msg_points.append(it.get_point_msg())

    pub.publish(
        disk_names=msg_names,
        disk_points=msg_points,
    )
    rate.sleep()
