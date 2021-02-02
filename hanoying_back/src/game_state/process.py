#! /usr/bin/python3
from typing import List

import rospy

from geometry_msgs.msg import Point
from hanoying_back.msg import GameRaw, Disk, Tower, GameState

ALLOWED_DISK_DISTANCE = rospy.get_param("/game/allowed_disk_distance", .1)

class PointObject:
    def __init__(self, x, y=None, z=None):
        if isinstance(x, str):
            self.x = rospy.get_param(x + "/x")
            self.y = rospy.get_param(x + "/y")
            self.z = rospy.get_param(x + "/z")
        elif isinstance(x, Point):
            self.x = x.x
            self.y = x.y
            self.z = x.z
        else:
            self.x = x
            self.y = y
            self.z = z

    def sq_xy_distance(self, to: 'PointObject') -> float:
        return (to.x - self.x)**2 + (to.y - self.y)**2

    def get_message(self) -> Point:
        return Point(x=self.x, y=self.y, z=self.z)

class TowerObject:
    def __init__(self, name: str, pos: PointObject):
        self.name = name
        self.pos = pos

        self._disks: 'List[DiskObject]' = []

    def append(self, disk: 'DiskObject'):
        self._disks.append(disk)

    def remove(self, disk: 'DiskObject'):
        self._disks.remove(disk)

    def get_message(self):
        return Tower(name=self.name, disks=[it.name for it in self._disks])

class DiskObject:
    def __init__(self, name: str, pos: PointObject, towers: List[TowerObject], floating: TowerObject):
        self.name = name
        self.pos = PointObject(0, 0, 0)
        self._tower = floating

        closest, shortest = floating, float("inf")
        for it in towers:
            distance = pos.sq_xy_distance(it.pos)
            if distance < shortest:
                shortest = distance
                closest = it

        if shortest < ALLOWED_DISK_DISTANCE:
            self._tower = closest

        self._tower.append(self)

    def get_message(self) -> Disk:
        return Disk(name=self.name, point=self.pos.get_message())

def process(raw: GameRaw) -> GameState:
    towerNames = rospy.get_param("/game/towers", "").split(';')

    towers = [TowerObject(name, PointObject("/game/tower" + name)) for name in towerNames if name]
    floating = TowerObject("floating", None)
    disks = [DiskObject(name, PointObject(point), towers, floating) for name, point in zip(raw.disk_names, raw.disk_points)]

    return GameState(
        disks=[it.get_message() for it in disks],
        towers=[it.get_message() for it in towers],
        floating=floating.get_message(),
    )
