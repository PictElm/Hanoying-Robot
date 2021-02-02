#! /usr/bin/python3
import rospy

from geometry_msgs.msg import Point
from hanoying_back.msg import Tower, GameState

rospy.init_node("game_state")

ALLOWED_DISK_DISTANCE = .1 # some random default value

class PointObject:
    def __init__(self, x, y=None, z=None):
        if z == None:
            self.x = rospy.get_param(x + "/x")
            self.y = rospy.get_param(x + "/y")
            self.z = rospy.get_param(x + "/z")
        else:
            self.x = x
            self.y = y
            self.z = z

    def sq_xy_distance(self, to: 'PointObject'):
        return (to.x - self.x)**2 + (to.y - self.y)**2

    def __str__(self):
        return f"{self.x}, {self.y}, {self.z}"

class TowerObject:
    def __init__(self, name: str, pos: 'PointObject'):
        self.name = name
        self.pos = pos

        self._disks: 'list[DiskObject]' = []

    def append(self, disk: 'DiskObject'):
        self._disks.append(disk)

    def remove(self, disk: 'DiskObject'):
        self._disks.remove(disk)

    def get_diskpositions(self):
        return [Point(x=it.pos.x, y=it.pos.y, z=it.pos.z) for it in self._disks]

    def get_disknames(self):
        return [it.name for it in self._disks]

    def get_message(self):
        return Tower(name=self.name, disks=self.get_disknames())

    def __str__(self):
        disks = " ".join(self.disks)
        return f"tower {self.name} ({disks})\n"

class DiskObject:
    def __init__(self, name: str, towers: list[TowerObject], floating: TowerObject):
        self.name = name
        self.pos = PointObject(0, 0, 0)

        self._firstcall = True

        self._towers = towers
        self._floating = floating
        self._current = floating

        msg_cb = lambda msg: self._callback(PointObject(msg.x, msg.y, msg.z))
        self._sub = rospy.Subscriber(f"/game/raw/disk{name}", Point, msg_cb)

    def _callback(self, pos: 'PointObject'):
        # TODO: `/game/raw/disk{name}` would be raw input from sensors
        # (e.g. camera, weights, honyarara and such nantoka)
        # to be processed into world coordinates here (`self.pos = pos`)
        self.pos = pos

        closest, shortest = self._floating, float("inf")
        for it in self._towers:
            distance = pos.sq_xy_distance(it.pos)
            if distance < shortest:
                shortest = distance
                closest = it

        if ALLOWED_DISK_DISTANCE < shortest:
            closest = self._floating

        if closest != self._current:
            self._current.remove(self)
            closest.append(self)
            self._current = closest

    def __str__(self):
        return f"disk {self.name} ({self.pos})"

towers: list[TowerObject] = []
floating: TowerObject = None
disks: list[DiskObject] = []

def load_game_param(towerNamesParam: str, diskNamesParam: str):
    global towers, floating, disks, ALLOWED_DISK_DISTANCE

    ALLOWED_DISK_DISTANCE = rospy.get_param("/game/allowed_disk_distance", ALLOWED_DISK_DISTANCE)
    ALLOWED_DISK_DISTANCE = ALLOWED_DISK_DISTANCE*ALLOWED_DISK_DISTANCE

    towerNames = towerNamesParam.split(';')
    diskNames = diskNamesParam.split(';')

    towers = [TowerObject(name, PointObject("/game/tower" + name)) for name in towerNames if name]
    floating = TowerObject("floating", None)
    disks = [DiskObject(name, towers, floating) for name in diskNames if name]

    for it in disks: floating.append(it)

towerNamesParam, diskNamesParam = "", ""
pub = rospy.Publisher("/game/state", GameState, queue_size=10)
rate = rospy.Rate(3)

while not rospy.is_shutdown():
    towerNamesChanged = towerNamesParam != (towerNamesParam := rospy.get_param("/game/towers", ""))
    diskNamesChanged = diskNamesParam != (diskNamesParam := rospy.get_param("/game/disks", ""))
    if towerNamesChanged or diskNamesChanged:
        load_game_param(towerNamesParam, diskNamesParam)

    pub.publish(
        towers=[it.get_message() for it in towers],
        floating_disks=floating.get_disknames(),
        floating_points=floating.get_diskpositions(),
    )
    rate.sleep()
