#! /usr/bin/python3
import rospy

from process import process

from hanoying_back.msg import GameRaw, GameState

rospy.init_node("game_state")

pub = rospy.Publisher("/game/state", GameState, queue_size=10)
sub = rospy.Subscriber("/game/raw", GameRaw, lambda msg: pub.publish(process(msg)))

rospy.spin()
