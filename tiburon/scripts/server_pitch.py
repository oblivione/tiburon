#!/usr/bin/env python
import rospy
from tiburon.cfg import pitchparamsConfig
from dynamic_reconfigure.server import Server

def callback(config, level):
	return config


def main():
    rospy.init_node("server_pitch")
    yaw_srv = Server(pitchparamsConfig,callback)
    rospy.spin()

if __name__ == '__main__':
    main()
