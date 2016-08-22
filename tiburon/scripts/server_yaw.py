#!/usr/bin/env python
import rospy
from vn_100.cfg import pidConfig
from dynamic_reconfigure.server import Server

def callback(config, level):
	return config


def main():
    rospy.init_node("server_yaw")
    yaw_srv = Server(pidConfig,callback)
    rospy.spin()

if __name__ == '__main__':
    main()
