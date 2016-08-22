#!/usr/bin/env python
import rospy
from vn_100.cfg import depthparamsConfig
from dynamic_reconfigure.server import Server

def callback(config, level):
	return config

def main():
    rospy.init_node("server_depth")
    yaw_srv = Server(depthparamsConfig,callback)
    rospy.spin()

if __name__ == '__main__':
    main()
