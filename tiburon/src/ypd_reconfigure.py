#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from tiburon.cfg import yawPitchDepthConfig 
#Note: ypdConfig name is from gen.generate in the cfg file

def callback(config, level):
    rospy.loginfo("ypd config successful!")
    return config

if __name__ == "__main__":
    rospy.init_node("ypd_reconfigure", anonymous = True)

    srv = Server(yawPitchDepthConfig, callback)
    rospy.spin()