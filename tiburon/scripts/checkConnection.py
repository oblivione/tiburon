#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt16
def main():
    rospy.init_node("checkConnection",anonymous=False)
    con=rospy.Publisher("/tiburonConnection",UInt16,queue_size=1)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        con.publish(1);
        rate.sleep()

if __name__=='__main__':
    main()
