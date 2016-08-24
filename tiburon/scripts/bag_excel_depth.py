#!/usr/bin/env python
import xlwt
import rospy
import xlrd
from xlutils.copy import copy
from std_msgs.msg import Float64


#FIRST OPEN BAG_EXCEL_SENSOR

rb = xlrd.open_workbook("pid_data.xls")
wb = copy(rb)
ws3 = wb.add_sheet("DEPTH VALUE")
count = 0

def heading3():
    global ws3,wb
    style_string = "font: bold on"
    style = xlwt.easyxf(style_string)
    DATA = ["Depth"]
    ws3.write(0,0,DATA[0],style=style)
    ws3.col(0).width=3500

def depth_callback(msg):
    global count,ws3,wb
    count = count+1
    rospy.loginfo("count=")
    rospy.loginfo(count)
    global ws3,wb
    ws3.write(count,0,str(msg.data))
    rospy.loginfo("depth appended,count=")
    rospy.loginfo(count)
    ws3.col(0).width=3500
    wb.save("pid_data.xls")

if __name__ == "__main__":
    global count
    rospy.init_node("bag_excel_depth")
    heading3()   
    rospy.Subscriber("/depth_value",Float64,depth_callback)
    wb.save("pid_data.xls")
    rospy.spin()
