#!/usr/bin/env python
import xlwt
import rospy
import xlrd
from xlutils.copy import copy
from std_msgs.msg import UInt16

#FIRST OPEN BAG_EXCEL_SENSOR

rb = xlrd.open_workbook("pid_data.xls")
wb = copy(rb)
ws4 = wb.add_sheet("FRONTPITCHSPEED")
count = 0

def heading4():
    global ws4,wb
    style_string = "font: bold on"
    style = xlwt.easyxf(style_string)
    DATA = ["FrontPitchSpeed"]
    ws4.write(0,0,DATA[0],style=style)
    ws4.col(0).width=3500

def frontpitchspeed_callback(msg):
    global count,ws4,wb
    count = count+1 #count value should remain updated
    global ws4,wb
    ws4.write(count,0,str(msg.data))
    rospy.loginfo("fps appended,count=")
    rospy.loginfo(count)
    ws4.col(0).width=3500
    wb.save("pid_data.xls")

if __name__ == "__main__":
    global count
    rospy.init_node("bag_excel_frontpitchspeed")
    heading4()
    rospy.Subscriber("/frontpitchspeed",UInt16,frontpitchspeed_callback)
    wb.save("pid_data.xls")
    rospy.spin()
