#!/usr/bin/env python
import xlwt
import xlrd
import rospy 
from xlutils.copy import copy
from vn_100.msg import ins_data#from vn_100.msg will be for all msgs defined in package

#RUN BAG_EXCEL_SENSOR FIRST

rb = xlrd.open_workbook("pid_data.xls")
wb = copy(rb) #to be used for writing
ws2 = wb.add_sheet("INS DATA")
count = 0

def heading2():
    global ws2,wb
    style_string = "font: bold on"
    style = xlwt.easyxf(style_string)
    DATA = ["Timestamp","YPR[x]","YPR[y]","YPR[z]","quat_data"]
    for i in range(0,len(DATA)):
        ws2.write(0,i,DATA[i],style=style)
        ws2.col(i).width=3500

def ins_data_callback(msg):
    global count,ws,wb
    count = count+1
    global ws2,wb
    DATA = [str(msg.header.stamp),str(msg.YPR.x),str(msg.YPR.y),str(msg.YPR.z),str(msg.quat_data)]
    for i in range(0,len(DATA)):
        ws2.write(count,i,DATA[i])
        rospy.loginfo("ins appended,count=")
        rospy.loginfo(count)
        ws2.col(i).width=3500
    wb.save("pid_data.xls")

if __name__ == "__main__":
    global count
    rospy.init_node("bag_excel_ins")
    heading2()
    rospy.Subscriber("/vn_100/ins_data",ins_data,ins_data_callback)
    wb.save("pid_data.xls")
    rospy.spin()
