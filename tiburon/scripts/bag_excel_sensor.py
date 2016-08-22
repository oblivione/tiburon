#!/usr/bin/env python
import xlwt
import rospy
from vn_100.msg import sensor_data 

#create new workbook
wb = xlwt.Workbook()
ws1 = wb.add_sheet("SENSOR DATA")
count = 0

def heading1():
    global ws1,wb
    style_string = "font: bold on"
    style = xlwt.easyxf(style_string)
    DATA = ["Acceleration[x]","Acceleration[y]","Acceleration[z]","Gyro[x]","Gyro[y]","Gyro[z]","Temp","Pressure","Timestamp"]
    for i in range(0,len(DATA)):
        ws1.write(0,i,DATA[i],style=style)
        ws1.col(i).width=3500
        
def sensor_data_callback(msg):
    global count,ws1,wb
    count = count+1
    global ws1,wb
    DATA = [str(msg.Accel.x),str(msg.Accel.y),str(msg.Accel.z),str(msg.Gyro.x),str(msg.Gyro.y),str(msg.Gyro.z),str(msg.Temp),str(msg.Pressure),str(msg.header.stamp)]
    for i in range(0,9):
        ws1.write(count,i,DATA[i])
        rospy.loginfo("sensor appended,count=")
        rospy.loginfo(count)
        if(i == 8):
            ws1.col(i).width=4700
        else:
            ws1.col(i).width=3500
    wb.save("pid_data.xls")

#problem:spacing problem in spreadsheet if subscribers run as separate threads n not sequentially    
if __name__ == "__main__":
    global count
    rospy.init_node("bag_excel_sensor")
    rospy.loginfo("Working")
    heading1()
    rospy.Subscriber("/vn_100/sensor_data",sensor_data,sensor_data_callback)
    wb.save("pid_data.xls")
    rospy.spin()
