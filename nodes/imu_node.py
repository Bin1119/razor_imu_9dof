#!/usr/bin/env python

import rospy
import serial
import string
import math
import sys

from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler


port='/dev/ttyACM0'

degrees2rad = math.pi/180.0
imu_yaw_calibration = 0.0

roll=0
pitch=0
yaw=0
seq=0
accel_factor = 9.806 / 256.0    # sensor reports accel as 256.0 = 1G (9.8m/s^2). Convert to m/s^2.

rospy.init_node("razor_node")
pub = rospy.Publisher('imu', Imu, queue_size=1)

rate = rospy.Rate(200)

imuMsg = Imu()

# Check your COM port and baud rate
rospy.loginfo("Opening %s...", port)
try:
    ser = serial.Serial(port=port, baudrate=115200, timeout=1)
except serial.serialutil.SerialException:
    rospy.logerr("IMU not found at port "+port + ". Did you specify the correct port in the launch file?")
    #exit
    sys.exit(0)

rospy.loginfo("Giving the razor IMU board 1 seconds to boot...")
rospy.sleep(1)

ser.write('#ox' + chr(13)) # To start display angle and sensor reading in text

rospy.loginfo("Flushing first 200 IMU entries...")
for x in range(0, 200):
    line = ser.readline()
rospy.loginfo("Publishing IMU data...")

while not rospy.is_shutdown():
    line = ser.readline()
    line = line.replace("#YPRAG=","")   # Delete "#YPRAG="
    words = string.split(line,",")    # Fields split

    if len(words) > 2:
        #in AHRS firmware z axis points down, in ROS z axis points up (see REP 103)
        yaw_deg = -float(words[0])
        yaw_deg = yaw_deg + imu_yaw_calibration
        if yaw_deg > 180.0:
            yaw_deg = yaw_deg - 360.0
        if yaw_deg < -180.0:
            yaw_deg = yaw_deg + 360.0
        yaw = yaw_deg*degrees2rad
        #in AHRS firmware y axis points right, in ROS y axis points left (see REP 103)
        pitch = -float(words[1])*degrees2rad
        roll = float(words[2])*degrees2rad
        # Publish message
        # AHRS firmware accelerations are negated
        # This means y and z are correct for ROS, but x needs reversing
        imuMsg.linear_acceleration.x = -float(words[3]) * accel_factor
        imuMsg.linear_acceleration.y = float(words[4]) * accel_factor
        imuMsg.linear_acceleration.z = float(words[5]) * accel_factor
        imuMsg.angular_velocity.x = float(words[6])
        #in AHRS firmware y axis points right, in ROS y axis points left (see REP 103)
        imuMsg.angular_velocity.y = -float(words[7])
        #in AHRS firmware z axis points down, in ROS z axis points up (see REP 103) 
        imuMsg.angular_velocity.z = -float(words[8])

    q = quaternion_from_euler(roll,pitch,yaw)
    imuMsg.orientation.x = q[0]
    imuMsg.orientation.y = q[1]
    imuMsg.orientation.z = q[2]
    imuMsg.orientation.w = q[3]

    imuMsg.header.stamp= rospy.Time.now()
    imuMsg.header.frame_id = 'imu_link'
    imuMsg.header.seq = seq
    seq = seq + 1

    pub.publish(imuMsg)
    print 'running!'
    rate.sleep()

ser.close
#f.close
