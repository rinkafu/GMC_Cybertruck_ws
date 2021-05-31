#!/usr/bin/env python3
#update 20/5/21
import sys
import serial
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from serial.tools import list_ports
import time

speed = 0.0
angRate = 0.0
multiple = 1

teensyPotr = ""
STR_USBPORT="USB VID:PID=16C0:0483 SER=7442840 LOCATION=2-2.3:1.0"

_Baudrate = 9600

v_range = 33970.276 # = 100 cm

wheel_left_set_old = 0.0
wheel_right_set_old = 0.0

buf_lr = 0
mortor_ender ={}

pub_l = rospy.Publisher('wheel_l_ender', Float32, queue_size=1)
pub_r = rospy.Publisher('wheel_r_ender', Float32, queue_size=1)

def Serial_Send(_serial, Speed_val):
    res = bytes(Speed_val, 'utf-8')
    # time.sleep(0.1)
    _serial.write(res)

def Serial_Read(_serial, range_buf=100):
    return _serial.read(range_buf)

def Serial_connect(SerialPort, Baudrate):
    _serial = serial.Serial(SerialPort, Baudrate, timeout=0)
    return _serial

def getTeensyPort():
    for port in list(list_ports.comports()):
        # print(port[2])
        if port[2] == STR_USBPORT:
            return port[0]

def callback(Twist):
    global wheel_left_set_old
    global wheel_right_set_old
    speed = Twist.linear.x
    angRate = Twist.angular.z
    wheel_left_set = ((speed - angRate) * v_range)
    wheel_right_set = ((speed + angRate) * v_range)
    # rospy.loginfo(wheel_left_set)
    # rospy.loginfo(wheel_right_set)
    if wheel_left_set_old != wheel_left_set and wheel_right_set != wheel_right_set_old:
        wheel_left_set_old = wheel_left_set
        wheel_right_set_old = wheel_right_set
        str_send = str(wheel_left_set) + "," + str(wheel_right_set) + '\n'
        rospy.loginfo(str_send)
        Serial_Send(_serial, str_send)

def _readline():
    eol = b'\r\n'
    leneol = len(eol)
    line = bytearray()
    while True:
        reader = Serial_Read(_serial, 1)
        if reader:
            line += reader
            if line[-leneol:] == eol:
                break
        else:
            break

    return bytes(line)

def timer_callback(event):
    try:
        reader = _readline().decode("utf-8")
        buf_lr = reader.split(",")
        mortor_ender["motro_L"] = int(buf_lr[0])
        mortor_ender["motro_R"] = int(buf_lr[1])
        if reader != "":
            rospy.loginfo(mortor_ender)
            pub_l.publish(mortor_ender["motro_L"])
            pub_r.publish(mortor_ender["motro_R"])
    except:
        pass

def listener():
    rospy.init_node('Motor', anonymous=False)
    rospy.Subscriber("cmd_vel", Twist, callback, queue_size=1)
    timer = rospy.Timer(rospy.Duration(0.015), timer_callback)
    rospy.spin()
    timer.shutdown()

if __name__ == '__main__':
    try:
        teensyPotr = getTeensyPort()
        _serial = Serial_connect(teensyPotr, _Baudrate)
        listener()
    except rospy.ROSInterruptException:
        pass
