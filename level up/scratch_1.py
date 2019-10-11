#!/usr/bin/env python
# license removed for brevity
import rospy
import serial
from builtins import bytearray, print, type, range, int
from sensor_msgs.msg import Joy, struct

serialData = serial.Serial('/dev/ttyACM0', 115200,
                           bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)

def callback(data):
    joy_axes = []
    byte_joy_axes = bytearray()
    joy_axes.append(data.axes[0])
    joy_axes.append(data.axes[1])
    rospy.loginfo(data.axes)
    print(joy_axes)
    byte_joy_axes += struct.pack('f', joy_axes[0])
    byte_joy_axes += struct.pack('f', joy_axes[1])
    print(byte_joy_axes)

    serialData.write(byte_joy_axes)

    reciver = []
    reciver.append(serialData.read(4))
    print(*reciver)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('joy', Joy, callback)
    rospy.spin()


while not rospy.is_shutdown():
    if __name__ == "__main__":
        listener()

# while serialData.readable():
# b+=struct.pack('f',i)
# b+=struct.pack('f',190.5)
# print(b)
# serialData.write(b)
# print(int.from_bytes(serialData.read(), byteorder='little'))
# print(serialData.read())
# print(i)
# b.clear()
# print(serialData.read().decode('Ascii'))
# serialString = serialData.readline()
# print(serialString.decode('Ascii'))
