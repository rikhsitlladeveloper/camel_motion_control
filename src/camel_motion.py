#!/usr/bin/env python3
import rospy
from Camel_Control import CamelControl
import time

cc = CamelControl()
#while True:
#    cc.get_velocity()
#print(cc.get_odom())
while True:
    cc.rotate(90)
    time.sleep(2)
    cc.rotate(-90)
    time.sleep(2)
# #cc.rotate(180)