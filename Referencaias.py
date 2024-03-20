#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Vector3

def referencia():
    global t
    global x, y, z
    global vel_deseadax, vel_deseaday, vel_deseadaz
    global yaw, yawpunto_deseada

    t = 0
    x, y, z = -5, 0, 0
    vel_deseadax, vel_deseaday, vel_deseadaz = 0, 0, 0
    yaw, yawpunto_deseada = 0, 0

    rospy.init_node('referencia', anonymous=True)

    pos_pub = rospy.Publisher('/pos', Vector3, queue_size=10)
    vel_pub = rospy.Publisher('/vel', Vector3, queue_size=10)
    yaw_pub = rospy.Publisher('/yaw', Vector3, queue_size=10)

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        t += 0.01
        if t >= 0 and t < 5:
            vel_deseadax, vel_deseaday, vel_deseadaz, yaw_deseada = 0, 0, -0.5, 0
        elif t >= 5 and t < 65:
            vel_deseadax = 0.5 * math.sin(0.1 * (t - 5))
            vel_deseaday = 0.5 * math.cos(0.1 * (t - 5))
            vel_deseadaz, yaw_deseada = 0, 0.1
        elif t >= 65:
            vel_deseadax, vel_deseaday, vel_deseadaz, yaw_deseada = 0, 0, 0, 0

        x += vel_deseadax * 0.01
        y += vel_deseaday * 0.01
        z += vel_deseadaz * 0.01
        yaw += yaw_deseada * 0.01

        pos_msg = Vector3()
        vel_msg = Vector3()
        yaw_msg = Vector3()

        #pos_msg.x = x
        #pos_msg.y = y
        #pos_msg.z = z

        pos_msg.x = x
        pos_msg.y = y
        pos_msg.z = z

        vel_msg.x = vel_deseadax
        vel_msg.y = vel_deseaday
        #vel_msg.z = vel_deseadaz
        #vel_msg.x = 0
        #vel_msg.y = 0
        vel_msg.z = 0

        yaw_msg.x = 0#yaw
        yaw_msg.y = 0#yaw_deseada
        yaw_msg.z = 0

        pos_pub.publish(pos_msg)
        vel_pub.publish(vel_msg)
        yaw_pub.publish(yaw_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        referencia()
    except rospy.ROSInterruptException:
        pass
