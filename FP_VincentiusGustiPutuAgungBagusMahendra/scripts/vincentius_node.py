#!/usr/bin/env python
import cv2
import numpy as np
import matplotlib as plt
from pyzbar.pyzbar import decode

import imutils
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import sys
from turtlesim.srv import *
from std_srvs.srv import Empty

global xspeed
global zspeed
global ismoving
global lambat
global turning
global steering

steering = 0
xspeed = 0.0
yspeed = 0.0

lambat = 0

ismoving = 0

turning = 0

vid= cv2.VideoCapture(0)
vid.set(3,640)
vid.set(4,480)


def pose_callback(pose):
    global robot_x
    global robot_y
    global robot_t

    rospy.loginfo("Robot x = %f\n RObot Y = %f\n Robot T = %f", pose.x, pose.y, pose.theta)
    robot_x = pose.x  # menyimpan float posisi turtlesim pada sumbu x
    robot_y = pose.y  # menyimpan float posisi turtlesim pada sumbu y
    robot_t = pose.theta  # menyimpan float posisi derajat turtlesim



def moving(lin_vel, ang_vel):
    rospy.init_node('move_turtle', anonymous=False)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist,
                          queue_size=10)

    rospy.Subscriber('/turtle1/pose', Pose,
                     pose_callback)

    rate = rospy.Rate(10)

    vel = Twist()

    vel.linear.x = lin_vel
    vel.linear.y = 0
    vel.linear.z = 0

    vel.angular.x = 0
    vel.angular.y = 0
    vel.angular.z = ang_vel

    pub.publish(vel)
    rate.sleep()


if __name__ == '__main__':
    try:
        while True:
            ret, frame = vid.read()
            frame = cv2.flip(frame, 1)
            if steering == 1:
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                value = (11, 11)
                blurred = cv2.GaussianBlur(hsv, value, 0)

                colourLower = np.array([100, 40, 140])
                colourUpper = np.array([140, 160, 255])

                height = frame.shape[0]
                width = frame.shape[1]

                mask = cv2.inRange(blurred, colourLower, colourUpper)
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

                kiri = mask[0:200, 0:270]
                kanan = mask[0:200, 360:640]
                bawah = mask[280:380, 200:450]
                bawahpol = mask[380:480, 200:450]

                tengah = mask[0:200,275:355]

                cnts_kanan = cv2.findContours(kanan, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cnts_kanan = imutils.grab_contours(cnts_kanan)

                cnts_kiri = cv2.findContours(kiri, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cnts_kiri = imutils.grab_contours(cnts_kiri)

                cnts_bawah = cv2.findContours(bawah, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cnts_bawah = imutils.grab_contours(cnts_bawah)

                cnts_bawahpol = cv2.findContours(bawahpol, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cnts_bawahpol = imutils.grab_contours(cnts_bawahpol)

                cnts_tengah = cv2.findContours(tengah, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cnts_tengah = imutils.grab_contours(cnts_tengah)


                if len(cnts_kanan)>0 and len(cnts_bawahpol) > 0:
                    moving(0.0,0.0)
                    moving(1.0,-0.8)

                elif len(cnts_kanan)>0 and len(cnts_bawah) > 0:
                    moving(0.0,0.0)
                    moving(0.4,-0.8)

                elif len(cnts_kiri)>0 and len(cnts_bawahpol) > 0:
                    moving(0.0,0.0)
                    moving(1,0.8)

                elif len(cnts_kiri)>0 and len(cnts_bawah) > 0:
                    moving(0.0,0.0)
                    moving(0.4,0.8)

                elif len(cnts_kanan)>0:
                    moving(0.0,0.0)
                    moving(0.0,-0.8)

                elif len(cnts_kiri)>0:
                    moving(0.0, 0.0)
                    moving(0.0,0.8)

                elif len(cnts_bawah)>0:
                    moving(0.4,0.0)

                elif len(cnts_bawahpol) > 0:
                    moving(1, 0.0)

                elif len(cnts_tengah)>5:
                    moving(0.0,0.0)

                lower_red = np.array([136, 87, 111])
                upper_red = np.array([180, 255, 255])

                mask1 = cv2.inRange(hsv, lower_red, upper_red)

                cnts1 = cv2.findContours(mask1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                cnts1 = imutils.grab_contours(cnts1)

                for c in cnts1:
                    area = cv2.contourArea(c)
                    if area > 5000:
                        cv2.drawContours(frame, [c], -1, (0, 255, 0), 3)

                        M = cv2.moments(c)

                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])

                        cv2.circle(frame, (cx, cy), 7, (255, 255, 255), -1)
                        cv2.putText(frame, "Merah", (cx - 20, cy - 20), cv2.FONT_HERSHEY_SIMPLEX, 2.5, (255, 255, 255),
                                    3)

                        steering = 0

                        xspeed = 0.0
                        yspeed = 0.0

                        ismoving = 0
                        turning = 0



                cv2.rectangle(frame, (0, 0), (270, 200), (0, 255, 0), 3)
                cv2.putText(frame, 'Kiri', (80, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

                cv2.rectangle(frame, (360, 0), (640, 200), (0, 255, 0), 3)
                cv2.putText(frame, 'Kanan', (500, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

                cv2.rectangle(frame, (200, 280), (450, 380), (0, 255, 0), 3)
                cv2.putText(frame, 'gas', (300, 320), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

                cv2.rectangle(frame, (200, 380), (450, 480), (0, 255, 0), 3)
                cv2.putText(frame, 'GASPOL', (270, 400), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

            else:
                if turning == 1 or ismoving == 1:
                    moving(xspeed, yspeed)

                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                grayframe = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

                lower_red = np.array([136, 87, 111])
                upper_red = np.array([180, 255, 255])

                lower_yellow = np.array([22, 93, 0])
                upper_yellow = np.array([45, 255, 255])

                lower_green = np.array([50, 50, 72])
                upper_green = np.array([70, 255, 255])

                lower_blue = np.array([94, 80, 2])
                upper_blue = np.array([120, 255, 255])

                mask1 = cv2.inRange(hsv, lower_red, upper_red)
                mask2 = cv2.inRange(hsv, lower_yellow, upper_yellow)
                mask3 = cv2.inRange(hsv, lower_green, upper_green)
                mask4 = cv2.inRange(hsv, lower_blue, upper_blue)

                cnts1 = cv2.findContours(mask1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                cnts1 = imutils.grab_contours(cnts1)

                cnts2 = cv2.findContours(mask2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                cnts2 = imutils.grab_contours(cnts2)

                cnts3 = cv2.findContours(mask3, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                cnts3 = imutils.grab_contours(cnts3)

                cnts4 = cv2.findContours(mask4, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                cnts4 = imutils.grab_contours(cnts4)

                for c in cnts1:
                    area = cv2.contourArea(c)
                    if area > 5000:
                        cv2.drawContours(frame, [c], -1, (0, 255, 0), 3)

                        M = cv2.moments(c)

                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])

                        cv2.circle(frame, (cx, cy), 7, (255, 255, 255), -1)
                        cv2.putText(frame, "Merah", (cx - 20, cy - 20), cv2.FONT_HERSHEY_SIMPLEX, 2.5, (255, 255, 255),
                                    3)

                        xspeed = 0.0
                        yspeed = 0.0

                        ismoving = 0
                        turning = 0

                for c in cnts2:
                    area = cv2.contourArea(c)
                    if area > 5000:
                        cv2.drawContours(frame, [c], -1, (0, 255, 0), 3)

                        M = cv2.moments(c)

                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])

                        if turning == 1:
                            turning = 0
                        if ismoving == 0:
                            ismoving = 1
                        xspeed = 0.3
                        yspeed = 0.0

                        cv2.circle(frame, (cx, cy), 7, (255, 255, 255), -1)
                        cv2.putText(frame, "Kuning", (cx - 20, cy - 20), cv2.FONT_HERSHEY_SIMPLEX, 2.5, (255, 255, 255),
                                    3)

                for c in cnts3:
                    area = cv2.contourArea(c)
                    if area > 5000:
                        cv2.drawContours(frame, [c], -1, (0, 255, 0), 3)

                        M = cv2.moments(c)

                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])

                        if turning == 1:
                            turning = 0
                        if ismoving == 0:
                            ismoving = 1
                        xspeed = 0.7
                        yspeed = 0.0

                        cv2.circle(frame, (cx, cy), 7, (255, 255, 255), -1)
                        cv2.putText(frame, "Hijau", (cx - 20, cy - 20), cv2.FONT_HERSHEY_SIMPLEX, 2.5, (255, 255, 255),
                                    3)


                code = decode(frame)
                for barcode in decode(frame):
                    myData = barcode.data.decode('utf-8')
                    pts = np.array([barcode.polygon], np.int32)
                    pts = pts.reshape((-1, 1, 2))
                    cv2.polylines(frame, [pts], True, (0, 127, 255), 5)
                    pts2 = barcode.rect
                    cv2.putText(frame, myData, (pts2[0], pts2[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 127, 255), 2)

                    if myData == 'kiri':
                        if turning == 0:
                            turning = 1
                        xspeed = 0.0
                        yspeed = 0.8
                    elif myData == 'kanan':
                        if turning == 0:
                            turning = 1
                        xspeed = 0.0
                        yspeed = -0.8

                gray_blurred = cv2.blur(grayframe, (3, 3))
                circle = cv2.HoughCircles(gray_blurred, cv2.HOUGH_GRADIENT, 1, 20, param1=100, param2=80, minRadius=1,
                                          maxRadius=250)
                if circle is not None:
                    circle = np.uint16(np.around(circle))

                    for pt in circle[0, :]:
                        a, b, r = pt[0], pt[1], pt[2]
                        cv2.putText(frame, "Circles Detected", (a, b), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 127, 255), 2)
                        cv2.circle(frame, (a, b), r, (0, 255, 0), 2)

                    if steering == 0:
                        steering = 1

                cv2.imshow("result", frame)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break




            cv2.imshow("result", frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        cap.release()
        cv2.destroyAllWindows()
    except rospy.ROSInterruptException:
        pass