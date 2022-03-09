import cv2
import numpy as np
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import string

vid = cv2.VideoCapture(-1)
while(True):
    ret,frame = vid.read()

    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break

    cv2.imshow("frame",frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
vid.release()
cv2.destroyAllWindows()