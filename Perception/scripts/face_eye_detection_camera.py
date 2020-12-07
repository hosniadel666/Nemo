#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

detector = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
eye_cascade = cv2.CascadeClassifier('haarcascade_eye.xml')

class image_converter:

  def __init__(self):
    rospy.init_node('image_converter', anonymous=True)
    self.image_pub = rospy.Publisher("/face_eyes_det", Image, queue_size=10)
    self.image_sub = rospy.Subscriber("/usb_cam_node/image_raw",Image,self.callback)
    self.bridge = CvBridge()
    rospy.spin()

  def callback(self, data):
    try:
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
        
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    faces = detector.detectMultiScale(gray, 1.3, 5)
    for (x, y, w, h) in faces:
        cv2.rectangle(cv_image, (x, y), (x + w, y + h), (255, 0, 0), 2)
        roi_gray = gray[y:y + h, x:x + w]
        roi_color = cv_image[y:y + h, x:x + w]
        eyes = eye_cascade.detectMultiScale(roi_gray, minNeighbors=15)
        for (ex, ey, ew, eh) in eyes:
            cv2.rectangle(roi_color, (ex, ey), (ex + ew, ey + eh), (0, 255, 0), 2)


    #cv2.imshow("face_eyes_det window", cv_image)
    #cv2.waitKey(3)

    try:
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
        print(e)



if __name__ == '__main__':
    try:

        image_converter()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()


