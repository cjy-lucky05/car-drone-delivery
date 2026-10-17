#!/usr/bin/env python

import rospy
import cv2
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
class fake_camera:
    def __init__(self):
        self.image_path = rospy.get_param('~image_path','bingda.png')
        self.image_pub = rospy.Publisher("/image_raw", Image, queue_size=3)
        self.bridge = CvBridge() 
        self.pub_image()
    def pub_image(self):
        
        self.rate = rospy.Rate(30)
        self.cv_image = cv2.imread(self.image_path,1)
        rospy.loginfo("Start Publish Fake Camera Image")
        while not rospy.is_shutdown():
            img_msg = self.bridge.cv2_to_imgmsg(self.cv_image, "bgr8")
            img_msg.header.stamp = rospy.Time.now()
            self.image_pub.publish(img_msg)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node("fake_camera",anonymous=False)
        fake_camera()
    except rospy.ROSInternalException:
        pass