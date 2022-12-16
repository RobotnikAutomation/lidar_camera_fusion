#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from sony_camera_bridge_msgs.srv import TakePicture
from std_srvs.srv import Trigger
import cv2
from cv_bridge import CvBridge, CvBridgeError
import time
import glob
import os

class ImagePublisher(object):

  def __init__(self):
    self.node_rate = 1
    self.latest_file = ""
    self.path = ""
    rospy.init_node('image_publisher')
    self.image_pub = rospy.Publisher("image_topic", Image, queue_size=10)
    self.bridge = CvBridge()

    rospy.wait_for_service('/sony_camera_bridge/get_folder_path')
    try:
        get_image_path = rospy.ServiceProxy('/sony_camera_bridge/get_folder_path', Trigger)
        result = get_image_path()
        self.path = result.message
        print(self.path)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

    # time.sleep(5)
    # self.image_pub.publish(self.image_message)

  def doSmth(self):
      list_of_files = glob.glob(self.path + '/*') # * means all if need specific format then *.csv
      file = max(list_of_files, key=os.path.getctime)
      print(file)

      if(len(list_of_files) == 0 or file == self.latest_file):
        return

      cv_image = cv2.imread(file, 0)
      try:
        image_message = self.bridge.cv2_to_imgmsg(cv_image, "passthrough")
      except CvBridgeError as e:
        print(e)

      self.image_pub.publish(self.image_message)

      self.latest_file = file

  def run(self):
    loop = rospy.Rate(self.node_rate)
    while not rospy.is_shutdown():
      self.doSmth()
      loop.sleep()

if __name__=='__main__':
    try:
        node = ImagePublisher()
    except rospy.ROSInterruptException:
        pass
    node.run()
    rospy.spin()
    rospy.loginfo("Exiting")

