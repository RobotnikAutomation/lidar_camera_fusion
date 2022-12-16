#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import time

class ImageRePublisher(object):

  def __init__(self):
    self.node_rate = 10
    self.image_message = Image()
    self.image_subs = rospy.Subscriber('/image_topic', Image, self.republish_cb)
    self.image_re_pub = rospy.Publisher("image_republisher_topic", Image, queue_size=1)

  def republish_cb(self, msg):
    self.image_message = msg
    self.image_message.header.stamp = rospy.Time.now()
    self.image_re_pub.publish(self.image_message)

if __name__ == '__main__':
    rospy.init_node("image_republisher", anonymous=True)
    my_node = ImageRePublisher()
    rospy.spin()
    rospy.loginfo("Exiting")

