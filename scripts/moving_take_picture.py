#!/usr/bin/env python2
  
import rospy
import tf2_ros
import math
import geometry_msgs.msg
from camera_meta_msgs.srv import TakePictureCameraMeta

class MovingTakePicture(object):

  def __init__(self):
    
    self.tfBuffer = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.tfBuffer)
    
    self.service_client = rospy.ServiceProxy("/robot/camera_meta/take_picture", TakePictureCameraMeta)
    try:
      trans = self.tfBuffer.lookup_transform('map', 'robot_base_footprint', rospy.Time())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      return
    self.last_point = trans.transform.translation

  def control_loop(self):
    try:
      trans = self.tfBuffer.lookup_transform('map','robot_base_footprint', rospy.Time())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      return
    trans_pos = trans.transform.translation
    distance = math.sqrt((self.last_point.x-trans.x) ** 2 + (self.last_point.y-trans.y) ** 2)
    if(distance > 1.5):
      srv_msg = TakePictureCameraMeta()
      self.service_client.call(srv_msg)
      self.last_point = trans.transform.translation

  def run(self):
    loop = rospy.Rate(5)
    while not rospy.is_shutdown():
      self.control_loop()
      loop.sleep()

if __name__ == '__main__':
    rospy.init_node("moving_take_picture", anonymous=True)
    my_node = MovingTakePicture()
    my_node.run()
