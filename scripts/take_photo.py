

 #!/usr/bin/env python
from __future__ import print_function
#import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    #self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=20)
    
    self.bridge = CvBridge()
    self.image_topic = '/camera/color/image_raw'
    #self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback)

  def take_image(self):
    ros_image = rospy.wait_for_message(self.image_topic, Image, rospy.Duration(4))
    cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
    return cv_image
  
  def save_image(self, image, filename='photo1.png'):
    cv2.imwrite(filename, image)
    
  def take_and_save_image(self):
    self.save_image(self.take_image())

  #def callback(self,data):
  #  try:
  #    cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
  #  except CvBridgeError as e:
  #    print(e)

  #  (rows,cols,channels) = cv_image.shape
    #if cols > 60 and rows > 60 :
    #  cv2.circle(cv_image, (50,50), 10, 255)

  #  cv2.imshow("Image window", cv_image)
  #  if not self.var:
  #    cv2.imwrite("photo1.png", cv_image)
  #  cv2.waitKey(2000)#show img za 2 sek
    
  #  self.var = 1


def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter() 
  ic.take_and_save_image()
  #rospy.sleep(4) 
  #cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)