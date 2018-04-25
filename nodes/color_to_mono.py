#!/usr/bin/env python

# ROS imports
import roslib, rospy

# opencv imports
import cv2

# numpy imports - basic math and matrix manipulation
import numpy as np

# imports for ROS image handling
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage

# command line arguments
from optparse import OptionParser

################################################################################

class Image_Converter:
    def __init__(self, topic, mode='rgb8'):
        # Define the source of the images, e.g. rostopic name
        self.image_source = topic
        self.mode = mode # node, the actual mode doesn't really matter if we're converting to mono
        
        # Initialize image aquisiti
        self.bridge = CvBridge()
        
        # Raw Image subscriber
        self.image_sub = rospy.Subscriber(self.image_source,Image,self.image_callback)

        # Mono image publisher
        self.image_pub = rospy.Publisher("/camera/mono", Image, queue_size=3)
        
    def image_callback(self,image):
        try:
            # Acquire the image, and convert to single channel gray image
            curr_image = self.bridge.imgmsg_to_cv2(image, desired_encoding='mono8')
                
            # Publish the image as a mono8 msg
            msg = self.bridge.cv2_to_imgmsg(curr_image)
            msg.encoding = 'mono8'
            self.image_pub.publish(msg)
            
        except CvBridgeError, e:
            print e
            
    def main(self):
      try:
        rospy.spin()
      except KeyboardInterrupt:
        print "Shutting down"
      cv2.destroyAllWindows()

################################################################################

#if __name__ == '__main__':
#    
#    main()

if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option("--topic", type="str", dest="topic", default='/usb_cam/image_raw',
                        help="name of the topic with your color image stream")
    (options, args) = parser.parse_args()

    rospy.init_node('color_to_gray_image_conversion', anonymous=True)
    image_converter = Image_Converter(options.image)    
    image_converter.main()
