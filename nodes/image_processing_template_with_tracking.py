#!/usr/bin/env python

# ROS imports
import roslib, rospy

# opencv imports
import cv2, cv

# numpy imports - basic math and matrix manipulation
import numpy as np

# imports for ROS image handling
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# message imports specific to this package
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point

################################################################################

def draw_optic_flow_field(gray_image, points):
    '''
    gray_image: opencv gray image, e.g. shape = (width, height)
    points: points at which optic flow is tracked, e.g. shape = (npoints, 1, 2)
    '''
    color_img = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)
    color_red = [0,0,255] # bgr colorspace
    linewidth = 1
    for i, point in enumerate(points):
        x = point[0]
        y = point[1]
        cv2.circle(color_img, (x,y), 2, color_red, linewidth) # draw a red line from the point with vector = [vx, vy]        
    
    cv2.imshow('optic_flow_field',color_img)
    cv2.waitKey(1)
    
def get_random_tracked_points():
    npoints = np.random.randint(10)
    points = []
    for n in range(npoints):
        p = [np.random.randint(10), np.random.randint(10), 0]
        points.append(p)
    return points

################################################################################

class Image_Processor:
    def __init__(self):
        # Define the source of the images, e.g. rostopic name
        self.image_source = "/camera/image_mono"
        
        # Initialize image aquisition
        self.bridge = CvBridge()
        
        # Previous image
        self.prev_image = None
        
        # Tracked points publisher
        self.tracked_points_pub = rospy.Publisher("tracked_points", PointCloud)
        
        # Raw Image Subscriber
        self.image_sub = rospy.Subscriber(self.image_source,Image,self.image_callback)
        
    def image_callback(self,image):
        try: # if there is an image
            # Acquire the image, and convert to single channel gray image
            curr_image = self.bridge.imgmsg_to_cv2(image, "mono8")
            if len(curr_image.shape) > 2:
                if curr_image.shape[2] > 1: # color image, convert it to gray
                    curr_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) # shape should now be (rows, columns)
                elif curr_image.shape[2] == 1: # mono image, with wrong formatting
                    curr_image = curr_image[:,:,0] # shape should now be (rows, columns)
                
            if self.prev_image is None:
                self.prev_image = curr_image
                return
                
            # optional: resize the image
            # curr_image = cv2.resize(curr_image, (0,0), fx=0.8, fy=0.8) 
            
            # Get time stamp
            secs = image.header.stamp.secs
            nsecs = image.header.stamp.nsecs
            curr_time = float(secs) + float(nsecs)*1e-9
            
            #########################################################################
            # Image processing stuff happens here. Image is acccessed at curr_image
            # Should end up with a list of 3D points, e.g.
            # [[x1,y1,0], [x2,y2,0], [x3,y3,0]]
            tracked_points = get_random_tracked_points() # for example - these should be based on the actual tracked points in the image
            #########################################################################
                        
            # draw the image, and tracking points
            draw_optic_flow_field(curr_image, tracked_points)
            
            # publish optic flow data to rostopic
            msg = PointCloud()
            msg.header.stamp.secs = secs
            msg.header.stamp.nsecs = nsecs
            msg.points = [Point( tracked_point[0], tracked_point[1], tracked_point[2] ) for tracked_point in tracked_points]
            self.tracked_points_pub.publish(msg)
            
            self.prev_image = curr_image
            
        except CvBridgeError, e:
            print e
            
################################################################################
  
def main():
  image_processor = Image_Processor()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv.DestroyAllWindows()

################################################################################

if __name__ == '__main__':
    rospy.init_node('image_processor', anonymous=True)
    main()
