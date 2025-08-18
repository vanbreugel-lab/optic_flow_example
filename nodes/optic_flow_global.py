#!/usr/bin/env python3

# Command line arguments
import serial
import rospy
import math
from optparse import OptionParser

# ROS imports
import roslib, rospy

# opencv imports
import cv2

# numpy imports - basic math and matrix manipulation
import numpy as np

# imports for ROS image handling
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import PoseStamped

# message imports specific to this package
from optic_flow_example.msg import OpticFlowMsg

################################################################################

def draw_optic_flow_field(gray_image, points, flow):
    '''
    gray_image: opencv gray image, e.g. shape = (width, height)
    points: points at which optic flow is tracked, e.g. shape = (npoints, 1, 2)
    flow: optic flow field, should be same shape as points
    '''
    color_img = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)
    color_red = [0,255,0] # bgr colorspace
    linewidth = 2
    for i, point in enumerate(points):
        x = int(point[0,0])
        y = int(point[0,1])
        vx = int(flow[i][0,0])
        vy = int(flow[i][0,1])
        # print('x: ', x, 'y: ', y, 'vx: ', vx, 'vy: ', vy)
        cv2.line(color_img, (x,y), (x+vx, y+vy), color_red, linewidth) # draw a red line from the point with vector = [vx, vy]        
    
    cv2.imshow('optic_flow_field',color_img)
    cv2.waitKey(1)

################################################################################
    
def define_points_at_which_to_track_optic_flow(image, spacing):
    points_to_track = []
    for x in range(0,image.shape[0],spacing):
        for y in range(0,image.shape[1],spacing):
            new_point = [y, x]
            points_to_track.append(new_point)
    points_to_track = np.array(points_to_track, dtype=np.float32) # note: float32 required for opencv optic flow calculations
    points_to_track = points_to_track.reshape(points_to_track.shape[0], 1, points_to_track.shape[1]) # for some reason this needs to be shape (npoints, 1, 2)
    return points_to_track

################################################################################

class Optic_Flow_Calculator:
    def __init__(self, topic):
        # Define the source of the images, e.g. rostopic name
        self.image_source = topic
        
        # Initialize image aquisition
        self.bridge = CvBridge()
        self.prev_image = None
        self.last_time = 0

        # initialize mocap variables
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0
        self.qw = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        # Lucas Kanade Optic Flow parameters
        self.lk_params = dict( winSize  = (15,15),
                               maxLevel = 2,
                               criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
        
        # Lucas Kanade Publishers
        self.optic_flow_pub = rospy.Publisher("optic_flow", OpticFlowMsg, queue_size=10)
        self.optic_flow_pub_mean_x = rospy.Publisher("/optic_flow/camera/mean/x", Float32, queue_size=10)
        self.optic_flow_pub_mean_y = rospy.Publisher("/optic_flow/camera/mean/y", Float32, queue_size=10)
        self.optic_flow_wind_mean_x = rospy.Publisher("/optic_flow/wind_sensor/mean/x", Float32, queue_size=10)
        self.optic_flow_wind_mean_y = rospy.Publisher("/optic_flow/wind_sensor/mean/y", Float32, queue_size=10)
        self.optic_flow_global_mean_x = rospy.Publisher("/optic_flow/global/mean/x", Float32, queue_size=10)
        self.optic_flow_global_mean_y = rospy.Publisher("/optic_flow/global/mean/y", Float32, queue_size=10)
        
        # Raw Image Subscriber
        self.image_sub = rospy.Subscriber(self.image_source,Image,self.image_callback)

        # Roll, Pitch, Yaw Subscribers (Float32MultiArray)
        # self.rpy_radians_sub = rospy.Subscriber('/drone/rpy/radians', Float32MultiArray, self.rpy_radians_callback)
        # self.yaw = 0.0

        #mocap callback
        self.mocap_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.mocap_callback)

    def mocap_callback(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z
        self.qx = msg.pose.orientation.x
        self.qy = msg.pose.orientation.y
        self.qz = msg.pose.orientation.z
        self.qw = msg.pose.orientation.w
        self.roll, self.pitch, self.yaw = self.euler_from_quaternion(self.qx, self.qy, self.qz, self.qw)

    def unroll_unpitch(self,R,P):
        C=np.matrix([[np.cos(P) , np.sin(R)*np.sin(P), np.cos(R)*np.sin(P)]
                        ,[0 , np.cos(R) , -1*np.sin(R)]
                        ,[-1*np.sin(P) , np.sin(R)*np.cos(P) , np.cos(R)*np.sin(P)]])
        return C

    def get_Cyaw(self,Y):
        Cyaw=np.matrix([[np.cos(Y) , 1*np.sin(Y), 0]
                        ,[-1*np.sin(Y) , np.cos(Y) , 0]
                        ,[0 , 0 , 1]])
        return Cyaw

    def get_Cpitch(self,P):
        Cpitch=np.matrix([[np.cos(P) , 0, -1*np.sin(P)]
                        ,[0, 1 , 0]
                        ,[np.sin(P) , 0 , np.cos(P)]])

        return Cpitch

    def get_Croll(self,R):
        Croll=np.matrix([[1 , 0, 0]
                        ,[0, np.cos(R) , np.sin(R)]
                        ,[0 , -1*np.sin(R) , np.cos(R)]])

        return Croll


    def euler_from_quaternion(self,x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians
        
    def rpy_radians_callback(self, msg):
        # Assuming the RPY is published in the order [roll, pitch, yaw]
        self.yaw = msg.data[-1]  # Extract yaw as the last element in the array
        

    def image_callback(self,image):
        try: # if there is an image
            # Acquire the image, and convert to single channel gray image
            curr_image = self.bridge.imgmsg_to_cv2(image, desired_encoding="mono8")

            # Define the region of interest (ROI)
            B = 140  # Example coordinates
            curr_image = curr_image[B:-B, B:-B]
            
            # Get time stamp
            secs = image.header.stamp.secs
            nsecs = image.header.stamp.nsecs
            curr_time = float(secs) + float(nsecs)*1e-9
            
            # If this is the first loop, initialize image matrices
            if self.prev_image is None:
                self.prev_image = curr_image
                self.last_time = curr_time
                self.points_to_track = define_points_at_which_to_track_optic_flow(curr_image, 30)
                return # skip the rest of this loop
                
            # get time between images
            dt = curr_time - self.last_time
            
            # calculate optic flow with lucas kanade
            new_position_of_tracked_points, status, error = cv2.calcOpticalFlowPyrLK(self.prev_image, curr_image, self.points_to_track, None, **self.lk_params)
              
            # calculate flow field
            flow = new_position_of_tracked_points - self.points_to_track
            
            # draw the flow field
            draw_optic_flow_field(curr_image, self.points_to_track, flow)
            
            # calculate and publish median optic flow per second
            mean_flow_x = Float32(1.0*np.median(flow[:,0,0])/dt)
            mean_flow_y = Float32(1.0*np.median(flow[:,0,1])/dt)

            self.optic_flow_pub_mean_x.publish(mean_flow_x)
            self.optic_flow_pub_mean_y.publish(mean_flow_y)

            # roll camera to wind sensor reference frame 180 degrees
            V_camera = np.matrix([[mean_flow_x.data],[mean_flow_y.data],[0]])
            C_roll = self.get_Croll(np.pi)
            V_wind = C_roll*V_camera

            # publish wind sensor optic flow
            mean_flow_x_wind = Float32(V_wind[0])
            mean_flow_y_wind = Float32(V_wind[1])
            self.optic_flow_wind_mean_x.publish(mean_flow_x_wind)
            self.optic_flow_wind_mean_y.publish(mean_flow_y_wind)

            # yaw the optic flow wind sensor to the global reference frame
            print('Yaw: ', self.yaw)
            C_yaw = self.get_Cyaw(self.yaw)
            V_global = C_yaw*V_wind

            # publish global optic flow
            mean_flow_x_global = Float32(V_global[1])
            mean_flow_y_global = Float32(V_global[0])
            self.optic_flow_global_mean_x.publish(mean_flow_x_global)
            self.optic_flow_global_mean_y.publish(mean_flow_y_global)


            # # bring from camera to wind sensor reference frame
            # mean_flow_x_s = 1.0*np.median(flow[:,0,0])/dt
            # mean_flow_y_s = -1.0*np.median(flow[:,0,1])/dt

            # # bring from wind sensor to body level reference frame
            # mean_flow_x_b = -1.0*mean_flow_y_s
            # mean_flow_y_b = mean_flow_x_s
            
            # # bring optic flow into global reference frame with a yaw rotation
            # # print('Yaw: ', self.yaw*180/np.pi)
            # # print('Mean Flow X: ', mean_flow_x_g)
            # # print('Mean Flow Y: ', mean_flow_y_g)
            # global_mean_flow_x_g = 1*(mean_flow_x_b * np.cos(self.yaw) + mean_flow_y_b * np.sin(self.yaw))
            # global_mean_flow_y_g = 1*(mean_flow_x_b * -1.0*np.sin(self.yaw) + mean_flow_y_b * np.cos(self.yaw))
            # # print('Global Mean Flow X: ', global_mean_flow_x_g)
            # # print('Global Mean Flow Y: ', global_mean_flow_y_g)
            # self.optic_flow_global_mean_x.publish(Float32(global_mean_flow_x_g))
            # self.optic_flow_global_mean_y.publish(Float32(global_mean_flow_y_g))
            
            # publish optic flow data to rostopic
            msg = OpticFlowMsg()
            msg.header.stamp.secs = secs
            msg.header.stamp.nsecs = nsecs
            msg.dt = dt
            msg.x = self.points_to_track[:,0,0]
            msg.y = self.points_to_track[:,0,1]
            msg.vx = flow[:,0,0]
            msg.vy = flow[:,0,1]
            self.optic_flow_pub.publish(msg)
            
            # save current image and time for next loop
            self.prev_image = curr_image
            self.last_time = curr_time
            
        except CvBridgeError as e:
            print(e)
            
    def main(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
            cv2.destroyAllWindows()
            
################################################################################

if __name__ == '__main__':    
    parser = OptionParser()
    parser.add_option("--topic", type="str", dest="topic", default='/camera/color/image_raw',
                      help="ROS topic with Image message for optic flow calculation")
    (options, args) = parser.parse_args()

    rospy.init_node('optic_flow_calculator', anonymous=True)
    optic_flow_calculator = Optic_Flow_Calculator(options.topic)
    optic_flow_calculator.main()
