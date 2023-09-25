#! /usr/bin/env python3

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospy
import cv2
import std_msgs.msg as std_msgs
import numpy as np

class tape_follower:

    def __init__(self):
        self.pub = rospy.Publisher("/cmd_vel",Twist, queue_size=1)
        self.sub = rospy.Subscriber("/rrbot/camera1/image_raw", Image, self.callback)
        self.bridge = CvBridge()

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            frame_height, frame_width, channels = cv_image.shape

            turn_value = self.find_road(cv_image, frame_height, frame_width)

            movement = Twist()
            movement.linear.x = 0.5
            movement.angular.z = turn_value * 2.5
            
            self.pub.publish(movement)

        except CvBridgeError as e:
            print(e)
    

    ## @brief Erodes a grayscale image to remove noise.
    #
    # This function performs morphological erosion on a given grayscale
    # image using a specified kernel size. The erosion operation is 
    # particularly useful for removing small noise particles from a binary 
    # image, and for separating closely connected objects.
    #
    # @param imgray The input grayscale image.
    # @return The eroded version of the input image.
    
    def erode_image(self, imgray):
        kernel_size = 3  # Adjust the size as needed
        kernel = np.ones((kernel_size, kernel_size), np.uint8)

        # Apply erosion on the binary image using the binary mask
        eroded_image = cv2.erode(imgray, kernel)
        return eroded_image

    ## @brief Computes the center of the road based on a row of pixel intensities.
    #
    # This function computes the median of the indices of non-zero pixels in a 
    # specified row of the input image. The row is selected based on the frame 
    # height input and the method returns the median of these indices as the 
    # estimated center of the road.
    #
    # @param image The input image.
    # @param frame_height The height of the frame to calculate which row to analyze.
    # @return The estimated center of the road, or 0 if no non-zero pixels are found.
    
    def center_of_road(self, image, frame_height):
        #Twentieth Row from bottom
        row_of_pixels = image[frame_height - 20, :]

        #Returns a tuple, one for row and one column. Values of each element hold
        #the index of a pixel on the original array that passed the condition
        #We only passed it one row, so we only care about columns, [0] is columns
        non_zero_indices = np.where(row_of_pixels > 0)[0]

        #If no non-zero values found
        if len(non_zero_indices) == 0:
            return 0

        #Get median value from these list of indices, round incase it is non-int
        median = np.median(non_zero_indices)

        return int(median)
    
    def find_road(self, image, frame_height, frame_width):

        imgray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ret,th1 = cv2.threshold(imgray,127,255,cv2.THRESH_BINARY_INV)
        eroded_image = self.erode_image(th1)

        # Find centre of road
        centerx_of_circle = self.center_of_road(eroded_image, frame_height)

        if centerx_of_circle < frame_width / 2:
           return -1
        elif centerx_of_circle > frame_width / 2:
           return 1
        else:
           return 0

def main():

    rospy.init_node('tape_follower', anonymous=True)
    tf_node = tape_follower()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
