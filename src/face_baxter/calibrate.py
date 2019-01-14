#!/usr/bin/python

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import sys
import numpy as np

'''global data common to all vision algorithms'''
isTracking = False
r = g = b = 0.0
image = np.zeros((640, 480, 3), np.uint8)
trackedImage = np.zeros((640, 480, 3), np.uint8)
imageWidth = imageHeight = 0;


def clickHandler( event, x, y, flags, d):
    global image
    if event == cv2.EVENT_LBUTTONUP:
        print('left button released', x, y)
        print(image[y,x])


# def captureVideo(src):
#     global image, isTracking, trackedImage
#     cap = cv2.VideoCapture(src)
#     if cap.isOpened() and src == 0:
#         ret = cap.set(3, 640) and cap.set(4, 480)
#         # if ret == False:
#         #     print 'Cannot set frame properties, returning'
#         #     return
#
#
#     #    waitTime = time/frame. Adjust accordingly.
#     if src == 0:
#         waitTime = 1
#     if cap:
#         print 'Succesfully set up capture device'
#     else:
#         print 'Failed to setup capture device'
#
#     windowName = 'Input View, press q to quit'
#     cv2.namedWindow(windowName)
#     cv2.setMouseCallback(windowName, clickHandler)
#     while (True):
#         # Capture frame-by-frame
#         ret, image = cap.read()
#         if ret == False:
#             break
#
#         cv2.imshow(windowName, image)
#         inputKey = cv2.waitKey(waitTime) & 0xFF
#         if inputKey == ord('q'):
#             break
#
#     cap.release()
#     cv2.destroyAllWindows()
#
#
# print 'Starting program'
# if __name__ == '__main__':
#     arglist = sys.argv
#     src = 0
#     print 'Argument count is ', len(arglist)
#     if len(arglist) == 2:
#         src = arglist[1]
#     else:
#         src = 0
#
#     captureVideo(src)
# else:
#     print 'Not in main'
#
# # Baxter Cameras: using habd camera
# right_camera = baxter_interface.CameraController("right_hand_camera")
# right_camera.open()
# camera_name = "right_hand_camera"
# sub = rospy.Subscriber('/cameras/' + camera_name + "/image", Image, adaptive_doer.callbackImg, None, 1)
#
# rospy.spin()


# Instantiate CvBridge
bridge = CvBridge()

def image_callback(msg):
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg
        cv2.imshow('camera_image.jpeg', cv2_img)
    windowName = 'Input View, press q to quit'
    cv2.namedWindow(windowName)
    cv2.setMouseCallback(windowName, clickHandler)
    while (True):
        # Capture frame-by-frame
        ret, image = cap.read()
        if ret == False:
            break

        cv2.imshow(windowName, image)
        inputKey = cv2.waitKey(waitTime) & 0xFF
        if inputKey == ord('q'):
            break
	cv2.waitKey(1)

def main():
    rospy.init_node('image_camera_node')
    # Define your image topic
    image_topic = "/cameras/right_hand_camera/image"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c


    rospy.spin()

