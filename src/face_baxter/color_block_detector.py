#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import Image

rospy.init_node("blob_finder")
'display for Baxter'


# display_pub= rospy.Publisher('/robot/xdisplay',Image, queue_size=10)
# name_pub = rospy.Publisher('name', String, queue_size=10)
location_pub = rospy.Publisher()

class BlobFinder(object):
    def __init__(self, color):

        eps = 30

        self.minB = max(color[0] - eps, 0)
        self.minG = max(color[1] - eps, 0)
        self.minR = max(color[2] - eps, 0)

        self.maxB = min(color[0] + eps, 255)
        self.maxG = min(color[1] + eps, 255)
        self.maxR = min(color[2] + eps, 255)

    def mask_img(self, img):
        maskB = np.logical_and(self.minB < img[:, :, 0], self.maxB > img[:, :, 0])
        # maskB = self.minB<img[:,:,0]

        maskG = np.logical_and(self.minG < img[:, :, 1], self.maxG > img[:, :, 1])
        maskR = np.logical_and(self.minR < img[:, :, 2], self.maxR > img[:, :, 2])
        #

        mask = np.logical_and(np.logical_and(maskB, maskG), maskR)

        img2 = np.copy(img)
        img2[np.logical_not(mask)] = (0, 0, 0)

        kernel = np.ones((3, 3), np.uint8)
        erosion = cv2.erode(img2, kernel, iterations=1)

        pts = []
        for y in range(erosion.shape[0]):
            for x in range(erosion.shape[1]):
                if mask[y, x]:
                    pts.append((y, x))

        if len(pts) < 70:  # arbitray criteria
            return erosion, None, None

        sum_pt = (0.0, 0.0)
        for pt in pts:
            sum_pt = (sum_pt[0] + pt[0], sum_pt[1] + pt[1])

        mean_pt = (int(sum_pt[1] / len(pts)), int(sum_pt[0] / len(pts)))

        return erosion, mean_pt[0], mean_pt[1]  # image, y, x


class AdaptiveDoer(object):
    def __init__(self):
        self.name_to_blobfinder = {}
        self.current_name = ''

    def enroll(self, name, blob_finder):
        self.name_to_blobfinder[name] = blob_finder

    def callbackImg(self, msg):
        # Capturing image of camera

        if not self.current_name:
            print("no active person seen")
            return

        if self.current_name not in self.name_to_blobfinder:
            print("active;y seen person name not assigned to color")
            return

        blob_finder = self.name_to_blobfinder[self.current_name]

        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        masked, x, y = blob_finder.mask_img(cv_image)
        print("object at X={}, Y={}".format(x, y))
        if x and y:
            cv2.circle(masked, (x,y), 3, (0, 0, 255), 2)
        cv2.imshow("cv_image", cv_image)
        cv2.imshow("testing", masked)
        cv2.waitKey(1)

    def callbackSighting(self,msg):
        self.current_name = msg.data



if __name__ == "__main__":
    adaptive_doer = AdaptiveDoer()
    adaptive_doer.enroll('Chelsey', BlobFinder(color=(126, 212, 148)))  # Greenish BGR
    adaptive_doer.enroll('Daniel', BlobFinder(color=(101, 74, 40)))  # Blueish BGR)
    adaptive_doer.enroll('rob', BlobFinder(color=(170, 190, 175)))  # Blueish BGR)

    #adaptive_doer.current_name=''
    "For testing with webcam"
    name_sub = rospy.Subscriber('/name', String, adaptive_doer.callbackSighting, None, 1)

    #WEbcam
    #sub = rospy.Subscriber('/cv_camera/image_raw', Image, adaptive_doer.callbackImg, None, 1)

    #Baxter Cameras: using habd camera
    right_camera = baxter_interface.CameraController("right_hand_camera")
    right_camera.open()
    # left__camera = baxter_interface.CameraController("left_hand_camera")
    # left_camera.close()

    head_camera = baxter_interface.CameraController("head_camera")

    head_camera.resolution = (960, 600)
    head_camera.close()
    camera_name = "right_hand_camera"
    sub = rospy.Subscriber('/cameras/' + camera_name + "/image", Image, adaptive_doer.callbackImg, None, 1)

    rospy.spin()
