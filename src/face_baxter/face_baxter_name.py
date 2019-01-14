#!/usr/bin/env python
# import baxter_interface
import rospy
import cv2
import face_recognition
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
rospy.init_node("face_finder")
'display for Baxter'
display_pub= rospy.Publisher('/robot/xdisplay',Image, queue_size=10)
name_pub = rospy.Publisher('name', String, queue_size=10)

bridge = CvBridge()

daniel_image = face_recognition.load_image_file("daniel.jpg")
chelsey_image = face_recognition.load_image_file("chelsey.jpg")
daniel_face_encoding = face_recognition.face_encodings(daniel_image)[0]
chelsey_face_encoding = face_recognition.face_encodings(chelsey_image)[0]
known_face_encodings = [daniel_face_encoding, chelsey_face_encoding]
known_face_names = ['Daniel', 'Chelsey']

def callback(msg):
    global known_face_encodings
    global known_face_names
    # Initialize some variables
    face_locations = []
    face_encodings = []
    face_names = []
    process_this_frame = True

    try:
        # Convert your ROS Image message to OpenCV2
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg

        # Resize frame of video to 1/4 size for faster face recognition processing
        small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)

        # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
        rgb_small_frame = small_frame[:, :, ::-1]

        # Only process every other frame of video to save time
        if process_this_frame:
            # Find all the faces and face encodings in the current frame of video
            t0 = time.time()
            face_locations = face_recognition.face_locations(rgb_small_frame)
            print(time.time()-t0, "sec detection")
            t0 = time.time()
            face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)
            print(time.time()-t0, "sec embedding")

            # face_names = []
            print(len(face_encodings), "faces to check")
            t0 = time.time()
            for face_encoding in face_encodings:
                # See if the face is a match for the known face(s)
                matches = face_recognition.compare_faces(known_face_encodings, face_encoding)
                name = "Unknown"

                # If a match was found in known_face_encodings, just use the first one.
                if True in matches:
                    first_match_index = matches.index(True)
                    name = known_face_names[first_match_index]

                    # face_names.append(name)
                    name_pub.publish(name)

            print(time.time()-t0, "sec search")


        process_this_frame = not process_this_frame

        # Display the results
        # for (top, right, bottom, left), name in zip(face_locations, face_names):
        #     # Scale back up face locations since the frame we detected in was scaled to 1/4 size
        #     top *= 4
        #     right *= 4
        #     bottom *= 4
        #     left *= 4
        #
        #     # Draw a box around the face
        #     cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)
        #
        #     # Draw a label with a name below the face
        #     cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
        #     font = cv2.FONT_HERSHEY_DUPLEX
        #     cv2.putText(frame, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)

        # Display the resulting image
        cv2.imshow('Video', frame)

        # Hit 'q' on the keyboard to quit!
        cv2.waitKey(1)


# "For testing with webcam"
# sub = rospy.Subscriber('/cv_camera/image_raw', Image, callback, None, 1)


"""For Baxter"""
right_camera = baxter_interface.CameraController("right_hand_camera")
right_camera.close()
#left__camera = baxter_interface.CameraController("left_hand_camera")
#left_camera.close()

head_camera = baxter_interface.CameraController("head_camera")

head_camera.resolution =(960, 600)
head_camera.open()
camera_name = "head_camera"
sub = rospy.Subscriber('/cameras/' + camera_name + "/image", Image,republish,None,1)
rospy.spin()
