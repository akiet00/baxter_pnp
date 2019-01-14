#!/usr/bin/env python

import sys
import copy
import rospy
import cv2
import cv_bridge
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import baxter_interface
from   baxter_interface import Gripper, RobotEnable
import numpy as np

from std_msgs.msg import (
    Header,
    String,
    Empty,
    Int32,
)

#Gazebo spawn service
from gazebo_msgs.srv import(
    SpawnModel,
    DeleteModel,
)
from geometry_msgs.msg import(
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from baxter_core_msgs.msg import EndEffectorState
from sensor_msgs.msg import Image
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose, PoseStamped
from baxter_core_msgs.srv import ( SolvePositionIK,
                                   SolvePositionIKRequest )

from baxter_builder.srv import *
#from PickAndPlace import PickAndPlace
import rospkg

sleep_flag = True
xpos = 0
ypos = 0
zpos = 0
grip_force = 0

def init():
    #Wake up Baxter
    baxter_interface.RobotEnable().enable()
    rospy.sleep(0.25)
    print "Baxter is enabled"

    print "Intitializing clients for services"
    global ik_service_left
    ik_service_left = rospy.ServiceProxy(
            "ExternalTools/left/PositionKinematicsNode/IKService",
            SolvePositionIK)

    global ik_service_right
    ik_service_right = rospy.ServiceProxy(
            "ExternalTools/right/PositionKinematicsNode/IKService",
            SolvePositionIK)

    global obj_loc_service
    obj_loc_service = rospy.ServiceProxy(
        "object_location_service", ObjLocation)

    global stopflag
    stopflag = False

    # Activate Baxter's interface
    global right_arm
    right_arm = baxter_interface.Limb('right')

    global right_gripper
    right_gripper = baxter_interface.Gripper('right')
    right_gripper.calibrate()
    rospy.loginfo( "Right gripper parameters: %s"  %(right_gripper.parameters()))

    global left_arm
    left_arm = baxter_interface.Limb('left')

    '''
    global end_effector_subs
    end_effector_subs = rospy.Subscriber("/robot/end_effector/right_gripper/state", EndEffectorState, end_effector_callback)
    rospy.sleep(1)'''

    global pubpic
    pubpic = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
    rospy.sleep(1)

def end_effector_callback(msg):
    global grip_force
    grip_force = msg.force

def load_gazebo_models(table_pose=Pose(position=Point(x=1.0, y=0.0, z=0.0)),
                       table_reference_frame="world",
                       block_pose=Pose(position=Point(x=0.6725, y=0.1265, z=0.7825)),
                       block_reference_frame="world"):
    # Get Models' Path
    model_path = rospkg.RosPack().get_path('baxter_builder')+"/models/"
    # Load Table SDF
    table_xml = ''
    with open (model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml=table_file.read().replace('\n', '')
    # Load RedBlock URDF
    block_xml = ''
    with open (model_path + "block/model.urdf", "r") as block_file:
        block_xml=block_file.read().replace('\n', '')

    # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
                             table_pose, table_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
    
    # Spawn RedBlock URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block", block_xml, "/",
                               block_pose, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

def delete_gazebo_models():
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("cafe_table")
        resp_delete = delete_model("block")
        resp_delete = delete_model("GreenBlock")
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))

# Method to pick up an object then drop it off at a designated
# location given the Cartesian coordiate of the object
# @param picker: A PickAndPlace object
# @param xpos, ypos, zpos: X,Y,Z coordinates of the object
# @param objColor: The color of the object
def pickup_and_dropoff(picker, xpos, ypos, zpos, objColor):
    # Create a pose out of the given object's coordinate
    goal_pose = Pose()
    goal_pose.position = Point(xpos,ypos,zpos)
    goal_pose.orientation = Quaternion(1.00, 0.00, 0.00, 0.00)

    # Send the goal pose to PickAndPlace object
    picker.pick(goal_pose)

    drop_pose_pos = Point(
            x=0.7,
            y=0.15,
            z=-0.129)
    drop_pose_orientation = Quaternion(
            x=-0.0249590815779,
            y=0.999649402929,
            z=.00737916180073)
    drop_pose = Pose(drop_pose_pos, drop_pose_orientation)
    picker.place(drop_pose)


class PickAndPlace:
    def __init__(self, limb, hover_distance = 0.15, verbose=True):
        self._limb_name = limb
        self._hover_distance = hover_distance # in meters
        self._verbose = verbose # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)
        # verify robot is enabled
        # print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

    def move_to_start(self):
        rospy.loginfo("Moving the {0} arm to start pose...".format(self._limb_name))
        # Predefined optimum start angles for the right arm
        '''start_angles = {'right_w0': 0.6699952259595108,
                             'right_w1': 1.030009435085784,
                             'right_w2': -0.4999997247485215,
                             'right_e0': -1.189968899785275,
                             'right_e1': 1.9400238130755056,
                             'right_s0': -0.08000397926829805,
                             'right_s1': -0.9999781166910306}'''
        '''start_pose_pos = Point(
            x= 0.75,
            y= 0.15,
            z= 0)
        start_pose_orientation=Quaternion(
                             x=-0.0249590815779,
                             y= 1.5,
                             z=0.00737916180073,
                             w=0.00486450832011)

        start_angles = self.ik_request(Pose(start_pose_pos, start_pose_orientation))'''
        
        #manually find these start angles
        start_angles =  {'right_s0': 1.4277416288545524,
                            'right_s1': 0.41811490551312985,
                            'right_w0': 2.880780461192421,
                            'right_w1': 1.7882523440818268,
                            'right_w2': 0.43036855388708734,
                            'right_e0': -2.856620708299036,
                            'right_e1': 0.6692594876579591}
        #rospy.loginfo('Starting angles:\n %s' % start_angles)
        if not start_angles:
            rospy.logerr("Unsucceffully move to start position")
            self.move_to_start() # try again
        #start_angles = dict(zip(self._joint_names, [0]*7)) #Error: _joint_names is not defined
        self._guarded_move_to_joint_position(start_angles)
        self.gripper_open()
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")

    def ik_request(self, pose):
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        try:
            resp = self._iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        limb_joints = {}
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            if self._verbose:
                rospy.logerr("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
                         (seed_str)))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
                rospy.loginfo("IK Joint Solution:\n{0}".format(limb_joints))
                print("-----------------------------------------------------\nclear")
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False
        return limb_joints

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)

    def _approach(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance
        joint_angles = self.ik_request(approach)
        self._guarded_move_to_joint_position(joint_angles)

    def _retract(self):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x 
        ik_pose.position.y = current_pose['position'].y 
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x 
        ik_pose.orientation.y = current_pose['orientation'].y 
        ik_pose.orientation.z = current_pose['orientation'].z 
        ik_pose.orientation.w = current_pose['orientation'].w
        joint_angles = self.ik_request(ik_pose)
        # servo up from current pose
        self._guarded_move_to_joint_position(joint_angles)

    def _servo_to_pose(self, pose):
        # servo down to release
        joint_angles = self.ik_request(pose)
        self._guarded_move_to_joint_position(joint_angles)

    # Method to pick up an object given the required pose position
    # @param pose: A valid ROS pose
    def pick(self, pose):
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # close gripper
        self.gripper_close()
        # retract to clear object
        self._retract()

    # Method to place an object down onto a platform given the required goal pose
    # @param pose: A valid ROS pose
    def place(self, pose):
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract()


def main():
    rospy.init_node('baxter_picking_colored_cubes_node')
    
    rospy.loginfo("Initializing Baxter Picking node")
    init()

    rospy.loginfo("Loading Gazebo Models")
    load_gazebo_models()

    # delete all Gazebo models on Exit
    rospy.on_shutdown(delete_gazebo_models)

    # Wait for all clear from gazebo
    rospy.wait_for_message("/robot/sim/started", Empty)
    
    # Initialize pick and place
    pnp = PickAndPlace('right')

    rospy.loginfo("Move to Starting Pose")
    pnp.move_to_start()
    rospy.sleep(1.0)
    
    while not rospy.is_shutdown():
        rospy.loginfo("In the loop")
        try:
            rospy.wait_for_service('object_location_service')
            response = obj_loc_service.call(ObjLocationRequest())
            rospy.loginfo("Response from Location Service\n")
            rospy.loginfo(response + "\n---------------------------")

            #if response.objfound == True:
            if response.objfound:
                pickup_and_dropoff(pnp, response.xb, response.yb, response.zb, response.objcolor)
            else:
                #Move to random pose
                rospy.loginfo("No objects found. Now moving to start position")
                pnp.move_to_start()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

if __name__ == '__main__':
    main()
