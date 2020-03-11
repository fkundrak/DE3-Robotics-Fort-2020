"""
FORT : Building a Fort with DE NIRO Baxter Robot
Phase 3 - Building A Fort In Gazebo Using Two Arms

Modified from Baxter SDK example (https://sdk.rethinkrobotics.com/wiki/Home)

Written by : Aisling Tai, Jingtong Ng, Rachel Brown
Reviewed and commented by: Clara Arcos, Rachel Brown
Last updated: 2020
"""

import argparse
import struct
import sys
import copy
import tf
import os
import rospy
import rospkg
import threading
import time


from gazebo_msgs.srv import (SpawnModel, DeleteModel)
from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion)
from std_msgs.msg import (Header, Empty)

from baxter_core_msgs.srv import (SolvePositionIK, SolvePositionIKRequest)

import baxter_interface

class PickAndPlace(object):
    def __init__(self, limb, hover_distance = 0.15, verbose=True):
        self._limb_name = limb # string
        self._hover_distance = hover_distance # in meters
        self._verbose = verbose # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)
        # verify that robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0]*7))
        self._guarded_move_to_joint_position(start_angles)
        self.gripper_open()
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")

    def move_to_position(self, angles=None):
        print("Moving the {0} arm to interim pose...".format(self._limb_name))
        if not angles:
            angles = dict(zip(self._joint_names, [0]*7))
        self._guarded_move_to_joint_position(angles)
        rospy.sleep(0.5)
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
                print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
                         (seed_str)))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
                print("IK Joint Solution:\n{0}".format(limb_joints))
                print("------------------")
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

    def gripper_missed(self):
        return self._gripper.missed()

    def gripper_position(self):
        return self._gripper.position()

    def gripper_gripping(self):
        self._gripper.set_moving_force(50.0)
        return self._gripper.gripping()

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

    def pick(self, pose):
        # open the gripper
        self.gripper_open()
        # servo above pose
        print('Approaching')
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        print('Ready to grip')
        # close gripper
        self.gripper_close()
        print('grip')
        # retract to clear object
        self._retract()

    def pick2(self, pose):
        # servo to pose
        self._servo_to_pose(pose)
        print('moving')

    def place(self, pose):
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract()


def poseratioant(px,py,pz,roll,pitch,yaw):
    """
    This function takes position and Euler angles and returns the pose
    It's used for the bricks that need to be rotated
    """
    quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    my_pose_msg = Pose()
    my_pose_msg.position.x = px
    my_pose_msg.position.y = py
    my_pose_msg.position.z = pz
    my_pose_msg.orientation.x = quat[0]
    my_pose_msg.orientation.y = quat[1]
    my_pose_msg.orientation.z = quat[2]
    my_pose_msg.orientation.w = quat[3]

    return my_pose_msg


def posedefined(px,py,pz,ox,oy,oz,ow):
    """
    This function takes position and quartenions and returns the pose
    It's used for the bricks that don't need to be rotated, as 'poseratioant'
    was found to misbehave with these bricks
    """
    my_pose_msg = Pose()
    my_pose_msg.position.x = px
    my_pose_msg.position.y = py
    my_pose_msg.position.z = pz
    my_pose_msg.orientation.x = ox
    my_pose_msg.orientation.y = oy
    my_pose_msg.orientation.z = oz
    my_pose_msg.orientation.w = ow

    return my_pose_msg


def load_gazebo_models_tables(
        table_pose=Pose(position=Point(x=0.8, y=0.0, z=0.0)),
        table_reference_frame="world",
        rtable_pose=Pose(position=Point(x=0.8, y=-1.075, z=0.0)),
        rtable_reference_frame="world",
        ltable_pose=Pose(position=Point(x=0.8, y=1.075, z=0.0)),
        ltable_reference_frame="world"):
    # Load Table SDF
    table_xml = ''
    with open ("models/cafe_table/model.sdf", "r") as table_file:
        table_xml=table_file.read().replace('\n', '')

    # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
                             table_pose, table_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

    # Load Table SDF
    ltable_xml = ''
    with open ("models/side_table/model.sdf", "r") as ltable_file:
        ltable_xml=ltable_file.read().replace('\n', '')

    # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("ltable", ltable_xml, "/",
                             ltable_pose, ltable_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

    # Load Table SDF
    rtable_xml = ''
    with open ("models/side_table/model.sdf", "r") as rtable_file:
        rtable_xml=rtable_file.read().replace('\n', '')

    # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("rtable", rtable_xml, "/",
                             rtable_pose, rtable_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))


# Spawning of Bricks in Gazebo - learn more in the Wiki -----------------------
# (Phase 1: Building A Fort In Gazebo Using One Arm) --------------------------
def load_rbrick(
        brickid, # Takes brickid (generated in main()) as an input to allow for multiple models
        brick_pose=Pose(position=Point(x=0.5897, y=-0.7, z=0.82)),
        brick_reference_frame="world"):

    # Load Brick SDF
    brick_xml = ''
    with open ("models/brick/model.sdf", "r") as brick_file:
        brick_xml=brick_file.read().replace('\n', '')

    # Spawn Brick SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_urdf = spawn_urdf("%s" %(brickid), brick_xml, "/",
                               brick_pose, brick_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))


def load_lbrick(
        brickid,
        brick_pose=Pose(position=Point(x=0.5897, y=0.7, z=0.82)),
        brick_reference_frame="world"):
        
    # Load Brick SDF
    brick_xml = ''
    with open ("models/brick/model.sdf", "r") as brick_file:
        brick_xml=brick_file.read().replace('\n', '')

    # Spawn Brick SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_urdf = spawn_urdf("%s" %(brickid), brick_xml, "/",
                               brick_pose, brick_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))


def load_rbrickv(
        brickid,
        brick_pose=Pose(position=Point(x=0.5897,y= -0.7,z= 0.775+0.192/2)),
        brick_reference_frame="world"):
    # Load Vertical Brick SDF
    brick_xml = ''
    with open ("models/brickv/model.sdf", "r") as brick_file:
        brick_xml=brick_file.read().replace('\n', '')

    # Spawn Vertical Brick SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_urdf = spawn_urdf("%s" %(brickid), brick_xml, "/",
                               brick_pose, brick_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))


def load_lbrickv(
        brickid,
        brick_pose=Pose(position=Point(x=0.5897,y= 0.7,z= 0.775+0.192/2)),
        brick_reference_frame="world"):
    # Load Vertical Brick SDF
    brick_xml = ''
    with open ("models/brickv/model.sdf", "r") as brick_file:
        brick_xml=brick_file.read().replace('\n', '')

    # Spawn Vertical Brick SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_urdf = spawn_urdf("%s" %(brickid), brick_xml, "/",
                               brick_pose, brick_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
# -----------------------------------------------------------------------------


def delete_gazebo_models():
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

        # Base bricks
        resp_delete = delete_model("brickL1")
        resp_delete = delete_model("brickR1")
        resp_delete = delete_model("brickL2")
        resp_delete = delete_model("brickR2")
        resp_delete = delete_model("brickL3")
        resp_delete = delete_model("brickR3")

        # Column bricks
        resp_delete = delete_model("brickRc4")
        resp_delete = delete_model("brickRc5")
        resp_delete = delete_model("brickRc6")
        resp_delete = delete_model("brickLc4")
        resp_delete = delete_model("brickLc5")
        resp_delete = delete_model("brickLc6")

        # Top bricks
        resp_delete = delete_model("brickL7")
        resp_delete = delete_model("brickR7")
        resp_delete = delete_model("brickL8")
        resp_delete = delete_model("brickR8")
        resp_delete = delete_model("brickL9")
        resp_delete = delete_model("brickR9")
        resp_delete = delete_model("brickL10")
        resp_delete = delete_model("brickR10")

        # Tables
        resp_delete = delete_model("cafe_table")
        resp_delete = delete_model("ltable")
        resp_delete = delete_model("rtable")

    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))


# Threading For Usage of Both Arms - learn more in the Wiki -------------------
# (Phase 4: Building A Fort With Baxter Using Two Arms) -----------------------
class ArmThread(threading.Thread):
    def __init__(self, threadID, arm_name):
        super(ArmThread, self).__init__() # Initialise threads
        # threading.Thread.__init__(self)
        self.threadID = threadID # Two threads will be running, IDs are 1 and 2
        self.arm_name = arm_name # One thread per arm

    def run(self): # Run both arms
        if self.arm_name == "l":
            full_l() # Run left arm
        if self.arm_name == "r":
            full_r() # Run right arm
        else:
            print("error")
#------------------------------------------------------------------------------


def main():

    global full_l
    global full_r

    rospy.init_node("ik_pick_and_place_demo") # Initialise node
    delete_gazebo_models()

    hover_distance = 0.1 # In meters
    
    # Starting position -------------------------------------
    lstart_pose1 = Pose()
    lstart_pose1.position.x = 0.579679836383
    lstart_pose1.position.y = 0.3
    lstart_pose1.position.z = 0.413676720426
    lstart_pose1.orientation.x = -0.0249590815779
    lstart_pose1.orientation.y = 0.999649402929
    lstart_pose1.orientation.z = 0.00737916180073
    lstart_pose1.orientation.w = 0.00486450832011

    rstart_pose1 = Pose()
    rstart_pose1.position.x = 0.579679836383
    rstart_pose1.position.y = -0.3
    rstart_pose1.position.z = 0.413676720426
    rstart_pose1.orientation.x = -0.0249590815779
    rstart_pose1.orientation.y = 0.999649402929
    rstart_pose1.orientation.z = -0.00737916180073
    rstart_pose1.orientation.w = 0.00486450832011
    #--------------------------------------------------------

    # Horizontal Brick Picking Position ---------------------
    lpick_pose1 = Pose()
    lpick_pose1.position.x = 0.589679836383
    lpick_pose1.position.y = 0.7
    lpick_pose1.position.z = 0.1
    lpick_pose1.orientation.x = -0.0249590815779
    lpick_pose1.orientation.y = 0.999649402929
    lpick_pose1.orientation.z = 0.00737916180073
    lpick_pose1.orientation.w = 0.00486450832011

    rpick_pose1 = Pose()
    rpick_pose1.position.x = 0.589679836383
    rpick_pose1.position.y = -0.7
    rpick_pose1.position.z = 0.1
    rpick_pose1.orientation.x = -0.0249590815779
    rpick_pose1.orientation.y = 0.999649402929
    rpick_pose1.orientation.z = 0.00737916180073
    rpick_pose1.orientation.w = 0.00486450832011
    #--------------------------------------------------------

    # Horizontal Brick Temporary Position -------------------
    ltemp_pose = Pose()
    ltemp_pose.position.x = 0.619679836383
    ltemp_pose.position.y = 0.15
    ltemp_pose.position.z = 0.12
    ltemp_pose.orientation.x = -0.0249590815779
    ltemp_pose.orientation.y = 0.999649402929
    ltemp_pose.orientation.z = 0.00737916180073
    ltemp_pose.orientation.w = 0.00486450832011

    rtemp_pose = Pose()
    rtemp_pose.position.x = 0.639679836383
    rtemp_pose.position.y = -0.15
    rtemp_pose.position.z = 0.12
    rtemp_pose.orientation.x = -0.0249590815779
    rtemp_pose.orientation.y = 0.999649402929
    rtemp_pose.orientation.z = 0.00737916180073
    rtemp_pose.orientation.w = 0.00486450832011
    #--------------------------------------------------------

    # Vertical Brick Temporary Position ---------------------
    lctemp_pose = Pose()
    lctemp_pose.position.x = 0.629679836383
    lctemp_pose.position.y = 0.2
    lctemp_pose.position.z = 0.216
    lctemp_pose.orientation.x = -0.0249590815779
    lctemp_pose.orientation.y = 0.999649402929
    lctemp_pose.orientation.z = 0.00737916180073
    lctemp_pose.orientation.w = 0.00486450832011

    rctemp_pose = Pose()
    rctemp_pose.position.x = 0.629679836383
    rctemp_pose.position.y = -0.2
    rctemp_pose.position.z = 0.216
    rctemp_pose.orientation.x = -0.0249590815779
    rctemp_pose.orientation.y = 0.999649402929
    rctemp_pose.orientation.z = 0.00737916180073
    rctemp_pose.orientation.w = 0.00486450832011
    #--------------------------------------------------------

    # Standby Position (after placing a brick) --------------
    lstandby_pose = Pose()
    lstandby_pose.position.x = 0.639679836383
    lstandby_pose.position.y = 0.2
    lstandby_pose.position.z = 0.32
    lstandby_pose.orientation.x = -0.0249590815779
    lstandby_pose.orientation.y = 0.999649402929
    lstandby_pose.orientation.z = 0.00737916180073
    lstandby_pose.orientation.w = 0.00486450832011

    rstandby_pose = Pose()
    rstandby_pose.position.x = 0.639679836383
    rstandby_pose.position.y = -0.2
    rstandby_pose.position.z = 0.32
    rstandby_pose.orientation.x = -0.0249590815779
    rstandby_pose.orientation.y = 0.999649402929
    rstandby_pose.orientation.z = 0.00737916180073
    rstandby_pose.orientation.w = 0.00486450832011
    #--------------------------------------------------------

    # Vertical Brick Picking Position -----------------------
    lpick_poseL456 = posedefined(  0.5897,
                            0.7, 
                            0.2256,
                            -0.0249590815779,
                            0.999649402929,
                            0.00737916180073,
                            0.00486450832011)

    rpick_poseR456 = posedefined(  0.5897,
                            -0.7,
                            0.2256,
                            -0.0249590815779,
                            0.999649402929,
                            0.00737916180073,
                            0.00486450832011)
    #--------------------------------------------------------

    os.system('rosrun baxter_tools tuck_arms.py -u')

    # Initialise pick and place
    left_pnp = PickAndPlace('left', hover_distance)
    right_pnp = PickAndPlace('right', hover_distance)

    # Go to initial position
    left_pnp.move_to_start(left_pnp.ik_request(lstart_pose1))
    right_pnp.move_to_start(right_pnp.ik_request(rstart_pose1))

    print("\nLoading Table...")
    load_gazebo_models_tables()

    # Execute Left Arm --------------------------------------
    def full_l():
        
        ldata = [   ["brickL2",0, 0.66,  0.212, -0.255, -0.0249590815779, 0.999649402929, 0.00737916180073, 0.00486450832011],
                ["brickL1",0, 0.809, 0.169, -0.255, 3.14, 0, 3.14/2],
                ["brickL3",0, 0.597, 0.053, -0.255, 3.14, 0, -3.14/2],
                ["brickLc4",1, 0.799, 0.159, -0.06632796, 3.14, 0, 3.14/2],
                ["brickLc5",1, 0.597, 0.01, -0.07632796, -0.0249590815779 , 0.999649402929, 0.00737916180073, 0.00486450832011],
                ["brickLc6",1, 0.597, 0.159, -0.06632796, 3.14, 0, 3.14/2],
                ["brickL7",0, 0.597, 0.121, -0.001, 3.14, 0, 3.14/2],
                ["brickL8",0, 0.809, 0.121, -0.001, 3.14, 0, 3.14/2],
                ["brickL9",0, 0.703, 0.159, 0.061, -0.0249590815779, 0.999649402929, 0.00737916180073, 0.00486450832011] ]
        # [BrickName,0 for horizontal / 1 for vertical, x, y, z, roll, pitch, yaw) or [BrickName,0 for horizontal / 1 for vertical, x, y, z, x, y ,z ,w)

        for li in range(len(ldata)):
            print("\nLoading ", ldata[li][0], "...")

            if (ldata[li][1]==1):
                load_lbrickv(ldata[li][0])
                print("\nPicking ", ldata[li][0], "...")
                left_pnp.move_to_position(left_pnp.ik_request(lstandby_pose))
                
                # Gripper Checking - learn more in the Wiki (Phase 5: Gripper Checking)
                lgripping = False

                while not lgripping:
                    left_pnp.pick(lpick_poseL456) # Attempt to pick the brick
                    print("\nFLOAT position", left_pnp.gripper_position())
                    if left_pnp.gripper_position() > 20: # If the brick has been picked...
                        lgripping = True # Break the loop

                print("\nPlacing ", ldata[li][0], "...")

                if (ldata[li][0] == 9):
                    left_pnp.pick2(lstart_pose1)
                else:
                    left_pnp.pick2(lctemp_pose)
            else:
                load_lbrick(ldata[li][0])

                print("\nPicking ", ldata[li][0], "...")

                # Gripper Checking
                lgripping = False

                while not lgripping:
                    left_pnp.pick(lpick_pose1)
                    print("\nFLOAT position", left_pnp.gripper_position())
                    if left_pnp.gripper_position() > 20:
                        lgripping = True

                print("\nPlacing ", ldata[li][0], "...")
                left_pnp.pick2(ltemp_pose)

            if len(ldata[li])==8:
                print("\nrationant ")
                lplace_pose = poseratioant(ldata[li][2],ldata[li][3]-.01,ldata[li][4]-0.01,ldata[li][5],ldata[li][6],ldata[li][7])
                # Creates the pose using the function 'poseratioant'
            else:
                print("\ndefined ")
                lplace_pose = posedefined(ldata[li][2],ldata[li][3]-.01,ldata[li][4]-0.01,ldata[li][5],ldata[li][6],ldata[li][7],ldata[li][8])
                # Creates the pose using the function 'posedefined'

            left_pnp.place(lplace_pose)
            left_pnp.move_to_position(left_pnp.ik_request(ltemp_pose))
        left_pnp.move_to_start(left_pnp.ik_request(lstart_pose1))
    # -------------------------------------------------------

    # Execute Right Arm -------------------------------------
    def full_r ():
        time.sleep(15)

        rdata = [   ["brickR1",0, 0.597, -0.149, -0.255, 3.14, 0, -3.14/2],
                ["brickR3",0, 0.799, -0.033, -0.255, 3.14, 0, 3.14/2],
                ["brickR2",0, 0.746, -0.192, -0.255, -0.0249590815779, 0.999649402929, 0.00737916180073, 0.00486450832011],
                ["brickRc4",1, 0.819, 0.01, -0.07632796, -0.0249590815779 , 0.999649402929, 0.00737916180073, 0.00486450832011],
                ["brickRc5",1, 0.819, -0.139, -0.07632796, 3.14, 0, 3.14/2],
                ["brickRc6",1, 0.587,-0.139, -0.07632796, 3.14, 0, 3.14/2],
                ["brickR7",0, 0.819, -0.111, -0.001, 3.14, 0, -3.14/2],
                ["brickR8",0, 0.597, -0.101, -0.001, 3.14, 0, -3.14/2],
                ["brickR9",0, 0.703, 0, 0.061, -0.0249590815779, 0.999649402929, 0.00737916180073, 0.00486450832011],
                ["brickR10",0, 0.703, -0.139, 0.061, -0.0249590815779, 0.999649402929, 0.00737916180073, 0.00486450832011] ]
        # [BrickName,0 for horizontal / 1 for vertical, x, y, z, roll, pitch, yaw) or [BrickName,0 for horizontal / 1 for vertical, x, y, z, x, y ,z ,w)

        for ri in range(len(rdata)):
            print("\nLoading ", rdata[ri][0], "...")

            if (rdata[ri][1]==1):

                load_rbrickv(rdata[ri][0])

                print("\nPicking ", rdata[ri][0], "...")
                right_pnp.move_to_position(right_pnp.ik_request(rstandby_pose))

                # Gripper Checking
                rgripping = False

                while not rgripping:
                    right_pnp.pick(rpick_poseR456)
                    print("\nFLOAT position", right_pnp.gripper_position())
                    if right_pnp.gripper_position() > 20:
                        rgripping = True

                print("\nPlacing ", rdata[ri][0], "...")
                right_pnp.pick2(rctemp_pose)

            else:

                load_rbrick(rdata[ri][0])
                print("\nPicking ", rdata[ri][0], "...")

                # Gripper Checking
                rgripping = False

                while not rgripping:
                    right_pnp.pick(rpick_pose1)
                    print("\nFLOAT position", right_pnp.gripper_position())
                    if right_pnp.gripper_position() > 20:
                        rgripping = True

                print("\nPlacing ", rdata[ri][0], "...")
                right_pnp.pick2(rtemp_pose)

            if len(rdata[ri])==8:
                print("\nrationant ")
                rplace_pose = poseratioant(rdata[ri][2],rdata[ri][3]-.01,rdata[ri][4]-0.01,rdata[ri][5],rdata[ri][6],rdata[ri][7])
                # Creates the pose using the function 'poseratioant'
            else:
                print("\ndefined ")
                rplace_pose = posedefined(rdata[ri][2],rdata[ri][3]-.01,rdata[ri][4]-0.01,rdata[ri][5],rdata[ri][6],rdata[ri][7],rdata[ri][8])
                # Creates the pose using the function 'posedefined'

            right_pnp.place(rplace_pose)
            right_pnp.move_to_position(right_pnp.ik_request(rtemp_pose))

        right_pnp.move_to_start(right_pnp.ik_request(rstart_pose1))
    # -------------------------------------------------------

    thread1 = ArmThread(1, "l") # Initialise threads
    thread2 = ArmThread(2, "r")

    thread1.start() # Start both threads
    thread2.start()

    print("\nFort Building Complete!")

    rospy.sleep(5.0)

    rospy.on_shutdown(delete_gazebo_models)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        rospy.on_shutdown(delete_gazebo_models)
