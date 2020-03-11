"""
FORT : Building a Fort with DE NIRO Baxter Robot
Phase 0 - Pick a Brick

Modified from Baxter SDK example (https://sdk.rethinkrobotics.com/wiki/Home)

Written by : Aisling Tai, Jingtong Ng, Rachel Brown
Reviewed and commented by: Clara Arcos, Rachel Brown
Last updated: 2020
"""

import argparse
import struct
import sys
import copy

import rospy
import rospkg

from gazebo_msgs.srv import (SpawnModel, DeleteModel)
from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion)
from std_msgs.msg import (Header, Empty)

from baxter_core_msgs.srv import (SolvePositionIK, SolvePositionIKRequest)

import baxter_interface

class PickAndPlace(object):
    def __init__(self, limb, hover_distance = 0.10, verbose=True):
        self._limb_name = limb # string
        self._hover_distance = hover_distance # in meters
        self._verbose = verbose # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)
        # verify robot is enabled
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

    def place(self, pose):
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract()

def load_gazebo_models(table_pose=Pose(position=Point(x=0.95, y=0.425, z=0.0)),
                       table_reference_frame="world",
                       table1_pose=Pose(position=Point(x=0.95, y=-0.425, z=0.0)),
                       table1_reference_frame="world",
                       brick1_pose=Pose(position=Point(x=0.5897, y=0.7, z=0.82)),
                       brick1_reference_frame="world",
                       brick2_pose=Pose(position=Point(x=0.5897, y=-0.7, z=0.82)),
                       brick2_reference_frame="world"):
    # Load Table SDF
    table_xml = ''
    with open ("models/cafe_table/model.sdf", "r") as table_file:
        table_xml=table_file.read().replace('\n', '')
    # Load Table1 SDF    
    table1_xml = ''
    with open ("models/cafe_table/model.sdf", "r") as table1_file:
        table1_xml=table1_file.read().replace('\n', '')

    # Load brick1 sdf
    brick1_xml = ''
    with open ("models/brick/model.sdf", "r") as brick1_file:
        brick1_xml=brick1_file.read().replace('\n', '')

    # Load brick2 sdf
    brick2_xml = ''
    with open ("models/brick/model.sdf", "r") as brick2_file:
        brick2_xml=brick2_file.read().replace('\n', '')

    # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
                             table_pose, table_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

    # Spawn Table1 SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("cafe_table1", table_xml, "/",
                             table1_pose, table1_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

    # Spawn Brick1 sdf
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("brick1", brick1_xml, "/",
                             brick1_pose, brick1_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

    # Spawn Brick2 sdf
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("brick2", brick2_xml, "/",
                             brick2_pose, brick2_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))



def load_gazebo_models_brick_only(
                       brick3_pose=Pose(position=Point(x=0.5897, y=0.7, z=0.82)),
                       brick3_reference_frame="world"):

    # Load brick sdf
    brick3_xml = ''
    with open ("models/brick/model.sdf", "r") as brick3_file:
        brick3_xml=brick3_file.read().replace('\n', '')

    # Spawn Brick sdf
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("brick3", brick3_xml, "/",
                             brick3_pose, brick3_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

def delete_gazebo_models():
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("cafe_table")
        resp_delete = delete_model("cafe_table1")
        resp_delete = delete_model("brick1")
        resp_delete = delete_model("brick2")
        resp_delete = delete_model("brick3")
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))

def main():
    """Simple pick and place example"""
    rospy.init_node("ik_pick_and_place_demo")

    hover_distance = 0.1 # meters
    # Starting Pose for left arm round 1
    lstart_pose1 = Pose()
    lstart_pose1.position.x = 0.579679836383
    lstart_pose1.position.y = 0.283311769707
    lstart_pose1.position.z = 0.413676720426
    lstart_pose1.orientation.x = -0.0249590815779
    lstart_pose1.orientation.y = 0.999649402929
    lstart_pose1.orientation.z = 0.00737916180073
    lstart_pose1.orientation.w = 0.00486450832011

    # Starting Pose for right arm
    rstart_pose1 = Pose()
    rstart_pose1.position.x = 0.579679836383
    rstart_pose1.position.y = -0.283311769707
    rstart_pose1.position.z = 0.413676720426
    rstart_pose1.orientation.x = -0.0249590815779
    rstart_pose1.orientation.y = 0.999649402929
    rstart_pose1.orientation.z = -0.00737916180073
    rstart_pose1.orientation.w = 0.00486450832011

    # Picking pose left
    lpick_pose1 = Pose()
    lpick_pose1.position.x = 0.589679836383
    lpick_pose1.position.y = 0.70
    lpick_pose1.position.z = 0.153676720426
    lpick_pose1.orientation.x = -0.0249590815779
    lpick_pose1.orientation.y = 0.999649402929
    lpick_pose1.orientation.z = 0.00737916180073
    lpick_pose1.orientation.w = 0.00486450832011

    # Picking pose right
    rpick_pose1 = Pose()
    rpick_pose1.position.x = 0.589679836383
    rpick_pose1.position.y = -0.70
    rpick_pose1.position.z = 0.153676720426
    rpick_pose1.orientation.x = -0.0249590815779
    rpick_pose1.orientation.y = 0.999649402929
    rpick_pose1.orientation.z = 0.00737916180073
    rpick_pose1.orientation.w = 0.00486450832011

    # Placing pose left arm round 1
    lplace_pose1 = Pose()
    lplace_pose1.position.x = 0.589679836383
    lplace_pose1.position.y = 0.0
    lplace_pose1.position.z = 0.153676720426
    lplace_pose1.orientation.x = -0.0249590815779
    lplace_pose1.orientation.y = 0.999649402929
    lplace_pose1.orientation.z = 0.00737916180073
    lplace_pose1.orientation.w = 0.00486450832011

    # Starting Pose for left arm round 2

    lstart_pose2 = Pose()
    lstart_pose2.position.x = 0.579679836383
    lstart_pose2.position.y = 0.283311769707
    lstart_pose2.position.z = 0.413676720426
    lstart_pose2.orientation.x = -0.0249590815779
    lstart_pose2.orientation.y = 0.999649402929
    lstart_pose2.orientation.z = 0.00737916180073
    lstart_pose2.orientation.w = 0.00486450832011

    # Picking pose round 2
    lpick_pose2 = Pose()
    lpick_pose2.position.x = 0.589679836383
    lpick_pose2.position.y = 0.70
    lpick_pose2.position.z = 0.153676720426
    lpick_pose2.orientation.x = -0.0249590815779
    lpick_pose2.orientation.y = 0.999649402929
    lpick_pose2.orientation.z = 0.00737916180073
    lpick_pose2.orientation.w = 0.00486450832011

    # Picking pose round 2
    lplace_pose2 = Pose()
    lplace_pose2.position.x = 0.589679836383
    lplace_pose2.position.y = 0.0
    lplace_pose2.position.z = 0.213676720426
    lplace_pose2.orientation.x = -0.0249590815779
    lplace_pose2.orientation.y = 0.999649402929
    lplace_pose2.orientation.z = 0.00737916180073
    lplace_pose2.orientation.w = 0.00486450832011

    # Picking pose round 2
    rplace_pose1 = Pose()
    rplace_pose1.position.x = 0.589679836383
    rplace_pose1.position.y = 0.0
    rplace_pose1.position.z = 0.263676720426
    rplace_pose1.orientation.x = -0.0249590815779
    rplace_pose1.orientation.y = 0.999649402929
    rplace_pose1.orientation.z = 0.00737916180073
    rplace_pose1.orientation.w = 0.00486450832011



    # Initialise pick and place
    left_pnp = PickAndPlace('left', hover_distance)
    right_pnp = PickAndPlace('right', hover_distance)

    # Go to initial position
    left_pnp.move_to_start(left_pnp.ik_request(lstart_pose1))
    right_pnp.move_to_start(right_pnp.ik_request(rstart_pose1))

    # Load Gazebo Models via Spawning Services
    load_gazebo_models()

    print("\nPicking...")
    left_pnp.pick(lpick_pose1)

    print("\nPicking...")
    right_pnp.pick(rpick_pose1)

    print("\nPlacing...")
    left_pnp.place(lplace_pose1)

    load_gazebo_models_brick_only()

    print("\nstarting...")
    left_pnp.move_to_start(left_pnp.ik_request(lstart_pose2))

    print("\nPicking...")
    left_pnp.pick(lpick_pose2)

    print("\nPlacing...")
    left_pnp.place(lplace_pose2)

    rospy.sleep(5.0)

    rospy.on_shutdown(delete_gazebo_models)

if __name__ == '__main__':
    sys.exit(main())