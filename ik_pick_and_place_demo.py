#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Baxter RSDK Inverse Kinematics Pick and Place Demo
"""
import argparse
import struct
import sys
import copy

import rospy
import rospkg

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Header,
    Empty,
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import baxter_interface

#Pick and Place Class Use is to move the robot 

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
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # close gripper
        self.gripper_close()
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


# don't mess with this
def load_gazebo_models(table_pose=Pose(position=Point(x=1.0, y=0.0, z=0.0)),
                       table_pose2=Pose(position=Point(x=0.0, y=1.0, z=0.0)),
		       table_reference_frame="world",
                       block_pose=Pose(position=Point(x=0.7334, y=-0.0291, z=0.7749)),
                       block_reference_frame="world"):
    # Get Models' Path
    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"
    # Load Table SDF
    table_xml = ''
    with open (model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml=table_file.read().replace('\n', '')
    # Load Block URDF
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
    # Spawn Table2 SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("cafe_table2", table_xml, "/",
                             table_pose2, table_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
    # Spawn Block URDF
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
        resp_delete = delete_model("cafe_table2")
        resp_delete = delete_model("block")
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))

# main function for all the execution of the class functions
def main():
    """RSDK Inverse Kinematics Pick and Place Example

    A Pick and Place example using the Rethink Inverse Kinematics
    Service which returns the joint angles a requested Cartesian Pose.
    This ROS Service client is used to request both pick and place
    poses in the /base frame of the robot.

    Note: This is a highly scripted and tuned demo. The object location
    is "known" and movement is done completely open loop. It is expected
    behavior that Baxter will eventually mis-pick or drop the block. You
    can improve on this demo by adding perception and feedback to close
    the loop.
    """
    rospy.init_node("ik_pick_and_place_demo")
    # Load Gazebo Models via Spawning Services
    # Note that the models reference is the /world frame
    # and the IK operates with respect to the /base frame
    load_gazebo_models()
    # Remove models from the scene on shutdown
    rospy.on_shutdown(delete_gazebo_models)

    # Wait for the All Clear from emulator startup
    rospy.wait_for_message("/robot/sim/started", Empty)

    limb = 'left'
    hover_distance = 0.15 # meters
    # Starting Joint angles for left arm
    # starting_joint_angles = {'left_w0': 0.7702527147204759,
    #                          'left_w1': 1.0699003547293966,
    #                          'left_w2': -0.7439340610665388,
    #                          'left_e0': -0.6731041905255042,
    #                          'left_e1': 1.006470431827732,
    #                          'left_s0': -0.8243352082669739,
    #                          'left_s1': -0.28686986561936934}

    pos1 = 					{'left_w0': 0.7702527147204759,
                             'left_w1': 1.0699003547293966,
                             'left_w2': -0.7439340610665388,
                             'left_e0': -0.6731041905255042,
                             'left_e1': 1.006470431827732,
                             'left_s0': -0.8243352082669739,
                             'left_s1': -0.28686986561936934}

    pos2 = 					{'left_w0': 1.0076871239985483,
                             'left_w1': 0.9303633017356626,
                             'left_w2': -0.47020928086742075,
                             'left_e0': -0.8198530633155379,
                             'left_e1': 0.901134657320731,
                             'left_s0': -0.9611180955602562,
                             'left_s1': -0.38781057495300336}

    pos3 = 					{'left_w0': 0.9282036274686233,
                             'left_w1': 1.0434510960566956,
                             'left_w2': -0.5795751876307987,
                             'left_e0': -1.030509576467691,
                             'left_e1': 0.8414031411274612,
                             'left_s0': -0.9224510175811869,}

    pos4 = 					{'left_w0': 0.8487201309386982,
                             'left_w1': 1.1565388903777285,
                             'left_w2': -0.6889410943941767,
                             'left_e0': -1.2411660896198442,
                             'left_e1': 0.7816716249341913,
                             'left_s0': -0.8837839396021174,
                             'left_s1': -0.3958645969615503}

    pos5 = 					{'left_w0': 0.7692366344087731,
                             'left_w1': 1.2696266846987614,
                             'left_w2': -0.7983070011575547,
                             'left_e0': -1.4518226027719974,
                             'left_e1': 0.7219401087409214,
                             'left_s0': -0.845116861623048,
                             'left_s1': -0.3998916079658238}

    pos6 = 					{'left_w0': 0.6897531378788481,
                             'left_w1': 1.3827144790197945,
                             'left_w2': -0.9076729079209327,
                             'left_e0': -1.6624791159241505,
                             'left_e1': 0.6622085925476516,
                             'left_s0': -0.8064497836439786,
                             'left_s1': -0.4039186189700973}

    pos7 = 					{'left_w0': 0.6450479737078589,
                             'left_w1': 1.4669651067070815,
                             'left_w2': -0.9722079226074919,
                             'left_e0': -1.8211779008923088,
                             'left_e1': 0.6211254060965746,
                             'left_s0': -0.7661101644082909,
                             'left_s1': -0.39966932474607153}
 
    pos8 = 					{'left_w0': 0.6003428095368699,
                             'left_w1': 1.5512157343943682,
                             'left_w2': -1.036742937294051,
                             'left_e0': -1.979876685860467,
                             'left_e1': 0.5800422196454976,
                             'left_s0': -0.7257705451726033,
                             'left_s1': -0.3954200305220458}

    pos9 = 					{'left_w0': 0.5457108135538279,
                             'left_w1': 1.6908798231351958,
                             'left_w2': -1.120982074590351,
                             'left_e0': -2.245316527612789,
                             'left_e1': 0.5165241764855364,
                             'left_s0': -0.6434187654446097,
                             'left_s1': -0.378645136845695}

    pos10 = 				{'left_w0': 0.5606354822886579,
                             'left_w1': 1.7728695786085311,
                             'left_w2': -1.1155594277330134,
                             'left_e0': -2.406840912997121,
                             'left_e1': 0.49030279280996075,
                             'left_s0': -0.5577219032033797,
                             'left_s1': -0.34531763271274585}

    pos11 = 				{'left_w0': 0.6499131299449006,
                             'left_w1': 1.8107624190011968,
                             'left_w2': -1.0168624589109103,
                             'left_e0': -2.4914799802672087,
                             'left_e1': 0.4975832875789655,
                             'left_s0': -0.45264091388866284,
                             'left_s1': -0.28875702469944353}

    pos12 = 				{'left_w0': 0.8183400707260966,
                             'left_w1': 1.8181357625000154,
                             'left_w2': -0.8212786303129128,
                             'left_e0': -2.5262638676767977,
                             'left_e1': 0.5345708797527454,
                             'left_s0': -0.3121367529402093,
                             'left_s1': -0.20228281938203357}

    pos13 = 				{'left_w0': 1.0707126188357867,
                             'left_w1': 1.8085670272918097,
                             'left_w2': -0.5251954041278932,
                             'left_e0': -2.538222713479633,
                             'left_e1': 0.5974707882914954,
                             'left_s0': -0.12017037579776882,
                             'left_s1': -0.07921452333676134}

    pos14 = 				{'left_w0': 1.2412698536566493,
                             'left_w1': 1.802100329427606,
                             'left_w2': -0.3250978233363083,
                             'left_e0': -2.546304684704547,
                             'left_e1': 0.6399795102234527,
                             'left_s0': 0.009563443150569978,
                             'left_s1': 0.003956916761683235}

    pos15 = 				{'left_w0': 1.4118270884775117,
                             'left_w1': 1.7956336315634018,
                             'left_w2': -0.12500024254472342,
                             'left_e0': -2.554386655929461,
                             'left_e1': 0.68248823215541,
                             'left_s0': 0.13929726209890875,
                             'left_s1': 0.08712835686012779}

    pos16 = 				{'left_w0': 1.604569803681383,
                             'left_w1': 1.7883257685691474,
                             'left_w2': 0.10112532209639913,
                             'left_e0': -2.5635199013161234,
                             'left_e1': 0.7305263379186585,
                             'left_s0': 0.2859063962358068,
                             'left_s1': 0.18111844299647656}

    pos17 = 				{'left_w0': 1.797312518885254,
                             'left_w1': 1.7810179055748931,
                             'left_w2': 0.3272508867375217,
                             'left_e0': -2.5726531467027853,
                             'left_e1': 0.778564443681907,
                             'left_s0': 0.43251553037270485,
                             'left_s1': 0.27510852913282535}

    pos18 = 				{'left_w0': 2.0344261948551425,
                             'left_w1': 1.772027712320538,
                             'left_w2': 0.6054324190777194,
                             'left_e0': -2.5838889404129435,
                             'left_e1': 0.8376613171077378,
                             'left_s0': 0.6128752948867214,
                             'left_s1': 0.3907359073449826}

    pos19 = 				{'left_w0': 2.271539870825031,
                             'left_w1': 1.7630375190661827,
                             'left_w2': 0.8836139514179172,
                             'left_e0': -2.5951247341231016,
                             'left_e1': 0.8967581905335688,
                             'left_s0': 0.793235059400738,
                             'left_s1': 0.5063632855571398}

    pos20 = 				{'left_w0': 2.4296156548049566,
                             'left_w1': 1.7570440568966124,
                             'left_w2': 1.0690683063113824,
                             'left_e0': -2.602615263263207,
                             'left_e1': 0.9361561061507894,
                             'left_s0': 0.9134749024100824,
                             'left_s1': 0.5834482043652447}

    pos20 = 				{'left_w0': 2.745767222764808,
                             'left_w1': 1.745057132557472,
                             'left_w2': 1.4399770160983127,
                             'left_e0': -2.617596321543418,
                             'left_e1': 1.0149519373852305,
                             'left_s0': 1.1539545884287712,
                             'left_s1': 0.7376180419814543}

    pnp = PickAndPlace(limb, hover_distance)
    # An orientation for gripper fingers to be overhead and parallel to the obj
    overhead_orientation = Quaternion(
                             x=-0.0249590815779,
                             y=0.999649402929,
                             z=0.00737916180073,
                             w=0.00486450832011)
    block_poses = list()
    # The Pose of the block in its initial location.
    # You may wish to replace these poses with estimates
    # from a perception node.
    # block_poses.append(Pose(
    #     position=Point(x=0.7, y=0.15, z=-0.129),
    #     orientation=overhead_orientation))
    # Feel free to add additional desired poses for the object.
    # Each additional pose will get its own pick and place.
    block_poses.append(Pose(
        position=Point(x=0.75, y=-0.1, z=-0.129),
        orientation=overhead_orientation))
    
    # block_poses.append(Pose(
    #     position=Point(x=0.0, y=1.0, z=-0.129),
    #     orientation=overhead_orientation))
    # Move to the desired starting angles
    # pnp.move_to_start(starting_joint_angles)

    # pnp.pick(block_poses[0])
    # pnp._guarded_move_to_joint_position(pos1)
    pnp._guarded_move_to_joint_position(pos2)
    pnp._guarded_move_to_joint_position(pos3)
    pnp._guarded_move_to_joint_position(pos4)
    pnp._guarded_move_to_joint_position(pos5)
    pnp._guarded_move_to_joint_position(pos6)
    pnp._guarded_move_to_joint_position(pos7)
    pnp._guarded_move_to_joint_position(pos8)
    pnp._guarded_move_to_joint_position(pos9)
    pnp._guarded_move_to_joint_position(pos10)
    pnp._guarded_move_to_joint_position(pos11)
    pnp._guarded_move_to_joint_position(pos12)
    pnp._guarded_move_to_joint_position(pos13)
    pnp._guarded_move_to_joint_position(pos14)
    pnp._guarded_move_to_joint_position(pos15)
    pnp._guarded_move_to_joint_position(pos16)
    pnp._guarded_move_to_joint_position(pos17)
    pnp._guarded_move_to_joint_position(pos18)
    pnp._guarded_move_to_joint_position(pos19)
    pnp._guarded_move_to_joint_position(pos20)
    
    idx = 0
    # while not rospy.is_shutdown():
    #     print("\nPicking...")
    #     pnp.pick(block_poses[idx])
    #     print("\nPlacing...")
    #     idx = (idx+1) % len(block_poses)
    #     pnp.place(block_poses[idx])
    # return 0

if __name__ == '__main__':
    sys.exit(main())
