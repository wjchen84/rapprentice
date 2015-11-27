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
Baxter RSDK Inverse Kinematics Example
"""
import argparse
import struct
import sys

import traceback
import threading
import Queue
import math

import rospy
import baxter_dataflow
import baxter_interface
from baxter_interface import CHECK_VERSION

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

from rapprentice.yes_or_no import yes_or_no
from rapprentice import transformations

DEBUG = False

def jnt_thread_run(limb_move, limb_joints, tol_factor=1):
    limb_queue = {}
    limb_thread = {}

    def move_thread(limb, angle, queue, tol_factor=1, timeout=15.0):
        """
        Threaded joint movement allowing for simultaneous joint moves.
        """
        try:
            limb.set_joint_position_speed(1.0)
            limb.move_to_joint_positions(angle, timeout,
                                         threshold=baxter_interface.settings.JOINT_ANGLE_TOLERANCE*tol_factor)
            queue.put(None)
        except Exception, exception:
            queue.put(traceback.format_exc())
            queue.put(exception)

    for lr in 'lr':
        limb_queue[lr] = Queue.Queue()
        limb_thread[lr] = threading.Thread(
            target=move_thread,
            args=(limb_move[lr],
                  limb_joints[lr],
                  limb_queue[lr],
                  tol_factor
                  )
        )
        limb_thread[lr].daemon = True
        limb_thread[lr].start()

    baxter_dataflow.wait_for(
        lambda: not (limb_thread['l'].is_alive() or
                     limb_thread['r'].is_alive()),
        timeout=20.0,
        timeout_msg=("Timeout while waiting for arm move threads"
                     " to finish"),
        rate=10,
    )

    for lr in 'lr':
        limb_thread[lr].join()
        result = limb_queue[lr].get()
        if not result is None:
            raise limb_queue[lr].get()

    #rospy.sleep(1.0)


def ik_run(poses, tol_factor=1):
    limb_joints = {}
    limb_move = {}
    gripper = {}

    for lr in 'lr':
        limb = {"l":"left", "r":"right"}[lr]
        limb_move[lr] = baxter_interface.Limb(limb)
        gripper[lr] = baxter_interface.Gripper(limb, CHECK_VERSION)
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()

        adjust_stepsize = math.pi/20
        adjust_step = 0
        adjust_stepmax = 7
        solnFound = False
        gripper_closed = (gripper[lr].position() < 10)

        """
        tmp_pos = [poses[limb].pose.position.x, poses[limb].pose.position.y, poses[limb].pose.position.z]
        if lr == 'l':
            poses[limb].pose.position = Point(x=tmp_pos[0], y=tmp_pos[1]+0.00, z=tmp_pos[2])
        else:
            poses[limb].pose.position = Point(x=tmp_pos[0], y=tmp_pos[1]-0.02, z=tmp_pos[2])
        """

        while (not solnFound) and ((adjust_step <= adjust_stepmax*2 and (not gripper_closed))
                                   or (adjust_step <= adjust_stepmax*4 and gripper_closed)):
            tmp_quat = [poses[limb].pose.orientation.x, poses[limb].pose.orientation.y,
                        poses[limb].pose.orientation.z, poses[limb].pose.orientation.w]
            if (adjust_step <= adjust_stepmax*2):  # adjust y axis rotation, will not affect the gripper grasping the cable
                tmp_euler = transformations.euler_from_quaternion(tmp_quat, 'rzxy')
                tmp_quat = transformations.quaternion_from_euler(tmp_euler[0], tmp_euler[1],
                                                                 tmp_euler[2]+adjust_stepsize*
                                                                 int((adjust_step+1)/2)*(-1)**adjust_step, 'rzxy')
            else:  # adjust x axis rotation, gripper is closed
                tmp_euler = transformations.euler_from_quaternion(tmp_quat, 'ryzx')
                tmp_quat = transformations.quaternion_from_euler(tmp_euler[0], tmp_euler[1],
                                                                 tmp_euler[2]+adjust_stepsize*
                                                                 int((adjust_step-adjust_stepmax*2+1)/2)*(-1)**adjust_step, 'ryzx')

            tmp_euler = transformations.euler_from_quaternion(tmp_quat, 'szxy')
            if (abs(tmp_euler[1]) < math.pi/180.0*110.0) and (abs(tmp_euler[2]) < math.pi/180.0*110.0):
                gripper_down = False
            else:
                gripper_down = True

            poses[limb].pose.orientation = Quaternion(x=tmp_quat[0], y=tmp_quat[1], z=tmp_quat[2], w=tmp_quat[3])

            ikreq.pose_stamp.append(poses[limb])
            try:
                rospy.wait_for_service(ns, 5.0)
                resp = iksvc(ikreq)
            except (rospy.ServiceException, rospy.ROSException), e:
                rospy.logerr("Service call failed: %s" % (e,))
                return 1

            # Check if result valid, and type of seed ultimately used to get solution
            if (resp.isValid[0]) and gripper_down:
                # Format solution into Limb API-compatible dictionary
                limb_joints[lr] = dict(zip(resp.joints[0].name, resp.joints[0].position))
                solnFound = True
            else:
                adjust_step += 1

        if solnFound and (adjust_step > 0):
            print("Pose adjusted for valid Inverse Kinematics. adjust_step: "), adjust_step,  "  Limb:", limb

        if not solnFound:
            print("INVALID POSE - No Valid Joint Solution Found by Inverse Kinematics. Limb:"), limb
            if DEBUG:
                if yes_or_no("Do you want to continue to next step?"):
                    limb_joints[lr] = limb_move[lr].joint_angles()
                else:
                    raise NameError("Task Aborted!")
            else:
                limb_joints[lr] = limb_move[lr].joint_angles()

    jnt_thread_run(limb_move,limb_joints,tol_factor)

    return 0


def main():
    """RSDK Inverse Kinematics Example

    A simple example of using the Rethink Inverse Kinematics
    Service which returns the joint angles and validity for
    a requested Cartesian Pose.

    Run this example, passing the *limb* to test, and the
    example will call the Service with a sample Cartesian
    Pose, pre-defined in the example code, printing the
    response of whether a valid joint solution was found,
    and if so, the corresponding joint angles.
    """
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument(
        '-l', '--limb', choices=['left', 'right'], required=True,
        help="the limb to test"
    )
    args = parser.parse_args(rospy.myargv()[1:])
    """
    rospy.init_node("rsdk_ik_service_client")
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        'left': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=6.97734143e-01, #0.530, #0.657579481614,
                    y=-7.71852517e-02, #0.268, #0.851981417433,
                    z=-8.28954126e-02, #0.451, #0.0388352386502,
                ),
                orientation=Quaternion(
                    x=1, #-0.298, #-0.366894936773,
                    y=0, #0.132, #0.885980397775,
                    z=0, #0.892, #0.108155782462,
                    w=0, #0.314, #0.262162481772,
                ),
            ),
        ),
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=0.530, #0.656982770038,
                    y=-0.468, #-0.852598021641,
                    z=0.251, #0.0388609422173,
                ),
                orientation=Quaternion(
                    x=1, #0.367048116303,
                    y=0, #0.885911751787,
                    z=0, #-0.108908281936,
                    w=0, #0.261868353356,
                ),
            ),
        ),
    }

    return ik_run(poses)

if __name__ == '__main__':
    sys.exit(main())
