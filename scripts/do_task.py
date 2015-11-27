#!/usr/bin/env python

DEBUGPLOT = False
MOVEIT_DEBUG = False

import argparse
usage="""

Run in simulation with a translation and a rotation of fake data:
./do_task.py ../sampledata/overhand/overhand.h5 --fake_data_segment=overhand0_seg00 --execution=0  --animation=1 --fake_data_transform .1 .1 .1 .1 .1 .1
./do_task.py ~/Data/sampledata/overhand/overhand.h5 --fake_data_segment=overhand0_seg00 --execution=0  --animation=1 --select_manual --fake_data_transform .1 .1 .1 .1 .1 .1

Run in simulation choosing the closest demo, single threaded
./do_task.py ~/Data/all.h5 --fake_data_segment=demo1-seg00 --execution=0  --animation=1  --parallel=0

For Calibration
./do_task.py ../data/sampledata/overhand/overhand.h5 --calibration="l"

Run in simulation but with actual kinect 2 sensor readings
# for baxter
./do_task.py ../data/overhand_redcable/overhand.h5 --execution=1
./do_task.py ../data/overhand/overhand.h5 --execution=1  --animation=1
./do_task.py ../data/sampledata/overhand/overhand.h5 --execution=1  --animation=1  --learn_from=pr2
./do_task.py ../data/sampledata/overhand/overhand.h5 --execution=1  --animation=1  --select_manual
# for PR2
./do_task.py ../data/sampledata/overhand/overhand.h5 --execution=0  --animation=1  --fake_data_transform 0 0 1 0 0 0

Actually run on the robot without pausing or animating
./do_task.py ../data/sampledata/overhand/overhand.h5 --execution=1 --animation=0

"""
parser = argparse.ArgumentParser(usage=usage)
parser.add_argument("h5file", type=str)
parser.add_argument("--cloud_proc_func", default="extract_red")
parser.add_argument("--cloud_proc_mod", default="rapprentice.cloud_proc_funcs")

parser.add_argument("--execution", type=int, default=0)
parser.add_argument("--animation", type=int, default=0)
parser.add_argument("--parallel", type=int, default=1)
parser.add_argument("--calibration", type=str, default="0")   # 0: no calibration, l: left arm, r: right arm
parser.add_argument("--learn_from", type=str, default="baxter")  # baxter or pr2

parser.add_argument("--prompt", action="store_true")
parser.add_argument("--show_neighbors", action="store_false")
parser.add_argument("--select_manual", action="store_true")
parser.add_argument("--log", action="store_true")

parser.add_argument("--fake_data_segment",type=str)
parser.add_argument("--fake_data_transform", type=float, nargs=6, metavar=("tx","ty","tz","rx","ry","rz"),
    default=[0,0,0,0,0,0], help="translation=(tx,ty,tz), axis-angle rotation=(rx,ry,rz)")

parser.add_argument("--interactive",action="store_true")

args = parser.parse_args()

#if args.fake_data_segment is None: assert args.execution==1

###################


"""
Workflow:
1. Fake data + animation only
    --fake_data_segment=xxx --execution=0
2. Fake data + Gazebo. Set Gazebo to initial state of fake data segment so we'll execute the same thing.
    --fake_data_segment=xxx --execution=1
    This is just so we know the robot won't do something stupid that we didn't catch with openrave only mode.
3. Real data + Gazebo
    --execution=1
    The problem is that the gazebo robot is in a different state from the real robot, in particular, the head tilt angle. TODO: write a script that       sets gazebo head to real robot head
4. Real data + Real execution.
    --execution=1

The question is, do you update the robot's head transform.
If you're using fake data, don't update it.

"""


from rapprentice import registration, colorize, berkeley_pr2, \
     animate_traj, ros2rave, plotting_openrave, task_execution, \
     planning, tps, func_utils, resampling, clouds, transformations
from rapprentice import math_utils as mu
from rapprentice.yes_or_no import yes_or_no
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header, String
import copy
from rapprentice import ik_service_client_dual as baxter_ik
import baxter_interface
from baxter_interface import CHECK_VERSION
import moveit_commander
import moveit_msgs.msg


try:
    from rapprentice import kinect2
    import rospy
except ImportError:
    print "Couldn't import ros stuff"


import trajoptpy, openravepy
import os, numpy as np, h5py, time
from numpy import asarray
import importlib
import math

cloud_proc_mod = importlib.import_module(args.cloud_proc_mod)
cloud_proc_func = getattr(cloud_proc_mod, args.cloud_proc_func)

def redprint(msg):
    print colorize.colorize(msg, "red", bold=True)

def split_trajectory_by_gripper(seg_info):
    rgrip = asarray(seg_info["r_gripper_joint"])
    lgrip = asarray(seg_info["l_gripper_joint"])

    thresh = .04 # open/close threshold

    n_steps = len(lgrip)


    # indices BEFORE transition occurs
    l_openings = np.flatnonzero((lgrip[1:] >= thresh) & (lgrip[:-1] < thresh))
    r_openings = np.flatnonzero((rgrip[1:] >= thresh) & (rgrip[:-1] < thresh))
    l_closings = np.flatnonzero((lgrip[1:] < thresh) & (lgrip[:-1] >= thresh))
    r_closings = np.flatnonzero((rgrip[1:] < thresh) & (rgrip[:-1] >= thresh))

    before_transitions = np.r_[l_openings, r_openings, l_closings, r_closings]
    after_transitions = before_transitions+1
    seg_starts = np.unique(np.r_[0, after_transitions])
    seg_ends = np.unique(np.r_[before_transitions, n_steps-1])

    return seg_starts, seg_ends

def binarize_gripper(angle):
    open_angle = .08
    closed_angle = 0
    thresh = .04
    if angle > thresh: return open_angle
    else: return closed_angle





def set_gripper_maybesim(lr, value):
    if args.execution:
        if not value:   # value == 0, close gripper
            Globals.gripper[lr].close()
        else:
            Globals.gripper[lr].open()

        """
        gripper = {"l":Globals.pr2.lgrip, "r":Globals.pr2.rgrip}[lr]
        gripper.set_angle(value)
        Globals.pr2.join_all()
        """
    else:
        Globals.robot.SetDOFValues([value*5], [Globals.robot.GetJoint("%s_gripper_l_finger_joint"%lr).GetDOFIndex()])
    return True


def align_to_nearest_pt(tcp_traj, new_xyz, steps, seg_info, i_start, i_end):
    for lr in 'lr':
        if steps[lr]:
            need_align = False
            critical_pt = np.zeros((4, 4))
            if i_start > 0:  # previously open at i_start-1, now close at i_start
                if (not seg_info["%s_gripper_joint"%lr][i_start]) and seg_info["%s_gripper_joint"%lr][i_start-1]:
                    critical_pt = copy.deepcopy(tcp_traj[lr][0])
                    need_align = True
            if i_end < len(seg_info["%s_gripper_joint"%lr])-1:  # now open at i_end, then close at i_end+1
                if seg_info["%s_gripper_joint"%lr][i_end] and (not seg_info["%s_gripper_joint"%lr][i_end+1]):
                    critical_pt = copy.deepcopy(tcp_traj[lr][-1])
                    need_align = True

            if need_align:
                #if lr == 'r':  # right limb has accuracy problem, with offset on tcp frame y axis
                    #critical_pt[:3, 3] += np.dot(np.linalg.inv(critical_pt[:3, :3]), np.array([0, 0.02, 0]))
                offset_xyz = new_xyz - critical_pt[:3, 3].transpose()
                offset_d = np.linalg.norm(offset_xyz, 2, 1)
                offset = offset_xyz[np.argmin(offset_d)]
                if lr == 'r':  # right limb has accuracy problem, with offset on tcp frame y axis
                    offset -= np.dot(np.linalg.inv(critical_pt[:3, :3]), np.array([0, 0.02, 0]))
                if min(offset_d) < 0.05:
                    trans_offset = np.zeros((4, 4))
                    trans_offset[:3, 3] += offset.transpose()
                    tcp_traj[lr] += trans_offset
                    print "Arm", lr, "aligned to nearest cable point with offset as: ", offset*1000, min(offset_d)*1000

    return tcp_traj


def exec_traj_maybesim(bodypart2traj, tcp_traj, tcp_poses, new_xyz, seg_info, i_start, i_end):
    if args.animation:
        dof_inds = []
        trajs = []
        for (part_name, traj) in bodypart2traj.items():
            manip_name = {"larm":"leftarm","rarm":"rightarm"}[part_name]
            dof_inds.extend(Globals.robot.GetManipulator(manip_name).GetArmIndices())
            trajs.append(traj)
        full_traj = np.concatenate(trajs, axis=1)
        Globals.robot.SetActiveDOFs(dof_inds)
        animate_traj.animate_traj(full_traj, Globals.robot, restore=False, pause=True)
    if args.execution:
        if not args.animation or yes_or_no("Do you want to execute on the real robot?"):
            steps = {'l': 0, 'r': 0}
            for (lr, trajs) in tcp_traj.items():
                steps[lr] = len(tcp_traj[lr])
            tcp_traj = align_to_nearest_pt(tcp_traj, new_xyz, steps, seg_info, i_start, i_end)

            if MOVEIT_DEBUG:
                step_inc = max(int(max(steps['l'], steps['r'])/2), 1)
            else:
                step_inc = 5
            step_range = range(0, max(steps['l'], steps['r']), step_inc)
            if step_range[-1] < max(steps['l'], steps['r'])-1:
                step_range.append(max(steps['l'], steps['r'])-1)
            for ii in step_range:
                if ii < steps['l']:
                    tcp_poses['l'].position = Point(x=tcp_traj['l'][ii][0, 3], y=tcp_traj['l'][ii][1, 3], z=tcp_traj['l'][ii][2, 3])
                    tmp_euler = transformations.euler_from_matrix(tcp_traj['l'][ii], 'syzx')
                    if args.learn_from == "pr2":
                        # add orientation offset from PR2 to Baxter
                        tmp_quat = transformations.quaternion_from_euler(tmp_euler[0] + math.pi/2, tmp_euler[1], tmp_euler[2], 'syzx')
                    else:
                        tmp_quat = transformations.quaternion_from_euler(tmp_euler[0], tmp_euler[1], tmp_euler[2], 'syzx')
                    tcp_poses['l'].orientation = Quaternion(x=tmp_quat[0], y=tmp_quat[1], z=tmp_quat[2], w=tmp_quat[3])
                if ii < steps['r']:
                    tcp_poses['r'].position = Point(x=tcp_traj['r'][ii][0, 3], y=tcp_traj['r'][ii][1, 3], z=tcp_traj['r'][ii][2, 3])
                    tmp_euler = transformations.euler_from_matrix(tcp_traj['r'][ii], 'syzx')
                    if args.learn_from == "pr2":
                        # add orientation offset from PR2 to Baxter
                        tmp_quat = transformations.quaternion_from_euler(tmp_euler[0] + math.pi/2, tmp_euler[1], tmp_euler[2], 'syzx')
                    else:
                        tmp_quat = transformations.quaternion_from_euler(tmp_euler[0], tmp_euler[1], tmp_euler[2], 'syzx')
                    tcp_poses['r'].orientation = Quaternion(x=tmp_quat[0], y=tmp_quat[1], z=tmp_quat[2], w=tmp_quat[3])
                if ii < step_range[-1]:
                    tol_factor = 20
                else:
                    tol_factor = 1

                limb_enable = 0
                if steps['l']:
                    limb_enable |= 1
                if steps['r']:
                    limb_enable |= 2
                baxter_move(tcp_poses, tol_factor, limb_enable)

                # print "Left: ", Globals.gripper['l'].position() < 10, "   Right: ", Globals.gripper['r'].position() < 10

            # pr2_trajectories.follow_body_traj(Globals.pr2, bodypart2traj)
        else:
            return False

    return True


def find_closest_manual(demofile, _new_xyz):
    "for now, just prompt the user"
    seg_names = demofile.keys()
    print "choose from the following options (type an integer)"
    for (i, seg_name) in enumerate(seg_names):
        print "%i: %s"%(i,seg_name)
    choice_ind = task_execution.request_int_in_range(len(seg_names))
    chosen_seg = seg_names[choice_ind]
    return chosen_seg

def registration_cost(xyz0, xyz1):
    scaled_xyz0, _ = registration.unit_boxify(xyz0)
    scaled_xyz1, _ = registration.unit_boxify(xyz1)
    f,g = registration.tps_rpm_bij(scaled_xyz0, scaled_xyz1, rot_reg=1e-3, n_iter=30)
    cost = registration.tps_reg_cost(f) + registration.tps_reg_cost(g)
    return cost

DS_SIZE = .025

@func_utils.once
def get_downsampled_clouds(demofile):
    return [clouds.downsample(seg["cloud_xyz"], DS_SIZE) for seg in demofile.values()]

def find_closest_auto(demofile, new_xyz):
    if args.parallel:
        from joblib import Parallel, delayed
    demo_clouds = [asarray(seg["cloud_xyz"]) for seg in demofile.values()]
    keys = demofile.keys()
    if args.parallel:
        costs = Parallel(n_jobs=3,verbose=100)(delayed(registration_cost)(demo_cloud, new_xyz) for demo_cloud in demo_clouds)
    else:
        costs = []
        for (i,ds_cloud) in enumerate(demo_clouds):
            costs.append(registration_cost(ds_cloud, new_xyz))
            print "completed %i/%i"%(i+1, len(demo_clouds))

    print "costs\n",costs
    if args.show_neighbors:
        nshow = min(1, len(keys))
        import cv2, rapprentice.cv_plot_utils as cpu
        sortinds = np.argsort(costs)[:nshow]
        near_rgbs = [asarray(demofile[keys[i]]["rgb"]) for i in sortinds]
        bigimg = cpu.tile_images(near_rgbs, 1, nshow, row_titles=None, col_titles=None, max_width=700)
        #bigimg = cpu.tile_images(near_rgbs, 1, nshow)
        cv2.imshow("closest demos", bigimg)
        cv2.moveWindow("closest demos", 0, 490)
        #cv2.moveWindow("closest demos", 0, 900)
        cv2.waitKey(1000)

        choice = raw_input("Press [y] to confirm or [0/1/2] to choose the closest demo, [n] to abort: ")
        if choice == "n":
            return False
        if choice in ["0", "1", "2"]:
            return keys[sortinds[choice]]

    ibest = np.argmin(costs)
    return keys[ibest]


def arm_moved(joint_traj):
    if len(joint_traj) < 2: return False
    return ((joint_traj[1:] - joint_traj[:-1]).ptp(axis=0) > .01).any()

def tpsrpm_plot_cb(x_nd, y_md, targ_Nd, corr_nm, wt_n, f):
    ypred_nd = f.transform_points(x_nd)
    handles = []
    handles.append(Globals.env.plot3(ypred_nd, 3, (0,1,0)))
    handles.extend(plotting_openrave.draw_grid(Globals.env, f.transform_points, x_nd.min(axis=0), x_nd.max(axis=0), xres = .1, yres = .1, zres = .04))
    Globals.viewer.Step()


def unif_resample(traj, max_diff, wt = None):
    """
    Resample a trajectory so steps have same length in joint space
    """
    import scipy.interpolate as si
    tol = .005
    if wt is not None:
        wt = np.atleast_2d(wt)
        traj = traj*wt

    dl = mu.norms(traj[1:] - traj[:-1],1)
    l = np.cumsum(np.r_[0,dl])
    goodinds = np.r_[True, dl > 1e-8]
    deg = min(3, sum(goodinds) - 1)
    if deg < 1: return traj, np.arange(len(traj))

    nsteps = max(int(np.ceil(float(l[-1])/max_diff)),2)
    newl = np.linspace(0,l[-1],nsteps)

    ncols = traj.shape[1]
    colstep = 10
    traj_rs = np.empty((nsteps,ncols))
    for istart in xrange(0, traj.shape[1], colstep):
        (tck,_) = si.splprep(traj[goodinds, istart:istart+colstep].T,k=deg,s = tol**2*len(traj),u=l[goodinds])
        traj_rs[:,istart:istart+colstep] = np.array(si.splev(newl,tck)).T
    if wt is not None: traj_rs = traj_rs/wt

    newt = np.interp(newl, l, np.arange(len(traj)))

    return traj_rs, newt



def baxter_move(tcp_poses_cmd, tol_factor=1, limb_enable=3):
    if MOVEIT_DEBUG:
        moveit_move_pose(tcp_poses_cmd, limb_enable)
    else:
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        poses = {
            'left': PoseStamped(
                header=hdr,
                pose=tcp_poses_cmd['l'],
            ),
            'right': PoseStamped(
                header=hdr,
                pose=tcp_poses_cmd['r'],
            ),
        }
        baxter_ik.ik_run(poses, tol_factor)


def baxter_move_to_initial():
    """
    Command the joints to the center of their joint ranges

    Neutral is defined as::
    ['*_s0', '*_s1', '*_e0', '*_e1', '*_w0', '*_w1', '*_w2']
    [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0]

    """
    limb_jnt_angles = {}

    for lr in 'lr':
        limb_jnt_angles[lr] = dict(zip(Globals.arm[lr].joint_names(),
                                       [0.0, -0.85, 0.0, 0.75, 0.0, 1.26, 0.0]))

    if MOVEIT_DEBUG:
        both_arm_jnt_angles = limb_jnt_angles['l'].copy()
        both_arm_jnt_angles.update(limb_jnt_angles['r'])
        moveit_move_joint(both_arm_jnt_angles)
    else:
        baxter_ik.jnt_thread_run(Globals.arm, limb_jnt_angles, 10)

    import time
    time.sleep(0.5)


def abort_task():

    if MOVEIT_DEBUG:
        moveit_commander.roscpp_shutdown()
    raise NameError("Task Aborted!")


def moveit_setup():

    ## First initialize moveit_commander
    moveit_commander.roscpp_initialize('/joint_states:=/robot/joint_states')

    ## Instantiate a RobotCommander object.  This object is an interface to the robot as a whole.
    robot = moveit_commander.RobotCommander()

    ## Instantiate a MoveGroupCommander object.
    group = moveit_commander.MoveGroupCommander("both_arms")

    ## We create this DisplayTrajectory publisher which is used below to publish
    ## trajectories for RVIZ to visualize.
    display_trajectory_publisher = rospy.Publisher(
        '/move_group/display_planned_path',
        moveit_msgs.msg.DisplayTrajectory)

    ## We can get the name of the reference frame for this robot
    print "============ Reference frame: %s" % group.get_planning_frame()

    ## We can get a list of all the groups in the robot
    print "============ Robot Groups:"
    print robot.get_group_names()

    group.set_goal_joint_tolerance(0.03)
    group.set_goal_position_tolerance(0.03)
    group.set_goal_orientation_tolerance(0.3)

    Globals.moveit['robot'] = robot
    Globals.moveit['group'] = group
    Globals.moveit['display_trajectory_publisher'] = display_trajectory_publisher


def moveit_move_joint(joint_cmd):
    Globals.moveit['group'].clear_pose_targets()
    Globals.moveit['group'].set_joint_value_target(joint_cmd)
    plan = Globals.moveit['group'].plan()
    moveit_display_traj_and_run(plan)


def moveit_move_pose(tcp_poses_cmd, limb_enable=0):
    Globals.moveit['group'].clear_pose_targets()

    ## Planning to a Pose goal
    if limb_enable & 1:
        Globals.moveit['group'].set_pose_target(tcp_poses_cmd['l'], "left_gripper")
    if limb_enable & 2:
        Globals.moveit['group'].set_pose_target(tcp_poses_cmd['r'], "right_gripper")

    plan = Globals.moveit['group'].plan()
    moveit_display_traj_and_run(plan)


def moveit_display_traj_and_run(plan):
    print "============ Waiting while RVIZ displays the planned trajectory..."
    if DEBUGPLOT:  # and yes_or_no("Do you want to see the trajectory again?"):
        ## You can ask RVIZ to visualize a plan (aka trajectory) for you again
        print "============ Visualizing the planned trajectory"
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = Globals.moveit['robot'].get_current_state()
        display_trajectory.trajectory.append(plan)
        Globals.moveit['display_trajectory_publisher'].publish(display_trajectory)

    if ~DEBUGPLOT or yes_or_no("Do you want to execute on the real robot?"):
        ## Moving to a pose goal
        Globals.moveit['group'].go(wait=True)
    else:
        abort_task()



"""
def list_addvalue(oldlist, addvalue):
    oldlist_array = np.array(oldlist)
    oldlist_array += addvalue
    newlist = oldlist_array.tolist()
    return newlist
"""

###################


class Globals:
    robot = None
    env = None
    pr2 = None
    k2 = None
    gripper = {'l': None, 'r': None}
    arm = {'l': None, 'r': None}
    moveit = {'robot': None, 'group': None, 'display_trajectory_publisher': None}

def main():

    demofile = h5py.File(args.h5file, 'r')
    tcp_poses0 = {'l': Pose(), 'r': Pose()}
    baxter2pr2offset = np.zeros((4, 4))
    baxter2pr2offset[:3, 3] += (0, 0, 0.6)

    trajoptpy.SetInteractive(args.interactive)


    if args.log:
        LOG_DIR = osp.join(osp.expanduser("~/Data/do_task_logs"), datetime.datetime.now().strftime("%Y%m%d-%H%M%S"))
        os.mkdir(LOG_DIR)
        LOG_COUNT = 0


    rospy.init_node("softproj_rapp")    # Create a node of name rapp_k2

    if args.execution or args.calibration != "0":                  # Use actual robot (Baxter)
        if MOVEIT_DEBUG:
            moveit_setup()

        for lr in 'lr':
            manip_name = {"l": "left", "r": "right"}[lr]
            Globals.gripper[lr] = baxter_interface.Gripper(manip_name, CHECK_VERSION)
            #Globals.gripper[lr].calibrate()
            Globals.gripper[lr].open()
            Globals.gripper[lr].set_holding_force(20)  # 20% force of 30N

            Globals.arm[lr] = baxter_interface.Limb(manip_name)

        baxter_move_to_initial()
        for lr in 'lr':
            tcp_poses0[lr].position = Globals.arm[lr].endpoint_pose()["position"]
            tcp_poses0[lr].orientation = Globals.arm[lr].endpoint_pose()["orientation"]
        tcp_poses = copy.deepcopy(tcp_poses0)

        """
        rospy.init_node("exec_task",disable_signals=True)
        Globals.pr2 = PR2.PR2()
        Globals.env = Globals.pr2.env
        Globals.robot = Globals.pr2.robot
        """

    if args.animation or not args.execution:
        Globals.env = openravepy.Environment()
        Globals.env.StopSimulation()
        Globals.env.Load("robots/pr2-beta-static.zae")
        Globals.robot = Globals.env.GetRobots()[0]
        Globals.viewer = trajoptpy.GetViewer(Globals.env)

    if not args.fake_data_segment:      # Use Kinect 2
        Globals.k2 = kinect2.Kinect2(rospy.get_name)
        import time
        time.sleep(3)
        if DEBUGPLOT:
            import cv2
            time.sleep(1)
            cv2.imshow("Depth image", Globals.k2.cv_depth_img)          # Showing the depth image
            cv2.imshow("RGB image", Globals.k2.cv_rgb_img)              # Showing the RGB image
            cv2.waitKey(10000)
            cv2.destroyAllWindows()
        """
        grabber = cloudprocpy.CloudGrabber()
        grabber.startRGBD()
        """

    #####################

    while True:

        redprint("Acquire point cloud")
        if args.fake_data_segment:
            fake_seg = demofile[args.fake_data_segment]
            new_xyz = np.squeeze(fake_seg["cloud_xyz"])
            hmat = openravepy.matrixFromAxisAngle(args.fake_data_transform[3:6])
            hmat[:3,3] = args.fake_data_transform[0:3]
            new_xyz = new_xyz.dot(hmat[:3,:3].T) + hmat[:3,3][None,:]
            r2r = ros2rave.RosToRave(Globals.robot, asarray(fake_seg["joint_states"]["name"]))
            r2r.set_values(Globals.robot, asarray(fake_seg["joint_states"]["position"][0]))
        else:
            rgb = Globals.k2.cv_rgb_img
            depth = Globals.k2.cv_depth_img
            T_w_k = berkeley_pr2.get_kinect_transform()
            new_xyz = cloud_proc_func(rgb, depth, T_w_k)

            if args.calibration != "0":
                tcp_poses[args.calibration].orientation = Quaternion(x=0, y=1, z=0, w=0)
                tcp_poses[args.calibration].position = Point(x=new_xyz[-1][0], y=new_xyz[-1][1], z=new_xyz[-1][2]+0.1)
                baxter_move(tcp_poses)
                tcp_poses[args.calibration].position = Point(x=new_xyz[-1][0], y=new_xyz[-1][1], z=new_xyz[-1][2])
                baxter_move(tcp_poses)
                raw_input("Press ENTER to continue...")
                tcp_poses[args.calibration].position = Point(x=new_xyz[-1][0], y=new_xyz[-1][1], z=new_xyz[-1][2]+0.1)
                baxter_move(tcp_poses)
                baxter_move_to_initial()
                return

            # In case there is offset for mis-calibration and from baxter to PR2
            new_xyz = new_xyz + args.fake_data_transform[0:3] + baxter2pr2offset[:3, 3]

            """
            Globals.pr2.head.set_pan_tilt(0,1.2)
            Globals.pr2.rarm.goto_posture('side')
            Globals.pr2.larm.goto_posture('side')
            Globals.pr2.join_all()
            time.sleep(.5)

            Globals.pr2.update_rave()

            rgb, depth = grabber.getRGBD()
            T_w_k = berkeley_pr2.get_kinect_transform(Globals.robot)
            new_xyz = cloud_proc_func(rgb, depth, T_w_k)

            #grab_end(new_xyz)
            """


        if args.log:
            LOG_COUNT += 1
            import cv2
            cv2.imwrite(osp.join(LOG_DIR,"rgb%i.png"%LOG_COUNT), rgb)
            cv2.imwrite(osp.join(LOG_DIR,"depth%i.png"%LOG_COUNT), depth)
            np.save(osp.join(LOG_DIR,"xyz%i.npy"%LOG_COUNT), new_xyz)

        ################################
        redprint("Finding closest demonstration")
        if args.select_manual:
            seg_name = find_closest_manual(demofile, new_xyz)
        else:
            seg_name = find_closest_auto(demofile, new_xyz)
            if seg_name is False:
                redprint("Task Aborted!")
                return

        seg_info = demofile[seg_name]
        redprint("closest demo: %s"%(seg_name))

        if "done" in seg_name:
            redprint("DONE!")
            break

        if not args.fake_data_segment and not args.execution:
            r2r = ros2rave.RosToRave(Globals.robot, asarray(seg_info["joint_states"]["name"]))
            r2r.set_values(Globals.robot, asarray(seg_info["joint_states"]["position"][0]))

        if args.log:
            with open(osp.join(LOG_DIR,"neighbor%i.txt"%LOG_COUNT),"w") as fh: fh.write(seg_name)
        ################################

        redprint("Generating end-effector trajectory")

        if args.learn_from == "pr2":
            old_xyz = np.squeeze(seg_info["cloud_xyz"])
        else:  # add offset for animation with pr2
            old_xyz = np.squeeze(seg_info["cloud_xyz"]) + baxter2pr2offset[:3, 3]
        if args.animation or not args.execution:
            handles = []
            handles.append(Globals.env.plot3(old_xyz, 5, (1, 0, 0)))
            handles.append(Globals.env.plot3(new_xyz, 5, (0, 0, 1)))

        scaled_old_xyz, src_params = registration.unit_boxify(old_xyz)
        scaled_new_xyz, targ_params = registration.unit_boxify(new_xyz)
        f,_ = registration.tps_rpm_bij(scaled_old_xyz, scaled_new_xyz, plot_cb = tpsrpm_plot_cb,
                                       plotting=5 if args.animation else 0,rot_reg=np.r_[1e-4,1e-4,1e-1], n_iter=50, reg_init=10, reg_final=.1)
        f = registration.unscale_tps(f, src_params, targ_params)

        if args.animation or not args.execution:
            handles.extend(plotting_openrave.draw_grid(Globals.env, f.transform_points, old_xyz.min(axis=0)-np.r_[0,0,.1], old_xyz.max(axis=0)+np.r_[0,0,.1], xres = .1, yres = .1, zres = .04))

        link2eetraj = {}
        for lr in 'lr':
            link_name = "%s_gripper_tool_frame"%lr
            if args.learn_from == "pr2":
                old_ee_traj = asarray(seg_info[link_name]["hmat"])
            else:  # add offset for animation with pr2
                old_ee_traj = asarray(seg_info[link_name]["hmat"]) + baxter2pr2offset
            new_ee_traj = f.transform_hmats(old_ee_traj)
            link2eetraj[link_name] = new_ee_traj

            if args.animation or not args.execution:
                handles.append(Globals.env.drawlinestrip(old_ee_traj[:,:3,3], 2, (1,0,0,1)))
                handles.append(Globals.env.drawlinestrip(new_ee_traj[:,:3,3], 2, (0,1,0,1)))

        miniseg_starts, miniseg_ends = split_trajectory_by_gripper(seg_info)
        success = True
        print colorize.colorize("mini segments:", "red"), miniseg_starts, miniseg_ends
        for (i_miniseg, (i_start, i_end)) in enumerate(zip(miniseg_starts, miniseg_ends)):

            """
            if args.execution=="real": Globals.pr2.update_rave()
            """

            ################################
            redprint("Generating joint trajectory for segment %s, part %i"%(seg_name, i_miniseg))

            # figure out how we're gonna resample stuff
            lr2oldtraj = {}
            for lr in 'lr':
                manip_name = {"l":"leftarm", "r":"rightarm"}[lr]
                old_joint_traj = asarray(seg_info[manip_name][i_start:i_end+1])
                #print (old_joint_traj[1:] - old_joint_traj[:-1]).ptp(axis=0), i_start, i_end
                if arm_moved(old_joint_traj):
                    lr2oldtraj[lr] = old_joint_traj
            if len(lr2oldtraj) > 0:
                old_total_traj = np.concatenate(lr2oldtraj.values(), 1)
                JOINT_LENGTH_PER_STEP = .1
                _, timesteps_rs = unif_resample(old_total_traj, JOINT_LENGTH_PER_STEP)
            ####

            ### Generate fullbody traj
            bodypart2traj = {}
            tcp_traj = {}
            for (lr,old_joint_traj) in lr2oldtraj.items():

                manip_name = {"l":"leftarm", "r":"rightarm"}[lr]

                old_joint_traj_rs = mu.interp2d(timesteps_rs, np.arange(len(old_joint_traj)), old_joint_traj)

                ee_link_name = "%s_gripper_tool_frame"%lr
                new_ee_traj = link2eetraj[ee_link_name][i_start:i_end+1]
                new_ee_traj_rs = resampling.interp_hmats(timesteps_rs, np.arange(len(new_ee_traj)), new_ee_traj)

                tcp_traj[lr] = new_ee_traj_rs - baxter2pr2offset

                # if args.execution: Globals.pr2.update_rave()

                if args.animation or not args.execution:
                    new_joint_traj = planning.plan_follow_traj(Globals.robot, manip_name,
                                                               Globals.robot.GetLink(ee_link_name), new_ee_traj_rs, old_joint_traj_rs)
                else:
                    new_joint_traj = old_joint_traj_rs
                part_name = {"l":"larm", "r":"rarm"}[lr]
                bodypart2traj[part_name] = new_joint_traj

            ################################
            redprint("Executing joint trajectory for segment %s, part %i using arms '%s'"%(seg_name, i_miniseg, bodypart2traj.keys()))

            for lr in 'lr':
                success &= set_gripper_maybesim(lr, binarize_gripper(seg_info["%s_gripper_joint"%lr][i_start]))
                # Doesn't actually check if grab occurred, unfortunately

            if not success: break

            if len(bodypart2traj) > 0:
                success &= exec_traj_maybesim(bodypart2traj, tcp_traj, tcp_poses,
                                              new_xyz-baxter2pr2offset[:3, 3], seg_info, i_start, i_end)

            if not success: break


        redprint("Segment %s result: %s"%(seg_name, success))

        # move back to initial position
        baxter_move_to_initial()
        tcp_poses = copy.deepcopy(tcp_poses0)

        if not yes_or_no("Do you want to continue?"):
            return

        if args.fake_data_segment: break

    ## When finished shut down moveit_commander.
    if MOVEIT_DEBUG:
        moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    main()
