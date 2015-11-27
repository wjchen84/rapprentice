from rapprentice import ros2rave, func_utils, berkeley_pr2, transformations
import fastrapp
import numpy as np
import cv2
import openravepy
import os.path as osp
from baxter_pykdl import baxter_kinematics
from shutil import copyfile


def extract_joints(bag, topic):
    """returns (names, traj) 
    """
    traj = []
    stamps = []
    for (_, msg, _) in bag.read_messages(topics=[topic]):
        traj.append(msg.position)
        stamps.append(msg.header.stamp.to_sec())
    assert len(traj) > 0
    names = msg.name
    return names, stamps, traj


def extract_gripper(bag, topic):
    """returns (names, traj)
    """
    traj = []
    stamps = []
    for (_, msg, _) in bag.read_messages(topics=[topic]):
        traj.append(msg.position>50)
        stamps.append(msg.timestamp.to_sec())
    assert len(traj) > 0
    return stamps, traj


def button_log(bag, topic0, meaning0, stamps, meanings, t=0):
    message_stream = bag.read_messages(topics=[topic0])
    (_,last_msg,_) = message_stream.next()
    button_dt = 0.01
    for (_, msg, _) in message_stream:
        if msg.state and not last_msg.state:
                stamps.append(t)
                meanings.append(meaning0)
        last_msg = msg
        t += button_dt

    return stamps, meanings


def get_t0(bag):
    joint_states_msg = bag.read_messages(topics=["/robot/joint_states"])
    (_,js_msg,_) = joint_states_msg.next()
    t0 = js_msg.header.stamp.to_sec()
    return t0


def extract_joy(bag):
    """sounds morbid    
    """

    stamps = []
    meanings = []
    t0 = get_t0(bag)

    button2meaning = {
        "/robot/digital_io/torso_left_itb_button0/state": "look",
        "/robot/digital_io/torso_left_itb_button2/state": "start",
        "/robot/digital_io/torso_left_itb_button1/state": "stop",
        "/robot/digital_io/left_lower_button/state": "l_open",
        "/robot/digital_io/left_upper_button/state": "l_close",
        "/robot/digital_io/right_lower_button/state": "r_open",
        "/robot/digital_io/right_upper_button/state": "r_close",
        "/robot/digital_io/left_itb_button1/state": "done"
    }
    check_buttons = button2meaning.keys()
    for topic0 in check_buttons:
        stamps, meanings = button_log(bag, topic0, button2meaning[topic0], stamps, meanings, t0)

    stamps_sorted = [x for (x,y) in sorted(zip(stamps,meanings), key=lambda pair:pair[0])]
    meanings_sorted = [y for (x,y) in sorted(zip(stamps,meanings), key=lambda pair:pair[0])]

    return stamps_sorted, meanings_sorted

"""
def extract_joy(bag):

    stamps = []
    meanings = []

    button2meaning = {
        12: "look",
        0: "start",
        3: "stop",
        7: "l_open",
        5: "l_close",
        15: "r_open",
        13: "r_close",
        14: "done"
    }
    check_buttons = button2meaning.keys()
    message_stream = bag.read_messages(topics=['/joy'])
    (_,last_msg,_) = message_stream.next()
    for (_, msg, _) in message_stream:
        for i in check_buttons:
            if msg.buttons[i] and not last_msg.buttons[i]:
                stamps.append(msg.header.stamp.to_sec())
                meanings.append(button2meaning[i])
        last_msg = msg

    return stamps, meanings
"""

        
def find_disjoint_subsequences(li, seq):
    """
    Returns a list of tuples (i,j,k,...) so that seq == (li[i], li[j], li[k],...)
    Greedily find first tuple, then second, etc.
    """
    subseqs = []
    cur_subseq_inds = []
    for (i_el, el) in enumerate(li):
        if el == seq[len(cur_subseq_inds)]:
            cur_subseq_inds.append(i_el)
            if len(cur_subseq_inds) == len(seq):
                subseqs.append(cur_subseq_inds)
                cur_subseq_inds = []
    return subseqs
    
def joy_to_annotations(stamps, meanings):
    """return a list of dicts giving info for each segment
    [{"look": 1234, "start": 2345, "stop": 3456},...]
    """
    out = []
    ind_tuples = find_disjoint_subsequences(meanings, ["look","start","stop"])
    for tup in ind_tuples:
        out.append({"look":stamps[tup[0]], "start":stamps[tup[1]], "stop":stamps[tup[2]]})
    
    done_inds = [i for (i,meaning) in enumerate(meanings) if meaning=="done"]
    for ind in done_inds:
        out.append({"done":None,"look":stamps[ind], "start":stamps[ind], "stop":stamps[ind]+1})
    
    return out

def add_kinematics_to_group(group, linknames, manipnames):
    "do forward kinematics on those links"
    limbnames = {"leftarm":"left", "rightarm":"right"}
    link2hmats = dict([(linkname, []) for linkname in linknames.values()])
    manipjoints = dict([(manipname, []) for manipname in manipnames])
    baxter_kin = {"left": baxter_kinematics("left"),
                  "right": baxter_kinematics("right")}
    for ros_vals in group["joint_states"]["position"]:
        joint_traj = dict(zip(group["joint_states"]["name"], ros_vals))

        for manipname in manipnames:
            joint_traj_limb = dict((jointname, joint_traj[jointname])
                                   for jointname in group["joint_states"]["name"] if limbnames[manipname] in jointname)
            manipjoints[manipname].append(joint_traj_limb.values())
            end_point_limb_posquat = baxter_kin[limbnames[manipname]].forward_position_kinematics(joint_traj_limb)
            end_point_limb_matrix = transformations.quaternion_matrix(end_point_limb_posquat[3:7])
            end_point_limb_matrix[:3, 3] = end_point_limb_posquat[:3]
            link2hmats[linknames[manipname]].append(end_point_limb_matrix)

    for (linkname, hmats) in link2hmats.items():
        group.create_group(linkname)
        group[linkname]["hmat"] = np.array(hmats)      

    for manipname in manipnames:
        group[manipname] = manipjoints[manipname]


"""
def add_kinematics_to_group(group, linknames, manipnames, jointnames, robot):
    "do forward kinematics on those links"
    if robot is None: robot = get_robot()
    r2r = ros2rave.RosToRave(robot, group["joint_states"]["name"])
    link2hmats = dict([(linkname, []) for linkname in linknames])
    links = [robot.GetLink(linkname) for linkname in linknames]
    rave_traj = []
    rave_inds = r2r.rave_inds
    for ros_vals in group["joint_states"]["position"]:
        r2r.set_values(robot, ros_vals)
        rave_vals = r2r.convert(ros_vals)
        robot.SetDOFValues(rave_vals, rave_inds)
        rave_traj.append(rave_vals)
        for (linkname,link) in zip(linknames, links):
            link2hmats[linkname].append(link.GetTransform())
    for (linkname, hmats) in link2hmats.items():
        group.create_group(linkname)
        group[linkname]["hmat"] = np.array(hmats)

    rave_traj = np.array(rave_traj)
    rave_ind_list = list(rave_inds)
    for manipname in manipnames:
        arm_inds = robot.GetManipulator(manipname).GetArmIndices()
        group[manipname] = rave_traj[:,[rave_ind_list.index(i) for i in arm_inds]]

    for jointname in jointnames:
        joint_ind = robot.GetJointIndex(jointname)
        group[jointname] = rave_traj[:,rave_ind_list.index(joint_ind)]
"""


    
@func_utils.once
def get_robot():
    """
    env = openravepy.Environment()
    env.Load("robots/pr2-beta-static.zae")
    robot = env.GetRobots()[0]
    """
    robot = None
    return robot

def add_gripper_to_group(group, bag, stamps_ds, stamps_grip, traj_grip, name):
    i_ds_grip = np.searchsorted(stamps_grip, stamps_ds)
    traj_grip_ds = [traj_grip[min(i, len(traj_grip)-1)] for i in i_ds_grip]
    group[name] = traj_grip_ds


def add_bag_to_hdf(bag, annotations, hdfroot, demo_name):
    joint_names, stamps, traj = extract_joints(bag, "/robot/joint_states")
    stamps_grip_l, traj_grip_l = extract_gripper(bag, "/robot/end_effector/left_gripper/state")
    stamps_grip_r, traj_grip_r = extract_gripper(bag, "/robot/end_effector/right_gripper/state")
    traj = np.asarray(traj)
    stamps = np.asarray(stamps)
    
    robot = get_robot()

    for seg_info in annotations:

        if seg_info["name"] == "done":
            break

        group = hdfroot.create_group(demo_name + "_" + seg_info["name"])

        start = seg_info["start"]
        stop = seg_info["stop"]
        
        [i_start, i_stop] = np.searchsorted(stamps, [start, stop])
        
        stamps_seg = stamps[i_start:i_stop+1]
        traj_seg = traj[i_start:i_stop+1]
        sample_inds = fastrapp.resample(traj_seg, np.arange(len(traj_seg)), .01, np.inf, np.inf)
        print "demo: ", group.name, " trajectory has length", len(sample_inds),len(traj_seg)

        traj_ds = traj_seg[sample_inds,:]
        stamps_ds = stamps_seg[sample_inds]

        group["description"] = seg_info["description"]
        group["stamps"] = stamps_ds
        group.create_group("joint_states")
        group["joint_states"]["name"] = joint_names
        group["joint_states"]["position"] = traj_ds
        link_names = {"leftarm":"l_gripper_tool_frame", "rightarm":"r_gripper_tool_frame"}
        manip_names = ["leftarm", "rightarm"]
        
        add_kinematics_to_group(group, link_names, manip_names)
        add_gripper_to_group(group, bag, stamps_ds, stamps_grip_l, traj_grip_l, "l_gripper_joint")
        add_gripper_to_group(group, bag, stamps_ds, stamps_grip_r, traj_grip_r, "r_gripper_joint")


"""
def add_bag_to_hdf(bag, annotations, hdfroot, demo_name):
    joint_names, stamps, traj = extract_joints(bag)
    traj = np.asarray(traj)
    stamps = np.asarray(stamps)

    robot = get_robot()

    for seg_info in annotations:


        group = hdfroot.create_group(demo_name + "_" + seg_info["name"])

        start = seg_info["start"]
        stop = seg_info["stop"]

        [i_start, i_stop] = np.searchsorted(stamps, [start, stop])

        stamps_seg = stamps[i_start:i_stop+1]
        traj_seg = traj[i_start:i_stop+1]
        sample_inds = fastrapp.resample(traj_seg, np.arange(len(traj_seg)), .01, np.inf, np.inf)
        print "trajectory has length", len(sample_inds),len(traj_seg)


        traj_ds = traj_seg[sample_inds,:]
        stamps_ds = stamps_seg[sample_inds]

        group["description"] = seg_info["description"]
        group["stamps"] = stamps_ds
        group.create_group("joint_states")
        group["joint_states"]["name"] = joint_names
        group["joint_states"]["position"] = traj_ds
        link_names = ["l_gripper_tool_frame","r_gripper_tool_frame","l_gripper_r_finger_tip_link","l_gripper_l_finger_tip_frame","r_gripper_r_finger_tip_link","r_gripper_l_finger_tip_frame"]
        special_joint_names = ["l_gripper_joint", "r_gripper_joint"]
        manip_names = ["leftarm", "rightarm"]

        add_kinematics_to_group(group, link_names, manip_names, special_joint_names, robot)
"""

def get_video_frames(video_dir, frame_stamps, t0):
    video_stamps = np.loadtxt(osp.join(video_dir,"stamps.txt"))
    video_stamps += t0 - video_stamps[0]
    frame_inds = np.searchsorted(video_stamps, frame_stamps)
    
    rgbs = []
    depths = []
    for frame_ind in frame_inds:
        rgb = cv2.imread(osp.join(video_dir,"rgb%i.jpg"%frame_ind))
        copyfile(osp.join(video_dir,"rgb%i.jpg"%frame_ind), osp.join(video_dir,"look_rgb%i.jpg"%frame_ind))
        assert rgb is not None
        rgbs.append(rgb)
        depth = cv2.imread(osp.join(video_dir,"depth%i.png"%frame_ind),2)
        copyfile(osp.join(video_dir,"depth%i.png"%frame_ind), osp.join(video_dir,"look_depth%i.png"%frame_ind))
        assert depth is not None
        depths.append(depth)
    return rgbs, depths


def add_rgbd_to_hdf(video_dir, annotations, hdfroot, demo_name, bag):
    
    frame_stamps = [seg_info["look"] for seg_info in annotations]
    
    rgb_imgs, depth_imgs = get_video_frames(video_dir, frame_stamps, get_t0(bag))
    
    for (i_seg, seg_info) in enumerate(annotations):        

        if seg_info["name"] == "done":
            break

        group = hdfroot[demo_name + "_" + seg_info["name"]]
        group["rgb"] = rgb_imgs[i_seg]
        group["depth"] = depth_imgs[i_seg]

        """
        robot = get_robot()
        r2r = ros2rave.RosToRave(robot, group["joint_states"]["name"])
        r2r.set_values(robot, group["joint_states"]["position"][0])
        T_w_h = robot.GetLink("head_plate_frame").GetTransform()
        T_w_k = T_w_h.dot(berkeley_pr2.T_h_k)
        """

        T_w_k = berkeley_pr2.get_kinect_transform()
        group["T_w_k"] = T_w_k

