"""
put Berkeley-specific parameters here
"""



import numpy as np
import math
from rapprentice import transformations


"""
Berkeley PR2's kinect transform
Here's what's in the URDF:
    <joint name="camera_rgb_optical_frame_joint" type="fixed">
    <parent link="head_plate_frame"/>
    <child link="camera_link"/>
    <origin rpy="0.005 0.035 0.02" xyz="-0.19 0.045 0.22"/>
    </joint>
    <link name="camera_link" type="camera"/>
"""

#T_b_o = np.array([[0,0,1,0],[-1,0,0,0],[0,-1,0,0],[0,0,0,1]])
#T_b_o = np.array([[0,0,1,0],[1,0,0,0],[0,1,0,0],[0,0,0,1]])
"""
T_h_k = np.array([[-0.02102462, -0.03347223,  0.99921848, -0.186996  ],
 [-0.99974787, -0.00717795, -0.02127621,  0.04361884],
 [ 0.0078845,  -0.99941387, -0.03331288,  0.22145804],
 [ 0.,          0.,          0.,          1.        ]])

f = 544.260779961

T_h_k = np.array([[-0.02102462, -0.03347223,  0.99921848, 0.2  ],
 [-0.99974787, -0.00717795, -0.02127621,  0.04361884],
 [ 0.0078845,  -0.99941387, -0.03331288,   0.22145804],
 [ 0.,          0.,          0.,          1.        ]])

T_tmp = np.array([[1, 0, 0, 0], [0, 0.5, 0.8667, 0], [0, -0.8667, 0.5, 0], [0, 0, 0, 1]])

T_h_k = T_h_k.dot(T_tmp)
"""

f = 1081.37

trans = np.array([1.456, -0.357, 0.587]) #[1.100, -0.506, 0.604], [1.160, -0.590, 0.505], [1.160, -0.620, 0.500]
quat = np.array([-0.268, 0.147, 0.913, 0.270]) # [-0.2348969 ,  0.15495554,  0.86150072,  0.42264494] (qx, qy, qz, qw)
#quat = np.array([-0.245, 0.133, 0.865, 0.417]) # (qx, qy, qz, qw)
#quat = np.array([-0.266, 0.090, 0.871, 0.404]) # (qx, qy, qz, qw)

def quaternion_to_R(quat, matrix):
    """Convert a quaternion into rotation matrix form.

    @param quat:    The quaternion.
    @type quat:     numpy 4D, rank-1 array
    @param matrix:  A 3D matrix to convert to a rotation matrix.
    @type matrix:   numpy 3D, rank-2 array
    """
    qx, qy, qz, qw = quat

    q_norm = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
    qx = qx / q_norm
    qy = qy / q_norm
    qz = qz / q_norm
    qw = qw / q_norm

    matrix[0, 0] = 1 - 2*qy**2 - 2*qz**2
    matrix[0, 1] = 2*qx*qy - 2*qz*qw
    matrix[0, 2] = 2*qx*qz + 2*qy*qw
    matrix[1, 0] = 2*qx*qy + 2*qz*qw
    matrix[1, 1] = 1 - 2*qx**2 - 2*qz**2
    matrix[1, 2] = 2*qy*qz - 2*qx*qw
    matrix[2, 0] = 2*qx*qz - 2*qy*qw
    matrix[2, 1] = 2*qy*qz + 2*qx*qw
    matrix[2, 2] = 1 - 2*qx**2 - 2*qy**2

    """
    # Repetitive calculations.
    q4_2 = quat[3]**2
    q12 = quat[0] * quat[1]
    q13 = quat[0] * quat[2]
    q14 = quat[0] * quat[3]
    q23 = quat[1] * quat[2]
    q24 = quat[1] * quat[3]
    q34 = quat[2] * quat[3]

    # The diagonal.
    matrix[0, 0] = 2.0 * (quat[0]**2 + q4_2) - 1.0
    matrix[1, 1] = 2.0 * (quat[1]**2 + q4_2) - 1.0
    matrix[2, 2] = 2.0 * (quat[2]**2 + q4_2) - 1.0

    # Off-diagonal.
    matrix[0, 1] = 2.0 * (q12 - q34)
    matrix[0, 2] = 2.0 * (q13 + q24)
    matrix[1, 2] = 2.0 * (q23 - q14)

    matrix[1, 0] = 2.0 * (q12 + q34)
    matrix[2, 0] = 2.0 * (q13 - q24)
    matrix[2, 1] = 2.0 * (q23 + q14)
    """

def get_kinect_transform():
    """
    matrix = {}
    quaternion_to_R(quat, matrix)
    T_w_k = np.array([[matrix[0,0], matrix[0,1], matrix[0,2], trans[0]],
                      [matrix[1,0], matrix[1,1], matrix[1,2], trans[1]],
                      [matrix[2,0], matrix[2,1], matrix[2,2], trans[2]],
                      [0, 0, 0, 1]])
    """
    T_w_k = transformations.quaternion_matrix(quat)
    T_w_k[:3,3] += trans
    return T_w_k

    """
    T_w_k[:2, :2] = matrix.copy()
    T_w_k[:2, 3] = trans.data
    T_w_k[3, :3] = np.array([0, 0, 0, 1])
    """

"""
def get_kinect_transform(robot):
    T_w_h = robot.GetLink("head_plate_frame").GetTransform()
    T_w_k = T_w_h.dot(T_h_k)
    return T_w_k
"""