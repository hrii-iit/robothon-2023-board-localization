from geometry_msgs.msg import Pose, Transform, Quaternion
from scipy.spatial.transform import Rotation as R
import numpy as np

def to_matrix(pose):
    matrix = np.identity(4)
    if pose.__class__.__name__ == "Transform":
        matrix[0:3,3] = np.array([pose.translation.x, pose.translation.y, pose.translation.z])
        matrix[0:3,0:3] = R.from_quat([pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w]).as_matrix()
    elif pose.__class__.__name__ == "Pose":
        matrix[0:3,3] = np.array([pose.position.x, pose.position.y, pose.position.z])
        matrix[0:3,0:3] = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]).as_matrix()
    return matrix

def to_pose(matrix):
    pose = Pose()
    pose.position.x = matrix[0,3]
    pose.position.y = matrix[1,3]
    pose.position.z = matrix[2,3]
    quat = R.from_matrix(matrix[0:3,0:3]).as_quat()
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    return pose

def to_transform(matrix):
    pose = Transform()
    pose.translation.x = matrix[0,3]
    pose.translation.y = matrix[1,3]
    pose.translation.z = matrix[2,3]
    quat = R.from_matrix(matrix[0:3,0:3]).as_quat()
    pose.rotation.x = quat[0]
    pose.rotation.y = quat[1]
    pose.rotation.z = quat[2]
    pose.rotation.w = quat[3]
    return pose

def concatenate_poses(pose1, pose2):
    # print("concatenate poses")
    matrix1 = to_matrix(pose1)
    matrix2 = to_matrix(pose2)
    # print(matrix1)
    # print(matrix2)
    res_matrix = np.matmul(matrix1, matrix2)
    # print(res_matrix)
    return to_pose(res_matrix)

def transform_to_pose(transform):
    pose = Pose()
    pose.position.x = transform.translation.x
    pose.position.y = transform.translation.y
    pose.position.z = transform.translation.z
    pose.orientation.x = transform.rotation.x
    pose.orientation.y = transform.rotation.y
    pose.orientation.z = transform.rotation.z
    pose.orientation.w = transform.rotation.w
    return pose

def pose_to_transform(pose):
    transform = Transform()
    transform.translation.x = pose.position.x
    transform.translation.y = pose.position.y
    transform.translation.z = pose.position.z
    transform.rotation.x = pose.orientation.x
    transform.rotation.y = pose.orientation.y
    transform.rotation.z = pose.orientation.z
    transform.rotation.w = pose.orientation.w
    return pose

def concatenate_quat(quat1, quat2):
    r_quat1 = R.from_quat([quat1.x, quat1.y, quat1.z, quat1.w])
    r_quat2 = R.from_quat([quat2.x, quat2.y, quat2.z, quat2.w])
    r_res_quat = (r_quat1 * r_quat2).as_quat()
    res_quat = Quaternion()
    res_quat.x = r_res_quat[0]
    res_quat.y = r_res_quat[1]
    res_quat.z = r_res_quat[2]
    res_quat.w = r_res_quat[3]
    return res_quat

def get_rpy(quat):
    # print(quat)
    r = R.from_quat([quat.x, quat.y, quat.z, quat.w])
    return r.as_euler('xyz')

# def get_rpy(matrix):
#     print(matrix)
#     r = R.from_matrix(matrix)
#     return r.as_euler()