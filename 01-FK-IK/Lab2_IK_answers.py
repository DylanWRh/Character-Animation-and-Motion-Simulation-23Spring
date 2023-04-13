import numpy as np
from scipy.spatial.transform import Rotation as R

def path_info(meta_data, joint_positions, joint_orientations):
    path, path_name, path1, path2 = meta_data.get_path_from_root_to_end()
    path_positions = []
    path_orientations = []
    for joint in path:
        path_positions.append(joint_positions[joint])
        path_orientations.append(R.from_quat(joint_orientations[joint]))
    
    path_offsets = [np.zeros(3)]
    for i in range(1, len(path)):
        path_offsets.append(meta_data.joint_initial_position[path[i]]-meta_data.joint_initial_position[path[i-1]])
    
    return path_positions, path_orientations, path_offsets


def CCD(meta_data, joint_positions, joint_orientations, target_pos):
    path_positions, path_orientations, path_offsets = path_info(meta_data, joint_positions, joint_orientations)
    path, path_name, path1, path2 = meta_data.get_path_from_root_to_end()
    epoches = 10
    EPS = 1e-2
    end_index = path_name.index(meta_data.end_joint)
    for epoch in range(epoches):
        if np.linalg.norm(joint_positions[path[end_index]] - target_pos) < EPS:
            break 
        for i in range(end_index-1, -1, -1):
            cur_pos = path_positions[i]
            end_pos = path_positions[end_index]

            cur_to_end_vec = end_pos - cur_pos
            cur_to_end_vec = cur_to_end_vec / np.linalg.norm(cur_to_end_vec)
            cur_to_des_vec = target_pos - cur_pos
            cur_to_des_vec = cur_to_des_vec / np.linalg.norm(cur_to_des_vec)

            rot_axis = np.cross(cur_to_end_vec, cur_to_des_vec)
            rot_axis = rot_axis / np.linalg.norm(rot_axis)
            # angle divided by 2 to avoid over rotating and improve effect
            rot_angle = np.arccos(np.clip(np.dot(cur_to_end_vec, cur_to_des_vec), -1, 1)) / 2 
            rot_vector = rot_angle * rot_axis
            rotation = R.from_rotvec(rot_vector) # Rotation

            path_orientations[i] = rotation * path_orientations[i]
            for j in range(i, end_index):
                path_positions[j + 1] = path_positions[j] + path_orientations[j].apply(path_offsets[j + 1])

    return path_positions, path_orientations

def calc_joint_from_path(meta_data, joint_positions, joint_orientations, path_positions, path_orientations):
    path, path_name, path1, path2 = meta_data.get_path_from_root_to_end()

    joint_rotations = R.identity(len(meta_data.joint_name))
    for i, parent in enumerate(meta_data.joint_parent):
        if parent == -1:
            joint_rotations[i] = R.from_quat(joint_orientations[i])
        else:
            joint_rotations[i] = R.inv(R.from_quat(joint_orientations[parent])) * R.from_quat(joint_orientations[i])
    
    # path FK
    # positions
    for joint, pos in zip(path, path_positions):
        joint_positions[joint] = pos
    # orientations, change orientation closer to RT
    # path = path2 + inverse(path1)
    for i in range(len(path2) - 1): # rt -> RT
        joint_orientations[path[i+1]] = path_orientations[i].as_quat()
    for i in range(len(path1) - 1): # RT -> end
        joint_orientations[path[i+len(path2)]] = path_orientations[i + len(path2)].as_quat()

    # non-path FK
    for i, parent in enumerate(meta_data.joint_parent):
        if parent == -1:
            continue
        if meta_data.joint_name[i] in path_name:
            continue 
        offset = meta_data.joint_initial_position[i] - meta_data.joint_initial_position[parent]
        rot = R.from_quat(joint_orientations[parent])
        joint_positions[i] = joint_positions[parent] + rot.apply(offset)
        joint_orientations[i] = (R.from_quat(joint_orientations[parent]) * joint_rotations[i]).as_quat()
    
    return joint_positions, joint_orientations


def part1_inverse_kinematics(meta_data, joint_positions, joint_orientations, target_pose):
    """
    完成函数，计算逆运动学
    输入: 
        meta_data: 为了方便，将一些固定信息进行了打包，见上面的meta_data类
        joint_positions: 当前的关节位置，是一个numpy数组，shape为(M, 3)，M为关节数
        joint_orientations: 当前的关节朝向，是一个numpy数组，shape为(M, 4)，M为关节数
        target_pose: 目标位置，是一个numpy数组，shape为(3,)
    输出:
        经过IK后的姿态
        joint_positions: 计算得到的关节位置，是一个numpy数组，shape为(M, 3)，M为关节数
        joint_orientations: 计算得到的关节朝向，是一个numpy数组，shape为(M, 4)，M为关节数
    """
    
    #### Write Your Code Here ####
    path_positions, path_orientations = CCD(meta_data, joint_positions, joint_orientations, target_pose)
    joint_positions, joint_orientations = calc_joint_from_path(meta_data, joint_positions, joint_orientations, path_positions, path_orientations)
    return joint_positions, joint_orientations

def part2_inverse_kinematics(meta_data, joint_positions, joint_orientations, target_height_ratio):
    """
    输入RightFoot，相较于bvh动作目标高度的比例，IK以外的部分与bvh一致
    """
    
    #### Write Your Code Here ####
    right_foot_idx = meta_data.joint_name.index('RightFoot')
    target_pose = joint_positions[right_foot_idx].copy()
    target_pose[1] *= target_height_ratio
    path_positions, path_orientations = CCD(meta_data, joint_positions, joint_orientations, target_pose)
    joint_positions, joint_orientations = calc_joint_from_path(meta_data, joint_positions, joint_orientations, path_positions, path_orientations)
    return joint_positions, joint_orientations

