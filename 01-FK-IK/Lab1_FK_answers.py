import numpy as np
from scipy.spatial.transform import Rotation as R

def load_motion_data(bvh_file_path):
    """part2 辅助函数，读取bvh文件"""
    with open(bvh_file_path, 'r') as f:
        lines = f.readlines()
        for i in range(len(lines)):
            if lines[i].startswith('Frame Time'):
                break
        motion_data = []
        for line in lines[i+1:]:
            data = [float(x) for x in line.split()]
            if len(data) == 0:
                break
            motion_data.append(np.array(data).reshape(1,-1))
        motion_data = np.concatenate(motion_data, axis=0)
    return motion_data



def part1_calculate_T_pose(bvh_file_path):
    """请填写以下内容
    输入： bvh 文件路径
    输出:
        joint_name: List[str]，字符串列表，包含着所有关节的名字
        joint_parent: List[int]，整数列表，包含着所有关节的父关节的索引,根节点的父关节索引为-1
        joint_offset: np.ndarray，形状为(M, 3)的numpy数组，包含着所有关节的偏移量

    Tips:
        joint_name顺序应该和bvh一致
    """
    joint_name = None
    joint_parent = None
    joint_offset = None
    
    #### Write Your Code Here ####
    joint_name = []
    joint_parent = []
    joint_offset = []

    my_stack = []
    cnt = -1

    with open(bvh_file_path, 'r') as f:
        lines = f.readlines()
        for line in lines:
            line_lst = line.split()
            if line_lst[0] == 'ROOT':
                joint_name.append(line_lst[1])
                joint_parent.append(-1)
            elif line_lst[0] == '{':
                cnt += 1
                my_stack.append(cnt)
            elif line_lst[0] == 'OFFSET':
                joint_offset.append([float(i) for i in line_lst[1:]])
            elif line_lst[0] == 'JOINT':
                joint_name.append(line_lst[1])
                joint_parent.append(my_stack[-1])
            elif line_lst[0] == '}':
                my_stack.pop()
            elif line_lst[0] == 'MOTION':
                break
    joint_offset = np.array(joint_offset)


    return joint_name, joint_parent, joint_offset


def part2_forward_kinematics(joint_name, joint_parent, joint_offset, motion_data, frame_id):
    """请填写以下内容
    输入: part1 获得的关节名字，父节点列表，偏移量列表
        motion_data: np.ndarray，形状为(N,X)的numpy数组，其中N为帧数，X为Channel数
        frame_id: int，需要返回的帧的索引
    输出:
        joint_positions: np.ndarray，形状为(M, 3)的numpy数组，包含着所有关节的全局位置
        joint_orientations: np.ndarray，形状为(M, 4)的numpy数组，包含着所有关节的全局旋转(四元数)
    
    Tips:
        1. joint_orientations的四元数顺序为(x, y, z, w)
        2. from_euler时注意使用大写的XYZ
    """
    joint_positions = None
    joint_orientations = None
    
    #### Write Your Code Here ####
    joint_positions = []
    joint_orientations = []

    channels = motion_data[frame_id].reshape(-1, 3)
    quats = R.from_euler('XYZ', channels[1:], degrees=True) # Rotation

    for i, parent in enumerate(joint_parent):
        if parent == -1:
            joint_positions.append(channels[0])
            joint_orientations.append(quats[0].as_quat())
        else:
            rot = R.from_quat(joint_orientations[parent]) * quats[i] 
            joint_orientations.append(R.as_quat(rot))

            rot_offset = R.from_quat(joint_orientations[parent]).apply(joint_offset[i])
            joint_positions.append(joint_positions[parent] + rot_offset)

    joint_positions = np.array(joint_positions)
    joint_orientations = np.array(joint_orientations)

    return joint_positions, joint_orientations


def part3_retarget_func(T_pose_bvh_path, A_pose_bvh_path):
    """
    将 A-pose的bvh重定向到T-pose上
    输入: 两个bvh文件的路径
    输出: 
        motion_data: np.ndarray，形状为(N,X)的numpy数组，其中N为帧数，X为Channel数。retarget后的运动数据
    
    Tips:
        两个bvh的joint name顺序可能不一致哦(
        as_euler时也需要大写的XYZ
    """
    motion_data = None
    
    #### Write Your Code Here ####
    joint_name_T, joint_parent_T, joint_offset_T = part1_calculate_T_pose(T_pose_bvh_path)
    joint_name_A, joint_parent_A, joint_offset_A = part1_calculate_T_pose(A_pose_bvh_path)
    motion_data_A = load_motion_data(A_pose_bvh_path)
    N, X = motion_data_A.shape

    pos_data_A = motion_data_A[:, :3]
    rot_data_A = motion_data_A[:, 3:]

    motion_data = np.zeros(motion_data_A.shape)

    motion_data_dict = {}

    for i, name in enumerate(joint_name_A):
        motion_data_dict[name] = rot_data_A[:, 3*i:3*(i+1)]
    
    for i, name in enumerate(joint_name_T):
        if name == 'LeftArm':
            motion_data_dict[name][:, -1] -= 45
        elif name == 'RightArm':
            motion_data_dict[name][:, -1] += 45
        elif name == 'LeftUpLeg':
            motion_data_dict[name][:, -1] += 15
        elif name == 'RightUpLeg':
            motion_data_dict[name][:, -1] -= 15
        motion_data[:, 3*(i+1):3*(i+2)] = motion_data_dict[name]
    
    motion_data[:, :3] = pos_data_A

    return motion_data