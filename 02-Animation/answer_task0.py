import numpy as np

# part 0
def load_meta_data(bvh_file_path):
    """
    请把lab1-FK-part1的代码复制过来
    请填写以下内容
    输入： bvh 文件路径
    输出:
        joint_name: List[str]，字符串列表，包含着所有关节的名字
        joint_parent: List[int]，整数列表，包含着所有关节的父关节的索引,根节点的父关节索引为-1
        channels: List[int]，整数列表，joint的自由度，根节点为6(三个平动三个转动)，其余节点为3(三个转动)
        joint_offset: np.ndarray，形状为(M, 3)的numpy数组，包含着所有关节的偏移量
    Tips:
        joint_name顺序应该和bvh一致
    """
    

    joint_name = None
    joint_parent = None
    channels = None
    joint_offset = None

    joint_name = []
    joint_parent = []
    channels = []
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
            elif line_lst[0] == 'CHANNELS':
                channels.append(int(line_lst[1]))
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

    
    return joint_name, joint_parent, channels, joint_offset