from bvh_utils import *
#---------------你的代码------------------#
# translation 和 orientation 都是全局的
def skinning(joint_translation, joint_orientation, T_pose_joint_translation, T_pose_vertex_translation, skinning_idx, skinning_weight):
    """
    skinning函数，给出一桢骨骼的位姿，计算蒙皮顶点的位置
    假设M个关节，N个蒙皮顶点，每个顶点受到最多4个关节影响
    输入：
        joint_translation: (M,3)的ndarray, 目标关节的位置
        joint_orientation: (M,4)的ndarray, 目标关节的旋转，用四元数表示
        T_pose_joint_translation: (M,3)的ndarray, T pose下关节的位置
        T_pose_vertex_translation: (N,3)的ndarray, T pose下蒙皮顶点的位置
        skinning_idx: (N,4)的ndarray, 每个顶点受到哪些关节的影响（假设最多受4个关节影响）
        skinning_weight: (N,4)的ndarray, 每个顶点受到对应关节影响的权重
    输出：
        vertex_translation: (N,3)的ndarray, 蒙皮顶点的位置
    """
    vertex_translation = np.zeros_like(T_pose_vertex_translation)
    vertex_translation = T_pose_vertex_translation
    
    N = len(skinning_idx)
    Q = R.from_quat(joint_orientation[skinning_idx].reshape([N * 4, 4])).as_matrix().reshape([N, 4, 3, 3]) # (N, 4, 3, 3)
    joint_targ = joint_translation[skinning_idx] # (N, 4, 3)
    joint_orig = T_pose_joint_translation[skinning_idx] # (N, 4, 3)

    r = (T_pose_vertex_translation[:, None, :] - joint_orig) # (N, 4, 3)
    Qr = np.einsum('...ij,...j->...i', Q, r) # (N, 4, 3)

    vertex_translation = np.einsum('...ij,...i->...j', (Qr + joint_targ), skinning_weight)

    return vertex_translation