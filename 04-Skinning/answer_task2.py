from bvh_utils import *
#---------------你的代码------------------#
# translation 和 orientation 都是全局的
def dq_skinning(joint_translation, joint_orientation, T_pose_joint_translation, T_pose_vertex_translation, skinning_idx, skinning_weight):
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

    # 默认使用 task1
    # from answer_task1 import skinning
    # vertex_translation = skinning(joint_translation, joint_orientation, T_pose_joint_translation, T_pose_vertex_translation, skinning_idx, skinning_weight)
    
    #---------------------------你的代码---------------------------#
    
    def qt2dq(q, t):
        # q: (N, 4[xyzw]), t: (N, 3)
        # DQ: (N, 8[xyzwxyzw])
        N = len(q)
        DQ = np.zeros((N, 8))
        DQ[:, :4] = q[:, :]
        DQ[:, 4] =  0.5 * ( q[:, 3] * t[:, 0] + q[:, 2] * t[:, 1] - q[:, 1] * t[:, 2])
        DQ[:, 5] =  0.5 * (-q[:, 2] * t[:, 0] + q[:, 3] * t[:, 1] + q[:, 0] * t[:, 2])
        DQ[:, 6] =  0.5 * ( q[:, 1] * t[:, 0] - q[:, 0] * t[:, 1] + q[:, 3] * t[:, 2])
        DQ[:, 7] = -0.5 * ( q[:, 0] * t[:, 0] + q[:, 1] * t[:, 1] + q[:, 2] * t[:, 2])
        return DQ

    q = R.from_quat(joint_orientation).as_quat()
    t = joint_translation - T_pose_joint_translation
    DQs = qt2dq(q, t)
    DQ = DQs[skinning_idx] # (N, 4, 8[xyzwxyzw])

    N = len(DQ)
    b = np.einsum('ij,ijk->ik', skinning_weight, DQ) # (N, 8[xyzwxyzw])

    x, y, z, w, t1, t2, t3, t0 = b.T # Each (N, )

    M = np.zeros((N, 3, 4))
    len2 = 1 / (x*x + y*y + z*z + w*w)
    M[:, 0, 0] = (w*w + x*x - y*y - z*z) * len2
    M[:, 0, 1] = (2*x*y - 2*w*z) * len2
    M[:, 0, 2] = (2*x*z + 2*w*y) * len2
    M[:, 1, 0] = (2*x*y + 2*w*z) * len2
    M[:, 1, 1] = (w*w + y*y - x*x - z*z) * len2
    M[:, 1, 2] = (2*y*z - 2*w*x) * len2
    M[:, 2, 0] = (2*x*z - 2*w*y) * len2
    M[:, 2, 1] = (2*y*z + 2*w*x) * len2
    M[:, 2, 2] = (w*w + z*z - x*x - y*y) * len2
    M[:, 0, 3] = (-2*t0*x + 2*t1*w - 2*t2*z + 2*t3*y) * len2
    M[:, 1, 3] = (-2*t0*y + 2*t1*z + 2*t2*w - 2*t3*x) * len2
    M[:, 2, 3] = (-2*t0*z - 2*t1*y + 2*t2*x + 2*t3*w) * len2

    avg_t = np.sum(T_pose_joint_translation[skinning_idx] * skinning_weight[..., None], axis=1)
    pts = T_pose_vertex_translation - avg_t
    vertex_translation[:, 0] = M[:, 0, 0] * pts[:, 0] + M[:, 0, 1] * pts[: ,1] + M[:, 0, 2] * pts[: ,2] + M[:, 0, 3]
    vertex_translation[:, 1] = M[:, 1, 0] * pts[:, 0] + M[:, 1, 1] * pts[: ,1] + M[:, 1, 2] * pts[: ,2] + M[:, 1, 3]
    vertex_translation[:, 2] = M[:, 2, 0] * pts[:, 0] + M[:, 2, 1] * pts[: ,1] + M[:, 2, 2] * pts[: ,2] + M[:, 2, 3]


    # from answer_task1 import skinning
    # vertex_translation = skinning(joint_translation, joint_orientation, T_pose_joint_translation, T_pose_vertex_translation, skinning_idx, skinning_weight)

    #-------------------------------------------------------------#
    return vertex_translation+avg_t