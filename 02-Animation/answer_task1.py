"""
注释里统一N表示帧数，M表示关节数
position, rotation表示局部平移和旋转
translation, orientation表示全局平移和旋转
"""
import numpy as np
import copy
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
from bvh_motion import BVHMotion
from smooth_utils import *

def quad_slerp(rot1, rot2, alpha):
    cos_val = np.dot(rot1, rot2)
    if cos_val < 0.:
        cos_val = -cos_val
        rot1 = -rot1 
    
    theta = np.arccos(cos_val)
    sin_val = np.sin(theta)

    EPS = 1e-3
    if sin_val > EPS:
        alpha1 = np.sin((1 - alpha) * theta) / sin_val
        alpha2 = np.sin(alpha * theta) / sin_val
    else:
        alpha1 = 1 - alpha
        alpha2 = alpha
    
    res_rot = alpha1 * rot1 + alpha2 * rot2
    res_rot /= np.linalg.norm(res_rot)

    return res_rot

# part1
def blend_two_motions(bvh_motion1:BVHMotion, bvh_motion2:BVHMotion, v:float=None, input_alpha:np.ndarray=None, target_fps=60) -> BVHMotion:
    '''
    输入: 两个将要blend的动作，类型为BVHMotion
          将要生成的BVH的速度v
          如果给出插值的系数alpha就不需要再计算了
          target_fps,将要生成BVH的fps
    输出: blend两个BVH动作后的动作，类型为BVHMotion
    假设两个动作的帧数分别为n1, n2
    首先需要制作blend 的权重适量 alpha
    插值系数alpha: 0~1之间的浮点数组，形状为(n3,)
    返回的动作有n3帧，第i帧由(1-alpha[i]) * bvh_motion1[j] + alpha[i] * bvh_motion2[k]得到
    i均匀地遍历0~n3-1的同时，j和k应该均匀地遍历0~n1-1和0~n2-1
    Tips:
        1. 计算速度，两个BVH已经将Root Joint挪到(0.0, 0.0)的XOZ位置上了
        2. 利用v计算插值系数alpha
        3. 线性插值以及Slerp
        4. 可能输入的两个BVH的fps不同，需要考虑
    '''
    
    
    res = bvh_motion1.raw_copy()
    res.joint_position = np.zeros_like(res.joint_position)
    res.joint_rotation = np.zeros_like(res.joint_rotation)
    res.joint_rotation[...,3] = 1.0
    res.frame_time = 1.0 / target_fps

    # TODO: 你的代码
    n1 = bvh_motion1.motion_length
    n2 = bvh_motion2.motion_length
    fps1 = 1.0 / bvh_motion1.frame_time
    fps2 = 1.0 / bvh_motion2.frame_time
    t1 = n1 / fps1 
    t2 = n2 / fps2
    dist1 = bvh_motion1.joint_position[-1, 0, [0, 2]]
    dist2 = bvh_motion2.joint_position[-1, 0, [0, 2]]
    v1 = np.linalg.norm(dist1) / t1 
    v2 = np.linalg.norm(dist2) / t2
    w1 = (v2 - v) / (v2 - v1)
    w2 = 1.0 - w1 

    dist3 = w1 * dist1 + w2 * dist2 
    t3 = np.linalg.norm(dist3) / v
    n3 = np.round(t3 * target_fps).astype(np.int32)
    alpha = np.ones((n3, )) * w2

    _, joint_num, position_channel_num = bvh_motion1.joint_position.shape
    _, joint_num, rotation_channel_num = bvh_motion1.joint_rotation.shape
    res.joint_position = np.zeros((n3, joint_num, position_channel_num))
    res.joint_rotation = np.zeros((n3, joint_num, rotation_channel_num))
    res.joint_rotation[..., 3] = 1.0 

    for i in range(n3):
        j = (i * n1) // n3 
        k = (i * n2) // n3 
        res.joint_position[i, :, :] = (1.0 - alpha[i]) * bvh_motion1.joint_position[j] + alpha[i] * bvh_motion2.joint_position[k]

        for l in range(joint_num):
            res.joint_rotation[i, l, :] = quad_slerp(bvh_motion1.joint_rotation[j,l,:], bvh_motion2.joint_rotation[k, l, :], alpha[i])

    

    return res

# part2
def build_loop_motion(bvh_motion:BVHMotion, ratio:float, half_life:float) -> BVHMotion:
    '''
    输入: 将要loop化的动作，类型为BVHMotion
          damping在前在后的比例ratio, ratio介于[0,1]
          弹簧振子damping效果的半衰期 half_life
          如果你使用的方法不含上面两个参数，就忽视就可以了，因接口统一保留
    输出: loop化后的动作，类型为BVHMotion
    
    Tips:
        1. 计算第一帧和最后一帧的旋转差、Root Joint位置差 (不用考虑X和Z的位置差)
        2. 如果使用"inertialization"，可以利用`smooth_utils.py`的
        `quat_to_avel`函数计算对应角速度的差距，对应速度的差距请自己填写
        3. 逐帧计算Rotations和Postions的变化
        4. 注意 BVH的fps需要考虑，因为需要算对应时间
        5. 可以参考`smooth_utils.py`的注释或者 https://theorangeduck.com/page/creating-looping-animations-motion-capture
    
    '''
    res = bvh_motion.raw_copy()
    
    # TODO: 你的代码
    dt = bvh_motion.frame_time

    rot = bvh_motion.joint_rotation
    pos = bvh_motion.joint_position
    avel = quat_to_avel(rot, dt)

    rot_diff = (R.from_quat(rot[-1]) * R.from_quat(rot[0]).inv()).as_rotvec()
    avel_diff = avel[-1] - avel[0]
    pos_diff = pos[-1] - pos[0]
    pos_diff[:, [0, 2]] = 0
    v_start = pos[1] - pos[0]
    v_end = pos[-1] - pos[-2]
    v_diff = (v_start - v_end) * dt

    n = bvh_motion.motion_length
    for i in range(n):
        offset_rot = R.from_rotvec(
            decay_spring_implicit_damping_rot(
                ratio * rot_diff, 
                ratio * avel_diff, 
                half_life, i * dt)[0] + \
                    decay_spring_implicit_damping_rot(
                        -(1.0 - ratio) * rot_diff, 
                        -(1.0 - ratio) * avel_diff,
                        half_life, (n - i - 1) * dt)[0]
        )
        res.joint_rotation[i] = (offset_rot * R.from_quat(rot[i])).as_quat()

        offset_pos = decay_spring_implicit_damping_pos(ratio * pos_diff, ratio * v_diff, half_life, i * dt)[0] + \
                decay_spring_implicit_damping_pos(-(1.0 - ratio) * pos_diff, -(1.0 - ratio) * v_diff, half_life, (n - i - 1) * dt)[0]
        res.joint_position[i] += offset_pos
    
    return res

# part3
def concatenate_two_motions(bvh_motion1:BVHMotion, bvh_motion2:BVHMotion, mix_frame1:int, mix_time:int):
    '''
    将两个bvh动作平滑地连接起来
    输入: 将要连接的两个动作，类型为BVHMotion
          混合开始时间是第一个动作的第mix_frame1帧
          mix_time表示用于混合的帧数
    输出: 平滑地连接后的动作，类型为BVHMotion
    
    Tips:
        你可能需要用到BVHMotion.sub_sequence 和 BVHMotion.append
    '''
    res = bvh_motion1.raw_copy()
    
    # TODO: 你的代码
    # 下面这种直接拼肯定是不行的(
    # res.joint_position = np.concatenate([res.joint_position[:mix_frame1], bvh_motion2.joint_position], axis=0)
    # res.joint_rotation = np.concatenate([res.joint_rotation[:mix_frame1], bvh_motion2.joint_rotation], axis=0)

    rot = bvh_motion1.joint_rotation[mix_frame1, 0]
    facing_dir = R.from_quat(rot).apply(np.array([0, 0, 1])).flatten()[[0, 2]]

    new_bvh_motion2 = bvh_motion2.translation_and_rotation(0, bvh_motion1.joint_position[mix_frame1, 0, [0, 2]], facing_dir)
    v_start = new_bvh_motion2.joint_position[1, 0, [0, 2]] - new_bvh_motion2.joint_position[0, 0, [0, 2]]
    v_end = bvh_motion1.joint_position[mix_frame1, 0, [0, 2]] - bvh_motion1.joint_position[mix_frame1-1, 0, [0, 2]]

    _, joint_num, pos_channel_num = new_bvh_motion2.joint_position.shape
    _, joint_num, rot_channel_num = new_bvh_motion2.joint_rotation.shape

    blend = bvh_motion1.raw_copy()
    blend.joint_position = np.zeros((mix_time, joint_num, pos_channel_num))
    blend.joint_rotation = np.zeros((mix_time, joint_num, rot_channel_num))

    for i in range(mix_time):
        w = i / mix_time
        blend.joint_position[i] = (1 - w) * res.joint_position[mix_frame1] + w * new_bvh_motion2.joint_position[0]
        for j in range(joint_num):
            blend.joint_rotation[i, j] = quad_slerp(res.joint_rotation[mix_frame1, j], new_bvh_motion2.joint_rotation[0, j], w)

    res = res.sub_sequence(0, mix_frame1)
    res.append(blend)
    res.append(new_bvh_motion2)
    return res

