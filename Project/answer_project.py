# 以下部分均为可更改部分，你可以把需要的数据结构定义进来
from typing import List
from bvh_loader import BVHMotion
from scipy.spatial.transform import Rotation as R
from physics_warpper import PhysicsInfo
from smooth_utils import quat_to_avel, build_loop_motion
import numpy as np

class PDController:
    def __init__(self, viewer) -> None:
        self.viewer = viewer
        self.physics_info = PhysicsInfo(viewer)
        self.cnt = 0
        self.get_pose = None
        pass
    
    def apply_pd_torque(self):
        pass

    def apply_root_force_and_torque(self):
        pass
    
    def apply_static_torque(self):
        pass

class CharacterController():
    def __init__(self, viewer, controller, pd_controller) -> None:
        # viewer 类，封装physics
        self.viewer = viewer
        # 手柄/键盘控制器
        self.controller = controller
        # pd controller
        self.pd_controller = pd_controller
        # motion
        self.motions = []
        # 添加motion
        self.motions.append(BVHMotion(bvh_file_name='./motion_material/idle_.bvh'))
        self.motions.append(BVHMotion(bvh_file_name='./motion_material/walk_forward_.bvh'))
        self.motions.append(BVHMotion(bvh_file_name='./motion_material/run_forward_.bvh'))
        self.motions.append(BVHMotion(bvh_file_name='./motion_material/walk_and_ture_right_.bvh'))
        self.motions.append(BVHMotion(bvh_file_name='./motion_material/walk_and_turn_left_.bvh'))

        self.joint_name = self.viewer.joint_name
        for motion in self.motions:
            motion.adjust_joint_name(self.joint_name)
        
        # 下面是你可能会需要的成员变量，只是一个例子形式
        # 当然，你可以任意编辑，来符合你的要求
        joint_translation, joint_orientation = self.motions[0].batch_forward_kinematics()
        # 当前角色的参考root位置
        self.cur_root_pos = joint_translation[0, 0]
        # 当前角色的参考root旋转
        self.cur_root_rot = joint_orientation[0, 0]

        facing_dir = R.from_quat(self.cur_root_rot).apply(np.array([0, 0, 1])).flatten()[[0, 2]]
        for i in range(len(self.motions)):
            self.motions[i] = self.motions[i].translation_and_rotation(0, self.cur_root_pos[[0, 2]], facing_dir)
        self.features = None 
        self.id2motion = {}
        self.build_features()
        self.pos_weight = 1.
        self.rot_weight = 1.
        self.vel_weight = 1.
        self.avel_weight = 1.     

        # 当前角色处于正在跑的BVH的第几帧
        self.cur_frame = 0
        self.cur_motionid = -1
        self.cur_motion = (joint_translation[0], joint_orientation[0])
        self.following_motions = []
        self.blending_frame = 20
        

    def build_features(self):
        poss = []
        avels = []
        vels = []
        rots = []
        cur_id = 0
        for motionid, motion in enumerate(self.motions):
            joint_translation, joint_orientation = motion.batch_forward_kinematics()

            poss.append(joint_translation[1:, 0, :] - joint_translation[[0], 0, :])

            avel = quat_to_avel(joint_orientation, motion.frame_time)
            avel[:, :, [0, 2]] = 0
            avels.append(avel[:, 0, :])

            vel = (joint_translation[1:] - joint_translation[: -1]) / motion.frame_time
            vel[:, :, 1] = 0
            vels.append(vel[:, 0, :])

            rots.append((R.from_quat(joint_orientation[1:, 0, :]) * R.from_quat(joint_orientation[[0], 0, :])).as_quat())

            self.id2motion.update({cur_id+i: {'motion': motionid, 'frame': i+1} for i in range(len(joint_orientation)-1)})
            cur_id += len(joint_orientation) - 1
        poss = np.concatenate(poss, axis=0)
        avels = np.concatenate(avels, axis=0)
        vels = np.concatenate(vels, axis=0)
        rots = np.concatenate(rots, axis=0)
        self.features = {'poss': poss, 'avels': avels, 'vels': vels, 'rots': rots}
        return


    def get_sim_motion(self, pos, rot, vel, avel):
        def get_sim(f, x, w=1):
            return w * np.linalg.norm(f - x[None, :], axis=-1) ** 2
        sim = get_sim(self.features['avels'], avel, self.avel_weight) + get_sim(self.features['vels'], vel, self.vel_weight) +\
            get_sim((R.from_quat(self.features['rots']).inv() * R.from_quat(rot)).as_quat(), np.array([0, 0, 0, 1]), self.rot_weight) +\
            get_sim(self.features['poss'], pos, self.pos_weight)
        
        tgt_id = np.argmin(sim)
        motionid, frame = self.id2motion[tgt_id].values()
        return motionid, frame


    def blending(self, tgt_translation, tgt_orientation, frame_num):
        cur_translation, cur_orientation = self.cur_motion
        delta_translation = tgt_translation - cur_translation
        dt = 1 / frame_num
        joint_num = tgt_orientation.shape[0]
        self.following_motions = []
        for i in range(1, frame_num + 1):
            new_orientation = np.array([quad_slerp(cur_orientation[j], tgt_orientation[j], dt * i) for j in range(joint_num)])
            self.following_motions.append((cur_translation + dt * i * delta_translation, new_orientation))
        return

    def update_state(self, 
                     desired_pos_list, 
                     desired_rot_list,
                     desired_vel_list,
                     desired_avel_list
                     ):
        '''
        Input: 平滑过的手柄输入,包含了现在(第0帧)和未来20,40,60,80,100帧的期望状态
            当然我们只是提供了速度和角速度的输入，如果通过pos和rot已经很好选择下一个动作了，可以不必须使用速度和角速度
            desired_pos_list: 期望位置, 6x3的矩阵, [x, 0, z], 每一行对应0，20，40...帧的期望位置(XoZ平面)， 期望位置可以用来拟合根节点位置
            desired_rot_list: 期望旋转, 6x4的矩阵, 四元数, 每一行对应0，20，40...帧的期望旋转(Y旋转), 期望旋转可以用来拟合根节点旋转
            desired_vel_list: 期望速度, 6x3的矩阵, [x, 0, z], 每一行对应0，20，40...帧的期望速度(XoZ平面), 期望速度可以用来拟合根节点速度
            desired_avel_list: 期望角速度, 6x3的矩阵, [0, y, 0], 每一行对应0，20，40...帧的期望角速度(Y旋转), 期望角速度可以用来拟合根节点角速度
        
        Output: 输出下一帧的关节名字,关节位置,关节旋转
            joint_name: List[str], 代表了所有关节的名字
            joint_translation: np.ndarray，形状为(M, 3)的numpy数组，包含着所有关节的全局位置
            joint_orientation: np.ndarray，形状为(M, 4)的numpy数组，包含着所有关节的全局旋转(四元数)
        Tips:
            1. 注意应该利用的期望位置和期望速度应该都是在XoZ平面内，期望旋转和期望角速度都是绕Y轴的旋转。其他的项没有意义

        '''
        # 一个简单的例子，循环播放第0个动画第0帧，不会响应输入信号
        # motion = self.motions[0]
        # motion.adjust_joint_name(self.viewer.joint_name)
        # joint_name = motion.joint_name
        # 
        # joint_translation, joint_orientation = motion.batch_forward_kinematics(root_pos=self.controller.viewer.root_pos)
        # joint_translation = joint_translation[self.cur_frame]
        # joint_orientation = joint_orientation[self.cur_frame]

        cur_pos, cur_rot, cur_vel, cur_avel = desired_pos_list[0], desired_rot_list[0], desired_vel_list[0], desired_avel_list[0]
        tgt_pos, tgt_rot, tgt_vel, tgt_avel = desired_pos_list[1], desired_rot_list[1], desired_vel_list[1], desired_avel_list[1]

        self.cur_root_pos = cur_pos
        self.cur_root_rot = cur_rot
        tgt_root_pos = tgt_pos
        tgt_root_rot, _ = BVHMotion.decompose_rotation_with_yaxis(tgt_rot)
        tgt_facing_dir = R.from_quat(tgt_root_rot).apply(np.array([0, 0, 1])).flatten()[[0, 2]]

        rot = (R.from_quat(tgt_rot) * R.from_quat(cur_rot).inv()).as_quat()
        pos = R.from_quat(cur_rot).inv().apply(tgt_pos - cur_pos)
        vel = R.from_quat(cur_rot).inv().apply(tgt_vel)
        avel = tgt_avel

        if not len(self.following_motions):
            #res = motion.translation_and_rotation(frame, )
            motionid, frame = self.get_sim_motion(pos, rot, vel, avel)
            tgt_motion = self.motions[motionid].translation_and_rotation(frame, tgt_root_pos[[0, 2]], tgt_facing_dir)
            joint_translation, joint_orientation = tgt_motion.batch_forward_kinematics()
            if self.cur_motionid == motionid:
                if frame + self.blending_frame >= len(joint_translation):
                    joint_translation = joint_translation - joint_translation[[0]] + joint_translation[[-1]]
                frame = (self.cur_frame + self.blending_frame) % len(joint_translation)
 
            self.blending(joint_translation[frame], joint_orientation[frame], self.blending_frame)

            self.cur_frame = frame
            self.cur_motionid = motionid

            joint_translation, joint_orientation = self.following_motions.pop(0)
            self.cur_motion = (joint_translation, joint_orientation)
        else:
            joint_translation, joint_orientation = self.following_motions.pop(0)
            self.cur_motion = (joint_translation, joint_orientation)
        joint_name = self.joint_name


        return joint_name, joint_translation, joint_orientation
    

    def sync_controller_and_character(self, character_state):
        '''
        这一部分用于同步手柄和你的角色的状态
        更新后很有可能会出现手柄和角色的位置不一致，
        这里用一个简单的方案，将手柄的位置对齐于角色我位置
        '''
        controller_pos = character_state[1][0] 
        self.controller.set_pos(controller_pos)
    

def quad_slerp(rot1, rot2, alpha):
    cos_val = np.dot(rot1, rot2)
    if cos_val < 0.:
        cos_val = -cos_val
        rot1 = -rot1 
    
    theta = np.arccos(np.clip(cos_val, -1, 1))
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