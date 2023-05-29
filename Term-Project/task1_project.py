from viewer.controller import  Controller
from viewer.viewer_new import SimpleViewer
from answer_project import *
import numpy as np

class InteractiveUpdate():
    def __init__(self, viewer, controller, character_controller):
        self.viewer = viewer
        self.controller = controller
        self.character_controller = character_controller
        
    def update(self, task):
        desired_pos_list, desired_rot_list, desired_vel_list, desired_avel_list, current_gait = \
            self.controller.get_desired_state()
        
        # 逐帧更新角色状态
        character_state = self.character_controller.update_state(
                desired_pos_list, desired_rot_list, 
                desired_vel_list, desired_avel_list
                )
        # 同步手柄和角色的状态
        self.character_controller.sync_controller_and_character(character_state)
        
        # viewer 渲染
        name, pos, rot = character_state[0], character_state[1], character_state[2]
        self.viewer.set_pose(name, pos, rot)
        
        return task.cont   

    

def main():
    # 创建一个控制器，是输入信号的一层包装与平滑
    # 可以使用键盘(方向键或wasd)或鼠标控制视角
    # 对xbox类手柄的支持在windows10下测试过，左手柄控制移动，右手柄控制视角
    # 其余手柄(如ps手柄)不能保证能够正常工作
    # 注意检测到手柄后，键盘输入将被忽略
    # simu_flag = 0 表示不使用physics character
    # simu_flag = 1 表示使用physics character
    viewer = SimpleViewer(float_base = True, substep = 32, simu_flag=0)
    pd_controller = PDController(viewer)
    controller = Controller(viewer)
    character_controller = CharacterController(controller, pd_controller)
    task = InteractiveUpdate(viewer, controller, character_controller)
    viewer.addTask(task.update)
    viewer.run()
    pass

if __name__ == '__main__':
    main()