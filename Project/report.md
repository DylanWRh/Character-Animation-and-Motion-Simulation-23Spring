# Motion Matching -- Report

## `bvh_loader.py`文件修改

修改内容
- `BVHMotion.__init__`时增加`self.frame_time`变量
- `load_motion_data`、`BVHMotion.load_motion`函数增加获取`frame_time`的代码
- 加入`decompose_rotation_with_yaxis`函数（直接引自Lab2中代码）

使用方式：直接替换原来的`bvh_loader.py`

## 运行方式

将`task1_project.py`中的`simu_flag`设置为0，直接运行即可

## 实现说明

以下功能的实现代码均在`answer_project.py`中

**Step 1** 构建特征：主要在`CharacterController.__init__`函数中实现，读取所需的BVH文件，并提取每个动作中根节点的位置(3D)、旋转(4D)、速度(3D)、角速度(3D)共13维的特征$f$

**Step 2** 动作选择：主要在`CharacterController.update_state`函数中实现，根据输入的期望位置变化、旋转变化、速度、角速度作为特征$\tilde{f}$，计算$\displaystyle\min_i ||f_i-\tilde{f}||$，得到目标动作的索引$i$

**Step 3** 插值：主要通过`CharacterController.blending`实现，利用当前动作和目标动作，通过lerp进行插值获取中间帧的动作，插值使用的`quad_slerp`也来自Lab2

## 功能说明

- 使用的BVH文件包括`idle_.bvh`、`run_forward_.bvh`、`walk_forward_.bvh`、`walk_and_turn_left.bvh`、`walk_and_turn_right.bvh`，故可支持静止、行走、跑步、转向等动作的控制
- 使用lerp插值进行动作的连接，使得动作转换较为平滑

## 缺陷分析

- 对走路和跑步的动作未进行预处理或循环化，故若当前动作位于走路/跑步BVH的后几帧，而预测出的目标动作为相应BVH的前几帧时，会有动作的不自然性；这一点在连续跑步时体现得较为明显，因为每一次预测新动作的间隔为20帧，而跑步BVH整体只有45帧
- 极其大幅的右转向（逆时针向后转）时可能会出现一定的打滑，而左转向时该现象不明显，原因在于左转向的BVH中有转至身后（即初始动作和末动作根节点方向相差180°左右）的动作，而右转向的BVH中只有转至右侧的动作，这将导致右转向角度大于90°时动作的不自然