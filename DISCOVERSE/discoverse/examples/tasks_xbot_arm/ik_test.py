# import mujoco
# import numpy as np
# from scipy.spatial.transform import Rotation
# import os
# from discoverse import DISCOVERSE_ROOT_DIR


# # (保留所有必要的import)
# from discoverse.robots_env.xbot_arm_base import XbotArmCfg
# from discoverse.utils import get_body_tmat
# from discoverse.task_base.xbot_arm_task_base import XbotArmTaskBase

# # (保留 solve_ik 函数的定义)
# def solve_ik(model, data, target_pos, target_quat, site_name="endpoint"):
#     # ... (函数内容不变) ...
#     site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
#     if site_id == -1:
#         raise ValueError("Site '{}' not found in model.".format(site_name))
#     data.site_xpos[site_id] = target_pos
#     data.site_xmat[site_id] = Rotation.from_quat(target_quat).as_matrix().flatten()
#     mujoco.mj_inverse(model, data)
#     pos_error = np.linalg.norm(data.site_xpos[site_id] - target_pos)
#     if pos_error > 0.01:
#         print("WARNING: IK solution found, but with a significant error of {:.4f}m.".format(pos_error))
#         return None
#     return data.qpos[:6].copy()

# # SimNode的父类可能需要一个Sim_Node的实例，我们在这里模拟一个
# class DummySimNode:
#     def __init__(self, cfg):
#         self.mj_model = mujoco.MjModel.from_xml_path(os.path.join(DISCOVERSE_ROOT_DIR, "models", cfg.mjcf_file_path))
#         self.mj_data = mujoco.MjData(self.mj_model)
#         mujoco.mj_forward(self.mj_model, self.mj_data)
#         # 你可能需要在这里添加更多SimNode需要的属性
#         self.running = True
        
        
# if __name__ == "__main__":
#     np.set_printoptions(precision=3, suppress=True, linewidth=500)

#     # --- 1. 在无头模式下初始化仿真环境 ---
#     print("正在以无头模式 (headless=True) 初始化仿真环境...")
#     cfg = XbotArmCfg()
#     cfg.mjcf_file_path = "mjcf/tasks_xbot_arm/place_block.xml"
#     cfg.headless = True # !! 关键修改：设置为True !!
    
#     try:
#         sim_node = DummySimNode(cfg) # 使用DummySimNode来避免GUI初始化
#         print("仿真环境初始化完成。")

#         # ... (后续的IK测试逻辑完全不变) ...
#         print("\n====== IK单元测试开始 ======")
#         target_pos_local = np.array([0.3, 0.0, 0.4])
#         # target_pos_local = np.array([0.2, 0.2, 0.5])
#         target_quat_local = Rotation.from_euler('y', 90, degrees=True).as_quat()

#         print("测试目标 (机械臂局部坐标): ...")

#         tmat_armbase_to_world = get_body_tmat(sim_node.mj_data, "arm_pose")
        
#         # !! 在这里加入一个断言或打印，来检查 tmat_armbase_to_world !!
#         print("\n获取到的基座变换矩阵 tmat_armbase_to_world:\n", tmat_armbase_to_world)
#         assert not np.all(tmat_armbase_to_world[:3, :3] == 0), "旋转矩阵部分为空！"
        
#         target_pos_world = (tmat_armbase_to_world @ np.append(target_pos_local, 1))[:3]
#         target_rot_world = Rotation.from_matrix(tmat_armbase_to_world[:3, :3]) * Rotation.from_quat(target_quat_local)
#         target_quat_world = target_rot_world.as_quat()

#         print("\n转换后的测试目标 (世界坐标): ...")
        
#         print("\n正在调用IK求解器...")
#         ik_solution = solve_ik(sim_node.mj_model, sim_node.mj_data, target_pos_world, target_quat_world)

#         if ik_solution is not None:
#             print("\n[SUCCESS] IK 求解成功!")
#             print("  - 请将下面这行关节角度复制到下一步的验证脚本中:")
#             print("  - 解算出的关节角度 (rad): np.array({})".format(np.array2string(ik_solution, separator=',')))
#         else:
#             print("\n[FAILURE] IK 求解失败!")

#     except Exception as e:
#         print(f"程序出错: {e}")

#     print("\n====== IK单元测试结束 ======")





import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
import time
import os

from discoverse import DISCOVERSE_ROOT_DIR
from discoverse.robots_env.xbot_arm_base import XbotArmCfg

# (我们不再需要 SimNode 或 get_body_tmat，因为我们要手动操作)
# from discoverse.task_base.xbot_arm_task_base import XbotArmTaskBase
# from discoverse.utils import get_body_tmat

# (保留 solve_ik 函数，但我们可能暂时用不到它的坐标变换部分)
def solve_ik(model, data, target_pos, target_quat, site_name="endpoint"):
    # ... (函数内容保持不变) ...
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
    if site_id == -1: raise ValueError("Site '{}' not found.".format(site_name))
    data.site_xpos[site_id] = target_pos
    data.site_xmat[site_id] = Rotation.from_quat(target_quat).as_matrix().flatten()
    mujoco.mj_inverse(model, data)
    pos_error = np.linalg.norm(data.site_xpos[site_id] - target_pos)
    if pos_error > 0.01:
        print("WARNING: IK solution found, but with a significant error of {:.4f}m.".format(pos_error))
        return None
    return data.qpos[:6].copy()

if __name__ == "__main__":
    np.set_printoptions(precision=3, suppress=True, linewidth=500)

    # --- 1. 直接加载模型，绕开所有自定义类 ---
    print("正在直接加载MJCF模型...")
    mjcf_path = os.path.join(DISCOVERSE_ROOT_DIR, "models", "mjcf/tasks_xbot_arm/place_block.xml")
    try:
        model = mujoco.MjModel.from_xml_path(mjcf_path)
        data = mujoco.MjData(model)
        print("模型加载成功。")
    except Exception as e:
        print(f"模型加载失败: {e}")
        exit()

    # --- 2. 定义一个在【世界坐标系】下的、极其简单的目标 ---
    print("\n====== 终极IK健全性检查开始 ======")
    
    # 首先，获取机械臂基座在世界中的位置 (从XML中得知是 0.3, 0.92, 0.71)
    base_pos_world = np.array([0.3, 0.92, 0.71])
    
    # 目标：就在基座的正上方 30 厘米处
    target_pos_world = base_pos_world + np.array([0.0, 0.0, 0.3])
    
    # 目标姿态：默认姿态，无旋转
    target_quat_world = np.array([0, 0, 0, 1.0]) # [x, y, z, w]

    print("测试目标 (世界坐标):")
    print("  - 位置 (pos):", target_pos_world)

    # --- 3. 【关键】将机械臂的初始姿态强制设置为全零 ---
    print("\n将机械臂初始关节角强制设置为全零...")
    data.qpos[:7] = 0  # 假设前7个自由度是机械臂+夹爪
    
    # !! 必须调用一次 mj_forward !! 以更新基于qpos=0的模型状态
    mujoco.mj_forward(model, data)
    print("初始姿态设置完毕。")

    # --- 4. 调用IK求解器 ---
    print("\n正在调用IK求解器...")
    ik_solution = solve_ik(model, data, target_pos_world, target_quat_world)

    # --- 5. 检查结果 ---
    if ik_solution is not None:
        print("\n[SUCCESS] IK 求解成功!")
        print("  - 关节角度 (rad):", ik_solution)
        
        # 验证
        data.qpos[:6] = ik_solution
        mujoco.mj_forward(model, data)
        endpoint_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "endpoint")
        final_pos_world = data.site_xpos[endpoint_site_id]
        final_error = np.linalg.norm(final_pos_world - target_pos_world)
        print("  - 最终误差: {:.5f}m".format(final_error))
        
        if final_error < 0.01:
            print("\n[结论] IK功能和模型本身极大概率是好的！问题在于您原始代码中的坐标变换或任务逻辑。")
        else:
            print("\n[结论] IK求解成功但误差依然很大，问题极可能在模型文件(MJCF/XML)的几何定义或关节限位上。")

    else:
        print("\n[FAILURE] IK 求解失败!")
        print("\n[结论] 连最简单的目标都无法求解，问题100%在您的模型文件(MJCF/XML)本身。请仔细检查：")
        print("  1. 所有连杆的pos/quat/euler是否正确。")
        print("  2. 所有关节的range是否正确且合理。")
        print("  3. 是否有不合理的自碰撞设置。")

    print("\n====== 检查结束 ======")