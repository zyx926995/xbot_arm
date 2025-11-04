import os
import sys
import numpy as np
import time
from datetime import datetime

# 添加项目根目录到Python路径
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# 导入xbot机械臂环境
from robots_env.xbot_arm_base import XbotArmBase
from task_base.xbot_arm_task_base import XbotArmTaskBase
from examples.tasks_xbot_arm.xbot_arm_pick_place import XbotArmPickPlace

def test_base_environment():
    """测试基础环境功能"""
    print("=== 测试基础环境功能 ===")
    
    # 创建基础环境实例
    env = XbotArmBase()
    
    # 重置环境
    obs = env.reset()
    print(f"初始观测维度: qpos={obs['qpos'].shape}, qvel={obs['qvel'].shape}")
    
    # 获取TCP位置
    tcp_pos = env.get_tcp_pos()
    print(f"初始TCP位置: {tcp_pos}")
    
    # 执行简单的关节运动
    print("执行简单的关节运动...")
    
    # 目标关节位置（略微调整初始位置）
    target_joint_pos = np.array([0.1, -0.5, 0.5, 0.0, 0.5, 0.0])
    
    # 执行动作
    obs, pri_obs, rew, ter, info = env.step(target_joint_pos)
    
    # 获取新的TCP位置
    new_tcp_pos = env.get_tcp_pos()
    print(f"运动后TCP位置: {new_tcp_pos}")
    print(f"TCP位置变化: {new_tcp_pos - tcp_pos}")
    
    # 测试多次步进
    print("执行多次步进...")
    steps = 5
    for i in range(steps):
        # 随机动作
        action = np.random.uniform(
            env.mj_model.actuator_ctrlrange[:env.nj, 0],
            env.mj_model.actuator_ctrlrange[:env.nj, 1]
        )
        obs, pri_obs, rew, ter, info = env.step(action)
        tcp_pos = env.get_tcp_pos()
        print(f"步骤 {i+1}/{steps}, TCP位置: {tcp_pos}")
        time.sleep(0.1)
    
    print("基础环境测试完成！")
    env.close()

def test_task_base_environment():
    """测试任务基础环境功能"""
    print("\n=== 测试任务基础环境功能 ===")
    
    # 创建任务基础环境实例
    env = XbotArmTaskBase()
    
    # 重置环境
    obs = env.reset()
    print(f"任务基础环境重置完成")
    
    # 测试领域随机化
    print("测试领域随机化...")
    for i in range(3):
        env.reset()
        table_height = env.mj_model.body_pos[env.table_body_id, 2]
        print(f"随机桌子高度 {i+1}: {table_height}")
    
    # 测试键盘交互（仅在非无头模式下）
    if not env.headless:
        print("\n键盘交互说明:")
        print("- WASD: 移动相机")
        print("- 上下左右箭头: 旋转相机")
        print("- ESC: 退出")
        print("按任意键继续...")
    
    print("任务基础环境测试完成！")
    env.close()

def test_pick_place_task():
    """测试抓取放置任务环境功能"""
    print("\n=== 测试抓取放置任务环境功能 ===")
    
    # 创建抓取放置任务实例
    env = XbotArmPickPlace(
        headless=False,
        render_mode='human'
    )
    
    # 重置环境
    obs = env.reset()
    print(f"抓取任务环境重置完成")
    
    # 获取物体和目标位置
    object_pos = env.get_object_pos()
    goal_pos = env.get_goal_pos()
    print(f"物体位置: {object_pos}")
    print(f"目标位置: {goal_pos}")
    
    # 测试几次重置，验证随机化
    print("\n测试物体和目标位置随机化...")
    for i in range(3):
        env.reset()
        obj_pos = env.get_object_pos()
        g_pos = env.get_goal_pos()
        print(f"随机化 {i+1}: 物体={obj_pos}, 目标={g_pos}")
    
    # 测试基本动作执行
    print("\n执行简单动作序列...")
    
    # 定义几个关键姿态的关节位置（简化示例）
    home_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    approach_pose = np.array([0.0, -0.3, 0.3, 0.0, 0.5, 0.0])
    grasp_pose = np.array([0.0, -0.4, 0.2, 0.0, 0.6, 0.0])
    
    # 执行动作序列
    poses = [home_pose, approach_pose, grasp_pose, home_pose]
    
    for i, pose in enumerate(poses):
        print(f"执行姿态 {i+1}/{len(poses)}")
        for _ in range(10):  # 重复执行以平滑过渡
            obs, pri_obs, rew, ter, info = env.step(pose)
            # 渲染环境
            env.render()
            time.sleep(0.05)
        
        # 检查是否成功
        if ter:
            print(f"任务状态: {'成功' if info.get('success', False) else '终止'}")
            break
    
    # 计算并显示奖励
    print(f"\n最终奖励: {rew}")
    
    # 显示键盘交互说明
    print("\n键盘交互说明:")
    print("- WASD: 移动相机")
    print("- 上下左右箭头: 旋转相机")
    print("- R: 重置环境")
    print("- ESC: 退出")
    print("\n按ESC退出测试...")
    
    # 让用户可以交互一段时间
    try:
        for _ in range(300):  # 运行约10秒
            env.render()
            time.sleep(0.05)
            
            # 检查键盘事件
            if env.check_key_press(27):  # ESC键
                print("检测到ESC键，退出测试")
                break
    except KeyboardInterrupt:
        print("用户中断测试")
    
    print("抓取放置任务测试完成！")
    env.close()

def main():
    """主测试函数"""
    print("XBot机械臂环境测试脚本")
    print(f"开始时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    
    try:
        # 运行基础环境测试
        test_base_environment()
        
        # 运行任务基础环境测试
        test_task_base_environment()
        
        # 运行抓取放置任务测试
        test_pick_place_task()
        
    except Exception as e:
        print(f"测试过程中出现错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print(f"\n测试结束时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")

if __name__ == '__main__':
    main()