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
    
    # 创建配置对象
    config = XbotArmCfg()
    config.headless = False
    
    # 创建基础环境实例
    env = XbotArmBase(config)
    
    # 重置环境
    obs = env.reset()
    print(f"初始观测数据: jq={obs['jq']}")
    
    # 获取TCP位置（从观测中获取）
    tcp_pos = np.array(obs['ep'])
    print(f"初始TCP位置: {tcp_pos}")
    
    # 执行简单的关节运动
    print("执行简单的关节运动...")
    
    # 目标关节位置（略微调整初始位置）
    target_joint_pos = np.array([0.1, -0.5, 0.5, 0.0, 0.5, 0.0])
    
    # 执行动作
    obs, pri_obs, rew, ter, info = env.step(target_joint_pos)
    
    # 获取新的TCP位置
    new_tcp_pos = np.array(obs['ep'])
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
        tcp_pos = np.array(obs['ep'])
        print(f"步骤 {i+1}/{steps}, TCP位置: {tcp_pos}")
        time.sleep(0.1)
    
    print("基础环境测试完成！")

def test_task_base_environment():
    """测试任务基础环境功能"""
    print("\n=== 测试任务基础环境功能 ===")
    
    # 创建配置对象
    config = XbotArmCfg()
    config.headless = False
    
    try:
        # 创建任务基础环境实例
        env = XbotArmTaskBase(config)
        
        # 重置环境
        print("正在重置环境...")
        obs = env.reset()
        print(f"任务基础环境重置完成")
        
        # 测试领域随机化
        print("测试领域随机化...")
        for i in range(3):
            env.reset()
            if hasattr(env, 'table_body_id') and env.table_body_id is not None and hasattr(env, 'mj_model'):
                try:
                    table_height = env.mj_model.body_pos[env.table_body_id, 2]
                    print(f"随机桌子高度 {i+1}: {table_height}")
                except IndexError:
                    print(f"警告: 无法获取桌子高度 - 索引错误")
        
        # 测试简单动作执行
        print("执行简单动作...")
        simple_action = np.array([0.1, -0.3, 0.3, 0.0, 0.5, 0.0])
        obs, pri_obs, rew, ter, info = env.step(simple_action)
        print(f"动作执行完成，TCP位置: {np.array(obs['ep'])}")
        
        # 测试键盘交互（仅在非无头模式下）
        if not env.config.headless and hasattr(env, 'render'):
            print("\n键盘交互说明:")
            print("- WASD: 移动相机")
            print("- 上下左右箭头: 旋转相机")
            print("- ESC: 退出")
            print("按任意键继续...")
            # 简短渲染几帧
            for _ in range(5):
                env.render()
                time.sleep(0.1)
    
    except Exception as e:
        print(f"任务基础环境测试过程中出现错误: {e}")
        import traceback
        traceback.print_exc()
    
    print("任务基础环境测试完成！")

def test_pick_place_task():
    """测试抓取放置任务环境功能"""
    print("\n=== 测试抓取放置任务环境功能 ===")
    
    # 创建配置对象
    config = XbotArmCfg()
    config.headless = False
    
    try:
        # 创建抓取放置任务实例
        print("创建抓取放置任务实例...")
        env = XbotArmPickPlace(config)
        
        # 重置环境
        print("重置环境...")
        obs = env.reset()
        print(f"抓取任务环境重置完成")
        
        # 获取物体和目标位置（如果方法存在）
        try:
            if hasattr(env, 'get_object_pos'):
                object_pos = env.get_object_pos()
                print(f"物体位置: {object_pos}")
            if hasattr(env, 'get_goal_pos'):
                goal_pos = env.get_goal_pos()
                print(f"目标位置: {goal_pos}")
        except Exception as e:
            print(f"获取位置信息时出错: {e}")
        
        # 测试几次重置，验证随机化
        print("\n测试物体和目标位置随机化...")
        for i in range(3):
            try:
                env.reset()
                if hasattr(env, 'get_object_pos') and hasattr(env, 'get_goal_pos'):
                    obj_pos = env.get_object_pos()
                    g_pos = env.get_goal_pos()
                    print(f"随机化 {i+1}: 物体={obj_pos}, 目标={g_pos}")
            except Exception as e:
                print(f"随机化测试中出错: {e}")
        
        # 测试基本动作执行（简化版本，减少潜在错误）
        print("\n执行简单动作...")
        simple_pose = np.array([0.1, -0.3, 0.3, 0.0, 0.5, 0.0])
        
        # 执行少量步骤，避免长时间运行
        try:
            for _ in range(5):
                obs, pri_obs, rew, ter, info = env.step(simple_pose)
                # 安全渲染
                if hasattr(env, 'render') and not config.headless:
                    try:
                        env.render()
                    except Exception as render_error:
                        print(f"渲染错误: {render_error}")
                time.sleep(0.05)
        except Exception as e:
            print(f"动作执行过程中出错: {e}")
        
        # 简单检查奖励
        if rew is not None:
            print(f"\n当前奖励: {rew}")
        
        # 简要的键盘交互说明
        print("\n键盘交互说明:")
        print("- WASD: 移动相机")
        print("- 上下左右箭头: 旋转相机")
        print("- R: 重置环境")
        print("- ESC: 退出")
        
    except Exception as e:
        print(f"抓取放置任务测试过程中出现错误: {e}")
        import traceback
        traceback.print_exc()
    
    print("抓取放置任务测试完成！")

def main():
    """主测试函数"""
    print("XBot机械臂环境测试脚本")
    print(f"开始时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    
    try:
        # 运行基础环境测试
        test_base_environment()
        
        # 运行任务基础环境测试
        test_task_base_environment()
        
        # 运行抓取放置任务测试（可选，先确保基础功能正常）
        print("\n注意: 基础功能测试完成后，可以选择性地运行抓取放置任务测试")
        # 暂时注释掉，先确保基础和任务基础环境正常工作
        # test_pick_place_task()
        
    except Exception as e:
        print(f"测试过程中出现错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print(f"\n测试结束时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")

if __name__ == '__main__':
    main()