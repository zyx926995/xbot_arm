import os
import sys
import numpy as np
from scipy.spatial.transform import Rotation
from task_base.xbot_arm_task_base import XbotArmTaskBase, XbotArmCfg

class XbotArmPickPlaceCfg(XbotArmCfg):
    def __init__(self):
        super().__init__()
        self.mjcf_file_path = "mjcf/xbot_arm_pick_place.xml"  # 包含抓取对象的MJCF文件
        self.headless = False
        self.use_gaussian_renderer = False

class XbotArmPickPlace(XbotArmTaskBase):
    def __init__(self, config=None):
        if config is None:
            config = XbotArmPickPlaceCfg()
        super().__init__(config)
        self.state = "reset"
        self.reset_times = 0
        self.object_init_pos = None
        self.object_target_pos = None
        self.success_threshold = 0.05  # 成功距离阈值
        self.max_steps = 500
        self.current_step = 0

    def resetState(self):
        super().resetState()
        self.state = "reset"
        self.current_step = 0
        self.reset_times += 1
        
        # 随机生成物体的初始位置和目标位置
        self.object_init_pos = np.array([0.3, 0.0, 0.02])  # 物体初始位置
        self.object_init_pos[:2] += np.random.uniform(-0.05, 0.05, 2)  # 轻微随机扰动
        
        self.object_target_pos = np.array([0.3, 0.2, 0.02])  # 目标位置
        self.object_target_pos[:2] += np.random.uniform(-0.05, 0.05, 2)  # 轻微随机扰动
        
        # 设置物体位置
        try:
            self.object_pose("object")[:3] = self.object_init_pos.copy()
        except KeyError:
            print("Warning: Object 'object' not found in model")
        
        mujoco.mj_forward(self.mj_model, self.mj_data)

    def domain_randomization(self):
        # 随机化光照
        self.random_light()
        # 随机化物体颜色
        try:
            self.mj_model.geom("object_geom").rgba[:3] = np.random.rand(3)
        except KeyError:
            pass

    def get_action_space(self):
        # 返回动作空间的维度和范围
        return {
            "shape": (self.nj,),
            "low": self.mj_model.actuator_ctrlrange[:self.nj, 0].tolist(),
            "high": self.mj_model.actuator_ctrlrange[:self.nj, 1].tolist()
        }

    def get_observation_space(self):
        # 返回观测空间
        obs_dim = {
            "jq": self.nj,
            "jv": self.nj,
            "ep": 3,  # 末端位置
            "eq": 4,  # 末端姿态（四元数）
            "obj_pos": 3,  # 物体位置
            "target_pos": 3  # 目标位置
        }
        return obs_dim

    def get_full_observation(self):
        # 获取完整的观测，包括物体位置和目标位置
        obs = self.getObservation()
        try:
            obj_pos = self.object_pose("object")[:3]
        except KeyError:
            obj_pos = np.zeros(3)
        
        full_obs = {
            **obs,
            "obj_pos": obj_pos.tolist(),
            "target_pos": self.object_target_pos.tolist()
        }
        return full_obs

    def check_success(self):
        # 检查是否成功将物体移动到目标位置
        try:
            obj_pos = self.object_pose("object")[:3]
            distance = np.linalg.norm(obj_pos - self.object_target_pos)
            return distance < self.success_threshold
        except KeyError:
            return False

    def getReward(self):
        # 计算奖励
        try:
            obj_pos = self.object_pose("object")[:3]
            # 与目标位置的距离
            distance_to_target = np.linalg.norm(obj_pos - self.object_target_pos)
            # 与物体的距离
            distance_to_object = np.linalg.norm(self.sensor_endpoint_posi_local - obj_pos)
            
            # 基础奖励：距离目标越近奖励越高
            reward = -distance_to_target
            
            # 额外奖励：如果成功
            if self.check_success():
                reward += 10.0
            
            # 额外奖励：靠近物体
            if distance_to_object < 0.05:
                reward += 0.1
            
            return reward
        except KeyError:
            return 0.0

    def checkTerminated(self):
        # 检查是否终止
        self.current_step += 1
        # 如果成功或达到最大步数则终止
        return self.check_success() or self.current_step >= self.max_steps

    def printMessage(self):
        super().printMessage()
        print(f"    State: {self.state}")
        print(f"    Current step: {self.current_step}/{self.max_steps}")
        print(f"    Success: {self.check_success()}")

if __name__ == "__main__":
    # 简单的测试脚本
    cfg = XbotArmPickPlaceCfg()
    env = XbotArmPickPlace(cfg)
    
    # 重置环境
    obs = env.reset()
    
    # 简单的控制循环
    while env.running:
        # 获取当前观测
        full_obs = env.get_full_observation()
        
        # 这里可以添加策略逻辑
        # 暂时使用随机动作
        action = np.random.uniform(
            env.mj_model.actuator_ctrlrange[:env.nj, 0],
            env.mj_model.actuator_ctrlrange[:env.nj, 1]
        )
        
        # 执行动作
        obs, pri_obs, rew, ter, info = env.step(action)
        
        # 打印信息
        env.printMessage()
        print(f"    Reward: {rew}")
        print(f"    Terminated: {ter}")
        
        # 如果终止则重置
        if ter:
            obs = env.reset()