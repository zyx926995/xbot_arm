import numpy as np
from diffusion_policy.common.pytorch_util import dict_apply
from diffusion_policy.dataset.diffusion_dataset import DiffusionDataset
from diffusion_policy.env.pusht.pusht_state_env import PushTStateEnv
from diffusion_policy.policy.diffusion_policy import DiffusionPolicy
from diffusion_policy.model.diffusion.conditional_unet1d import ConditionalUnet1D

# 配置字典
config = dict()

# 环境配置
config['env'] = dict(
    # 要导入的模块
    module_name='discoverse.examples.tasks_xbot_arm.xbot_arm_pick_place',
    # 要使用的环境类
    class_name='XbotArmPickPlace',
    # 环境参数
    env_kwargs=dict(
        # 可以在这里设置特定于环境的参数
    ),
    # 数据收集参数
    n_train=100,
    n_val=20,
    horizon=100,
    # 采样参数
    sample_kwargs=dict(
        deterministic=True,
    )
)

# 数据集配置
config['dataset'] = dict(
    # 使用的数据集类
    cls=DiffusionDataset,
    # 数据集参数
    kwargs=dict(
        # 数据路径
        data_dir='data/xbot_arm_pick_place',
        # 历史长度
        horizon=100,
        # 动作长度
        n_action_steps=20,
        # 观测维度配置
        obs_key='obs',
        # 观测和动作的归一化
        normalize=True,
        # 数据转换
        transform_obs=None,
        transform_action=None,
    )
)

# 模型配置
config['model'] = dict(
    # 使用的模型类
    cls=ConditionalUnet1D,
    # 模型参数
    kwargs=dict(
        # 动作维度（xbot有6个关节）
        input_dim=6,
        # 条件维度（观测维度）
        global_cond_dim=6 + 6 + 3 + 4 + 3 + 3,  # jq + jv + ep + eq + obj_pos + target_pos
        # 网络配置
        down_dims=[256, 512, 1024],
        kernel_size=5,
        n_groups=8,
        # 注意力层配置
        n_heads=4,
        cross_attention_dim=None,
    )
)

# 策略配置
config['policy'] = dict(
    # 使用的策略类
    cls=DiffusionPolicy,
    # 策略参数
    kwargs=dict(
        # 扩散模型配置
        model=config['model'],
        # 噪声调度器配置
        noise_scheduler=dict(
            num_train_timesteps=100,
            beta_schedule='squaredcos_cap_v2',
            beta_start=0.0001,
            beta_end=0.02,
            clip_sample=True,
        ),
        # 采样配置
        num_inference_steps=25,
        # 动作采样参数
        action_horizon=20,
        obs_horizon=1,
        pred_horizon=20,
    )
)

# 训练配置
config['train'] = dict(
    # 训练轮数
    num_epochs=100,
    # 批次大小
    batch_size=32,
    # 学习率
    learning_rate=1e-4,
    # 权重衰减
    weight_decay=1e-6,
    # 学习率调度
    lr_scheduler=dict(
        cls='ReduceLROnPlateau',
        kwargs=dict(
            mode='min',
            factor=0.5,
            patience=10,
            verbose=True,
        )
    ),
    # 梯度裁剪
    gradient_clip_val=1.0,
    # 设备
    device='cuda' if torch.cuda.is_available() else 'cpu',
    # 随机种子
    seed=42,
    # 保存配置
    save_dir='outputs/xbot_arm_dp',
    # 每N轮保存一次检查点
    save_every_n_epochs=10,
    # 每N轮验证一次
    val_every_n_epochs=5,
)

# 数据收集配置
config['data_collection'] = dict(
    # 每个环境收集的轨迹数
    n_trajectories_per_env=5,
    # 每条轨迹的最大长度
    max_steps=500,
    # 收集的观测键
    obs_keys=['jq', 'jv', 'ep', 'eq', 'obj_pos', 'target_pos'],
    # 收集的动作键
    action_keys=['action'],
)

# 评估配置
config['eval'] = dict(
    # 评估的环境数
    n_envs=10,
    # 每个环境评估的轨迹数
    n_trajectories_per_env=5,
    # 每条轨迹的最大长度
    max_steps=500,
    # 评估指标
    metrics=['success_rate', 'mean_reward', 'mean_episode_length'],
)

# 导入必要的模块
import torch
from diffusion_policy.common.robomimic_util import get_robomimic_config