import os
import sys
import numpy as np
import torch
import argparse
import importlib.util
import matplotlib.pyplot as plt
from tqdm import tqdm

# 添加项目根目录到Python路径
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from diffusion_policy.policy.diffusion_policy import DiffusionPolicy
from diffusion_policy.dataset.diffusion_dataset import DiffusionDataset
from diffusion_policy.common.trainer import Trainer
from diffusion_policy.common.logger import Logger

# 解析命令行参数
def parse_args():
    parser = argparse.ArgumentParser(description='Train Diffusion Policy for XBot Arm')
    parser.add_argument('--config', type=str, default='configs/xbot_arm_config.py',
                        help='Path to configuration file')
    parser.add_argument('--output', type=str, default='outputs/xbot_arm_dp',
                        help='Output directory for checkpoints and logs')
    parser.add_argument('--resume', type=str, default=None,
                        help='Path to checkpoint to resume training')
    parser.add_argument('--device', type=str, default='cuda' if torch.cuda.is_available() else 'cpu',
                        help='Device to use for training')
    return parser.parse_args()

# 加载配置文件
def load_config(config_path):
    spec = importlib.util.spec_from_file_location('config', config_path)
    config_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(config_module)
    return config_module.config

# 初始化环境
def init_environment(config):
    module_name = config['env']['module_name']
    class_name = config['env']['class_name']
    
    # 动态导入环境模块
    module = importlib.import_module(module_name)
    env_class = getattr(module, class_name)
    
    # 创建环境实例
    env_kwargs = config['env'].get('env_kwargs', {})
    env = env_class(**env_kwargs)
    
    return env

# 收集训练数据
def collect_data(env, config, output_dir):
    print("Collecting training data...")
    n_trajectories = config['env']['n_train'] + config['env']['n_val']
    max_steps = config['data_collection']['max_steps']
    obs_keys = config['data_collection']['obs_keys']
    
    trajectories = []
    
    for i in tqdm(range(n_trajectories)):
        trajectory = {
            'observations': [],
            'actions': []
        }
        
        # 重置环境
        obs = env.reset()
        
        # 收集轨迹
        for step in range(max_steps):
            # 这里可以使用不同的策略生成动作
            # 暂时使用简单的关节随机运动
            action = np.random.uniform(
                env.mj_model.actuator_ctrlrange[:env.nj, 0],
                env.mj_model.actuator_ctrlrange[:env.nj, 1]
            )
            
            # 执行动作
            next_obs, pri_obs, rew, ter, info = env.step(action)
            
            # 收集观测和动作
            full_obs = env.get_full_observation()
            obs_dict = {key: full_obs[key] for key in obs_keys}
            
            trajectory['observations'].append(obs_dict)
            trajectory['actions'].append(action.tolist())
            
            # 更新观测
            obs = next_obs
            
            # 如果终止则停止当前轨迹
            if ter:
                break
        
        # 添加到轨迹列表
        trajectories.append(trajectory)
    
    # 保存数据
    os.makedirs(output_dir, exist_ok=True)
    
    # 分割训练和验证数据
    train_trajectories = trajectories[:config['env']['n_train']]
    val_trajectories = trajectories[config['env']['n_train']:]
    
    # 保存训练数据
    np.save(os.path.join(output_dir, 'train_trajectories.npy'), train_trajectories)
    # 保存验证数据
    np.save(os.path.join(output_dir, 'val_trajectories.npy'), val_trajectories)
    
    print(f"Collected {len(train_trajectories)} training trajectories and {len(val_trajectories)} validation trajectories")
    print(f"Data saved to {output_dir}")

# 初始化数据集
def init_dataset(config):
    data_dir = config['dataset']['kwargs']['data_dir']
    
    # 创建数据集实例
    dataset_cls = config['dataset']['cls']
    dataset_kwargs = config['dataset']['kwargs']
    
    # 初始化训练数据集
    train_dataset = dataset_cls(
        data_path=os.path.join(data_dir, 'train_trajectories.npy'),
        **dataset_kwargs
    )
    
    # 初始化验证数据集
    val_dataset = dataset_cls(
        data_path=os.path.join(data_dir, 'val_trajectories.npy'),
        **dataset_kwargs
    )
    
    return train_dataset, val_dataset

# 初始化数据加载器
def init_data_loaders(train_dataset, val_dataset, config):
    batch_size = config['train']['batch_size']
    
    # 创建数据加载器
    train_loader = torch.utils.data.DataLoader(
        train_dataset,
        batch_size=batch_size,
        shuffle=True,
        num_workers=4,
        pin_memory=True
    )
    
    val_loader = torch.utils.data.DataLoader(
        val_dataset,
        batch_size=batch_size,
        shuffle=False,
        num_workers=4,
        pin_memory=True
    )
    
    return train_loader, val_loader

# 初始化模型
def init_model(config, device):
    # 创建策略实例
    policy_cls = config['policy']['cls']
    policy_kwargs = config['policy']['kwargs']
    
    policy = policy_cls(**policy_kwargs)
    policy.to(device)
    
    return policy

# 初始化优化器
def init_optimizer(model, config):
    learning_rate = config['train']['learning_rate']
    weight_decay = config['train']['weight_decay']
    
    optimizer = torch.optim.AdamW(
        model.parameters(),
        lr=learning_rate,
        weight_decay=weight_decay
    )
    
    return optimizer

# 初始化学习率调度器
def init_lr_scheduler(optimizer, config):
    scheduler_config = config['train']['lr_scheduler']
    scheduler_cls = getattr(torch.optim.lr_scheduler, scheduler_config['cls'])
    scheduler = scheduler_cls(optimizer, **scheduler_config['kwargs'])
    return scheduler

# 训练函数
def train(model, train_loader, val_loader, optimizer, scheduler, config, device, output_dir, resume=None):
    # 初始化训练器
    trainer = Trainer(
        model=model,
        optimizer=optimizer,
        scheduler=scheduler,
        device=device,
        output_dir=output_dir
    )
    
    # 初始化日志记录器
    logger = Logger(output_dir)
    
    # 加载检查点（如果有）
    start_epoch = 0
    if resume is not None:
        checkpoint = torch.load(resume)
        model.load_state_dict(checkpoint['model_state_dict'])
        optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
        scheduler.load_state_dict(checkpoint['scheduler_state_dict'])
        start_epoch = checkpoint['epoch'] + 1
        print(f"Resumed training from epoch {start_epoch}")
    
    # 训练循环
    num_epochs = config['train']['num_epochs']
    save_every = config['train']['save_every_n_epochs']
    val_every = config['train']['val_every_n_epochs']
    
    best_val_loss = float('inf')
    
    for epoch in range(start_epoch, num_epochs):
        print(f"Epoch {epoch+1}/{num_epochs}")
        
        # 训练轮次
        train_loss = trainer.train_epoch(train_loader)
        
        # 记录训练损失
        logger.log_scalar('train_loss', train_loss, epoch)
        
        print(f"  Train Loss: {train_loss:.6f}")
        
        # 验证轮次
        if (epoch + 1) % val_every == 0:
            val_loss = trainer.val_epoch(val_loader)
            
            # 记录验证损失
            logger.log_scalar('val_loss', val_loss, epoch)
            
            print(f"  Val Loss: {val_loss:.6f}")
            
            # 更新学习率调度器
            if scheduler is not None:
                scheduler.step(val_loss)
            
            # 保存最佳模型
            if val_loss < best_val_loss:
                best_val_loss = val_loss
                trainer.save_checkpoint(epoch, is_best=True)
        
        # 定期保存检查点
        if (epoch + 1) % save_every == 0:
            trainer.save_checkpoint(epoch)
    
    # 保存最终模型
    trainer.save_checkpoint(num_epochs - 1)
    
    print("Training completed!")
    print(f"Best validation loss: {best_val_loss:.6f}")

# 主函数
def main():
    # 解析参数
    args = parse_args()
    
    # 加载配置
    config = load_config(args.config)
    
    # 设置输出目录
    output_dir = args.output
    os.makedirs(output_dir, exist_ok=True)
    
    # 设置设备
    device = torch.device(args.device)
    
    # 初始化环境
    env = init_environment(config)
    
    # 收集训练数据
    data_dir = config['dataset']['kwargs']['data_dir']
    os.makedirs(data_dir, exist_ok=True)
    collect_data(env, config, data_dir)
    
    # 初始化数据集
    train_dataset, val_dataset = init_dataset(config)
    
    # 初始化数据加载器
    train_loader, val_loader = init_data_loaders(train_dataset, val_dataset, config)
    
    # 初始化模型
    model = init_model(config, device)
    
    # 初始化优化器
    optimizer = init_optimizer(model, config)
    
    # 初始化学习率调度器
    scheduler = init_lr_scheduler(optimizer, config)
    
    # 开始训练
    train(model, train_loader, val_loader, optimizer, scheduler, config, device, output_dir, args.resume)

if __name__ == '__main__':
    main()