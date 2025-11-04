import os
import sys
import subprocess
import argparse
from datetime import datetime

def parse_args():
    parser = argparse.ArgumentParser(description='Run XBot Arm Training')
    parser.add_argument('--policy', type=str, default='dp',
                        choices=['dp', 'act'], help='Policy type to use')
    parser.add_argument('--config', type=str, default='configs/xbot_arm_config.py',
                        help='Path to configuration file')
    parser.add_argument('--output', type=str, default='outputs/xbot_arm_dp',
                        help='Output directory for checkpoints and logs')
    parser.add_argument('--resume', type=str, default=None,
                        help='Path to checkpoint to resume training')
    parser.add_argument('--device', type=str, default='cuda' if torch.cuda.is_available() else 'cpu',
                        help='Device to use for training')
    parser.add_argument('--headless', action='store_true',
                        help='Run in headless mode')
    return parser.parse_args()

def ensure_directories(output_dir):
    """确保必要的目录存在"""
    # 创建输出目录
    os.makedirs(output_dir, exist_ok=True)
    
    # 创建数据目录
    data_dir = os.path.join(output_dir, 'data')
    os.makedirs(data_dir, exist_ok=True)
    
    return data_dir

def generate_run_id():
    """生成运行ID"""
    return datetime.now().strftime('%Y%m%d_%H%M%S')

def run_training(args):
    """运行训练过程"""
    # 确保目录存在
    data_dir = ensure_directories(args.output)
    
    # 生成运行ID
    run_id = generate_run_id()
    log_file = os.path.join(args.output, f'training_log_{run_id}.txt')
    
    print(f"开始训练 XBot Arm，策略类型: {args.policy}")
    print(f"配置文件: {args.config}")
    print(f"输出目录: {args.output}")
    print(f"运行ID: {run_id}")
    print(f"日志文件: {log_file}")
    
    # 构建训练命令
    if args.policy == 'dp':
        # 使用Diffusion Policy
        train_script = os.path.join('policies', 'dp', 'train_xbot_arm.py')
    else:
        # 使用ACT Policy
        train_script = os.path.join('policies', 'act', 'train.py')
    
    # 构建命令参数
    cmd_args = [
        sys.executable,
        train_script,
        '--config', args.config,
        '--output', args.output,
    ]
    
    # 添加可选参数
    if args.resume:
        cmd_args.extend(['--resume', args.resume])
    
    if args.device:
        cmd_args.extend(['--device', args.device])
    
    if args.headless:
        # 如果是无头模式，需要设置环境变量
        os.environ['MUJOCO_HEADLESS'] = '1'
    
    # 执行训练命令
    try:
        print(f"\n执行命令: {' '.join(cmd_args)}")
        print(f"训练输出将保存到 {log_file}")
        print("按 Ctrl+C 停止训练...\n")
        
        # 重定向输出到日志文件
        with open(log_file, 'w') as log:
            process = subprocess.Popen(
                cmd_args,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1
            )
            
            # 实时打印输出并写入日志
            for line in process.stdout:
                print(line, end='', flush=True)
                log.write(line)
            
            # 等待进程完成
            process.wait()
            
            if process.returncode == 0:
                print("\n训练成功完成！")
            else:
                print(f"\n训练异常终止，返回码: {process.returncode}")
                print(f"详细信息请查看日志文件: {log_file}")
                
    except KeyboardInterrupt:
        print("\n用户中断训练")
        if 'process' in locals():
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
    except Exception as e:
        print(f"\n训练过程中发生错误: {e}")
        import traceback
        traceback.print_exc()

def main():
    """主函数"""
    # 添加项目根目录到Python路径
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    sys.path.insert(0, project_root)
    
    # 尝试导入torch
    try:
        import torch
    except ImportError:
        print("错误: 未找到PyTorch，请先安装依赖")
        sys.exit(1)
    
    # 解析参数
    args = parse_args()
    
    # 运行训练
    run_training(args)

if __name__ == '__main__':
    main()