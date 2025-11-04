import importlib
import numpy as np
import cv2


class Env():
    def __init__(self, args):
        module = importlib.import_module(args.task_path.replace("/", ".").replace(".py", ""))
        SimNode = getattr(module, "SimNode") # SimNode
        cfg = getattr(module, "cfg") # cfg
        self.cfg = cfg
        cfg.headless = True
        self.simnode = SimNode(cfg)
        self.args = args
        self.obs_steps = args.obs_steps
        self.obs_que = None
        self.video_list = list()
        # 适配xbot机械臂：添加图像尺寸参数
        self.image_size = getattr(args, 'image_size', 224)

    def reset(self):
        obs, t = self.simnode.reset(), 0
        self.video_list = list()
        from collections import  deque
        self.obs_que = deque([obs], maxlen=self.obs_steps+1) 
        return self.obs_que_ext(), t
    
    def obs_que_ext(self):
        result = dict()
        # 处理所有非图像类型的观测
        for key in self.args.obs_keys:
            if not key.startswith('image'):
                # 适配xbot机械臂：确保观测数据存在
                valid_obs = [obs for obs in self.obs_que if key in obs]
                if valid_obs:
                    result[key] = self.stack_last_n_obs(
                        [np.array(obs[key]) for obs in valid_obs]
                    )
        # 处理图像类型的观测
        image_ids = [int(key[5:]) for key in self.args.obs_keys if key.startswith('image')]
        imgs = {id: [] for id in image_ids}
        for obs in self.obs_que:
            # 适配xbot机械臂：检查img是否存在且非None
            if 'img' in obs and obs['img'] is not None:
                img = obs['img']
                for id in image_ids:
                    if id in img and img[id] is not None:
                        # 确保图像是RGB格式
                        img_data = img[id]
                        if len(img_data.shape) == 2:
                            img_data = cv2.cvtColor(img_data, cv2.COLOR_GRAY2RGB)
                        # 调整大小
                        img_data = cv2.resize(img_data, (self.image_size, self.image_size))
                        # 转置并归一化
                        img_trans = np.transpose(img_data / 255, (2, 0, 1))
                        imgs[id].append(img_trans)

        for id in image_ids:
            if imgs[id]:  # 确保有数据再堆叠
                result[f'image{id}'] = self.stack_last_n_obs(imgs[id])
        
        # 适配xbot机械臂：添加tcp位置和姿态信息
        if hasattr(self.simnode, 'env'):
            env = self.simnode.env
            if hasattr(env, 'get_tcp_pos'):
                result['tcp_pos'] = env.get_tcp_pos()
            if hasattr(env, 'get_tcp_quat'):
                result['tcp_quat'] = env.get_tcp_quat()
            # 对于抓取任务，添加物体和目标位置
            if hasattr(env, 'get_object_pos'):
                result['object_pos'] = env.get_object_pos()
            if hasattr(env, 'get_goal_pos'):
                result['goal_pos'] = env.get_goal_pos()
                
        return result
    
    def step(self, action):
        success = 0
        for act in action: #依次执行每个动作
            # 适配xbot机械臂：限制动作范围
            if hasattr(self.simnode, 'env') and hasattr(self.simnode.env, 'mj_model'):
                env = self.simnode.env
                # 获取关节控制范围
                ctrl_range = env.mj_model.actuator_ctrlrange
                # 限制动作在有效范围内
                act_clamped = np.clip(act, ctrl_range[:, 0], ctrl_range[:, 1])
            else:
                act_clamped = act
                
            for _ in range(int(round(1. / self.simnode.render_fps / (self.simnode.delta_t)))):
                obs, _, _, _, _ = self.simnode.step(act_clamped)
            self.obs_que.append(obs) #添加单个obs
            # 适配xbot机械臂：检查img是否存在
            if 'img' in obs and obs['img'] is not None:
                self.video_list.append(obs['img'])
            
            if self.simnode.check_success():
                success = 1
                break
        return self.obs_que_ext(), success 
       
    def stack_last_n_obs(self, all_obs):
        # 适配xbot机械臂：处理空观测列表情况
        if not all_obs:
            return None
            
        assert(len(all_obs) > 0)
        result = np.zeros((self.obs_steps,) + all_obs[-1].shape, 
            dtype=all_obs[-1].dtype)
        start_idx = -min(self.obs_steps, len(all_obs))
        result[start_idx:] = np.array(all_obs[start_idx:])
        if self.obs_steps > len(all_obs):
            # pad
            result[:start_idx] = result[start_idx]
        return result