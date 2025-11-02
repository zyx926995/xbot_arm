import sys
import rospy
import moveit_commander
import geometry_msgs.msg

# 初始化 moveit_commander 和 rospy 节点
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

# 实例化 RobotCommander 对象.
robot = moveit_commander.RobotCommander()

# 实例化 PlanningSceneInterface 对象.
scene = moveit_commander.PlanningSceneInterface()

# 实例化 MoveGroupCommander 对象. 这是最关键的对象！
# 注意！教程里用的是 "panda_arm". 您需要把它改成您机械臂的规划组名，也就是 "xarm".
group_name = "xarm"  # <--- 这是您需要做的第一个关键修改！
move_group = moveit_commander.MoveGroupCommander(group_name)

# 创建一个 Pose 消息实例
pose_goal = geometry_msgs.msg.Pose()

# --- 这里将是您VLA算法的输出对接处 ---
# 您的VLA算法需要输出目标位置 (x, y, z) 和姿态 (四元数 qx, qy, qz, qw)
# 然后把这些值赋给 pose_goal 对象
pose_goal.position.x = 0.4  # 替换成您的目标x
pose_goal.position.y = 0.1  # 替换成您的目标y
pose_goal.position.z = 0.4  # 替换成您的目标z
pose_goal.orientation.w = 1.0 # 替换成您的目标四元数w
# (如果VLA也输出了qx, qy, qz, 也要相应地设置 pose_goal.orientation.x/y/z)
# ------------------------------------

# 设置目标位姿
move_group.set_pose_target(pose_goal)

# 规划并执行. 这是您需要的“魔法指令”
plan = move_group.go(wait=True)

# 确保没有残余运动
move_group.stop()
# 清除目标，这是个好习惯
move_group.clear_pose_targets()