# 调试指南

## 构建

### 构建单个包（快速迭代）

```bash
# AUV 工作空间
cd workspace_auv
colcon build --packages-select uv_control uv_task  --symlink-install
source install/setup.bash

# 仿真工作空间（必须先 source auv_ws）
cd workspace_sim
colcon build --packages-select uv_sim  --symlink-install
source install/setup.bash
```

`--symlink-install` 让 Python 节点的改立即生效，无需重新构建。

### 完整构建

```bash
cd workspace_auv && colcon build && source install/setup.bash
cd workspace_sim && colcon build && source install/setup.bash
```

## ros2 CLI 调试

### 检查节点是否运行

```bash
ros2 node list
ros2 node info /vision
ros2 node info /position
ros2 node info /basic_motion
ros2 node info /task_runner
ros2 node info /sim_bridge
```

### 检查话题

```bash
# 列出所有活跃话题
ros2 topic list

# 查看话题信息（类型、发布者、订阅者）
ros2 topic info /topic_name

# 实时查看话题内容
ros2 topic echo /basic_motion/pose_info
ros2 topic echo /zit6/cmd/setpoint
ros2 topic echo /zit6/state/status
ros2 topic echo /task/status
ros2 topic echo /auv/odometry

# 感知系统
ros2 topic echo /perception/objects
ros2 topic echo /perception/detection/front_left
ros2 topic echo /perception/detection/front_right
ros2 topic echo /perception/detection/down_left
ros2 topic echo /perception/detection/down_right

# 查看话题频率
ros2 topic hz /basic_motion/pose_info
ros2 topic hz /perception/objects
ros2 topic hz /zit6/cmd/setpoint
ros2 topic hz /auv/odometry
```

### 检查服务

```bash
# 列出所有服务
ros2 service list

# 调用 task_runner 服务
ros2 service call /task/stop std_srvs/srv/Trigger
```

### 发送 BasicMotion Action 测试

```bash
# 检查 action server
ros2 action list
ros2 action info /basic_motion

# 发送 START goal（初始化 odom 原点）
ros2 action send_goal /basic_motion uv_msgs/action/BasicMotion \
  "{cmd_type: 6, axes: '', target: [0, 0, 0, 0], timeout: 5.0}"

# SET — 下潜到 z=3.0
ros2 action send_goal /basic_motion uv_msgs/action/BasicMotion \
  "{cmd_type: 3, axes: '', target: [0, 0, 3.0, 0], timeout: 30.0}"

# SET — 转到 yaw=50°
ros2 action send_goal /basic_motion uv_msgs/action/BasicMotion \
  "{cmd_type: 3, axes: '', target: [0, 0, 0, 50.0], timeout: 30.0}"

# BMOVE — 向前 2m
ros2 action send_goal /basic_motion uv_msgs/action/BasicMotion \
  "{cmd_type: 2, axes: '', target: [2.0, 0, 0, 0], timeout: 60.0}"

# SET — 回到原点
ros2 action send_goal /basic_motion uv_msgs/action/BasicMotion \
  "{cmd_type: 3, axes: '', target: [0, 0, 0, 0], timeout: 60.0}"

# WTRAVEL — 向世界系 (3,4) 方向直线前进（先转向，再直走）
ros2 action send_goal /basic_motion uv_msgs/action/BasicMotion \
  "{cmd_type: 4, axes: '', target: [3.0, 4.0, 0.0, 0.0], timeout: 60.0}"

# BTRAVEL — 沿当前机头方向直线前进 3m（body→world 后再转向+直走）
ros2 action send_goal /basic_motion uv_msgs/action/BasicMotion \
  "{cmd_type: 5, axes: '', target: [3.0, 0.0, 0.0, 0.0], timeout: 60.0}"
```

给 action 发送 goal 时加上 `--feedback` 可以看实时反馈：

```bash
ros2 action send_goal --feedback /basic_motion uv_msgs/action/BasicMotion \
  "{cmd_type: 2, axes: '', target: [2.0, 0, 0, 0], timeout: 60.0}"
```

### 检查参数

```bash
# 列出节点参数
ros2 param list /sim_bridge

# 获取参数值
ros2 param get /sim_bridge hil_mode
```

### 日志

所有节点的 `output='screen'`，运行时直接看终端输出。也可以单独看某个节点的日志：

```bash
# 设置日志级别（DEBUG 最详细）
ros2 run uv_control basic_motion --ros-args --log-level debug
```

## 仿真调试

### 启动仿真

```bash
cd workspace_sim
source install/setup.bash
ros2 launch uv_bringup sim_bringup.py
```

按需启用/禁用组件：

```bash
ros2 launch uv_bringup sim_bringup.py enable_ai:=false enable_nav:=true enable_task:=true
```

- `enable_ai:=false` — 关闭视觉
- `enable_nav:=true` — 开启导航
- `enable_task:=true` — 开启任务执行器

### 仿真桥（sim_bridge）状态输出

sim_bridge 用 `output='screen'`，所有 PID 计算、推进器混合、状态更新都会打印到终端。

### PID 调参

sim_bridge 的 8 个 PID 控制器运行时可调，无需改代码重启：

| 参数路径 | 控制器 | 级联层级 |
|---|---|---|
| `chassis.pid.pos.x` | X 位置环 | 外环 |
| `chassis.pid.pos.y` | Y 位置环 | 外环 |
| `chassis.pid.pos.z` | Z 位置环 | 外环 |
| `chassis.pid.pos.yaw` | Yaw 位置环 | 外环 |
| `chassis.pid.vel.x` | X 速度环 | 内环 |
| `chassis.pid.vel.y` | Y 速度环 | 内环 |
| `chassis.pid.vel.z` | Z 速度环 | 内环 |
| `chassis.pid.vel.yaw` | Yaw 速度环 | 内环 |

每个控制器有 4 个增益：kp（比例）、ki（积分）、kd（微分）、i_limit（积分限幅）。

**查看当前值：**

```bash
ros2 service call /zit6/get_params zit6_interfaces/srv/GetParams "{paths: []}"
```

返回全部 8 个 PID 的完整配置 JSON。

**运行时修改 PID 参数：**

```bash
# 改单个 PID 的所有增益（推荐）
ros2 service call /zit6/update_params zit6_interfaces/srv/UpdateParams \
  "{json: '{\"chassis.pid.pos.z\": {\"kp\": 800.0, \"ki\": 50.0, \"kd\": 600.0, \"i_limit\": 5000.0}}'}"

# 只改 kp（传单个数值）
ros2 service call /zit6/update_params zit6_interfaces/srv/UpdateParams \
  "{paths: ['chassis.pid.pos.z'], values: ['800.0']}"
```

**永久改参数**：编辑 `sim_bridge.py`，改 `_init_full()` 中的 `_pid_params` 字典和 `_Pid` 构造参数（两个地方都需要改）。建议运行时调参试出好值后再写死。

### 查看 Stonefish 传感器数据

```bash
# 压力传感器
ros2 topic echo /auv/pressure

# IMU
ros2 topic echo /auv/imu

# DVL
ros2 topic echo /auv/dvl

# 里程计
ros2 topic echo /auv/odometry
```

### 查看 ZIT6 协议层

```bash
# 下位机设定值
ros2 topic echo /zit6/cmd/setpoint

# 下位机反馈状态
ros2 topic echo /zit6/state/status
ros2 topic echo /zit6/state/thr
ros2 topic echo /zit6/state/pos
ros2 topic echo /zit6/state/vel
```

## task_runner 调试

### task_runner 状态

task_runner 以 1Hz 发布 `/task/status`：

```bash
ros2 topic echo /task/status
```

输出示例：

```
current: 0, total: 6, name: "start", status: "running"
current: 1, total: 6, name: "setz", status: "running"
current: 1, total: 6, name: "setz", status: "succeeded"
...
current: 5, total: 6, name: "wait", status: "succeeded"
status: "ALL_DONE"
```

停止正在执行的任务：

```bash
ros2 service call /task/stop std_srvs/srv/Trigger
```

### task_runner 日志级别

task_runner 有详细的 INFO 日志，包括每个任务的执行状态和结果：

- 发送 goal 时打印命令类型、目标值、超时
- goal 被接受/拒绝时打印结果
- 每个任务完成时打印 success + message
- 异常时打印 error traceback

### task_runner 支持的任务列表

| 任务名 | 命令类型 | params | 说明 |
|---|---|---|---|
| `start` | START | `{}` | 初始化 odom 原点 |
| `setx` | SET | `{x}` | 绝对定位 X 轴 |
| `sety` | SET | `{y}` | 绝对定位 Y 轴 |
| `setz` | SET | `{z}` | 绝对定位 Z 轴 |
| `setrz` | SET | `{rz}` | 绝对定位偏航 |
| `setxy` | SET | `{x, y}` | 绝对定位 XY |
| `setxyz` | SET | `{x, y, z}` | 绝对定位 XYZ |
| `setxyzrz` | SET | `{x, y, z, rz}` | 绝对定位 XYZ+偏航 |
| `setxyrz` | SET | `{x, y, rz}` | 绝对定位 XY+偏航 |
| `bmovex` | BMOVE | `{dx}` | 机体系步进 X |
| `bmovey` | BMOVE | `{dy}` | 机体系步进 Y |
| `bmovez` | BMOVE | `{dz}` | 机体系步进 Z |
| `bmoverz` | BMOVE | `{drz}` | 机体系步进偏航 |
| `bmovexy` | BMOVE | `{dx, dy}` | 机体系步进 XY |
| `bmovexyz` | BMOVE | `{dx, dy, dz}` | 机体系步进 XYZ |
| `wmovex` | WMOVE | `{x}` | 世界系步进 X |
| `wmovey` | WMOVE | `{y}` | 世界系步进 Y |
| `wmovez` | WMOVE | `{z}` | 世界系步进 Z |
| `wmoverz` | WMOVE | `{rz}` | 世界系步进偏航 |
| `wmovexy` | WMOVE | `{x, y}` | 世界系步进 XY |
| `wmovexyz` | WMOVE | `{x, y, z}` | 世界系步进 XYZ |
| `wtravelx` | WTRAVEL | `{x}` | 世界系直线 X |
| `wtravely` | WTRAVEL | `{y}` | 世界系直线 Y |
| `wtravelz` | WTRAVEL | `{z}` | 世界系直线 Z |
| `wtravelxy` | WTRAVEL | `{x, y}` | 世界系直线 XY |
| `wtravelxyz` | WTRAVEL | `{x, y, z}` | 世界系直线 XYZ |
| `btravelx` | BTRAVEL | `{dx}` | 机体系直线 X |
| `btravely` | BTRAVEL | `{dy}` | 机体系直线 Y |
| `btravelz` | BTRAVEL | `{dz}` | 机体系直线 Z |
| `btravelxy` | BTRAVEL | `{dx, dy}` | 机体系直线 XY |
| `btravelxyz` | BTRAVEL | `{dx, dy, dz}` | 机体系直线 XYZ |
| `navigate` | SET | `{x, y, z, rz}` | 绝对定位，超时 120s |
| `wait` | — | `{duration}` | 等待 N 秒 |

## 网络/通信调试

### 检查 DDS 发现

```bash
# 列出所有参与的 DDS 节点
ros2 topic list
ros2 node list
```

如果有节点频繁掉线，检查 DDS 配置：

```bash
# 查看 DDS 调试信息（仅限调试）
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

### micro-ROS（HIL/实物）

HIL 模式下 micro-ROS agent 连接 STM32 MCU：

```bash
# HIL 启动
ros2 launch uv_bringup hil_bringup.py serial_dev:=/dev/ttyUSB0 serial_baud:=921600

# 检查 micro-ROS agent 是否收到数据（agent log 有 -v 4 的详细输出）
```

micro-ROS agent 的输出直接打印到终端。如果 MCU 离线，agent 会报错：

```
[micro_ros_agent]: [ERROR] [1234567890.123] - Client lost
```

## rqt 可视化

```bash
# 安装 rqt
sudo apt install ros-jazzy-rqt ros-jazzy-rqt-common-plugins

# 启动 rqt
rqt

# 或直接用 rqt_graph 查看节点/话题拓扑
rqt_graph
```

## 常见问题排查

### "odom origin not set"

**问题**：发送运动命令前没有发 START。
**解决**：先发 START goal：

```bash
ros2 action send_goal /basic_motion uv_msgs/action/BasicMotion \
  "{cmd_type: 6, axes: '', target: [0, 0, 0, 0], timeout: 5.0}"
```

### Action server 不接受连接

**问题**：basic_motion 节点没启动或崩溃。
**解决**：检查节点列表和日志：

```bash
ros2 node list | grep basic_motion
# 如果有输出说明节点在运行
```

### 仿真启动后黑屏

Stonefish 在无 GPU 的环境（SSH、WSL）会挂。确保：

1. 使用 NO GPU 模式（`sim_bringup.py` 已设为 `quality: high`，但无 GPU 时需降级）
2. 或者用 VNC 连接桌面环境

### 机器人收到定深指令后转圈

**已知 bug**：sim_bridge.py 第 616 行 yaw 极性反转。定位方法：

```bash
# 看 setpoint 的值，yaw 分量是否正？
ros2 topic echo /zit6/cmd/setpoint
```

## 快速参考卡片

| 场景 | 命令 |
|---|---|
| 构建一个包 | `colcon build --packages-select uv_control  --symlink-install` |
| 查看节点 | `ros2 node list` |
| 查看话题 | `ros2 topic list` |
| 看话题内容 | `ros2 topic echo /topic` |
| 看话题频率 | `ros2 topic hz /topic` |
| 发 action goal | `ros2 action send_goal /basic_motion uv_msgs/action/BasicMotion "{...}"` |
| 发 action + 反馈 | `ros2 action send_goal --feedback /basic_motion uv_msgs/action/BasicMotion "{...}"` |
| 调服务 | `ros2 service call /task/stop std_srvs/srv/Trigger` |
| 列参数 | `ros2 param list /node_name` |
| 看日志 | 终端输出（所有节点 `output='screen'`） |
| 关视觉 | `ros2 launch uv_bringup sim_bringup.py enable_ai:=false` |
| 开任务 | `ros2 launch uv_bringup sim_bringup.py enable_task:=true` |
| 停止任务 | `ros2 service call /task/stop std_srvs/srv/Trigger` |
