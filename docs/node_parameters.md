# 节点参数与启动参数文档

## 所有可执行节点

| 可执行文件 | 包 | 工作区 | 说明 |
|---|---|---|---|
| `stonefish_simulator` | `stonefish_ros2` | workspace_sim | GPU 渲染仿真器 |
| `stonefish_simulator_nogpu` | `stonefish_ros2` | workspace_sim | 无头模式仿真器 |
| `sim_bridge` | `uv_sim` | workspace_sim | 仿真硬件桥接 |
| `hw_manager` | `uv_hm` | workspace_auv | 实车硬件管理 |
| `basic_motion` | `uv_control` | workspace_auv | 运动控制 |
| `vision` | `uv_perception` | workspace_auv | YOLO 目标检测 |
| `position` | `uv_perception` | workspace_auv | 3D 目标定位 |
| `navigator` | `uv_nav` | workspace_auv | A* 路径规划 + 避障 |
| `task_runner` | `uv_task` | workspace_auv | JSON 任务执行器 |

---

## vision 节点参数

### 运行模式

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `sim_mode` | bool | `true` | `true`: 订阅 `/auv/*/stitched` ROS 话题<br>`false`: 使用 V4L2 设备直接采集 |
| `front_cam_path` | str | `/dev/video0` | 前视摄像头 V4L2 设备路径（仅 real 模式） |
| `down_cam_path` | str | `/dev/video2` | 下视摄像头 V4L2 设备路径（仅 real 模式） |

### 图像发布

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `publish_images` | bool | `false` | 发布拆分后的原始图像到 `/perception/image/*` |
| `publish_annotated` | bool | `false` | 发布带 YOLO 检测框的标注图像到 `/perception/annotated/*` |

### 模型与数据集

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `model_path` | str | `""` | YOLO 模型 .pt 文件路径。为空时自动查找默认路径 |
| `save_dataset` | bool | `false` | 是否保存采集的图像到 `img/` 目录 |

### 相机标定

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `front_camera_matrix` | float[9] | 单位矩阵 | 前视相机内参 3x3 |
| `front_dist_coeffs` | float[5] | 全零 | 前视相机畸变系数 (k1,k2,p1,p2,k3) |
| `down_camera_matrix` | float[9] | 单位矩阵 | 下视相机内参 3x3 |
| `down_dist_coeffs` | float[5] | 全零 | 下视相机畸变系数 (k1,k2,p1,p2,k3) |

## position 节点参数

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `max_history` | int | `30` | 每条射线历史最大保留数量 |

---

## sim_bridge 节点参数

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `hil_mode` | bool | `false` | `true`: HIL 模式（相机直通 + 推力混合）<br>`false`: SIL 全仿真（PID + ZIT6 状态机） |

---

## 无参数节点

`basic_motion`、`navigator`、`task_runner`、`hw_manager` 不接受 ROS2 参数。

---

## 启动文件 Launch Arguments

### sim_bringup.py

| 参数 | 默认值 | 说明 |
|---|---|---|
| `enable_ai` | `true` | 启用 vision + position |
| `enable_nav` | `false` | 启用 navigator |
| `enable_task` | `false` | 启用 task_runner |
| `scenario_desc` | `wuurc_murc_2026_auv.scn` | Stonefish 场景文件 |
| `publish_annotated` | `false` | 发布 YOLO 带框标注图像 |

**用法：**
```bash
ros2 launch uv_bringup sim_bringup.py enable_ai:=true publish_annotated:=true
```

### real_bringup.py

| 参数 | 默认值 | 说明 |
|---|---|---|
| `enable_ai` | `true` | 启用 vision + position |
| `enable_nav` | `true` | 启用 navigator |
| `enable_task` | `false` | 启用 task_runner |
| `publish_annotated` | `false` | 发布 YOLO 带框标注图像 |

**用法：**
```bash
ros2 launch uv_bringup real_bringup.py publish_annotated:=true
```

### hil_bringup.py

| 参数 | 默认值 | 说明 |
|---|---|---|
| `enable_ai` | `false` | 启用 vision + position |
| `enable_nav` | `false` | 启用 navigator |
| `enable_task` | `false` | 启用 task_runner |
| `enable_motion` | `false` | 启用 basic_motion |
| `scenario` | `underwater_xunyun.scn` | Stonefish 场景文件 |
| `serial_dev` | `/dev/ttyUSB0` | MCU 串口设备 |
| `serial_baud` | `921600` | 串口波特率 |
| `publish_annotated` | `false` | 发布 YOLO 带框标注图像 |

**用法：**
```bash
ros2 launch uv_bringup hil_bringup.py enable_ai:=true publish_annotated:=true
```

### perception_launch.py

| 参数 | 说明 |
|---|---|
| `publish_annotated` | 发布 YOLO 带框标注图像（默认 `false`） |

**用法：**
```bash
ros2 launch uv_perception perception_launch.py publish_annotated:=true
```

## 话题参考

### vision 发布话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/perception/detection/front_left` | `DetectionArray` | 前视左检测结果 |
| `/perception/detection/front_right` | `DetectionArray` | 前视右检测结果 |
| `/perception/detection/down_left` | `DetectionArray` | 下视左检测结果 |
| `/perception/detection/down_right` | `DetectionArray` | 下视右检测结果 |
| `/perception/image/front_left` | `Image` | 前视左原始图像（需 enable） |
| `/perception/image/front_right` | `Image` | 前视右原始图像（需 enable） |
| `/perception/image/down_left` | `Image` | 下视左原始图像（需 enable） |
| `/perception/image/down_right` | `Image` | 下视右原始图像（需 enable） |
| `/perception/annotated/front_left` | `Image` | 前视左带框标注（需 enable） |
| `/perception/annotated/front_right` | `Image` | 前视右带框标注（需 enable） |
| `/perception/annotated/down_left` | `Image` | 下视左带框标注（需 enable） |
| `/perception/annotated/down_right` | `Image` | 下视右带框标注（需 enable） |

### position 发布话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/perception/objects` | `ObjectPositionArray` | 3D 物体世界坐标 |
