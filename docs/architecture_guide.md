# Parol6-Joycon 架构图使用指南

## 📐 架构图概览

本架构图展示了将 Nintendo Joycon 手柄集成到 Parol6 ROS2 MoveIt 机械臂的完整系统设计，包括7个核心层次和数据采集功能。

### 文件位置
- **DrawIO 文件**: `docs/parol6_joycon_architecture.drawio`
- **使用方法**:
  1. 在线打开: https://app.diagrams.net/ → 文件 → 打开 → 选择该文件
  2. VSCode 插件: 安装 "Draw.io Integration" 插件后直接打开
  3. 桌面版: 下载 DrawIO Desktop 应用

---

## 🎨 设计说明

### 1. 配色方案（技术感主题）

我采用了材料设计（Material Design）配色体系，每层使用独特的颜色标识：

| 层次 | 主色调 | 色系 | 设计意图 |
|------|--------|------|----------|
| **硬件输入层** | 🔵 蓝色 (#1e88e5) | Blue 600 | 代表物理输入设备 |
| **姿态解算层** | 🟠 橙色 (#ff9800) | Orange 500 | 代表数据处理和计算 |
| **ROS2 适配层** | 🟢 绿色 (#66bb6a) | Green 400 | 代表软件接口和适配 |
| **运动规划层** | 🟣 紫色 (#ab47bc) | Purple 400 | 代表算法和智能规划 |
| **机器人控制层** | 🩷 粉红 (#ec407a) | Pink 400 | 代表控制逻辑 |
| **硬件执行层** | 🔴 红色 (#ef5350) | Red 400 | 代表物理执行单元 |
| **数据采集层** | 🟡 黄色 (#ffeb3b) | Yellow 500 | 代表数据记录功能 |

**字体颜色对比**：
- 深色背景模块：使用白色字体 (#ffffff)
- 浅色背景模块：使用深色字体 (#1a1a1a)
- 标题：使用各层次的深色变体（如 Blue 900）

---

### 2. 布局优化策略

#### 空间分配
```
总画布: 1600px × 1200px

层次分布:
├─ 左侧 (80-380px): 输入和解算层
├─ 中央 (450-800px): ROS2 适配层
├─ 右侧 (870-1550px): 规划、控制、执行层
└─ 底部 (660-900px): 数据采集层
```

#### 避免遮挡的设计原则
1. **垂直间距**: 层与层之间至少保持 40px 间距
2. **水平偏移**: 相邻模块错开布局，避免连线穿过
3. **模块尺寸**:
   - 主要功能模块: 260-310px 宽
   - 子功能模块: 110-200px 宽
   - 高度根据内容自适应

---

### 3. 连线优化方案

#### 线条类型
| 类型 | 样式 | 宽度 | 用途 |
|------|------|------|------|
| **实线** | 正交连接 | 3px | 实时控制数据流 |
| **虚线** | 正交连接 | 2px | 数据采集流/反馈流 |

#### 连线走向策略
```
规则1: 主控制流从左至右流动
规则2: 反馈流从右至左，使用虚线标识
规则3: 数据采集流从上至下，使用虚线标识
规则4: 所有连线使用正交模式，避免斜线穿过模块
```

#### 标签位置
- 所有连线添加标签说明数据类型
- 标签背景设为 #f5f5f5 提高可读性
- 标签放置在连线中点位置，避免遮挡其他模块

---

## 📋 模块详细说明

### 第1层：硬件输入层

**主要模块**: Nintendo Joy-Con
- **文件**: 无（硬件设备）
- **关键参数**:
  - 蓝牙协议: Bluetooth 4.0
  - IMU 更新率: ~100Hz
  - 按键数量: 14个 + 1个摇杆
- **连接方式**: L/R 按键自动重连
- **校准方法**: Home 键长按3秒

---

### 第2层：姿态解算层

#### 主模块：joycon-robotics 库
- **文件**: `joyconrobotics/joyconrobotics.py`
- **核心类**: `JoyconRobotics`
- **关键函数**:
  ```python
  get_control() → (pose, gripper, button)
  reset_joycon()  # 校准
  set_posture_limits(glimit)
  set_dof_speed(dof_speed)
  ```

#### 子模块A：姿态估计器（AttitudeEstimator）
- **文件**: `joyconrobotics/gyro.py`
- **核心函数**:
  ```python
  update(gyro_in_rad, accel_in_g) → [roll, pitch, yaw]
  reset_yaw()
  set_yaw_diff(data)
  ```
- **算法参数**:
  - 互补滤波器: α = 0.55
  - 低通滤波器: α = 0.05
  - 采样周期: dt = 0.01秒

#### 子模块B：按键事件处理（ButtonEventJoyCon）
- **文件**: `joyconrobotics/event.py`
- **按键映射**:
  - A键 (button=1): 删除录制
  - Y键 (button=-1): 保存轨迹
  - Home键 (button=8): 复位
  - ZR键: 夹爪切换

---

### 第3层：ROS2 适配层

#### 主模块：Joycon 遥操作节点
- **建议文件**: `parol6_joycon_teleop/joycon_teleop_node.py`
- **核心类**: `JoyconTeleopNode(Node)`
- **关键函数**:
  ```python
  timer_callback()  # @ 100Hz
  check_workspace_limits()
  transform_coordinates()
  predict_collision()
  ```

**发布话题**:
- `/target_pose` (geometry_msgs/PoseStamped)
- `/gripper_command` (std_msgs/Float64)

**订阅话题**:
- `/joint_states` (sensor_msgs/JointState)
- `/planning_scene` (moveit_msgs/PlanningScene)

#### 配置模块：配置管理器
- **文件**: `parol6_joycon_teleop/config/parol6_config.yaml`
- **关键参数**:
  ```yaml
  parol6_工作空间:
    x: [0.05, 0.40]  # 米
    y: [-0.35, 0.35]
    z: [0.05, 0.50]

  初始位姿: [0.25, 0.0, 0.2, 0.0, 0.0, 0.0]

  joycon_映射:
    欧拉角反转: [1, -1, 1]
    方向反转: [-1, 1, 1]
    偏移欧拉角: [0, -3.14159, 0]

  控制参数:
    自由度速度: [1.0, 1.0, 1.0, 1.2, 1.2, 1.5]
    俯仰加倍: false
    纯XZ模式: false

  夹爪:
    打开: 1.0
    关闭: 0.0
  ```

---

### 第4层：运动规划层

#### MoveIt2 运动规划
- **包**: `moveit_ros_move_group`
- **节点**: `move_group`
- **配置包**: `parol6_moveit_config`

**关键功能**:
1. **逆运动学求解**:
   - 求解器: KDL 或 TRAC-IK
   - 超时: 0.05秒

2. **路径规划**:
   - 规划器: RRT-Connect (OMPL)
   - 规划时间: 5.0秒

3. **碰撞检测**:
   - 引擎: FCL（柔性碰撞检测库）
   - 检测频率: 实时

4. **轨迹优化**:
   - 时间参数化
   - 速度缩放: 0.3
   - 加速度缩放: 0.3

**配置文件**:
- `parol6_moveit_config/config/moveit.yaml`
- `parol6_moveit_config/config/kinematics.yaml`

---

### 第5层：机器人控制层

#### Parol6 驱动节点
- **建议文件**: `parol6_ros2/parol6_driver/parol6_driver.py`
- **核心功能**:
  ```python
  execute_trajectory(trajectory)
  get_joint_states() → JointState
  emergency_stop()
  set_velocity_limits(limits)
  ```

**通讯协议**:
- 接口: USB 串口
- 控制器: STM32F446
- 波特率: 115200（待确认）
- 数据格式: 关节角度（弧度）+ 速度（弧度/秒）

**发布话题**:
- `/joint_states` @ 100Hz

**订阅话题**:
- `/joint_trajectory` (trajectory_msgs/JointTrajectory)

---

### 第6层：硬件执行层

#### Parol6 机械臂
- **规格参数**:
  ```
  自由度: 6轴
  工作半径: 400mm
  负载: 1kg
  重复精度: 0.08mm
  重量: 6kg
  ```

- **硬件配置**:
  - 步进电机: 6个
  - 传动: 行星减速器 + 皮带传动
  - 编码器: 集成在电机
  - 末端执行器: 可选配夹爪

---

### 第7层：数据采集层

#### 模块A：轨迹记录器
- **建议文件**: `parol6_data_collection/trajectory_recorder.py`
- **核心类**: `TrajectoryRecorder`
- **关键函数**:
  ```python
  record_step(observation, action, gripper)
  save_episode(episode_id)
  start_new_episode()
  clear_data()
  ```

**数据格式（HDF5）**:
```python
episode_000.hdf5:
  ├─ 观测值/
  │   ├─ 关节位置: (N, 6) float32
  │   ├─ 关节速度: (N, 6) float32
  │   └─ 末端位姿: (N, 6) float32
  ├─ 动作/
  │   └─ 目标关节位置: (N, 6) float32
  ├─ 夹爪/
  │   └─ 状态: (N, 1) float32
  └─ 时间戳/
      └─ 纳秒时间: (N, 1) int64
```

#### 模块B：相机模块
- **建议文件**: `parol6_data_collection/camera_node.py`
- **依赖**: `cv2.VideoCapture`, `sensor_msgs/Image`
- **功能**:
  - RGB 图像采集: @ 30Hz
  - 深度图采集: @ 30Hz（可选）
  - 相机标定: 棋盘格标定

**发布话题**:
- `/camera/image_raw` (sensor_msgs/Image)
- `/camera/depth` (sensor_msgs/Image)
- `/camera/camera_info` (sensor_msgs/CameraInfo)

#### 模块C：数据集管理器
- **建议文件**: `parol6_data_collection/dataset_manager.py`
- **数据结构**:
  ```
  数据集/
    ├─ episode_000.hdf5
    ├─ episode_001.hdf5
    ├─ ...
    └─ metadata.json
  ```

**metadata.json 格式**:
```json
{
  "总轨迹数": 100,
  "总步数": 50000,
  "机器人": "Parol6",
  "控制频率": 100,
  "任务": ["抓取放置", "推动"],
  "创建日期": "2025-11-20",
  "parol6配置": {
    "工作空间": [[0.05, -0.35, 0.05], [0.40, 0.35, 0.50]],
    "初始位姿": [0.25, 0.0, 0.2, 0.0, 0.0, 0.0]
  }
}
```

---

## 🔄 数据流详解

### 1. 实时控制流（100Hz）

```
Joy-Con IMU 数据 (100Hz)
  ↓ [蓝牙] 陀螺仪 + 加速度计 + 按键
姿态估计器.update()
  ↓ [互补滤波] roll, pitch, yaw
JoyconRobotics.get_control()
  ↓ [坐标映射] [x, y, z, r, p, y] + 夹爪
Joycon遥操作节点.timer_callback()
  ↓ [ROS2 话题] PoseStamped
MoveIt2.computeCartesianPath()
  ↓ [逆运动学] 关节轨迹
Parol6驱动.execute_trajectory()
  ↓ [USB 串口] 电机指令
Parol6 机械臂执行
```

**关键时间节点**:
- Joy-Con 读取: ~0-5毫秒
- 姿态解算: ~5-10毫秒
- ROS2 传输: ~10-20毫秒
- MoveIt2 规划: ~20-40毫秒
- 串口发送: ~40-45毫秒
- **总延迟: <50毫秒**

---

### 2. 反馈流（100Hz）

```
Parol6 关节编码器
  ↓ [USB 串口] 关节位置 + 速度
Parol6驱动.publish_joint_states()
  ↓ [ROS2 话题] /joint_states
Joycon遥操作节点（接收反馈）
  ↓ 用于工作空间限制检查
MoveIt2 规划场景（接收反馈）
  ↓ 用于碰撞检测
```

---

### 3. 数据采集流（100Hz）

```
控制数据（Joycon遥操作节点）
  ↓
机器人状态（Parol6驱动）  ──┐
  ↓                        │
相机图像（相机节点）  ────────┤
  ↓                        │
轨迹记录器.record_step() ←──┘
  ↓
按键触发（Y键）
  ↓
轨迹记录器.save_episode()
  ↓
数据集管理器.organize()
  ↓
HDF5 文件 + metadata.json
```

---

## 🛠️ 使用和修改指南

### 在 DrawIO 中编辑

1. **修改文本**:
   - 双击任意文本框直接编辑
   - 支持富文本格式（粗体、颜色、大小）

2. **调整模块位置**:
   - 拖动模块到新位置
   - 连线会自动重新路由
   - 按住 Shift 多选模块批量移动

3. **修改颜色**:
   - 选中模块 → 右侧样式面板
   - fillColor: 填充色
   - strokeColor: 边框色
   - fontColor: 字体色

4. **添加新模块**:
   - 复制现有模块保持风格一致
   - 建议使用圆角矩形
   - strokeWidth=2（主模块），=1.5（子模块）

5. **连线技巧**:
   - 使用航点添加转折点
   - 右键连线 → 选择边 → 调整属性
   - 设置 edgeStyle=orthogonalEdgeStyle 保持正交

---

### 导出和分享

**导出为图片**:
```
文件 → 导出为 → PNG
- 分辨率: 200-300 DPI
- 边距: 10px
- 背景: 透明或白色
```

**导出为 PDF**:
```
文件 → 导出为 → PDF
- 适合文档嵌入
- 矢量格式，无损缩放
```

**嵌入到文档**:
- Markdown: `![架构图](docs/architecture.png)`
- LaTeX: `\includegraphics{docs/architecture.pdf}`
- Word/PPT: 直接插入 PNG 或 PDF

---

## 📊 性能指标说明

### 系统延迟分解

| 阶段 | 延迟 | 优化方法 |
|------|------|----------|
| 蓝牙传输 | 5毫秒 | 使用最新 Joy-Con 固件 |
| 姿态解算 | 5毫秒 | C++ 重写（可选）|
| ROS2 传输 | 10毫秒 | 使用 Fast DDS |
| MoveIt2 规划 | 20毫秒 | 缩小规划空间 |
| 串口发送 | 5毫秒 | 提高波特率 |
| **总计** | **45毫秒** | |

### 数据采集效率

**单条轨迹**:
- 时长: 30-180秒
- 数据点: 3,000-18,000个
- 文件大小: ~5-30MB（含图像）

**日采集量**（8小时工作）:
- 理想条件: 100-150条轨迹
- 实际预期: 50-80条轨迹（含调试）
- 总数据量: ~2.5-4GB/天

---

## 🔐 安全注意事项

### 软限位配置
```yaml
# 在 parol6_config.yaml 中设置
安全限位:
  位置:
    x: [0.10, 0.35]  # 比物理限位缩小 50mm
    y: [-0.30, 0.30]
    z: [0.10, 0.45]

  速度:
    最大笛卡尔速度: 0.5  # 米/秒
    最大关节速度: 1.0  # 弧度/秒

  力:
    最大接触力: 10.0  # 牛顿（如有力传感器）
```

### 碰撞检测
```python
# 在 JoyconTeleopNode 中实现
def check_collision(self, target_pose):
    # 1. 预测碰撞
    result = self.planning_scene.is_state_valid(target_pose)

    # 2. 如果检测到碰撞
    if not result.valid:
        # 震动警告
        self.joycon.vibrate(duration=0.1, intensity=1.0)
        # 拒绝执行
        return False

    return True
```

### 急停机制
- **硬件急停**: 外接按钮（推荐）
- **软件急停**: Plus/Minus 键长按3秒
- **自动急停**: 检测异常速度/加速度

---

## 📚 参考资源

### 官方文档
1. **Parol6**: https://source-robotics.github.io/PAROL-docs/
2. **joycon-robotics**: https://github.com/box2ai-robotics/joycon-robotics
3. **MoveIt2**: https://moveit.picknik.ai/main/index.html
4. **ROS2 Humble**: https://docs.ros.org/en/humble/

### 社区支持
- **QQ 交流群**: 948755626
- **B站教程**: https://space.bilibili.com/122291348
- **GitHub Issues**: 各项目仓库的 Issues 页面

### 相关论文
1. ACT: 基于 Transformer 的动作分块
2. 扩散策略: 通过动作扩散的视觉运动策略学习

---

## 🔄 版本历史

| 版本 | 日期 | 更新内容 |
|------|------|----------|
| v1.0 | 2025-11-20 | 初始版本，完整架构设计 |
| v1.1 | 2025-11-20 | 全部内容中文化 |

---

## 📝 待办事项

- [ ] 添加错误处理流程图
- [ ] 补充性能测试结果
- [ ] 增加真实部署照片
- [ ] 编写故障排除指南
- [ ] 添加多机械臂协同架构

---

## 🤝 贡献指南

如果您想改进这个架构图或文档：

1. Fork 项目仓库
2. 在 DrawIO 中修改 `.drawio` 文件
3. 更新 `architecture_guide.md` 说明
4. 提交 Pull Request

**注意**: 请保持配色方案和布局风格的一致性！

---

## 📄 许可证

本架构图和文档遵循项目主仓库的开源协议。

---

**创建者**: Claude Code
**日期**: 2025-11-20
**联系方式**: 通过项目 Issues 联系
