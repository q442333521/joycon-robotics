# Parol6-Joycon 集成方案文档中心

## 📚 文档导航

欢迎来到 Parol6 + Joycon 集成方案的文档中心！这里包含了完整的技术文档、架构图和实施指南。

---

## 📁 文档列表

### 1. 核心架构文档

#### 🎨 [架构图 (DrawIO)](./parol6_joycon_architecture.drawio)
**推荐查看方式**:
- **在线**: https://app.diagrams.net/ → 打开文件
- **VSCode**: 安装 "Draw.io Integration" 插件
- **桌面版**: 下载 DrawIO Desktop

**内容概览**:
- 7层完整系统架构
- 实时控制流和数据采集流
- 详细的文件名、函数名、参数标注
- 技术感配色设计

#### 📖 [架构指南 (Markdown)](./architecture_guide.md)
**深度解析文档**，包含:
- 每层模块的详细说明
- 关键代码文件和函数
- 配置参数完整列表
- 数据流详解
- 性能指标分析
- 安全注意事项

---

## 🚀 快速开始

### 第一步: 查看架构图
```bash
# 在浏览器中打开 DrawIO 在线版
# 然后导入 docs/parol6_joycon_architecture.drawio
```

### 第二步: 理解系统架构
阅读 `architecture_guide.md`，重点关注:
1. 7层架构的职责划分
2. 数据流向 (控制流 + 反馈流 + 采集流)
3. 关键配置参数

### 第三步: 开始实施
参考架构图中的文件名和函数名，按以下顺序开发:

```
Week 1-2: ROS2 适配层
  └─ 文件: joycon_teleop_node.py
  └─ 配置: parol6_config.yaml

Week 3-4: 运动规划集成
  └─ 文件: parol6_moveit_config/
  └─ 测试: 基础遥操作

Week 5-6: 数据采集功能
  └─ 文件: trajectory_recorder.py
  └─ 文件: camera_node.py
  └─ 文件: dataset_manager.py

Week 7-8: 优化和测试
  └─ 性能调优
  └─ 安全验证
```

---

## 📊 架构图预览

系统包含以下7个核心层:

```
┌─────────────────────────────────────────────────────┐
│  Layer 1: 硬件输入层 (Nintendo Joy-Con)              │
│  🔵 蓝色 - 陀螺仪+加速度计+按键 @ 100Hz              │
└──────────────────┬──────────────────────────────────┘
                   │ 蓝牙数据流
                   ↓
┌─────────────────────────────────────────────────────┐
│  Layer 2: 姿态解算层 (joycon-robotics)               │
│  🟠 橙色 - 互补滤波+低通滤波 → [x,y,z,r,p,y]        │
└──────────────────┬──────────────────────────────────┘
                   │ 6DOF Pose
                   ↓
┌─────────────────────────────────────────────────────┐
│  Layer 3: ROS2 适配层 (Joycon Teleop Node)          │
│  🟢 绿色 - 坐标转换+限制检查 → PoseStamped          │
└──────────────────┬──────────────────────────────────┘
                   │ /target_pose
                   ↓
┌─────────────────────────────────────────────────────┐
│  Layer 4: 运动规划层 (MoveIt2)                       │
│  🟣 紫色 - 逆运动学+路径规划+碰撞检测               │
└──────────────────┬──────────────────────────────────┘
                   │ JointTrajectory
                   ↓
┌─────────────────────────────────────────────────────┐
│  Layer 5: 机器人控制层 (Parol6 Driver)               │
│  🩷 粉红 - 轨迹跟踪+电机控制 → USB Serial           │
└──────────────────┬──────────────────────────────────┘
                   │ 电机指令
                   ↓
┌─────────────────────────────────────────────────────┐
│  Layer 6: 硬件执行层 (Parol6 机械臂)                 │
│  🔴 红色 - 6轴机械臂 (400mm/1kg/0.08mm)             │
└─────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────┐
│  Layer 7: 数据采集层 (Trajectory Recorder + Camera) │
│  🟡 黄色 - HDF5数据集+图像流 @ 100Hz                │
└─────────────────────────────────────────────────────┘
```

**总延迟**: <50ms (端到端)
**控制频率**: 100Hz
**数据采集**: 100Hz (支持 ACT/Diffusion Policy)

---

## 🎯 核心功能模块

### 输入处理
| 模块 | 文件 | 功能 |
|------|------|------|
| Joy-Con连接 | joycon.py | 蓝牙设备管理 |
| 姿态估计 | gyro.py | 互补滤波+漂移补偿 |
| 按键事件 | event.py | 按键映射和处理 |

### ROS2接口
| 模块 | 文件 | 功能 |
|------|------|------|
| 遥操作节点 | joycon_teleop_node.py | 主控制逻辑 |
| 配置管理 | parol6_config.yaml | 参数配置 |

### 运动控制
| 模块 | 包名 | 功能 |
|------|------|------|
| MoveIt2规划 | moveit_ros_move_group | 逆运动学+路径规划 |
| 机器人驱动 | parol6_driver | 硬件接口 |

### 数据采集
| 模块 | 文件 | 功能 |
|------|------|------|
| 轨迹记录 | trajectory_recorder.py | HDF5数据集生成 |
| 相机采集 | camera_node.py | RGB/深度图像 |
| 数据管理 | dataset_manager.py | 数据集组织 |

---

## ⚙️ 关键配置参数

### Parol6 工作空间
```yaml
workspace:
  x: [0.05, 0.40]  # 米
  y: [-0.35, 0.35]
  z: [0.05, 0.50]

init_pose: [0.25, 0.0, 0.2, 0.0, 0.0, 0.0]
```

### Joycon 映射参数
```yaml
euler_reverse: [1, -1, 1]
direction_reverse: [-1, 1, 1]
offset_euler_rad: [0, -3.14159, 0]
pitch_down_double: false
pure_xz: false
```

### 控制参数
```yaml
max_velocity_scaling: 0.3
max_acceleration_scaling: 0.3
dof_speed: [1.0, 1.0, 1.0, 1.2, 1.2, 1.5]
```

### 数据采集
```yaml
recording_frequency: 100  # Hz
image_resolution: [640, 480]
save_format: "hdf5"
```

---

## 🔄 数据流说明

### 实时控制流 (主线)
```
Joy-Con → joycon-robotics → ROS2 Node → MoveIt2 → Parol6 Driver → 机械臂
(100Hz)     (姿态解算)      (坐标转换)   (路径规划)   (电机控制)    (执行)
```

### 反馈流 (虚线)
```
机械臂 → Parol6 Driver → ROS2 Node (工作空间检查)
                      → MoveIt2 (碰撞检测)
```

### 数据采集流 (虚线)
```
ROS2 Node ──┐
Parol6 Driver ─┼→ Trajectory Recorder → Dataset Manager → HDF5
Camera ────┘
```

---

## 📈 性能指标

| 指标 | 数值 | 说明 |
|------|------|------|
| **端到端延迟** | <50ms | Joy-Con到机械臂响应 |
| **控制频率** | 100Hz | 实时控制循环 |
| **采集频率** | 100Hz | 数据记录速率 |
| **位置精度** | 0.08mm | Parol6硬件限制 |
| **工作范围** | 400mm | 半径 |
| **负载能力** | 1kg | 含末端执行器 |

---

## 🛡️ 安全特性

1. **软限位保护**: 工作空间边界检查
2. **碰撞预测**: MoveIt2实时碰撞检测
3. **速度限制**: 最大速度/加速度缩放
4. **急停机制**: Plus/Minus长按或外接按钮
5. **震动反馈**: 碰撞警告

---

## 🔧 开发工具推荐

### 必备工具
- **ROS2**: Humble (Ubuntu 22.04)
- **MoveIt2**: 官方二进制包
- **DrawIO**: 架构图编辑
- **VSCode**: 代码编辑 + DrawIO插件

### 可选工具
- **RViz2**: 可视化调试
- **rqt**: ROS2调试工具
- **PlotJuggler**: 数据可视化
- **Rerun**: 机器人数据可视化

---

## 📚 学习资源

### 官方文档
1. **Parol6**: https://source-robotics.github.io/PAROL-docs/
2. **joycon-robotics**: https://github.com/box2ai-robotics/joycon-robotics
3. **MoveIt2 教程**: https://moveit.picknik.ai/main/doc/tutorials/tutorials.html
4. **ROS2 教程**: https://docs.ros.org/en/humble/Tutorials.html

### 视频教程
- **B站@盒子桥智能**: https://space.bilibili.com/122291348
- **ROS2官方教程**: YouTube → The Construct

### 社区支持
- **QQ群**: 948755626 (joycon-robotics)
- **GitHub Issues**: 各项目仓库
- **ROS Discourse**: https://discourse.ros.org/

---

## 🐛 故障排除

### 常见问题

#### Q1: Joy-Con无法连接？
**解决方案**:
1. 重新运行 `make install`
2. 删除蓝牙配对信息后重新配对
3. 检查 Ubuntu 蓝牙服务状态
4. 参考 `architecture_guide.md` 的连接章节

#### Q2: 姿态漂移严重？
**解决方案**:
1. 长按 Home 键重新校准
2. 检查 `lowpassfilter_alpha_rate` 参数
3. 确保 Joy-Con 电量充足
4. 避免磁场干扰

#### Q3: MoveIt2规划失败？
**解决方案**:
1. 检查目标位姿是否在工作空间内
2. 增加 `planning_time` 参数
3. 调整 `max_velocity_scaling`
4. 检查碰撞场景配置

#### Q4: 数据采集卡顿？
**解决方案**:
1. 降低图像分辨率
2. 使用 SSD 存储
3. 检查 CPU/内存占用
4. 考虑使用 ROS2 bag录制

---

## 🎓 进阶主题

### 1. 模仿学习训练
完成数据采集后，可以训练：
- **ACT** (Action Chunking Transformers)
- **Diffusion Policy**
- **VLA** (Vision-Language-Action) - lerobot-joycon2

### 2. 多机械臂协同
扩展架构支持：
- 双臂协同操作
- 主从控制
- 双Joy-Con双臂控制

### 3. 力控集成
如果 Parol6 配备力传感器：
- 力反馈控制
- 柔顺操作
- 接触检测

---

## 🤝 贡献指南

欢迎为此项目贡献！

**贡献方式**:
1. 改进架构图设计
2. 补充文档内容
3. 提供实际部署案例
4. 分享调试经验

**提交流程**:
1. Fork 仓库
2. 创建分支 (`git checkout -b feature/your-feature`)
3. 提交更改 (`git commit -m 'Add some feature'`)
4. 推送分支 (`git push origin feature/your-feature`)
5. 创建 Pull Request

---

## 📄 许可证

本文档集遵循项目主仓库的开源协议。

---

## 📞 联系方式

- **Issues**: https://github.com/box2ai-robotics/joycon-robotics/issues
- **QQ群**: 948755626
- **Email**: boxjod@163.com

---

## 🌟 致谢

感谢以下项目和团队：
- **Box2AI Robotics** - joycon-robotics开发
- **Source Robotics** - Parol6机械臂
- **MoveIt** - 运动规划框架
- **ROS2** - 机器人操作系统

---

**文档版本**: v1.0
**最后更新**: 2025-11-20
**维护者**: Claude Code

---

## 📖 文档更新日志

| 日期 | 版本 | 更新内容 |
|------|------|----------|
| 2025-11-20 | v1.0 | 初始版本发布 |

---

**提示**: 开始之前，建议先完整阅读 `architecture_guide.md`，然后在 DrawIO 中打开架构图对照学习。祝您集成顺利！🚀
