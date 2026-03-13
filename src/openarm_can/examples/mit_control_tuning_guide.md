# MIT 模式位置控制误差分析与调优指南

## 你遇到的问题

**现象**：目标位置 0.5 rad，实际只到 0.255 rad，误差 14 度。

**原因**：不是控制写得不行，而是 **MIT 模式的物理本质** 决定的！

---

## MIT 模式原理

MIT 模式本质是 **虚拟弹簧阻尼器**，控制律为：

```
τ_motor = Kp × (q_target - q_current) + Kd × (0 - dq_current) + τ_feedforward
          └─────── 弹簧力 ─────────┘   └──── 阻尼力 ────┘   └── 前馈补偿 ──┘
```

当电机停止时，必然满足：**弹簧力 = 外部负载力**

### 你的情况

- 当前配置：`Kp=20`，`τ_feedforward=0`
- 机械臂自重产生下拉力矩 `τ_gravity ≈ -5 N·m`（假设）
- 平衡点：`20 × (0.5 - q) = 5` → `q ≈ 0.25`（正好和你实测的 0.255 吻合！）

**结论**：你的控制器工作正常，只是刚度不够强到"硬拉"到目标。

---

## 解决方案

### 方案 1：提高刚度（推荐先试）✅

```cpp
// 分级配置：承重关节用高刚度
Motor 1-2 (DM8009 基座)：kp = 80, kd = 3.0   // 承载最大
Motor 3-4 (DM4340 中段)：kp = 50, kd = 2.0   // 中等负载
Motor 5-7 (DM4310 腕部)：kp = 30, kd = 1.5   // 轻载
```

**优点**：简单，立即见效  
**缺点**：刚度过高可能震荡、发热、对冲击敏感

### 方案 2：添加重力补偿（最优但复杂）

```cpp
// 需要机械臂动力学模型
τ_feedforward = compute_gravity_torque(q_current, arm_parameters);

openarm::damiao_motor::MITParam{kp, kd, q_target, 0, τ_feedforward}
                                           //            ↑ 这里加补偿
```

**优点**：低刚度+精确跟踪，电机柔顺安全  
**缺点**：需要准确的质量分布参数和运动学模型

### 方案 3：混合（工业常用）

```cpp
// 中等刚度 + 粗略重力补偿
kp = 50;
τ_gravity_rough = estimate_gravity_by_joint_index(joint_id);  // 查表法
```

---

## 调参策略

### 第 1 步：测试当前改进版

```bash
cd ~/OpenArm/src/openarm_can/build
./openarm-demo
```

**预期**：误差应该降到 < 3 度。

### 第 2 步：若仍不满意，逐步加强

修改 demo.cpp 第 95-96 行，取消注释：

```cpp
// OPTION 2: 激进增益
std::vector<double> kp_values = {150, 150, 150, 150, 100, 100, 100};
std::vector<double> kd_values = {5.0, 5.0, 5.0, 5.0, 3.0, 3.0, 3.0};
```

### 第 3 步：观察副作用

刚度过高的表现：
- ❌ 电机发热严重
- ❌ 到位后高频震荡（抖动）
- ❌ 碰到障碍物直接卡死（不柔顺）
- ❌ 启动时猛烈加速（不平滑）

**如果出现以上问题 → 必须降低 kp 并加重力补偿！**

---

## 为什么末端关节误差小？

Motor 5-7 误差 < 1 度，因为：
1. **负载轻**：腕部只承受夹爪重量（~200g）
2. **力臂短**：靠近末端，力矩=力×距离，距离小则力矩小
3. **同样的 kp=20，能克服小负载**

---

## 进阶：如何实现重力补偿

### 简化方法（适合快速原型）

```cpp
// 基于关节角度的查表法（需实测）
double estimate_gravity_torque(int joint_id, double q) {
    // 预先手动拖动机械臂，记录不同角度下的平衡力矩
    if (joint_id == 0) {  // Motor 1
        return -4.5 * sin(q);  // 近似公式
    } else if (joint_id == 1) {
        return -3.2 * sin(q);
    }
    // ... 其他关节
    return 0.0;
}
```

### 标准方法（工业级）

使用 **递归牛顿-欧拉算法**（RNE）计算重力项：

```cpp
#include <kdl/chaindynparam.hpp>  // KDL库（Kinematics & Dynamics Library）

KDL::ChainDynParam dyn_solver(robot_chain);
KDL::JntArray q_current(7), gravity_torques(7);
dyn_solver.JntToGravity(q_current, gravity_torques);

// 然后在控制中使用
mit_param.tau = gravity_torques(i);  // 第 i 个关节
```

需要：
- 机械臂的完整 URDF 模型（含质量、惯量）
- 链接质心坐标
- 集成 KDL 或 Pinocchio 动力学库

---

## 总结

| 方法 | 误差 | 安全性 | 实现难度 | 适用场景 |
|------|------|--------|----------|----------|
| 低刚度（kp=20） | 大（14度）| ★★★★★ | ★ | 遥操作、拖动示教 |
| 高刚度（kp=100）| 小（<2度）| ★★ | ★ | 刚性夹取、定点操作 |
| 重力补偿+中刚度 | 极小 | ★★★★ | ★★★★★ | 工业生产、人机协作 |

**你当前的改进版用的是"分级高刚度"方案，应该能解决大部分问题。**

如果机械臂需要长期使用或需要更高性能，后续可以考虑集成动力学模型。

---

## 调试 Checklist

- [ ] 确认 `set_zero_all()` 已执行（零点对齐）
- [ ] 使用分级 kp：基座 80，腕部 30
- [ ] 观察是否震荡（若震荡，降 kp 或加 kd）
- [ ] 检查电机温度（若>60°C，降 kp）
- [ ] 误差 < 3 度即为可接受范围
- [ ] （可选）添加重力补偿降低发热

