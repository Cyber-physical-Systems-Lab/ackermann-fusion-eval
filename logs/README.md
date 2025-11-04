cat >> README.md << 'EOF'



## 录包（已完成）
**基线 bag（多进程，200 Hz）**  
- 名称：`baseline_sim_multiproc_200hz_2025-10-30_1756`
- 时长：89.419 s  
- 话题计数：
  - `/wheel_vel`: 17884 → ≈200.00 Hz
  - `/wheel_angle`: 17884 → ≈200.00 Hz
  - `/imu`: 8943 → ≈100.01 Hz

**可调频率三组（已完成）**
- `sim_multiproc_50hz_2025-10-30_1958`：/wheel_vel≈50 Hz，/imu≈100 Hz  
- `sim_multiproc_100hz_2025-10-30_2001`：/wheel_vel≈100 Hz，/imu≈100 Hz  
- `sim_multiproc_200hz_2025-10-30_2006`：/wheel_vel≈200 Hz，/imu≈100 Hz

---

## 回放自检（复现步骤）
```bash
# 回放 200 Hz 包（循环，静默后台）
cd ~/zml/project/bags
ros2 bag play sim_multiproc_200hz_2025-10-30_2006 --loop \
  --disable-keyboard-controls </dev/null >/dev/null 2>&1 &

# 速率检查（应≈200 Hz）
ros2 topic hz /wheel_vel -w 50


##QoS中继
# 运行中继：订阅 /wheel_vel（BestEffort），发布 /wheel_vel_reliable（Reliable）
cd ~/zml/project/ros2_ws
python3 scripts/relay_wheel_vel_qos.py &
# 验证发布 QoS:
ros2 topic info /wheel_vel_reliable -v | grep -A2 "PUBLISHER"   # 应显示 Reliability: RELIABLE


## 探针统计（每秒）
BestEffort  avg_hz=199.97  max_dt_max=0.0054s  avg_recv=200.0/s  samples=24  
Reliable    avg_hz=199.97  max_dt_max=0.0056s  avg_recv=200.0/s  samples=24

> 结论：200 Hz 轮速话题在两种可靠性策略下均可稳定消费；系统链路与 QoS 设置符合实验要求。
EOF
