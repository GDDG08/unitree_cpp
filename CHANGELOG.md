# CHANGELOG

## Versions

### 1.0.2
- Fix: shutdown as damping mode

### 1.0.3 [IMPORTANT FIX]
- Fix control delay, send command immediately after step.
    - this bug could lead to jittering and stability issue, see https://github.com/GDDG08/RoboJuDo/issues/2

### 1.0.4
- Add torso (secondary) IMU subscription, exposed via `RobotState.torso_imu_state`. see https://github.com/HansZ8/unitree_cpp/issues/5
    - Opt-in via `enable_torso_imu` / `torso_imu_topic` (default `rt/secondary_imu`).
