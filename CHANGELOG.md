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

### dummy_sdk (opt-in)
- Add a bundled header-only dummy `unitree_sdk2` (`dummy_sdk/`) so the module can
  build and import on machines without the real SDK (e.g. macOS). Opt-in via
  `-DUSE_DUMMY_SDK=ON`; a normal build still requires the real SDK and fails
  loudly if it is missing, never silently falling back. The dummy simulates
  inbound data (stationary robot, identity base IMU, tilted torso IMU, zero
  odometry) so `self_check()` passes and the pipeline runs. Test only — never
  deploy. See the TESTING guide in the README.
