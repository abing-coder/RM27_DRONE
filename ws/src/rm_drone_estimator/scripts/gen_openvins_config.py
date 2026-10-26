#!/usr/bin/env python3
"""根据机体模型参数（需与 rm_drone.urdf.xacro 一致）生成 OpenVINS 单目/双目配置。
实机标定后，请用 Kalibr 结果替换 kalibr_imucam_chain.yaml / kalibr_imu_chain.yaml。"""
import math
import os

import numpy as np

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(PKG, 'config', 'openvins')

W, H, HFOV = 640, 480, 1.6
CAM_PITCH = 0.65  # Dedicated navigation stereo; nearby texture gives useful depth parallax.
BASELINE = 0.05
CAM_POS = (0.045, 0.008)  # x, z
IMU_RATE = 500.0
# 比仿真真实噪声略大，留一点保守余量
GYRO_ND, GYRO_RW, ACC_ND, ACC_RW = 0.0015, 3e-5, 0.015, 6e-4
NS = '/drone'

fx = (W / 2.0) / math.tan(HFOV / 2.0)
cx, cy = (W + 1) / 2.0, (H + 1) / 2.0


def R_imu_cam():
    c, s = math.cos(CAM_PITCH), math.sin(CAM_PITCH)
    Ry = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    R_link_opt = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)
    return Ry @ R_link_opt


def T_rows(y):
    R = R_imu_cam()
    p = np.array([CAM_POS[0], y, CAM_POS[1]])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return '\n'.join('    - [' + ', '.join(f'{v:.9f}' for v in row) + ']' for row in T)


def imucam(n_cam):
    s = '%YAML:1.0\n\n'
    names = [('left', BASELINE / 2), ('right', -BASELINE / 2)][:n_cam]
    for i, (name, y) in enumerate(names):
        overlaps = '[1]' if (n_cam == 2 and i == 0) else ('[0]' if n_cam == 2 else '[]')
        s += f"""cam{i}:
  T_imu_cam: # R_CtoI, p_CinI
{T_rows(y)}
  cam_overlaps: {overlaps}
  camera_model: pinhole
  distortion_coeffs: [0.0, 0.0, 0.0, 0.0]
  distortion_model: radtan
  intrinsics: [{fx:.4f}, {fx:.4f}, {cx:.2f}, {cy:.2f}]
  resolution: [{W}, {H}]
  rostopic: {NS}/stereo/{name}/image_raw
"""
    return s


IMU_CHAIN = f"""%YAML:1.0

imu0:
  T_i_b:
    - [1.0, 0.0, 0.0, 0.0]
    - [0.0, 1.0, 0.0, 0.0]
    - [0.0, 0.0, 1.0, 0.0]
    - [0.0, 0.0, 0.0, 1.0]
  accelerometer_noise_density: {ACC_ND:.4e}
  accelerometer_random_walk: {ACC_RW:.4e}
  gyroscope_noise_density: {GYRO_ND:.4e}
  gyroscope_random_walk: {GYRO_RW:.4e}
  rostopic: {NS}/imu
  time_offset: 0.0
  update_rate: {IMU_RATE}
  model: "kalibr"
  Tw:
    - [ 1.0, 0.0, 0.0 ]
    - [ 0.0, 1.0, 0.0 ]
    - [ 0.0, 0.0, 1.0 ]
  R_IMUtoGYRO:
    - [ 1.0, 0.0, 0.0 ]
    - [ 0.0, 1.0, 0.0 ]
    - [ 0.0, 0.0, 1.0 ]
  Ta:
    - [ 1.0, 0.0, 0.0 ]
    - [ 0.0, 1.0, 0.0 ]
    - [ 0.0, 0.0, 1.0 ]
  R_IMUtoACC:
    - [ 1.0, 0.0, 0.0 ]
    - [ 0.0, 1.0, 0.0 ]
    - [ 0.0, 0.0, 1.0 ]
  Tg:
    - [ 0.0, 0.0, 0.0 ]
    - [ 0.0, 0.0, 0.0 ]
    - [ 0.0, 0.0, 0.0 ]
"""


def estimator(n_cam, calib):
    return f"""%YAML:1.0 # need to specify the file type at the top!

verbosity: "INFO"

use_fej: true
integration: "rk4"
use_stereo: {'true' if n_cam == 2 else 'false'}
max_cameras: {n_cam}

calib_cam_extrinsics: {calib}
calib_cam_intrinsics: {calib}
calib_cam_timeoffset: {calib}
calib_imu_intrinsics: false
calib_imu_g_sensitivity: false

max_clones: 11
max_slam: 40
max_slam_in_update: 25
max_msckf_in_update: 40
dt_slam_delay: 1

gravity_mag: 9.81

feat_rep_msckf: "GLOBAL_3D"
feat_rep_slam: "ANCHORED_MSCKF_INVERSE_DEPTH"
feat_rep_aruco: "ANCHORED_MSCKF_INVERSE_DEPTH"

# 停在机库里时用 ZUPT 完成静止初始化（不需要等待起飞时的“急动”）
try_zupt: true
zupt_chi2_multipler: 1
zupt_max_velocity: 0.1
zupt_noise_multiplier: 10
zupt_max_disparity: 0.0
zupt_only_at_beginning: true

init_window_time: 1.0
init_imu_thresh: 4.0
init_max_disparity: 3.0
init_max_features: 120

init_dyn_use: false
init_dyn_mle_opt_calib: false
init_dyn_mle_max_iter: 50
init_dyn_mle_max_time: 0.05
init_dyn_mle_max_threads: 4
init_dyn_num_pose: 6
init_dyn_min_deg: 10.0
init_dyn_inflation_ori: 10
init_dyn_inflation_vel: 100
init_dyn_inflation_bg: 10
init_dyn_inflation_ba: 100
init_dyn_min_rec_cond: 1e-12
init_dyn_bias_g: [ 0.0, 0.0, 0.0 ]
init_dyn_bias_a: [ 0.0, 0.0, 0.0 ]

record_timing_information: false
record_timing_filepath: "/tmp/traj_timing.txt"

save_total_state: false
filepath_est: "/tmp/ov_estimate.txt"
filepath_std: "/tmp/ov_estimate_std.txt"
filepath_gt: "/tmp/ov_groundtruth.txt"

use_klt: true
num_pts: {150 if n_cam == 1 else 120}
fast_threshold: 15
grid_x: 5
grid_y: 4
min_px_dist: 12
knn_ratio: 0.70
track_frequency: 31.0
downsample_cameras: false
num_opencv_threads: 2
histogram_method: "NONE"

use_aruco: false
num_aruco: 1024
downsize_aruco: true

up_msckf_sigma_px: 1
up_msckf_chi2_multipler: 1
up_slam_sigma_px: 1
up_slam_chi2_multipler: 1
up_aruco_sigma_px: 1
up_aruco_chi2_multipler: 1

use_mask: false

relative_config_imu: "kalibr_imu_chain.yaml"
relative_config_imucam: "kalibr_imucam_chain.yaml"
"""


def main():
    for name, n in (('mono', 1), ('stereo', 2)):
        d = os.path.join(OUT, name)
        os.makedirs(d, exist_ok=True)
        open(os.path.join(d, 'estimator_config.yaml'), 'w').write(estimator(n, 'false'))
        open(os.path.join(d, 'kalibr_imu_chain.yaml'), 'w').write(IMU_CHAIN)
        open(os.path.join(d, 'kalibr_imucam_chain.yaml'), 'w').write(imucam(n))
        print('wrote', d)
    print(f'fx=fy={fx:.3f} cx={cx} cy={cy}')
    print('R_imu_cam=\n', R_imu_cam())


if __name__ == '__main__':
    main()
