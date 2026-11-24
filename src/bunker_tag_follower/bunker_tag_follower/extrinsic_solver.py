"""Offline hand-eye solver for JSONL samples from extrinsic_recorder."""

import argparse
import json
import math
import os
import sys

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def transform(data):
    return (Rotation.from_quat(data['quaternion_xyzw']),
            np.asarray(data['translation_m'], dtype=np.float64))


def inverse(rotation, translation):
    inv_rotation = rotation.inv()
    return inv_rotation, -inv_rotation.apply(translation)


def compose(a, b):
    ar, at = a
    br, bt = b
    return ar * br, at + ar.apply(bt)


def load_samples(path, max_odom_offset):
    samples, rejected = [], 0
    with open(path, encoding='utf-8') as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                row = json.loads(line)
                if abs(float(row.get('odom_time_offset_sec', 0.0))) > max_odom_offset:
                    rejected += 1
                    continue
                samples.append((transform(row['odom_T_base']),
                                transform(row['camera_T_tag']), row))
            except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
                raise ValueError(f'{path}:{line_number}: invalid sample: {exc}') from exc
    return samples, rejected


def solve(samples, position_sigma, rotation_sigma_rad):
    # Seed from the project's measured mount estimate and REP-103 optical axes.
    x0_r = Rotation.from_euler('xyz', [-math.pi / 2, 0.0, -math.pi / 2])
    x0_t = np.array([0.09, 0.0, 0.539], dtype=np.float64)
    y_guesses = [compose(compose(a, (x0_r, x0_t)), c) for a, c, _ in samples]
    y0_t = np.mean([t for _, t in y_guesses], axis=0)
    y0_r = Rotation.concatenate([r for r, _ in y_guesses]).mean()
    initial = np.r_[x0_t, x0_r.as_rotvec(), y0_t, y0_r.as_rotvec()]

    def residual(params):
        x = (Rotation.from_rotvec(params[3:6]), params[:3])
        y = (Rotation.from_rotvec(params[9:12]), params[6:9])
        y_inv = inverse(*y)
        errors = []
        for a, c, _ in samples:
            error = compose(y_inv, compose(compose(a, x), c))
            errors.extend(error[1] / position_sigma)
            errors.extend(error[0].as_rotvec() / rotation_sigma_rad)
        return np.asarray(errors)

    return least_squares(residual, initial, loss='soft_l1', f_scale=1.0,
                         max_nfev=500, x_scale='jac')


def pose_record(pose):
    rotation, translation = pose
    return {'translation_m': translation.tolist(),
            'quaternion_xyzw': rotation.as_quat().tolist(),
            'rpy_rad': rotation.as_euler('xyz').tolist()}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', nargs='?', default='camera_extrinsic_samples.jsonl')
    parser.add_argument('--output', default='camera_extrinsic_result.json')
    parser.add_argument('--min-samples', type=int, default=10)
    parser.add_argument('--max-odom-offset', type=float, default=0.10)
    parser.add_argument('--position-sigma', type=float, default=0.03)
    parser.add_argument('--rotation-sigma-deg', type=float, default=5.0)
    parser.add_argument('--max-position-rms', type=float, default=0.05)
    parser.add_argument('--max-rotation-rms-deg', type=float, default=3.0)
    args = parser.parse_args(argv)

    try:
        samples, rejected = load_samples(args.input, args.max_odom_offset)
    except (OSError, ValueError) as exc:
        print(f'无法读取采集数据：{exc}', file=sys.stderr)
        return 2
    if len(samples) < args.min_samples:
        print(f'有效样本只有 {len(samples)} 组（时间偏差筛掉 {rejected} 组）；'
              f'至少需要 {args.min_samples} 组。', file=sys.stderr)
        return 2

    odom_xy = np.asarray([a[1][:2] for a, _, _ in samples])
    yaw = np.unwrap([a[0].as_euler('xyz')[2] for a, _, _ in samples])
    travel = float(np.max(np.linalg.norm(odom_xy - odom_xy[0], axis=1)))
    yaw_span = float(np.ptp(yaw))
    if travel < 0.25 or yaw_span < math.radians(20):
        print(f'运动激励不足：平移范围 {travel:.3f} m，转角范围 '
              f'{math.degrees(yaw_span):.1f} deg。至少需要 0.25 m 平移和 20 deg 转向；'
              '请补采后再求解。', file=sys.stderr)
        return 2

    result = solve(samples, args.position_sigma,
                   math.radians(args.rotation_sigma_deg))
    p = result.x
    base_optical = (Rotation.from_rotvec(p[3:6]), p[:3])
    world_tag = (Rotation.from_rotvec(p[9:12]), p[6:9])

    translation_errors, rotation_errors = [], []
    for a, c, _ in samples:
        error = compose(inverse(*world_tag), compose(compose(a, base_optical), c))
        translation_errors.append(np.linalg.norm(error[1]))
        rotation_errors.append(error[0].magnitude())
    trans_rms = float(np.sqrt(np.mean(np.square(translation_errors))))
    rot_rms = float(np.sqrt(np.mean(np.square(rotation_errors))))
    singular_values = np.linalg.svd(result.jac, compute_uv=False)
    rank_tolerance = (singular_values[0] * max(result.jac.shape) * np.finfo(float).eps
                      if singular_values.size else 0.0)
    effective_rank = int(np.count_nonzero(singular_values > rank_tolerance))
    condition_number = (float(singular_values[0] / singular_values[-1])
                        if singular_values.size and singular_values[-1] > 0 else math.inf)

    base_xyz = np.asarray([a[1] for a, _, _ in samples])
    base_rpy = np.unwrap(np.asarray([a[0].as_euler('xyz') for a, _, _ in samples]), axis=0)
    planar_motion = (float(np.ptp(base_xyz[:, 2])) < 0.02 and
                     float(np.ptp(base_rpy[:, 0])) < math.radians(1.0) and
                     float(np.ptp(base_rpy[:, 1])) < math.radians(1.0))
    vertical_observable = not planar_motion
    if (not result.success or not vertical_observable or effective_rank < len(result.x) or
            condition_number > 1e8 or trans_rms > args.max_position_rms or
            math.degrees(rot_rms) > args.max_rotation_rms_deg):
        quality = 'poor'
    elif trans_rms <= 0.03 and math.degrees(rot_rms) <= 2.0:
        quality = 'good'
    else:
        quality = 'review'

    link_optical = [transform(row['camera_link_T_optical'])
                    for _, _, row in samples if row.get('camera_link_T_optical')]
    if link_optical:
        internal = (Rotation.concatenate([r for r, _ in link_optical]).mean(),
                    np.mean([t for _, t in link_optical], axis=0))
        base_link = compose(base_optical, inverse(*internal))
        camera_link_frame = samples[0][2].get('camera_link_frame', 'camera_link')
    else:
        base_link = None
        camera_link_frame = None

    output = {
        'samples_used': len(samples),
        'samples_rejected_time_offset': rejected,
        'position_rms_m': trans_rms,
        'rotation_rms_deg': math.degrees(rot_rms),
        'position_error_p95_m': float(np.percentile(translation_errors, 95)),
        'rotation_error_p95_deg': math.degrees(float(np.percentile(rotation_errors, 95))),
        'jacobian_effective_rank': effective_rank,
        'jacobian_parameter_count': int(len(result.x)),
        'jacobian_condition_number': (condition_number if math.isfinite(condition_number)
                                      else 'Infinity'),
        'vertical_translation_observable': vertical_observable,
        'quality': quality,
        'quality_note': ('poor means do not apply this transform; RMS, observability, or conditioning failed'),
        'base_frame': samples[0][2].get('base_frame', 'base_link'),
        'camera_optical_frame': samples[0][2].get('camera_frame', 'camera_color_optical_frame'),
        'base_T_camera_optical': pose_record(base_optical),
        'camera_link_frame': camera_link_frame,
        'base_T_camera_link': pose_record(base_link) if base_link else None,
        'optimizer_success': bool(result.success),
        'optimizer_message': result.message,
    }
    output_path = os.path.expanduser(args.output)
    with open(output_path, 'w', encoding='utf-8') as stream:
        json.dump(output, stream, ensure_ascii=False, indent=2)
        stream.write('\n')

    print(f'使用 {len(samples)} 组样本；因时间差筛除 {rejected} 组。')
    print(f'残差 RMS：平移 {trans_rms:.4f} m，旋转 {math.degrees(rot_rms):.2f} deg')
    print(f'残差 P95：平移 {output["position_error_p95_m"]:.4f} m，'
          f'旋转 {output["rotation_error_p95_deg"]:.2f} deg')
    print(f'可观测秩 {effective_rank}/{len(result.x)}；条件数 {condition_number:.3g}；'
          f'垂向可观测={vertical_observable}；质量={quality}')
    print(f'base_T_camera_optical: xyz={base_optical[1].round(6).tolist()}, '
          f'rpy={base_optical[0].as_euler("xyz").round(6).tolist()}')
    if base_link:
        print(f'base_T_camera_link: xyz={base_link[1].round(6).tolist()}, '
              f'rpy={base_link[0].as_euler("xyz").round(6).tolist()}')
    else:
        print('样本未包含 camera_link→optical TF；只输出 optical frame 外参。')
    print(f'结果文件：{output_path}')
    if not result.success:
        print('优化器未收敛；请勿直接采用结果。', file=sys.stderr)
        return 1
    if quality == 'poor':
        print('质量检查未通过；请勿直接采用结果。', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
