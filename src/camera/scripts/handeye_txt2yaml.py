#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
calibrateHandEye.txt -> camera_tf.yaml 手眼外参转换脚本

背景
----
cv::calibrateHandEye 输出的 Hcg_camera 即 T_cam2gripper
（相机在机械臂末端坐标系中的位姿），与 camera_tf.yaml 中发布的静态 TF
parent_frame_id -> color_frame_id 同向，因此**不需要求逆**，只需三步：

    1. 拆块      : 取 4x4 齐次矩阵的 R 与 t（文件里的采样行直接忽略）
    2. 单位换算  : t 由毫米换为米（旋转无量纲）
    3. R -> 四元数: 输出 xyzw 顺序（节点按 [x,y,z,w] 读取，见 astras_node.cpp:234-237）

第 3 步是最容易踩的坑：xyzw / wxyz 写反、或忘记归一化符号，都会得到一个
"看起来是四元数、但朝向完全不同"的值。

用法
----
    # 1) 只转换并打印
    python3 handeye_txt2yaml.py /path/to/calibrateHandEye.txt

    # 2) 转换并与 camera_tf.yaml 当前生效值对比
    python3 handeye_txt2yaml.py /path/to/calibrateHandEye.txt \\
        --yaml src/camera/config/camera_tf.yaml

    # 3) 转换并直接写回 yaml（改前请先 git diff 备份）
    python3 handeye_txt2yaml.py /path/to/calibrateHandEye.txt \\
        --yaml src/camera/config/camera_tf.yaml --write
"""

import argparse
import datetime
import math
import os
import re
import sys

import numpy as np

# ---------------------------------------------------------------- 解析

_MATRIX_NUM_RE = re.compile(r'\[([^\[\]]*)\]', re.S)


def _read_text_best_effort(path):
    """按 UTF-8 -> GBK -> latin-1(忽略错误) 依次尝试解码。

    标定工具输出的表头常为 GBK 编码（如 "Hcg_camera 含义为："），
    但矩阵本体是纯 ASCII，因此解码失败不影响取数。
    """
    with open(path, 'rb') as f:
        raw = f.read()
    for enc in ('utf-8', 'gbk'):
        try:
            return raw.decode(enc)
        except UnicodeDecodeError:
            continue
    return raw.decode('latin-1', errors='ignore')


def parse_handeye_txt(path):
    """从标定输出文件中解析出 4x4 齐次矩阵。

    取第一个恰好含 16 个数字的方括号块，因此对换行 / `;` / `,` 的分隔方式
    都兼容；文件尾部的位姿采样行（`0: [...]`）不会被误取。
    """
    if not os.path.isfile(path):
        raise FileNotFoundError('找不到标定文件: %s' % path)

    text = _read_text_best_effort(path)

    for match in _MATRIX_NUM_RE.finditer(text):
        nums = [x for x in re.split(r'[;,\s]+', match.group(1).strip()) if x]
        if len(nums) != 16:
            continue
        try:
            values = [float(x) for x in nums]
        except ValueError:
            continue
        return np.array(values, dtype=float).reshape(4, 4)

    raise ValueError('%s 中未找到 4x4 齐次矩阵（期望 16 个数）' % path)


# ---------------------------------------------------------------- 校验


def validate(H):
    """校验齐次矩阵形状与旋转块合法性。

    返回 (是否合法刚体变换, 警告列表)。不合法时绝不允许写回 yaml ——
    一个带缩放或有镜像的 R 会被静默转成"看着合理"的四元数。
    """
    warnings = []

    R = H[:3, :3]
    if abs(H[3, 3] - 1.0) > 1e-6:
        warnings.append('H[3,3] = %g，期望 1.0' % H[3, 3])
    if np.abs(H[3, :3]).max() > 1e-9:
        warnings.append('末行前三项非零，这不是标准齐次矩阵')

    det = float(np.linalg.det(R))
    if abs(det - 1.0) > 1e-3:
        warnings.append('det(R) = %.6f，偏离 +1（非纯旋转，可能含缩放或镜像）' % det)

    orth = float(np.abs(R @ R.T - np.eye(3)).max())
    if orth > 1e-3:
        warnings.append('R 非正交，max|R·Rᵀ - I| = %.2e' % orth)

    is_rigid = abs(det - 1.0) <= 1e-3 and orth <= 1e-3
    return is_rigid, warnings


# ---------------------------------------------------------------- 旋转转换


def mat2quat(R):
    """旋转矩阵 -> 四元数，返回归一化的 [x, y, z, w]（Shepperd 分支法）。"""
    trace = float(np.trace(R))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s

    q = np.array([qx, qy, qz, qw], dtype=float)
    q /= np.linalg.norm(q)
    return q


def quat2mat(q):
    """四元数 [x, y, z, w] -> 旋转矩阵（用于往返校验）。"""
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rotation_angle_deg(qa, qb):
    """两个四元数所代表旋转之间的夹角（度）。"""
    dot = abs(float(np.dot(qa, qb)))
    dot = min(1.0, max(-1.0, dot))
    return math.degrees(2.0 * math.acos(dot))


# ---------------------------------------------------------------- yaml 读写

_KEY_T_RE = re.compile(r'^(?P<indent>\s*)tf_translation\s*:\s*\[(?P<body>[^\]]*)\]')
_KEY_Q_RE = re.compile(r'^(?P<indent>\s*)tf_rotation\s*:\s*\[(?P<body>[^\]]*)\]')
_PARENT_RE = re.compile(r'^\s*parent_frame_id\s*:\s*["\']?(?P<name>[^"\'#]+)')

# 本脚本写入的溯源注释，用于重复写入时原地覆盖而非累加
_AUTO_NOTE_RE = re.compile(
    r'^\s*#\s*由\s+\S+\s+@\s+\d{4}-\d{2}-\d{2}\s+\d{2}:\d{2}\s+自动转换\s*$')


def read_yaml_values(path):
    """读取 yaml 中当前**生效**的 tf_translation / tf_rotation（跳过注释行）。"""
    translation = rotation = parent = None
    with open(path, encoding='utf-8') as f:
        for line in f:
            if line.lstrip().startswith('#'):
                continue

            if parent is None:
                m = _PARENT_RE.match(line)
                if m:
                    parent = m.group('name').strip()

            if translation is None:
                m = _KEY_T_RE.match(line)
                if m:
                    translation = [float(x) for x in m.group('body').split(',')]

            if rotation is None:
                m = _KEY_Q_RE.match(line)
                if m:
                    rotation = [float(x) for x in m.group('body').split(',')]

    return (np.array(translation) if translation else None,
            np.array(rotation) if rotation else None,
            parent)


def write_yaml_values(path, translation, rotation, source_note):
    """就地替换 yaml 中生效的 tf_translation / tf_rotation 两行，返回替换情况。

    只动第一处非注释匹配行，注释掉的历史值保持原样。若目标行上方已是本脚本
    生成的溯源注释，则原地覆盖它，保证反复执行不会累加注释行。
    """
    with open(path, encoding='utf-8') as f:
        lines = f.readlines()

    t_str = ', '.join('%.10g' % v for v in translation)
    q_str = ', '.join('%.10g' % v for v in rotation)

    def emit(out, indent, value_line):
        note_line = '%s# 由 %s 自动转换\n' % (indent, source_note)
        if out and _AUTO_NOTE_RE.match(out[-1]):
            out[-1] = note_line
        else:
            out.append(note_line)
        out.append(value_line)

    replaced = {'t': False, 'q': False}
    out = []
    for line in lines:
        if not line.lstrip().startswith('#'):
            m = _KEY_T_RE.match(line)
            if m and not replaced['t']:
                emit(out, m.group('indent'), '%stf_translation: [%s]\n' % (m.group('indent'), t_str))
                replaced['t'] = True
                continue

            m = _KEY_Q_RE.match(line)
            if m and not replaced['q']:
                emit(out, m.group('indent'), '%stf_rotation: [%s]    # xyzw\n' % (m.group('indent'), q_str))
                replaced['q'] = True
                continue

        out.append(line)

    if not (replaced['t'] and replaced['q']):
        raise RuntimeError('yaml 中未找到生效的 tf_translation / tf_rotation 行，未做修改')

    with open(path, 'w', encoding='utf-8') as f:
        f.writelines(out)

    return replaced


# ---------------------------------------------------------------- 输出


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='把 calibrateHandEye.txt 的 Hcg_camera 矩阵转换为 camera_tf.yaml 的外参',
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('txt', help='标定输出文件路径（calibrateHandEye.txt）')
    parser.add_argument('--yaml', metavar='PATH',
                        help='camera_tf.yaml 路径；给出后会自动与其中当前生效值对比')
    parser.add_argument('--write', action='store_true',
                        help='把转换结果写回 --yaml 指定的文件（先自行 git diff 备份）')
    parser.add_argument('--no-negate', action='store_true',
                        help='不强制四元数归一化为 w>=0（默认取 w>=0，与 tf2_echo 输出一致）')
    args = parser.parse_args(argv)

    if args.write and not args.yaml:
        parser.error('--write 需要同时指定 --yaml')

    # --- 转换 ---
    H = parse_handeye_txt(args.txt)

    is_rigid, warnings = validate(H)
    for warning in warnings:
        print('[警告] %s' % warning)

    R = H[:3, :3]
    t_m = H[:3, 3] / 1000.0          # mm -> m
    q = mat2quat(R)

    if not args.no_negate and q[3] < 0.0:
        q = -q

    # --- 往返校验 ---
    err_R = float(np.abs(quat2mat(q) - R).max())
    err_t = float(np.abs(t_m * 1000.0 - H[:3, 3]).max())
    print('往返校验: 旋转 max 误差 = %.2e %s | 平移 max 误差 = %.2e mm %s' % (
        err_R, 'OK' if err_R < 1e-9 else 'FAIL',
        err_t, 'OK' if err_t < 1e-9 else 'FAIL'))

    print()
    print('=== 转换结果 (可直接粘贴) ===')
    print('    tf_translation: [%s]' % ', '.join('%.10g' % v for v in t_m))
    print('    tf_rotation: [%s]    # xyzw' % ', '.join('%.10g' % v for v in q))
    print()
    print('  相机各轴在末端系下的方向 (R 的列向量):')
    for axis, name in ((0, 'x 轴(图像右)'), (1, 'y 轴(图像下)'), (2, 'z 轴(光轴)  ')):
        print('    %s : %s' % (name, np.round(R[:, axis], 4)))

    # --- 与 yaml 对比 / 写回 ---
    if args.yaml:
        print()
        if not os.path.isfile(args.yaml):
            print('[跳过] 找不到 yaml: %s' % args.yaml)
            return 0

        t_cur, q_cur, parent = read_yaml_values(args.yaml)
        print('=== 与 %s 当前生效值对比 ===' % args.yaml)
        if parent:
            print('  当前 parent_frame_id : %s' % parent)
            if 'arm1' in parent:
                print('  [注意] 目标是 arm1；若本次标定挂在另一条臂上，参考系是错的')

        if t_cur is None or q_cur is None:
            print('  [跳过] yaml 中未读到生效的 tf_translation / tf_rotation')
        else:
            delta = t_m - t_cur
            angle = rotation_angle_deg(q, q_cur)
            print('  平移 当前(mm): [%.3f, %.3f, %.3f]' % tuple(t_cur * 1000))
            print('  平移 新  (mm): [%.3f, %.3f, %.3f]' % tuple(t_m * 1000))
            print('  平移 差值(mm): [%.3f, %.3f, %.3f]   |Δ| = %.2f mm'
                  % (delta[0] * 1000, delta[1] * 1000, delta[2] * 1000,
                     float(np.linalg.norm(delta)) * 1000))
            print('  姿态夹角      : %.2f 度' % angle)
            if angle > 90.0:
                print('  [注意] 夹角 > 90 度，两者朝向完全不同，请确认是否同一安装/同一参考臂')

        if args.write:
            if not is_rigid:
                print()
                print('[中止] 旋转块不是合法刚体旋转，拒绝写回 yaml'
                      '（请先修标定数据，或去掉 --write 仅查看转换结果）')
                return 1

            note = '%s @ %s' % (os.path.basename(args.txt),
                                datetime.datetime.now().strftime('%Y-%m-%d %H:%M'))
            write_yaml_values(args.yaml, t_m, q, note)
            print()
            print('已写回 %s，请执行 git diff 复核' % args.yaml)

    return 0


if __name__ == '__main__':
    sys.exit(main())
