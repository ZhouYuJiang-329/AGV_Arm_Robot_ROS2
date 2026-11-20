#!/usr/bin/python3
"""Export navigation config references/snapshots; no ROS init or hardware access."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import yaml

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'docs/导航'
ACTIVE = {
 'docs/validation/seer_low_speed_2026-11-13/nav2_locked_arm_site.yaml': 'params_file：当前现场 Nav2 参数',
 'docs/validation/seer_low_speed_2026-11-13/localization_observe_site.yaml': 'localization_params：当前现场定位参数',
 'src/robot_move/config/navigation_guard.yaml': 'guard_params：执行保护与速度上限',
 'src/agv_bringup/config/agv_navigation_commissioning.yaml': 'commissioning_file：现场确认与地图身份',
 'src/robot_move/config/seer_odometry_confirmed.yaml': 'odom_params 默认：已确认参考轴的里程计',
 'src/lidar/config/seer_lidar.yaml': 'lidar_params 默认：前后雷达采集',
 'src/agv_description/config/lidar_mounts.yaml': 'lidar_mounts_file 默认：雷达相对 base_link 外参',
 'src/agv_bringup/config/agv_navigation_real.rviz': 'rviz=true 时加载：当前实机显示配置',
 'maps/seer/20251120141209997_navigable.yaml': 'map：当前可导航地图元数据',
}
OTHER = {
 'src/agv_bringup/config/agv_nav2_real.yaml': '实机通用模板；当前命令已被现场 params_file 覆盖',
 'src/robot_move/config/seer_localization.yaml': '定位通用模板；当前命令已被现场定位参数覆盖',
 'src/robot_move/config/seer_odometry.yaml': '里程计通用模板；当前使用 confirmed 版本',
 'src/agv_bringup/config/agv_nav2_sim.yaml': '仿真 Nav2 参数，不用于当前实机启动',
 'src/agv_bringup/config/agv_slam_sim.yaml': '仿真建图参数',
 'src/agv_bringup/config/agv_navigation_scene.yaml': '仿真场景、车体和障碍物',
 'src/agv_bringup/config/agv_static_planning.yaml': '静态离线规划参数',
 'src/agv_bringup/config/ros2_controllers.yaml': '整机 ros2_control 关联配置；当前实机导航入口不加载',
 'src/pathplannernode/src/config/params.yaml': '旧独立路径规划器参数；当前 Nav2 实机入口不加载',
 'maps/seer/20251120141209997.yaml': '原始转换地图；当前使用 navigable 版本',
 'docs/validation/seer_low_speed_2026-11-13/nav2_locked_arm_site_before_stop.yaml': '历史版本：添加独立停车区域之前，不用于当前启动',
 'docs/validation/seer_low_speed_2026-11-13/lidar_mounts_measured_candidate.yaml': '测量候选外参，未作为当前默认挂载文件',
}
REFERENCES = [
 'src/agv_bringup/launch/agv_navigation_real.launch.py',
 'src/agv_bringup/launch/agv_navigation_sim.launch.py',
 'src/agv_bringup/launch/agv_navigation.launch.py',
 'src/agv_bringup/launch/seer_localization_demo.launch.py',
 'src/agv_bringup/launch/agv_gazebo_preview.launch.py',
 'src/agv_bringup/launch/agv_static_planning.launch.py',
 'src/agv_bringup/launch/agv_mapping_sim.launch.py',
 'src/robot_move/launch/seer_odometry.launch.py',
 'src/robot_move/launch/seer_localization_view.launch.py',
 'src/robot_move/launch/navigation_guard.launch.py',
 'src/lidar/launch/seer_lidar.launch.py',
 'src/agv_description/launch/robot_description.launch.py',
 'src/agv_description/urdf/lidar_only.urdf.xacro',
 'src/agv_description/urdf/lidar_mounts.xacro',
 'src/agv_description/urdf/agv_robot.urdf.xacro',
 'src/agv_bringup/agv_bringup_sim/real_navigation.py',
 'src/agv_bringup/agv_bringup_sim/execution_gate.py',
 'src/robot_move/src/navigation/node.cpp',
 'src/robot_move/src/navigation/guard.cpp',
 'src/robot_move/src/localization/node.cpp',
 'tools/record_navigation.py', 'tools/record_steering.py',
 'tools/check_agv_navigation_inputs.py', 'tools/record_seer_localization.py',
 'tools/probe_seer_odometry.py', 'tools/seer_readonly_capability_probe.py',
 'tools/convert_seer_smap.py',
 'src/pathplannernode/src/pathplanner/launch/ros2_pathplanner_launch.py',
]


def flatten(value, prefix=''):
    if isinstance(value, dict):
        for key, item in value.items():
            yield from flatten(item, f'{prefix}.{key}' if prefix else str(key))
    else:
        yield prefix, value


def display(value):
    return json.dumps(value, ensure_ascii=False).replace('|', '\\|').replace('\n', '\\n')


def export():
    OUT.mkdir(parents=True, exist_ok=True)
    files = dict(ACTIVE, **OTHER)
    for directory in ('src/agv_bringup/config', 'src/robot_move/config'):
        for p in sorted((ROOT / directory).glob('*.rviz')):
            if p.name != 'agv_moveit.rviz':
                files.setdefault(str(p.relative_to(ROOT)), '其他导航/定位/里程计显示配置，不是当前实机 RViz 默认')
    for path in (ROOT / 'docs/validation/seer_low_speed_2026-11-13').glob('*.yaml'):
        files.setdefault(str(path.relative_to(ROOT)), '其他现场/验证参数；是否启用以启动命令为准')
    for file in files:
        if not (ROOT / file).is_file():
            raise FileNotFoundError(file)
    manifest = {'generated_utc': datetime.now(timezone.utc).isoformat(),
                'note': 'Documentation snapshots only; launch continues reading original files. Not runtime parameter proof.',
                'files': [], 'related_assets': []}
    index = ['# 导航配置文件清单', '', '由 `tools/export_navigation_config_docs.py` 从当前磁盘文件生成。', '',
             '“当前实机”按已记录的完整 execute 命令及该入口默认依赖判定，不代表正在运行的进程已加载这些值。', '',
             '| 分类 | 原始文件（修改这里） | 作用 | 阅读快照 |', '| --- | --- | --- | --- |']
    catalog = ['# 全部 YAML 参数当前值', '', '自动生成的磁盘参数清单。数组原样保留，RViz 完整内容见配置快照。', '',
               '这些文件属于不同模式，不能将它们合并成一份启动参数。以“配置文件清单”的分类为准。', '']
    for number, (file, role) in enumerate(files.items(), 1):
        source = ROOT / file
        snapshot = OUT / '配置快照' / file
        snapshot.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, snapshot)
        digest = hashlib.sha256(source.read_bytes()).hexdigest()
        group = '当前实机' if file in ACTIVE else '模板/仿真/历史/关联'
        manifest['files'].append({'source': file, 'snapshot': str(snapshot.relative_to(OUT)),
                                  'category': group, 'role': role, 'sha256': digest})
        index.append(f'| {group} | [{file}](../../{file}) | {role} | [快照](配置快照/{file}) |')
        if source.suffix in ('.yaml', '.yml'):
            values = yaml.safe_load(source.read_text())
            catalog += [f'## {number}. {source.name}', '', f'分类：{group}。{role}。', '',
                        f'原文件：[{file}](../../{file})；SHA256：`{digest}`。', '',
                        '| 参数路径 | 当前磁盘值 |', '| --- | --- |']
            catalog += [f'| `{key}` | `{display(value)}` |' for key, value in flatten(values)]
            catalog.append('')
    index += ['', '## 启动、模型、执行逻辑与工具', '',
              '以下保留原文件链接，未复制可执行代码，阅读/导出不会执行它们。', '']
    for file in REFERENCES:
        assert (ROOT / file).is_file(), file
        index.append(f'- [{file}](../../{file})')
    for map_name in ('20251120141209997.yaml', '20251120141209997_navigable.yaml'):
        path = ROOT / 'maps/seer' / map_name
        image = (path.parent / yaml.safe_load(path.read_text())['image']).resolve()
        assert image.is_relative_to(ROOT) and image.is_file()
        relative = str(image.relative_to(ROOT))
        manifest['related_assets'].append({'source': relative, 'sha256': hashlib.sha256(image.read_bytes()).hexdigest()})
    index += ['', '## 地图图像资源', '',
              '地图图像保留原位置，不重复复制。快照中的地图 YAML 仅供阅读，不是独立可启动地图包。', '']
    index += [f'- [{r["source"]}](../../{r["source"]})，SHA256 `{r["sha256"]}`。' for r in manifest['related_assets']]
    index += ['', '实车测试轨迹和 rosbag 位于 `logs/navigation/`（被 Git 忽略），不混入当前配置目录。历史验证文件位于 `docs/validation/`。', '']
    (OUT / '配置文件清单.md').write_text('\n'.join(index))
    (OUT / '全部参数参考.md').write_text('\n'.join(catalog))
    (OUT / '配置快照/manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    print(f'Exported {len(files)} configuration snapshots, full YAML parameter reference and source index to {OUT}')


if __name__ == '__main__':
    export()
