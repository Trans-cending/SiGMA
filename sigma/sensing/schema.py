"""Node labels and sensor configuration inherited from the source data generator.

The Stage II runner selects CARLA RGB, depth, semantic, LiDAR, GNSS and IMU
blueprints from these data-only defaults. Ray-traced radar is a virtual
calibration entry consumed by Stage III.
"""

def safe_path_component(value, fallback='node'):
    text = str(value or '').strip()
    cleaned = ''.join(c if (c.isalnum() or c in ('-', '_', '.')) else '_' for c in text)
    cleaned = cleaned.strip('._')
    return cleaned or str(fallback)

def node_label(kind, index):
    return f"{str(kind).lower()}_{int(index) + 1:03d}"

def sensor_modality(sensor_type):
    typ = str(sensor_type or '').lower()
    if typ in ('camera.rgb', 'camera', 'scene'):
        return 'rgb'
    if typ in ('camera.depth', 'depth'):
        return 'depth'
    if typ in ('camera.semantic_segmentation', 'semantic', 'semantic_segmentation'):
        return 'semantic'
    if typ in ('gnss',):
        return 'gps'
    if typ in ('lidar', 'sensor.lidar.ray_cast', 'ray_cast'):
        return 'lidar_3d'
    if typ in ('lidar_2d', 'laser_2d', 'ray_cast_2d'):
        return 'lidar_2d'
    if typ in ('lidar_3d', 'laser_3d', 'ray_cast_3d'):
        return 'lidar_3d'
    return typ or 'sensor'

def trajectory_file_stem(modality, trajectory_id, frame_idx):
    return f"{sensor_modality(modality)}_{int(frame_idx):06d}"

def default_multimodal_sensor_items(kind):
    """Return the source RGB/depth/semantic/LiDAR/GNSS/IMU suites for RSU/CAV nodes."""
    prefix = str(kind).lower()
    if prefix == 'cav':
        rgb_tf = {'location': [1.5, 0.0, 1.7], 'rotation': [0.0, 0.0, 0.0]}
        cav_camera_tfs = {
            'front': {'location': [1.5, 0.0, 1.7], 'rotation': [0.0, 0.0, 0.0]},
            'left': {'location': [0.0, -0.8, 1.7], 'rotation': [0.0, -90.0, 0.0]},
            'back': {'location': [-1.5, 0.0, 1.7], 'rotation': [0.0, 180.0, 0.0]},
            'right': {'location': [0.0, 0.8, 1.7], 'rotation': [0.0, 90.0, 0.0]},
        }
        lidar_tf = {'location': [0.0, 0.0, 2.2], 'rotation': [0.0, 0.0, 0.0]}
        gps_tf = {'location': [0.0, 0.0, 0.0], 'rotation': [0.0, 0.0, 0.0]}
        imu_tf = {'location': [0.0, 0.0, 1.6], 'rotation': [0.0, 0.0, 0.0]}
    else:
        rgb_tf = {'location': [0.0, 0.0, 0.0], 'rotation': [0.0, 0.0, 0.0]}
        lidar_tf = {'location': [0.0, 0.0, 0.0], 'rotation': [0.0, 0.0, 0.0]}
        gps_tf = {'location': [0.0, 0.0, 0.0], 'rotation': [0.0, 0.0, 0.0]}
        imu_tf = {'location': [0.0, 0.0, 0.0], 'rotation': [0.0, 0.0, 0.0]}

    if prefix == 'cav':
        camera_items = []
        for modality, blueprint in (
                ('rgb', 'sensor.camera.rgb'),
                ('depth', 'sensor.camera.depth'),
                ('semantic', 'sensor.camera.semantic_segmentation')):
            for direction in ('front', 'left', 'back', 'right'):
                item = {
                    'name': f'{modality}_{direction}',
                    'file_prefix': f'{modality}_{direction}',
                    'type': modality,
                    'blueprint': blueprint,
                    'enabled': direction == 'front',
                    'image_size_x': 1280,
                    'image_size_y': 720,
                    'fov': 90,
                    'transform': cav_camera_tfs[direction],
                }
                if modality == 'depth':
                    item['depth_max_m'] = 200.0
                camera_items.append(item)
    elif prefix == 'rsu':
        camera_height = 1.5
        rsu_camera_tfs = {
            'front': {'location': [0.0, 0.0, 0.0], 'rotation': [0.0, 0.0, 0.0]},
            'left': {'location': [0.0, 0.0, 0.0], 'rotation': [0.0, -90.0, 0.0]},
            'back': {'location': [0.0, 0.0, 0.0], 'rotation': [0.0, 180.0, 0.0]},
            'right': {'location': [0.0, 0.0, 0.0], 'rotation': [0.0, 90.0, 0.0]},
        }
        camera_items = []
        for modality, blueprint in (
                ('rgb', 'sensor.camera.rgb'),
                ('depth', 'sensor.camera.depth'),
                ('semantic', 'sensor.camera.semantic_segmentation')):
            for direction in ('front', 'left', 'back', 'right'):
                item = {
                    'name': f'{modality}_{direction}',
                    'file_prefix': f'{modality}_{direction}',
                    'type': modality,
                    'blueprint': blueprint,
                    'enabled': True,
                    'image_size_x': 1280,
                    'image_size_y': 720,
                    'fov': 90,
                    'mount_z_mode': 'above_sign',
                    'camera_height_above_sign_m': camera_height,
                    'transform': rsu_camera_tfs[direction],
                }
                if modality == 'depth':
                    item['depth_max_m'] = 200.0
                camera_items.append(item)
    else:
        raise ValueError('Default sensor suites are defined only for cav and rsu')

    gps_item = {
        'name': f'{prefix}_gps',
        'type': 'gps',
        'blueprint': 'sensor.other.gnss',
        'transform': gps_tf,
    }

    return camera_items + [
        {'name': f'{prefix}_lidar_2d_top', 'file_prefix': 'lidar_2d_top', 'type': 'lidar_2d',
         'modality': 'lidar_2d', 'direction': 'top', 'blueprint': 'sensor.lidar.ray_cast',
         'channels': 1, 'range': 120.0, 'points_per_second': 60000,
         'rotation_frequency': 10, 'sync_rotation_frequency_to_sensor_tick': True,
         'upper_fov': 0, 'lower_fov': 0, 'transform': lidar_tf},
        {'name': f'{prefix}_lidar_3d_top', 'file_prefix': 'lidar_3d_top', 'type': 'lidar_3d',
         'modality': 'lidar_3d', 'direction': 'top', 'blueprint': 'sensor.lidar.ray_cast',
         'channels': 32, 'range': 120.0, 'points_per_second': 120000,
         'rotation_frequency': 10, 'sync_rotation_frequency_to_sensor_tick': True,
         'upper_fov': 10, 'lower_fov': -25, 'transform': lidar_tf},
        {'name': f'{prefix}_radar', 'type': 'radar', 'radar_backend': 'sionna_rcs',
         'radar_profile': 'deepsense_s9',
         'transform': {'location': [2.0, 0.0, 1.0] if prefix == 'cav' else [0.0, 0.0, 0.0],
                       'rotation': [0.0, 0.0, 0.0]}},
        gps_item,
        {'name': f'{prefix}_imu', 'type': 'imu', 'blueprint': 'sensor.other.imu', 'transform': imu_tf},
    ]

def sensor_items_for_node(cfg, kind, index, label=None):
    cfg = dict(cfg or {})
    label = label or node_label(kind, index)
    items = (cfg.get('items', []) if kind == 'ris' else
             cfg.get('items') or default_multimodal_sensor_items(kind))
    items = [dict(item or {}) for item in list(items or [])]

    per_node = cfg.get('per_node') or {}
    node_cfg = {}
    if isinstance(per_node, dict):
        node_cfg = dict(per_node.get(label) or per_node.get(str(index + 1)) or {})
    elif isinstance(per_node, list) and index < len(per_node) and isinstance(per_node[index], dict):
        node_cfg = dict(per_node[index])
    if node_cfg.get('enabled') is False:
        return []
    if node_cfg.get('items') or (kind == 'ris' and 'items' in node_cfg):
        items = [dict(item or {}) for item in list(node_cfg.get('items') or [])]

    lidar_range = node_cfg.get('lidar_range', cfg.get('lidar_range', None))
    radar_range = node_cfg.get('radar_range', cfg.get('radar_range', None))

    out = []
    for item in items:
        typ = sensor_modality(item.get('type'))
        if lidar_range is not None and typ in ('lidar_2d', 'lidar_3d'):
            item['range'] = float(lidar_range)
        if radar_range is not None and typ == 'radar':
            item['range'] = float(radar_range)
        out.append(item)
    return out
