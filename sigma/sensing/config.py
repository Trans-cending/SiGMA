"""Validate the implemented Stage II YAML contract independently of CARLA.

Use ``sigma validate --config configs/town03.yaml``. Paths are resolved by the
runner against ``config['_config_dir']``. Sample intervals are integer world
steps; per-sensor ``every_n_samples`` multiplies the shared capture stride.
Routes default to explicit endpoints; ``selection: seeded_spawn_points`` selects
map endpoints with the pipeline seed and a bounded reachability search.
"""
import math
from .schema import sensor_items_for_node, sensor_modality

BLUEPRINTS = {
    'rgb': 'sensor.camera.rgb', 'depth': 'sensor.camera.depth',
    'semantic': 'sensor.camera.semantic_segmentation',
    'lidar_2d': 'sensor.lidar.ray_cast', 'lidar_3d': 'sensor.lidar.ray_cast',
    'gps': 'sensor.other.gnss', 'imu': 'sensor.other.imu',
}


def sensor_items(config, kind, index=0):
    """Select supported paper modalities from the inherited sensor schema."""
    cfg = (config.get('sensors') or {}).get(kind) or {}
    if cfg.get('enabled') is False:
        return []
    items = sensor_items_for_node(cfg, kind, index)
    return [item for item in items if item.get('enabled', True)
            and sensor_modality(item.get('type')) in BLUEPRINTS]


def virtual_radar_items(config, kind, index=0):
    cfg = (config.get('sensors') or {}).get(kind) or {}
    if cfg.get('enabled') is False:
        return []
    return [item for item in sensor_items_for_node(cfg, kind, index)
            if item.get('enabled', True) and item.get('type') == 'radar'
            and item.get('radar_backend', item.get('backend')) == 'sionna_rcs']


def validate_stage2_config(config):
    """Return actionable validation errors; leave the supplied mapping untouched."""
    cfg = config.get('stage2', config)
    errors = []
    if not isinstance(cfg, dict):
        return ['stage2 must be a mapping']
    spec = cfg.get('map') or {}
    if not isinstance(spec, dict) or bool(spec.get('name')) == bool(spec.get('opendrive')):
        errors.append('stage2.map must specify exactly one of name or opendrive')
    elif spec.get('name') and not str(spec['name']).rsplit('/', 1)[-1].startswith('Town'):
        if spec.get('assets_version') != '0.9.16':
            errors.append('stage2.map.assets_version must be 0.9.16 for a reimported custom map')
    try:
        dt = float(cfg.get('fixed_delta_seconds', .05))
        if not math.isfinite(dt) or not 0 < dt <= .1:
            raise ValueError()
    except (TypeError, ValueError):
        errors.append('stage2.fixed_delta_seconds must be finite and in (0, 0.1]')
    for key, default in [('max_frames', 400), ('sample_every_n_frames', 1),
                         ('world_state_every_n_frames', 1), ('trajectory_id', 1)]:
        value = cfg.get(key, default)
        if isinstance(value, bool) or not isinstance(value, int) or value < 1:
            errors.append(f'stage2.{key} must be a positive integer')
    try:
        if int(cfg.get('world_state_every_n_frames', 1)) % int(cfg.get('sample_every_n_frames', 1)):
            errors.append('stage2.world_state_every_n_frames must be a multiple of sample_every_n_frames')
    except (ValueError, TypeError, ZeroDivisionError):
        pass
    route = cfg.get('route') or {}
    if not isinstance(route, dict):
        errors.append('stage2.route must be a mapping')
        route = {}
    selection = route.get('selection', 'explicit')
    if selection not in ('explicit', 'seeded_spawn_points'):
        errors.append('stage2.route.selection must be explicit or seeded_spawn_points')
    candidates = route.get('max_route_candidates', 64)
    if type(candidates) is not int or candidates < 1:
        errors.append('stage2.route.max_route_candidates must be a positive integer')
    if selection == 'seeded_spawn_points':
        if any(key in route for key in ('start', 'destination', 'via')):
            errors.append('stage2.route.seeded_spawn_points selects start/destination/via from map spawn points; configure selection alone')
        points = []
    else:
        for key in ('start', 'destination'):
            if not isinstance(route.get(key), dict):
                errors.append(f'stage2.route.{key} must contain x/y[/z] or geolocation')
        via = route.get('via') or []
        if not isinstance(via, list):
            errors.append('stage2.route.via must be a list of route points')
            via = []
        points = [route.get('start'), route.get('destination'), *via]
    for point in points:
        if not isinstance(point, dict):
            continue
        geo = point.get('geolocation')
        try:
            values = [geo['latitude'], geo['longitude'], geo.get('altitude', 0)] if geo else [point['x'], point['y'], point.get('z', 0)]
            if not all(math.isfinite(float(x)) for x in values):
                raise ValueError()
        except (ValueError, TypeError, KeyError):
            errors.append('Route points require finite x/y/z or latitude/longitude/altitude')
    if not isinstance(cfg.get('rsus', []), list):
        errors.append('stage2.rsus must be a list')
    fov = cfg.get('fov') or {}
    if fov.get('enabled'):
        if fov.get('node_kind', 'rsu') == 'ris':
            if not (config.get('ris') or {}).get('enabled'):
                errors.append('RIS FoV acquisition requires top-level ris.enabled')
        elif not cfg.get('rsus'):
            errors.append('stage2.fov.enabled requires at least one RSU')
    for kind in ('cav', 'rsu', 'ris'):
        sensor_cfg = (cfg.get('sensors') or {}).get(kind) or {}
        for item in sensor_cfg.get('items') or []:
            if (item.get('enabled', True) and sensor_modality(item.get('type')) not in BLUEPRINTS
                    and not (item.get('type') == 'radar'
                             and item.get('radar_backend', item.get('backend')) == 'sionna_rcs')):
                errors.append(f'Unsupported Stage II sensor type {item.get("type")!r}')
        try:
            for item in sensor_items(cfg, kind):
                step = item.get('every_n_samples', 1)
                if not isinstance(step, int) or isinstance(step, bool) or step < 1:
                    errors.append(f'{kind}/{item.get("name")}: every_n_samples must be a positive integer')
                if item.get('sensor_tick', 0) not in (0, 0.0, '0', '0.0'):
                    errors.append(f'{kind}/{item.get("name")}: use every_n_samples; sensor_tick must be 0 for exact-frame acquisition')
        except (TypeError, ValueError):
            errors.append(f'stage2.sensors.{kind} must contain valid sensor items')
    return errors
