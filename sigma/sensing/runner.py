"""Stage II: official CARLA 0.9.16 road trajectories and synchronized sensing.

Run ``sigma stage2 --config configs/town03.yaml --output outputs/site --seed 42``.
The output retains ``stage1_multimodal/<trajectory_id:06d>`` as the on-disk
contract consumed by the existing Blender/radio converters. Canonical ``time_s`` is the absolute CARLA elapsed time shared by sensors and
Stage III; ``trajectory_time_s`` is relative to the first captured world step.
``route.json`` records the resolved endpoints, route lane stations and selection
method; seeded map selection also records the seed and original spawn indices.
State YAMLs are committed only after exact-frame sensor files succeed. Failed trajectories keep
intermediate files and a failed manifest; rerunning requires a new trajectory ID
or a clean output directory, avoiding accidental overwrite of prior samples.
"""
from pathlib import Path
import csv
import json
import math
import random
import time
import warnings
import numpy as np
from .config import validate_stage2_config, sensor_items, virtual_radar_items
from .runtime import check_versions, load_world, synchronous_settings
from .routes import RoadRoutePlanner, select_seeded_spawn_route
from .fov import FieldOfView, actor_is_alive
from .geo import parse_xodr_tmerc_georeference, geolocation_to_xodr_tmerc_carla_location, geolocation_to_carla_location
from .sensors import CarlaSensor, transform_from_config, virtual_radar_calibration
from .frame_sync import capture_due, validate_sensor_payloads, FrameSyncError
from .writer import (write_json, write_yaml, build_cav_payloads, build_rsu_payloads,
                     build_ris_payloads, attach_cav_gnss_measurements, write_world_state_yaml)


def _xyz(value):
    return [float(value.x), float(value.y), float(value.z)]


def frame_identity(site_id, trajectory_id, frame_idx, world_frame, timestamp, trajectory_origin_time):
    """Canonical absolute simulation seconds plus a separate trajectory clock."""
    return {'site_id': str(site_id), 'trajectory_id': int(trajectory_id),
            'frame_idx': int(frame_idx), 'world_frame': int(world_frame),
            'timestamp': float(timestamp), 'time_s': float(timestamp),
            'trajectory_time_s': float(timestamp) - float(trajectory_origin_time)}


def _route_distance(location, route):
    point = np.asarray([location.x, location.y], dtype=float)
    best = math.inf
    for a, b in zip(route, route[1:]):
        start = np.asarray([a.x, a.y], dtype=float)
        end = np.asarray([b.x, b.y], dtype=float)
        delta = end - start
        norm = float(np.dot(delta, delta))
        fraction = np.clip(float(np.dot(point - start, delta)) / norm, 0., 1.) if norm > 1e-12 else 0.
        best = min(best, float(np.linalg.norm(point - start - fraction * delta)))
    return best


def _resolve_location(carla, carla_map, entry, projection):
    geo = entry.get('geolocation')
    if geo:
        kwargs = {key: float(geo.get(key, 0.)) for key in ('latitude', 'longitude', 'altitude')}
        if projection:
            loc, _ = geolocation_to_xodr_tmerc_carla_location(carla, **kwargs, projection=projection)
        else:
            loc, _ = geolocation_to_carla_location(carla_map, carla, **kwargs)
        return loc
    xyz = entry.get('location')
    if xyz is not None:
        return carla.Location(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))
    return carla.Location(x=float(entry.get('x', 0)), y=float(entry.get('y', 0)), z=float(entry.get('z', 0)))


def _plan_route(carla, world, route_cfg, projection, seed):
    """Resolve explicit points or select a seeded forward driving-lane route."""
    carla_map = world.get_map()
    planner = RoadRoutePlanner(world, carla, route_cfg)
    step_m = float(route_cfg.get('step_m', 2.0))
    max_projection = float(route_cfg.get('max_projection_distance_m', 3.0))
    if route_cfg.get('selection', 'explicit') == 'seeded_spawn_points':
        waypoints, metadata = select_seeded_spawn_route(
            carla_map, carla, planner, seed,
            max_route_candidates=route_cfg.get('max_route_candidates', 64),
            max_projection_distance_m=max_projection, step_m=step_m)
        return waypoints, [], metadata
    start = _resolve_location(carla, carla_map, route_cfg['start'], projection)
    destination = _resolve_location(carla, carla_map, route_cfg['destination'], projection)
    via = [_resolve_location(carla, carla_map, raw, projection) for raw in route_cfg.get('via', [])]
    for label, location in [('start', start), *[(f'via_{i}', point) for i, point in enumerate(via)], ('destination', destination)]:
        waypoint = carla_map.get_waypoint(location, project_to_road=True, lane_type=carla.LaneType.Driving)
        if waypoint is None or math.hypot(waypoint.transform.location.x - location.x, waypoint.transform.location.y - location.y) > max_projection:
            raise RuntimeError(f'Route {label} is outside the driving road projection tolerance ({max_projection} m)')
    waypoints = planner.plan(start, destination, via, step_m=step_m)
    return waypoints, via, {'selection': 'explicit', 'start_carla_m': _xyz(start),
                            'destination_carla_m': _xyz(destination)}


def _scene_vehicle(actor):
    tf, bb, velocity = actor.get_transform(), actor.bounding_box, actor.get_velocity()
    return {'bp_id': actor.type_id, 'location': _xyz(tf.location), 'center': _xyz(bb.location),
            'angle': [float(tf.rotation.roll), float(tf.rotation.yaw), float(tf.rotation.pitch)],
            'extent': _xyz(bb.extent), 'velocity': _xyz(velocity),
            'speed': float(math.sqrt(sum(v * v for v in _xyz(velocity))))}


def _resolve_height(carla, carla_map, location, mode='absolute', configured_height=None):
    """Preserve source nearest-driving-road AGL reference, rejecting missing roads."""
    if mode == 'absolute':
        return location, None
    if mode != 'relative_to_ground':
        raise ValueError('Infrastructure z_mode must be absolute or relative_to_ground')
    waypoint = carla_map.get_waypoint(location, project_to_road=True, lane_type=carla.LaneType.Driving)
    if waypoint is None:
        raise ValueError('Cannot resolve relative_to_ground infrastructure height without a driving road')
    ground_z = float(waypoint.transform.location.z)
    height = float(location.z if configured_height is None else configured_height)
    return carla.Location(x=float(location.x), y=float(location.y), z=ground_z + height), ground_z


def _static_nodes(carla, carla_map, cfg, projection):
    nodes = []
    for index, raw in enumerate(cfg.get('rsus') or []):
        configured_location = _resolve_location(carla, carla_map, raw, projection)
        height = (raw.get('geolocation') or {}).get('altitude', configured_location.z)
        location, ground_z = _resolve_height(carla, carla_map, configured_location, raw.get('z_mode', 'absolute'), height)
        tf = transform_from_config(carla, {'location': _xyz(location), 'rotation': raw.get('rotation', [0, 0, 0])})
        nodes.append({'label': f'rsu_{index+1:03d}', 'location': location,
                      'config_location': configured_location, 'reference_location': location,
                      'z_mode': raw.get('z_mode', 'absolute'), 'resolved_ground_z': ground_z,
                      'rotation': tf.rotation, 'transform': tf,
                      'bbox_top_alignment_verified': True,
                      'bbox_top_alignment_state': 'fixed_georeferenced_reference_no_marker'})
    return nodes


def _ris_nodes(carla, carla_map, config, projection, dataset_root):
    if not (config.get('ris') or {}).get('enabled'):
        return [], None
    from mosaic_core.ris import normalize_ris_config, panel_geometry, element_world_coordinates, ris_config_hash
    cfg = normalize_ris_config(config['ris'])
    nodes, metadata = [], {'schema_version': 1, 'config': cfg, 'nodes': {}}
    for index, raw in enumerate(cfg['nodes']):
        geo = {key: raw[key] for key in ('latitude', 'longitude', 'altitude')}
        location = _resolve_location(carla, carla_map, {'geolocation': geo}, projection)
        location, ground_z = _resolve_height(carla, carla_map, location, raw.get('z_mode', 'absolute'), geo['altitude'])
        geom = panel_geometry(cfg, index)
        label, rotation = raw['id'], raw['rotation']
        transform = carla.Transform(location, carla.Rotation(**rotation))
        node = {'label': label, 'location': location, 'rotation_dict': rotation,
                'rotation': transform.rotation, 'transform': transform,
                'config_hash': ris_config_hash(cfg), 'visualization_geometry': 'none'}
        nodes.append(node)
        metadata['nodes'][label] = {
            'node_id': label, 'geolocation': geo, 'location_carla_m': _xyz(location),
            'z_mode': raw.get('z_mode', 'absolute'), 'resolved_ground_z': ground_z,
            'rotation_carla_deg': rotation, 'panel_geometry': geom.as_dict(),
            'element_world_coordinates_carla_m': element_world_coordinates(_xyz(location), rotation, geom).tolist(),
            'config_hash': node['config_hash'], 'ordinary_rt_geometry': False,
            'visualization_geometry': 'none'}
    path = dataset_root / 'ris_metadata.yaml'
    write_yaml(path, metadata)
    return nodes, '../ris_metadata.yaml'


def _annotations(fov, sensors, actors, trajectory_dir, ids):
    for sensor in sensors:
        if sensor.modality != 'rgb':
            continue
        objects = []
        for actor in actors:
            if not actor_is_alive(actor):
                continue
            bbox = fov.bbox(sensor, actor)
            if bbox:
                objects.append({'object_id': f'vehicle_{int(actor.id):06d}', 'actor_id': int(actor.id),
                                'class_id': 0, **bbox})
        path = trajectory_dir / 'annotations' / 'camera_bbox2d' / sensor.node_label / sensor.name / 'bbox.jsonl'
        path.parent.mkdir(parents=True, exist_ok=True)
        record = dict(ids, image_size=[int(sensor.item.get('image_size_x', 1280)), int(sensor.item.get('image_size_y', 720))],
                      objects=objects, visibility_semantics='projected image clipping fraction; does not infer occlusion')
        with path.open('a', encoding='utf-8') as stream:
            stream.write(json.dumps(record, allow_nan=False) + '\n')


def run_stage2(config, output_dir, seed=0):
    """Collect one configured route; return its manifest path or raise clearly."""
    errors = validate_stage2_config(config)
    if errors:
        raise ValueError('; '.join(errors))
    cfg = config.get('stage2', config)
    try:
        import carla
    except ImportError as exc:
        raise RuntimeError('Stage II requires the official carla==0.9.16 wheel for Python 3.10–3.12') from exc
    output_dir = Path(output_dir).resolve()
    dataset_root = output_dir / 'stage1_multimodal'
    trajectory_id = int(cfg.get('trajectory_id', 1))
    trajectory_dir = dataset_root / f'{trajectory_id:06d}'
    if trajectory_dir.exists() and any(trajectory_dir.iterdir()):
        raise FileExistsError(f'Trajectory output already exists: {trajectory_dir}; use a new trajectory_id or output')
    trajectory_dir.mkdir(parents=True, exist_ok=True)
    manifest = trajectory_dir / 'manifest.json'
    site_id = str(config.get('site_id') or (config.get('site') or {}).get('id') or 'site')
    status = {'schema_version': 1, 'stage': 2, 'status': 'running', 'site_id': site_id,
              'trajectory_id': trajectory_id, 'seed': int(seed), 'samples': 0, 'frames': []}
    write_json(manifest, status)
    rng = random.Random(int(seed))
    client = carla.Client(str(cfg.get('host', '127.0.0.1')), int(cfg.get('port', 2000)))
    client.set_timeout(float(cfg.get('timeout', 20.0)))
    sensors, actors = [], []
    world, previous, tm = None, None, None
    try:
        status['carla_versions'] = check_versions(client)
        world = load_world(client, carla, cfg, config.get('_config_dir', '.'))
        dt = float(cfg.get('fixed_delta_seconds', .05))
        previous = synchronous_settings(world, dt)
        tm_port = int(cfg.get('traffic_manager_port', 8000))
        tm = client.get_trafficmanager(tm_port)
        tm.set_synchronous_mode(True)
        tm.set_random_device_seed(int(seed))
        tm.set_hybrid_physics_mode(False)
        carla_map = world.get_map()
        projection = parse_xodr_tmerc_georeference(carla_map.to_opendrive())
        # Apply source south-positive tmerc conversion only when declared by
        # the site configuration. Town maps keep native CARLA GNSS semantics.
        site_projection = projection if cfg.get('georeference_y_south', False) else None
        route_cfg = cfg['route']
        max_projection = float(route_cfg.get('max_projection_distance_m', 3.0))
        route_waypoints, via, route_selection = _plan_route(carla, world, route_cfg, site_projection, seed)
        route = [wp.transform.location for wp in route_waypoints]
        write_json(trajectory_dir / 'route.json', {'site_id': site_id, 'trajectory_id': trajectory_id,
            **route_selection,
            'waypoints_carla_m': [_xyz(loc) for loc in route], 'ordered_via_carla_m': [_xyz(loc) for loc in via],
            'lane_stations': [[int(wp.road_id), int(wp.section_id), int(wp.lane_id), float(wp.s)] for wp in route_waypoints]})
        blueprint_library = world.get_blueprint_library()
        vehicle_cfg = cfg.get('vehicles') or {}
        blueprint = blueprint_library.find(vehicle_cfg.get('blueprint', 'vehicle.tesla.model3'))
        blueprint.set_attribute('role_name', 'sigma_cav_001')
        spawn = route_waypoints[0].transform
        spawn.location.z += .5
        selected = world.try_spawn_actor(blueprint, spawn)
        if selected is None:
            raise RuntimeError('Selected vehicle could not spawn on its route start (occupied or invalid road)')
        actors.append(selected)
        selected.set_autopilot(True, tm_port)
        tm.auto_lane_change(selected, False)
        tm.set_path(selected, route)
        tm.set_desired_speed(selected, float(route_cfg.get('speed_kmh', 20)))
        spawn_points = list(carla_map.get_spawn_points())
        rng.shuffle(spawn_points)
        candidates = sorted(blueprint_library.filter('vehicle.*'), key=lambda bp: bp.id)
        requested_background = int(vehicle_cfg.get('background_count', 0))
        for transform in spawn_points:
            if len(actors) - 1 >= requested_background:
                break
            if transform.location.distance(spawn.location) < float(vehicle_cfg.get('spawn_separation_m', 8)):
                continue
            bp = rng.choice(candidates)
            bp.set_attribute('role_name', 'sigma_background')
            actor = world.try_spawn_actor(bp, transform)
            if actor is not None:
                actors.append(actor)
                actor.set_autopilot(True, tm_port)
        if len(actors) - 1 != requested_background:
            raise RuntimeError(f'Only {len(actors)-1}/{requested_background} background vehicles could spawn; lower background_count')
        rsu_nodes = _static_nodes(carla, carla_map, cfg, site_projection)
        ris_nodes, ris_file = _ris_nodes(carla, carla_map, config, site_projection, dataset_root)
        for item in sensor_items(cfg, 'cav'):
            sensors.append(CarlaSensor(world, carla, 'cav_001', item, parent=selected))
        for kind, nodes in [('rsu', rsu_nodes), ('ris', ris_nodes)]:
            for index, node in enumerate(nodes):
                for item in sensor_items(cfg, kind, index):
                    sensors.append(CarlaSensor(world, carla, node['label'], item, reference_transform=node['transform']))
        if not sensors:
            raise ValueError('Stage II requires at least one enabled sensor')
        names = [(sensor.node_label, sensor.name) for sensor in sensors]
        if len(set(names)) != len(names):
            raise ValueError('Sensor names must be unique within each node')
        fov = FieldOfView(carla, cfg.get('fov'))
        fov_cfg = cfg.get('fov') or {}
        fov_kind = str(fov_cfg.get('node_kind', 'rsu'))
        view_sensors = [sensor for sensor in sensors if sensor.node_label.startswith(fov_kind + '_') and sensor.modality == 'rgb']
        if fov_cfg.get('enabled') and not view_sensors:
            raise ValueError(f'FoV acquisition requires an enabled {fov_kind} RGB camera')
        registry = {f'vehicle_{int(actor.id):06d}': {'actor_id': int(actor.id), 'bp_id': actor.type_id,
                    'class_id': 0, 'selected_for_communication': actor.id == selected.id,
                    'communication_node_id': 'cav_001' if actor.id == selected.id else None} for actor in actors}
        write_yaml(trajectory_dir / 'object_registry.yaml', {'schema_version': 1, 'objects': registry})
        write_yaml(dataset_root / 'classes.yaml', {'classes': {0: 'vehicle', 3: 'rsu'}})
        write_yaml(dataset_root / 'sensor_config.yaml', {'schema_version': 1, 'sensors': cfg.get('sensors', {}),
                   'simulation': {'fixed_delta_seconds': dt}, 'ris': config.get('ris', {'enabled': False})})
        sample_stride = int(cfg.get('sample_every_n_frames', 1))
        world_stride = int(cfg.get('world_state_every_n_frames', 1))
        outside_count, origin, first_timestamp = 0, None, None
        calibrated = False
        stop_reason = 'max_frames'
        csv_path = trajectory_dir / 'samples.csv'
        with csv_path.open('w', newline='', encoding='utf-8') as stream:
            csv_writer = csv.DictWriter(stream, fieldnames=['site_id', 'trajectory_id', 'frame_idx', 'world_frame', 'timestamp', 'time_s', 'trajectory_time_s', 'world_state'])
            csv_writer.writeheader()
            for frame_idx in range(int(cfg.get('max_frames', 400))):
                world_frame = int(world.tick(float(cfg.get('timeout', 20))))
                snapshot = world.get_snapshot()
                if int(snapshot.frame) != world_frame:
                    raise FrameSyncError('Another client advanced the CARLA world; SiGMA must own all ticks')
                timestamp = float(snapshot.timestamp.elapsed_seconds)
                if first_timestamp is None:
                    first_timestamp = timestamp
                if not calibrated:
                    for kind, nodes in [('cav', [{'label': 'cav_001', 'transform': selected.get_transform(), 'actor': selected}]),
                                        ('rsu', rsu_nodes), ('ris', ris_nodes)]:
                        for index, node in enumerate(nodes):
                            label = node['label']
                            entries = {s.name: s.calibration() for s in sensors if s.node_label == label}
                            for item in virtual_radar_items(cfg, kind, index):
                                entry = virtual_radar_calibration(carla, item, node['transform'], parent=node.get('actor'))
                                if entry['name'] in entries:
                                    raise ValueError(f'Duplicate real/virtual sensor name: {label}/{entry["name"]}')
                                entries[entry['name']] = entry
                            write_yaml(trajectory_dir / 'calibration' / f'{label}.yaml', {'schema_version': 1,
                                       'node_id': label, 'sensors': entries})
                    calibrated = True
                location = selected.get_transform().location
                road_wp = carla_map.get_waypoint(location, project_to_road=True, lane_type=carla.LaneType.Driving)
                if road_wp is None or math.hypot(location.x-road_wp.transform.location.x, location.y-road_wp.transform.location.y) > max_projection:
                    raise RuntimeError(f'Communication vehicle left the driving road at frame {world_frame}')
                if _route_distance(location, route) > float(route_cfg.get('max_route_deviation_m', 4.0)):
                    raise RuntimeError(f'Communication vehicle deviated from its authoritative route at frame {world_frame}')
                specs = [s.camera_spec() for s in view_sensors]
                inside, _ = fov.hit(selected, specs, margin_px=float(fov_cfg.get('margin_px', 0))) if fov_cfg.get('enabled') else (True, None)
                if origin is None:
                    entry, hit = fov.hit(selected, specs, require_entry_edge=bool(fov_cfg.get('require_entry_edge', False))) if fov_cfg.get('enabled') else (True, None)
                    if entry and hit:
                        if fov_cfg.get('entry_side'):
                            entry = hit.get('entry_side') == fov_cfg['entry_side']
                        if fov_cfg.get('motion_direction'):
                            entry = entry and hit.get('motion_direction') == fov_cfg['motion_direction']
                        if fov_cfg.get('require_entry_heading_alignment', False):
                            wp = min(route_waypoints, key=lambda x: x.transform.location.distance(location))
                            yaw_error = math.radians(selected.get_transform().rotation.yaw - wp.transform.rotation.yaw)
                            entry = entry and math.cos(yaw_error) >= float(fov_cfg.get('entry_heading_min_dot', .5))
                    if entry:
                        origin = frame_idx
                    elif frame_idx + 1 >= int(fov_cfg.get('entry_timeout_frames', cfg.get('max_frames', 400))):
                        raise RuntimeError('Selected vehicle never entered the configured camera FoV before timeout')
                elif fov_cfg.get('enabled'):
                    outside_count = 0 if inside else outside_count + 1
                    if outside_count >= max(1, int(fov_cfg.get('stop_after_exit_frames', 1))):
                        stop_reason = 'camera_fov_exit'
                        break
                if not capture_due(frame_idx, origin, sample_stride):
                    continue
                sample_index = (frame_idx - origin) // sample_stride
                captured = {}
                due_sensors = [s for s in sensors if sample_index % s.every_n_samples == 0]
                for sensor in due_sensors:
                    measurement = sensor.buffer.get(world_frame, float(cfg.get('sensor_timeout', 5.0)))
                    if not math.isclose(float(measurement.timestamp), timestamp, rel_tol=0, abs_tol=max(1e-5, dt * 1e-3)):
                        raise FrameSyncError(f'{sensor.name}: sensor timestamp differs from snapshot for frame {world_frame}')
                    payload = sensor.save(measurement, trajectory_dir, frame_idx, world_frame,
                                          site_id=site_id, trajectory_id=trajectory_id, projection=site_projection,
                                          trajectory_origin_time=first_timestamp)
                    captured[f'{sensor.node_label}/{sensor.name}'] = payload
                validate_sensor_payloads((('sensors', captured),), world_frame)
                for payload in captured.values():
                    payload['sensor_sample_index'] = sample_index // next(s.every_n_samples for s in due_sensors if s.node_label == payload['node_label'] and s.name == payload['name'])
                with (trajectory_dir / 'sensor_samples.jsonl').open('a', encoding='utf-8') as sample_stream:
                    sample_stream.write(json.dumps({'site_id': site_id, 'trajectory_id': trajectory_id,
                        'frame_idx': frame_idx, 'world_frame': world_frame, 'timestamp': timestamp,
                        'time_s': timestamp, 'trajectory_time_s': timestamp-first_timestamp, 'sensors': captured}, allow_nan=False) + '\n')
                ids = frame_identity(site_id, trajectory_id, frame_idx, world_frame, timestamp, first_timestamp)
                _annotations(fov, due_sensors, actors, trajectory_dir, ids)
                if not capture_due(frame_idx, origin, world_stride):
                    continue
                def write_pose(label, payload, idx):
                    path = trajectory_dir / 'nodes' / label / 'pose' / f'pose_{idx:06d}.json'
                    write_json(path, dict(payload, **ids))
                    return path.relative_to(trajectory_dir).as_posix()
                cavs = build_cav_payloads([selected], actor_is_alive, write_pose, frame_idx)
                attach_cav_gnss_measurements(cavs, captured, world_frame)
                vehicles = {int(actor.id): _scene_vehicle(actor) for actor in world.get_actors().filter('vehicle.*') if actor_is_alive(actor)}
                # The Blender loader consumes these source-schema fields; uav
                # stays empty and airground groups the captured sidecar sensors.
                state = dict(ids, schema_version=1, map=carla_map.name, source_map=carla_map.name,
                             cadence={'source': 'carla_world_tick', 'fixed_delta_seconds': dt,
                                      'world_state_every_n_frames': world_stride, 'sensor_every_n_world_frames': sample_stride,
                                      'has_sidecar_sensor_capture': True, 'sensor_sample_index': sample_index},
                             nodes={'rsu': build_rsu_payloads(rsu_nodes, actor_is_alive, write_pose, frame_idx),
                                    'cav': cavs, 'ris': build_ris_payloads(ris_nodes, ris_file), 'static_rx': {}, 'uav': {}},
                             airground={'has_sidecar_sensor_capture': True,
                                        'vehicle_sidecar_sensors': {key: value for key, value in captured.items() if value['node_label'].startswith('cav_')},
                                        'rsu_sensors': {key: value for key, value in captured.items() if value['node_label'].startswith('rsu_')},
                                        'ris_sensors': {key: value for key, value in captured.items() if value['node_label'].startswith('ris_')}},
                             scene={'vehicles': vehicles}, sensors=captured)
                path = Path(write_world_state_yaml(trajectory_dir / 'world_state', trajectory_id, frame_idx, world_frame, state))
                csv_writer.writerow(dict(ids, world_state=path.relative_to(trajectory_dir).as_posix()))
                stream.flush()
                status['samples'] += 1
                status['frames'].append(dict(ids, world_state=path.relative_to(trajectory_dir).as_posix()))
                write_json(manifest, status)
                if location.distance(route[-1]) <= float(route_cfg.get('destination_tolerance_m', 2)):
                    stop_reason = 'route_destination'
                    break
        if status['samples'] == 0:
            raise RuntimeError('Trajectory produced no synchronized samples')
        status.update(status='complete', stop_reason=stop_reason)
        write_json(manifest, status)
        write_yaml(trajectory_dir / 'trajectory_meta.yaml', dict(status, frames=None))
        return manifest
    except BaseException as exc:
        status.update(status='failed', error=f'{type(exc).__name__}: {exc}')
        write_json(manifest, status)
        raise
    finally:
        for sensor in reversed(sensors):
            try:
                sensor.destroy()
            except RuntimeError as exc:
                warnings.warn(f'CARLA sensor cleanup failed: {exc}', RuntimeWarning)
        for actor in reversed(actors):
            try:
                if actor_is_alive(actor):
                    actor.destroy()
            except RuntimeError as exc:
                warnings.warn(f'CARLA actor cleanup failed: {exc}', RuntimeWarning)
        if tm is not None:
            try:
                tm.set_synchronous_mode(False)
            except RuntimeError as exc:
                warnings.warn(f'CARLA traffic manager cleanup failed: {exc}', RuntimeWarning)
        if world is not None and previous is not None:
            try:
                world.apply_settings(previous)
            except RuntimeError as exc:
                warnings.warn(f'CARLA world settings restore failed: {exc}', RuntimeWarning)
