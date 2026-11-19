"""Frame-state payloads preserving the source stage1_multimodal wire schema.

Stage II outputs ``stage1_multimodal/000001/world_state/*.yaml`` so existing
Blender and radio converters consume synchronized simulation actor geometry
and retain sensor measurements as separate payloads.
"""
import json
import math
import os
from pathlib import Path
import yaml
from .schema import node_label


def atomic_write_text(path, text):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name(path.name + '.tmp')
    temp.write_text(text, encoding='utf-8')
    temp.replace(path)


def write_yaml(path, payload):
    atomic_write_text(path, yaml.safe_dump(payload, sort_keys=False))


def write_json(path, payload):
    atomic_write_text(path, json.dumps(payload, indent=2, allow_nan=False))

class CustomDumper(yaml.Dumper):
    def increase_indent(self, flow=False, indentless=False):
        return super(CustomDumper, self).increase_indent(flow, False)

def build_rsu_payloads(rsu_nodes, actor_is_alive_fn, write_pose_fn, frame_idx):
    out = {}
    for i, node in enumerate(list(rsu_nodes or [])):
        actor = node.get('actor')
        label = node.get('label') or node_label('rsu', i)
        payload = {
            'node_id': label,
            'node_type': 'rsu',
            'index': int(i),
            'actor_id': int(actor.id) if actor_is_alive_fn(actor) else None,
            'location': [float(node['location'].x), float(node['location'].y), float(node['location'].z)],
            'rotation': {
                'pitch': float(node['rotation'].pitch),
                'yaw': float(node['rotation'].yaw),
                'roll': float(node['rotation'].roll),
            },
            'config_location': [float(node['config_location'].x), float(node['config_location'].y), float(node['config_location'].z)],
            'reference_location': [float(node['reference_location'].x), float(node['reference_location'].y), float(node['reference_location'].z)],
            'height_reference': 'bbox_top',
            'z_mode': str(node.get('z_mode', 'absolute')),
            'coordinate_system': str(node.get('coordinate_system', 'carla')),
        }
        if node.get('actor_location') is not None:
            actor_loc = node['actor_location']
            payload['actor_location'] = [float(actor_loc.x), float(actor_loc.y), float(actor_loc.z)]
        if node.get('actor_bbox_top_rel_z') is not None:
            payload['actor_bbox_top_rel_z'] = float(node['actor_bbox_top_rel_z'])
        if node.get('configured_actor_rotation') is not None:
            payload['configured_actor_rotation'] = dict(node.get('configured_actor_rotation') or {})
        if node.get('actual_actor_rotation') is not None:
            payload['actual_actor_rotation'] = dict(node.get('actual_actor_rotation') or {})
        if node.get('actor_rotation_error_deg') is not None:
            payload['actor_rotation_error_deg'] = float(node['actor_rotation_error_deg'])
        if node.get('actor_rotation_verified') is not None:
            payload['actor_rotation_verified'] = bool(node['actor_rotation_verified'])
        if node.get('actor_bbox_top_rel_location') is not None:
            payload['actor_bbox_top_rel_location'] = [
                float(v) for v in node['actor_bbox_top_rel_location']]
        if node.get('actual_bbox_top_location') is not None:
            actual_top = node['actual_bbox_top_location']
            payload['actual_bbox_top_location'] = [
                float(actual_top.x), float(actual_top.y), float(actual_top.z)]
        if node.get('bbox_top_alignment_error_m') is not None:
            payload['bbox_top_alignment_error_m'] = float(node['bbox_top_alignment_error_m'])
        payload['bbox_top_alignment_verified'] = bool(
            node.get('bbox_top_alignment_verified', False))
        if node.get('bbox_top_alignment_state') is not None:
            payload['bbox_top_alignment_state'] = str(node['bbox_top_alignment_state'])
        if node.get('face_yaw') is not None:
            payload['face_yaw'] = float(node['face_yaw'])
        if node.get('geolocation') is not None:
            geo = dict(node.get('geolocation') or {})
            payload['geolocation'] = {
                'latitude': float(geo.get('latitude')),
                'longitude': float(geo.get('longitude')),
                'altitude': float(geo.get('altitude')),
            }
        if node.get('geolocation_conversion') is not None:
            payload['geolocation_conversion'] = node.get('geolocation_conversion')
        pose_file = write_pose_fn(label, payload, frame_idx)
        if pose_file:
            payload['pose_file'] = pose_file
        out[label] = payload
    return out

def build_cav_payloads(selected_vehicles, actor_is_alive_fn, write_pose_fn, frame_idx):
    """CAV node payloads for world_state.

    NOTE: RF phase-center placement is a stage-3 (Sionna) concern. Stage1
    writes actor pose and bounding-box truth only; stage3 computes the RX
    position from ``location`` + bounding-box top (center.z + extent.z) +
    RX_ANTENNA_HEIGHT_M.
    """
    out = {}
    for i, veh in enumerate(list(selected_vehicles or [])):
        label = node_label('cav', i)
        if not actor_is_alive_fn(veh):
            out[label] = {'node_id': label, 'node_type': 'cav', 'index': int(i), 'available': False}
            continue
        tf = veh.get_transform()
        vel = veh.get_velocity()
        bbox_payload = None
        bbox_top_location = None
        try:
            bb = veh.bounding_box
            bbox_payload = {
                'location': [float(bb.location.x), float(bb.location.y), float(bb.location.z)],
                'extent': [float(bb.extent.x), float(bb.extent.y), float(bb.extent.z)],
            }
            top_rel = tf.location.__class__(
                x=float(bb.location.x),
                y=float(bb.location.y),
                z=float(bb.location.z) + float(bb.extent.z),
            )
            bbox_top_location = tf.transform(top_rel)
        except Exception:
            bbox_payload = None
            bbox_top_location = None
        payload = {
            'node_id': label,
            'node_type': 'cav',
            'index': int(i),
            'actor_id': int(veh.id),
            'bp_id': getattr(veh, 'type_id', 'unknown'),
            'location': [float(tf.location.x), float(tf.location.y), float(tf.location.z)],
            'rotation': {
                'pitch': float(tf.rotation.pitch),
                'yaw': float(tf.rotation.yaw),
                'roll': float(tf.rotation.roll),
            },
            'bounding_box': bbox_payload,
            'bbox_top_location': (
                [float(bbox_top_location.x), float(bbox_top_location.y), float(bbox_top_location.z)]
                if bbox_top_location is not None else None
            ),
            'height_reference': 'bbox_top',
            'velocity': [float(vel.x), float(vel.y), float(vel.z)],
            'speed': float(math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)),
        }
        pose_file = write_pose_fn(label, payload, frame_idx)
        if pose_file:
            payload['pose_file'] = pose_file
        out[label] = payload
    return out

def attach_cav_gnss_measurements(cav_payloads, sensor_payloads, world_frame):
    """Attach exact-frame CARLA GNSS measurements to CAV world-state nodes.

    Actor transforms remain under ``location``; latitude/longitude comes from
    the exact-frame CARLA GNSS sensor measurement.
    """
    expected_frame = int(world_frame)
    attached = set()

    for sensor in (sensor_payloads or {}).values():
        if not isinstance(sensor, dict):
            continue
        if str(sensor.get('type', '')).lower() not in ('gps', 'gnss'):
            continue
        node_label_value = str(sensor.get('node_label') or '')
        node = (cav_payloads or {}).get(node_label_value)
        if node is None:
            continue
        if node_label_value in attached:
            raise RuntimeError(f'{node_label_value}: multiple GNSS measurements for one world frame')
        sensor_frame = int(sensor.get('sensor_frame', -1))
        payload_world_frame = int(sensor.get('world_frame', -1))
        if (sensor.get('available') is False or sensor.get('error')
                or sensor.get('frame_sync_ok') is not True
                or sensor_frame != expected_frame
                or payload_world_frame != expected_frame):
            raise RuntimeError(
                f'{node_label_value}: GNSS is not synchronized to world_frame={expected_frame}'
            )
        coords = tuple(float(sensor.get(key)) for key in ('latitude', 'longitude', 'altitude'))
        if not all(math.isfinite(value) for value in coords):
            raise RuntimeError(f'{node_label_value}: GNSS contains non-finite coordinates')
        if sensor.get('measurement_source') != 'carla_gnss_sensor':
            raise RuntimeError(f'{node_label_value}: GPS payload is not from the CARLA GNSS sensor')

        geolocation = {
            'latitude': coords[0],
            'longitude': coords[1],
            'altitude': coords[2],
        }
        node['geolocation'] = geolocation
        node['geolocation_source'] = 'carla_gnss_sensor'
        node['gnss_measurement'] = {
            **geolocation,
            'coordinate_reference': str(sensor.get('coordinate_reference') or 'WGS84'),
            'sensor_name': str(sensor.get('name') or ''),
            'blueprint': str(sensor.get('blueprint') or 'sensor.other.gnss'),
            'world_frame': payload_world_frame,
            'sensor_frame': sensor_frame,
            'sensor_timestamp': float(sensor.get('sensor_timestamp', 0.0)),
            'file': sensor.get('file'),
            'measurement_source': 'carla_gnss_sensor',
        }
        raw = sensor.get('raw_carla_gnss')
        if isinstance(raw, dict):
            node['gnss_measurement']['raw_carla_gnss'] = dict(raw)
        conversion = sensor.get('geolocation_conversion')
        if isinstance(conversion, dict):
            node['geolocation_conversion'] = dict(conversion)
            node['gnss_measurement']['geolocation_conversion'] = dict(conversion)
        attached.add(node_label_value)
    return cav_payloads

def write_world_state_yaml(world_state_dir, trajectory_id, frame_idx, world_frame, payload):
    path = os.path.join(
        world_state_dir,
        f"world_state_{int(frame_idx):06d}.yaml")
    atomic_write_text(
        path,
        yaml.dump(payload, Dumper=CustomDumper, default_flow_style=False, sort_keys=False))
    return path

def build_ris_payloads(ris_nodes, metadata_file=None):
    """Build compact per-frame references to fixed RIS infrastructure."""
    out = {}
    for node in list(ris_nodes or []):
        loc = node.get('location')
        if loc is None:
            continue
        label = str(node.get('label') or node.get('id') or 'ris_001')
        out[label] = {
            'node_id': label,
            'node_type': 'ris',
            'static': True,
            'location': [float(loc.x), float(loc.y), float(loc.z)],
            'rotation': dict(node.get('rotation_dict') or {}),
            'config_hash': str(node.get('config_hash') or ''),
            'metadata_file': metadata_file,
            'ordinary_rt_geometry': False,
            'visualization_geometry': str(
                node.get('visualization_geometry') or 'unavailable'),
        }
    return out
