"""Official CARLA sensors with bounded exact-frame callback buffers.

RGB is PNG, metric depth is float32 NPY (metres, preserving CARLA precision),
semantic labels are uint8 NPY, LiDAR is float32 NPY [x,y,z,intensity] in the
sensor's CARLA frame, and GNSS/IMU are JSON. Canonical ``time_s`` and
``sensor_timestamp`` are the absolute CARLA elapsed seconds. ``every_n_samples`` controls saved
cadence; sensor_tick=0 keeps callback frames aligned with the single world tick.
"""
from collections import OrderedDict
from pathlib import Path
import math
import threading
import time
import numpy as np
from .config import BLUEPRINTS
from .schema import safe_path_component, sensor_modality
from .writer import write_json
from .geo import correct_carla_gnss_to_site_geolocation
from .frame_sync import FrameSyncError


class ExactFrameBuffer:
    """Bound memory and reject stale substitution when an exact frame is absent."""
    def __init__(self, max_frames=8):
        self.max_frames = max(1, int(max_frames))
        self.data = OrderedDict()
        self.condition = threading.Condition()

    def put(self, data):
        with self.condition:
            self.data[int(data.frame)] = data
            while len(self.data) > self.max_frames:
                self.data.popitem(last=False)
            self.condition.notify_all()

    def get(self, frame, timeout=5.0):
        deadline = time.monotonic() + max(0, timeout)
        with self.condition:
            while int(frame) not in self.data:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise FrameSyncError(f'No sensor callback for exact CARLA frame {frame}')
                self.condition.wait(remaining)
            data = self.data.pop(int(frame))
            for old in list(self.data):
                if old < frame:
                    self.data.pop(old)
            return data


def decode_depth(raw_data, width, height):
    """Decode CARLA BGRA depth with its 24-bit, 1000 m normalized encoding."""
    bgra = np.frombuffer(raw_data, dtype=np.uint8).reshape((height, width, 4))
    packed = (bgra[..., 2].astype(np.float64) + 256.0 * bgra[..., 1]
              + 65536.0 * bgra[..., 0])
    return (packed * (1000.0 / 16777215.0)).astype(np.float32)


def _vector(value):
    return [float(value.x), float(value.y), float(value.z)]


def transform_from_config(carla, config=None):
    config = config or {}
    location = config.get('location') or [0, 0, 0]
    rotation = config.get('rotation') or [0, 0, 0]
    if isinstance(rotation, dict):
        rotation = [rotation.get('pitch', 0), rotation.get('yaw', 0), rotation.get('roll', 0)]
    return carla.Transform(carla.Location(x=float(location[0]), y=float(location[1]), z=float(location[2])),
                           carla.Rotation(pitch=float(rotation[0]), yaw=float(rotation[1]), roll=float(rotation[2])))


def compose_transform(carla, parent, relative):
    """Compose mounts through full rotation matrices, including pitch and roll."""
    matrix = np.asarray(parent.get_matrix()) @ np.asarray(relative.get_matrix())
    rotation = carla.Rotation(pitch=math.degrees(math.asin(np.clip(matrix[2, 0], -1, 1))),
                              yaw=math.degrees(math.atan2(matrix[1, 0], matrix[0, 0])),
                              roll=math.degrees(math.atan2(-matrix[2, 1], matrix[2, 2])))
    return carla.Transform(carla.Location(x=matrix[0, 3], y=matrix[1, 3], z=matrix[2, 3]), rotation)


class CarlaSensor:
    """Spawn only documented 0.9.16 blueprints; invalid attributes fail early."""
    def __init__(self, world, carla, node_label, item, *, parent=None, reference_transform=None):
        self.item = dict(item)
        self.cfg = self.item
        self.node_label = node_label
        self.modality = sensor_modality(item.get('type'))
        self.name = safe_path_component(item.get('name'), self.modality)
        self.every_n_samples = int(item.get('every_n_samples', 1))
        self.blueprint = item.get('blueprint') or BLUEPRINTS[self.modality]
        if self.blueprint != BLUEPRINTS[self.modality]:
            raise ValueError(f'{self.name}: blueprint {self.blueprint!r} does not match modality {self.modality}')
        bp = world.get_blueprint_library().find(self.blueprint)
        self.attributes = {}
        # Interpret metadata/mount keys locally and validate every remaining
        # key against the CARLA blueprint attributes.
        metadata = {'name', 'type', 'blueprint', 'enabled', 'transform', 'file_prefix',
                    'direction', 'modality', 'every_n_samples', 'depth_max_m',
                    'sync_rotation_frequency_to_sensor_tick', 'mount_z_mode',
                    'camera_height_above_sign_m'}
        requested = {k: v for k, v in item.items() if k not in metadata}
        requested['sensor_tick'] = 0.0
        if item.get('sync_rotation_frequency_to_sensor_tick') and self.modality.startswith('lidar'):
            # Match source full-revolution scan rule at the world callback rate.
            requested['rotation_frequency'] = 1.0 / float(world.get_settings().fixed_delta_seconds)
        for key, value in requested.items():
            if not bp.has_attribute(key):
                raise ValueError(f'{self.name}: official CARLA blueprint {self.blueprint} has no attribute {key!r}')
            bp.set_attribute(key, str(value).lower() if isinstance(value, bool) else str(value))
            self.attributes[key] = str(bp.get_attribute(key))
        mount = transform_from_config(carla, item.get('transform'))
        if parent is not None:
            # Preserve source bbox-top mount reference for vehicle sensors.
            bb = parent.bounding_box
            mount.location.x += float(bb.location.x)
            mount.location.y += float(bb.location.y)
            mount.location.z += float(bb.location.z + bb.extent.z)
            self.sensor = world.spawn_actor(bp, mount, attach_to=parent,
                                            attachment_type=carla.AttachmentType.Rigid)
        else:
            if reference_transform is not None:
                mount = compose_transform(carla, reference_transform, mount)
            self.sensor = world.spawn_actor(bp, mount)
        self.mount_matrix = np.asarray(mount.get_matrix()).tolist()
        self.buffer = ExactFrameBuffer()
        self.sensor.listen(self.buffer.put)

    def camera_spec(self):
        if self.modality != 'rgb':
            return None
        return {'node_label': self.node_label, 'name': self.name,
                'world_to_camera': self.sensor.get_transform().get_inverse_matrix(),
                'width': int(self.item.get('image_size_x', 1280)),
                'height': int(self.item.get('image_size_y', 720)),
                'fov': float(self.item.get('fov', 90))}

    def calibration(self):
        result = {'name': self.name, 'type': self.modality, 'blueprint': self.blueprint,
                  'node_label': self.node_label, 'mount_reference': 'bbox_top',
                  'mount_matrix': self.mount_matrix,
                  'world_matrix': self.sensor.get_transform().get_matrix(),
                  'coordinate_system': 'CARLA: X forward, Y right, Z up',
                  'attributes': self.attributes, 'every_n_samples': self.every_n_samples,
                  'config': self.item,
                  'world_transform_at_calibration': {'location': _vector(self.sensor.get_transform().location),
                     'rotation': {key: float(getattr(self.sensor.get_transform().rotation, key)) for key in ('pitch', 'yaw', 'roll')}}}
        if self.modality in ('rgb', 'depth', 'semantic'):
            width, height = int(self.item.get('image_size_x', 1280)), int(self.item.get('image_size_y', 720))
            focal = width / (2 * math.tan(math.radians(float(self.item.get('fov', 90))) / 2))
            result['intrinsic_matrix'] = [[focal, 0, width / 2], [0, focal, height / 2], [0, 0, 1]]
        return result

    def save(self, data, trajectory_dir, frame_idx, world_frame, *, site_id, trajectory_id, projection=None, trajectory_origin_time=None):
        if int(data.frame) != int(world_frame):
            raise FrameSyncError(f'{self.name}: callback frame {data.frame} differs from world {world_frame}')
        root = Path(trajectory_dir)
        folder = root / 'nodes' / self.node_label / self.modality / self.name
        folder.mkdir(parents=True, exist_ok=True)
        suffix = '.png' if self.modality == 'rgb' else ('.json' if self.modality in ('gps', 'imu') else '.npy')
        path = folder / f'{self.modality}_{int(frame_idx):06d}{suffix}'
        payload = {'name': self.name, 'type': self.modality, 'node_label': self.node_label,
                   'blueprint': self.blueprint, 'available': True, 'frame_sync_ok': True,
                   'site_id': str(site_id), 'trajectory_id': int(trajectory_id),
                   'frame_idx': int(frame_idx), 'world_frame': int(world_frame),
                   'sensor_frame': int(data.frame), 'sensor_timestamp': float(data.timestamp),
                   'timestamp': float(data.timestamp), 'time_s': float(data.timestamp),
                   'trajectory_time_s': (float(data.timestamp) - float(trajectory_origin_time)
                                         if trajectory_origin_time is not None else None),
                   'file': path.relative_to(root).as_posix()}
        if self.modality == 'rgb':
            from PIL import Image
            bgra = np.frombuffer(data.raw_data, dtype=np.uint8).reshape((data.height, data.width, 4))
            Image.fromarray(bgra[..., [2, 1, 0]]).save(path)
            payload['encoding'] = 'rgb8'
        elif self.modality == 'depth':
            np.save(path, decode_depth(data.raw_data, data.width, data.height), allow_pickle=False)
            payload['encoding'] = 'float32_metric_depth_m'
        elif self.modality == 'semantic':
            labels = np.frombuffer(data.raw_data, dtype=np.uint8).reshape((data.height, data.width, 4))[..., 2]
            np.save(path, labels, allow_pickle=False)
            payload['encoding'] = 'uint8_carla_semantic_tag'
        elif self.modality in ('lidar_2d', 'lidar_3d'):
            array = np.frombuffer(data.raw_data, dtype=np.float32).reshape((-1, 4)).copy()
            if not np.isfinite(array).all():
                raise ValueError(f'{self.name}: nonfinite LiDAR measurements')
            np.save(path, array, allow_pickle=False)
            payload.update(encoding='float32_xyzi', coordinate_reference='sensor_local_carla', point_count=len(array))
        elif self.modality == 'gps':
            raw = {key: float(getattr(data, key)) for key in ('latitude', 'longitude', 'altitude')}
            corrected = raw
            if projection:
                values, conversion = correct_carla_gnss_to_site_geolocation(**raw, projection=projection)
                corrected = dict(zip(('latitude', 'longitude', 'altitude'), values))
                payload.update(raw_carla_gnss=raw, geolocation_conversion=conversion)
            payload.update(**corrected, coordinate_reference='WGS84', measurement_source='carla_gnss_sensor')
            write_json(path, payload)
        elif self.modality == 'imu':
            payload.update(accelerometer=_vector(data.accelerometer), gyroscope=_vector(data.gyroscope),
                           compass=float(data.compass), accelerometer_unit='m/s^2', gyroscope_unit='rad/s', compass_unit='rad')
            write_json(path, payload)
        payload.update(file_written=path.is_file(), file_size_bytes=path.stat().st_size)
        if not path.stat().st_size:
            raise IOError(f'{self.name}: empty sensor output {path}')
        return payload

    def destroy(self):
        if self.sensor is not None and self.sensor.is_alive:
            self.sensor.stop()
            self.sensor.destroy()


def virtual_radar_calibration(carla, item, reference_transform, *, parent=None):
    """Export a radar mount for the Sionna ray-tracing backend.

    Stage III reads this calibration to generate multi-point RCS CIR and FMCW
    IQ. Vehicle mounts use the same source bbox-top reference as real sensors;
    the exported config transform includes that offset for downstream readers.
    """
    from copy import deepcopy
    config = deepcopy(item)
    mount = transform_from_config(carla, config.get('transform'))
    if parent is not None:
        bb = parent.bounding_box
        mount.location.x += float(bb.location.x)
        mount.location.y += float(bb.location.y)
        mount.location.z += float(bb.location.z + bb.extent.z)
        config['transform'] = {'location': _vector(mount.location),
                              'rotation': [float(mount.rotation.pitch), float(mount.rotation.yaw), float(mount.rotation.roll)]}
    world = compose_transform(carla, reference_transform, mount)
    return {'name': safe_path_component(item.get('name'), 'radar'), 'type': 'radar',
            'config': config, 'virtual': True, 'acquisition_stage': 3,
            'mount_reference': 'actor_origin' if parent is not None else 'fixed_node_reference',
            'source_mount_reference': 'bbox_top',
            'world_transform_at_calibration': {'location': _vector(world.location),
                'rotation': {key: float(getattr(world.rotation, key)) for key in ('pitch', 'yaw', 'roll')}}}
