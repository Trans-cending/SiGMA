"""Stage II unit contracts using CARLA-compatible fixtures."""
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch
import numpy as np
import yaml
from sigma.sensing.config import validate_stage2_config, sensor_items, virtual_radar_items
from sigma.sensing.runtime import check_versions, load_world, synchronous_settings
from sigma.sensing.sensors import CarlaSensor, ExactFrameBuffer, decode_depth, virtual_radar_calibration
from sigma.sensing.frame_sync import FrameSyncError, capture_due, validate_sensor_payloads
from sigma.sensing.fov import FieldOfView
from sigma.sensing.routes import RoadRoutePlanner
from sigma.sensing.runner import _resolve_location, _route_distance, _resolve_height, frame_identity
from sigma.sensing.writer import attach_cav_gnss_measurements
from sigma.sensing.geo import geolocation_to_xodr_tmerc_carla_location, xodr_tmerc_carla_location_to_geolocation


class Location:
    def __init__(self, x=0., y=0., z=0.):
        self.x, self.y, self.z = x, y, z

    def distance(self, other):
        return math.sqrt((self.x-other.x)**2+(self.y-other.y)**2+(self.z-other.z)**2)

    def __sub__(self, other):
        return Location(self.x-other.x, self.y-other.y, self.z-other.z)


class Rotation:
    def __init__(self, pitch=0., yaw=0., roll=0.):
        self.pitch, self.yaw, self.roll = pitch, yaw, roll


class Transform:
    def __init__(self, location=None, rotation=None):
        self.location, self.rotation = location or Location(), rotation or Rotation()

    def get_matrix(self):
        from mosaic_core.ris import carla_rotation_matrix
        matrix = np.eye(4)
        matrix[:3, :3] = carla_rotation_matrix([self.rotation.pitch, self.rotation.yaw, self.rotation.roll])
        matrix[:3, 3] = [self.location.x, self.location.y, self.location.z]
        return matrix.tolist()

    def get_inverse_matrix(self):
        return np.linalg.inv(self.get_matrix()).tolist()

    def transform(self, point):
        result = np.asarray(self.get_matrix()) @ [point.x, point.y, point.z, 1.]
        return Location(*result[:3])


CARLA = NS(Location=Location, Vector3D=Location, Rotation=Rotation, Transform=Transform,
           LaneType=NS(Driving=1))


class Waypoint:
    def __init__(self, x, y=0., station=0.):
        self.transform = Transform(Location(x, y))
        self.road_id, self.section_id, self.lane_id, self.s = 1, 0, -1, station
        self.is_junction, self.lane_type = False, 1
        self.children, self.parents = [], []

    def next(self, step):
        return self.children

    def previous(self, step):
        return self.parents

    def get_left_lane(self):
        return None

    def get_right_lane(self):
        return None


def planner(nodes):
    carla_map = NS(get_waypoint=lambda loc, **kwargs: min(nodes, key=lambda wp: wp.transform.location.distance(loc)))
    return RoadRoutePlanner(NS(get_map=lambda: carla_map), CARLA)


def connect(a, b):
    a.children.append(b)
    b.parents.append(a)


def config():
    return {'stage2': {'map': {'name': 'Town01'},
        'route': {'start': {'x': 0, 'y': 0}, 'destination': {'x': 20, 'y': 0}}}}


class SensingContracts(unittest.TestCase):
    def test_exact_frame_buffer_rejects_stale_and_bounds_memory(self):
        buffer = ExactFrameBuffer(max_frames=2)
        for frame in [10, 11, 12]:
            buffer.put(NS(frame=frame))
        self.assertEqual(list(buffer.data), [11, 12])
        with self.assertRaisesRegex(FrameSyncError, 'exact CARLA frame 10'):
            buffer.get(10, timeout=0)
        self.assertEqual(buffer.get(12, timeout=0).frame, 12)
        self.assertEqual(list(buffer.data), [])

    def test_canonical_time_is_absolute_even_when_trajectory_starts_late(self):
        identity = frame_identity('site_a', 2, 7, 42, 123.45, 120.)
        self.assertEqual(identity['time_s'], 123.45)
        self.assertEqual(identity['timestamp'], identity['time_s'])
        self.assertAlmostEqual(identity['trajectory_time_s'], 3.45)
        sensor = CarlaSensor.__new__(CarlaSensor)
        sensor.modality = sensor.name = 'gps'
        sensor.node_label, sensor.blueprint = 'cav_001', 'sensor.other.gnss'
        data = NS(frame=42, timestamp=123.45, latitude=33.4, longitude=-111.9, altitude=5.)
        with tempfile.TemporaryDirectory() as tmp:
            payload = sensor.save(data, tmp, 7, 42, site_id='site_a', trajectory_id=2,
                                  trajectory_origin_time=120.)
            self.assertEqual(payload['sensor_timestamp'], identity['time_s'])
            self.assertEqual(payload['time_s'], identity['time_s'])
            self.assertAlmostEqual(payload['trajectory_time_s'], identity['trajectory_time_s'])

    def test_post_fov_cadence_uses_accepted_origin(self):
        self.assertFalse(capture_due(10, None, 3))
        self.assertTrue(capture_due(8, 5, 3))
        self.assertFalse(capture_due(9, 5, 3))
        self.assertFalse(capture_due(2, 5, 3))

    def test_depth_metric_endpoints(self):
        raw = bytes([0, 0, 0, 255, 255, 255, 255, 255, 0, 0, 255, 255])
        array = decode_depth(raw, 3, 1)
        self.assertEqual(array.dtype, np.float32)
        np.testing.assert_allclose(array[0, :2], [0., 1000.])
        self.assertAlmostEqual(float(array[0, 2]), 255 * 1000 / 16777215, places=6)

    def test_sensor_receipt_requires_exact_frame_and_nonempty_file(self):
        record = {'type': 'rgb', 'world_frame': 42, 'sensor_frame': 42, 'frame_sync_ok': True,
                  'file': 'image.png', 'file_written': True, 'file_size_bytes': 20}
        validate_sensor_payloads([('cav', {'camera': record})], 42)
        with self.assertRaises(FrameSyncError):
            validate_sensor_payloads([('cav', {'camera': dict(record, sensor_frame=41)})], 42)
        with self.assertRaises(FrameSyncError):
            validate_sensor_payloads([('cav', {'camera': dict(record, file_size_bytes=0)})], 42)

    def test_gnss_is_measurement_and_cannot_be_substituted(self):
        cav = {'cav_001': {'location': [1., 2., 3.]}}
        data = {'type': 'gps', 'node_label': 'cav_001', 'sensor_frame': 5, 'world_frame': 5,
                'frame_sync_ok': True, 'latitude': 31., 'longitude': 121., 'altitude': 4.,
                'measurement_source': 'carla_gnss_sensor'}
        attach_cav_gnss_measurements(cav, {'gps': data}, 5)
        self.assertEqual(cav['cav_001']['geolocation']['latitude'], 31.)
        with self.assertRaises(RuntimeError):
            attach_cav_gnss_measurements(cav, {'gps': dict(data, sensor_frame=4)}, 5)

    @patch("importlib.metadata.version", return_value="0.9.16")
    def test_official_versions_checked_before_world_access(self, _metadata_version):
        client = Mock()
        client.get_client_version.return_value = '0.9.15'
        client.get_server_version.return_value = '0.9.16'
        with self.assertRaisesRegex(RuntimeError, 'client='):
            check_versions(client)
        client.get_world.assert_not_called()
        client.get_client_version.return_value = '0.9.16'
        self.assertEqual(check_versions(client), {'client': '0.9.16', 'server': '0.9.16'})

    def test_matching_release_strings_reject_mismatched_wheel_metadata(self):
        client = Mock()
        client.get_client_version.return_value = '0.9.16'
        client.get_server_version.return_value = '0.9.16'
        with patch('importlib.metadata.version', return_value='0.9.15'):
            with self.assertRaisesRegex(RuntimeError, 'client='):
                check_versions(client)
        client.get_world.assert_not_called()

    def test_clean_official_commit_requires_matching_distribution_and_server(self):
        client = Mock()
        client.get_client_version.return_value = '294096eb1'
        client.get_server_version.return_value = '294096eb1'
        with patch('importlib.metadata.version', return_value='0.9.16'):
            self.assertEqual(check_versions(client)['client'], '294096eb1')
            client.get_client_version.return_value = '294096eb1-dirty'
            client.get_server_version.return_value = '294096eb1-dirty'
            with self.assertRaises(RuntimeError):
                check_versions(client)
        client.get_client_version.return_value = '294096eb1'
        client.get_server_version.return_value = '294096eb1'
        with patch('importlib.metadata.version', return_value='0.9.15'):
            with self.assertRaises(RuntimeError):
                check_versions(client)

    def test_custom_cooked_map_requires_version_provenance(self):
        client = Mock()
        with self.assertRaisesRegex(ValueError, 'assets_version'):
            load_world(client, CARLA, {'map': {'name': 'MySite'}})
        client.get_available_maps.return_value = ['/Game/Sites/MySite']
        load_world(client, CARLA, {'map': {'name': 'MySite', 'assets_version': '0.9.16'}})
        client.load_world.assert_called_once_with('MySite')

    def test_opendrive_loader_passes_content_and_does_not_load_cooked_scene(self):
        client = Mock()
        fake_carla = NS(OpendriveGenerationParameters=lambda **kw: kw)
        with tempfile.TemporaryDirectory() as tmp:
            Path(tmp, 'small.xodr').write_text('<OpenDRIVE/>')
            load_world(client, fake_carla, {'map': {'opendrive': 'small.xodr'}}, tmp)
        self.assertEqual(client.generate_opendrive_world.call_args.args[0], '<OpenDRIVE/>')
        client.load_world.assert_not_called()

    def test_sync_restores_distinct_previous_settings_and_enables_rendering(self):
        old, new = NS(synchronous_mode=False), NS(synchronous_mode=False)
        world = Mock()
        world.get_settings.side_effect = [old, new]
        self.assertIs(synchronous_settings(world, .05), old)
        self.assertTrue(new.synchronous_mode)
        self.assertFalse(new.no_rendering_mode)
        self.assertLessEqual(.05, new.max_substep_delta_time * new.max_substeps)

    def test_configuration_and_virtual_radar_do_not_need_carla(self):
        self.assertEqual(validate_stage2_config(config()), [])
        cfg = config()
        cfg['stage2']['sample_every_n_frames'] = 2
        cfg['stage2']['world_state_every_n_frames'] = 3
        self.assertTrue(validate_stage2_config(cfg))
        real = sensor_items({}, 'cav')
        self.assertTrue({'rgb', 'depth', 'gps', 'imu', 'lidar_3d'} <= {x['type'] for x in real})
        self.assertNotIn('radar', {x['type'] for x in real})
        radar = virtual_radar_items({}, 'cav')
        self.assertEqual(radar[0]['radar_backend'], 'sionna_rcs')

    def test_geodetic_tmerc_source_roundtrip_and_resolver(self):
        proj = {'lat_0': 33.4, 'lon_0': -111.9, 'k': 1., 'x_0': 0., 'y_0': 0.}
        kwargs = {'latitude': 33.4003, 'longitude': -111.8998, 'altitude': 8.}
        location, _ = geolocation_to_xodr_tmerc_carla_location(CARLA, **kwargs, projection=proj)
        lat, lon, alt = xodr_tmerc_carla_location_to_geolocation(location, projection=proj)
        np.testing.assert_allclose([lat, lon, alt], list(kwargs.values()), rtol=0, atol=2e-7)
        resolved = _resolve_location(CARLA, None, {'geolocation': kwargs}, proj)
        self.assertLess(location.distance(resolved), 1e-9)

    def test_ris_ground_relative_height_preserves_source_road_elevation(self):
        road = NS(get_waypoint=lambda *a, **k: NS(transform=Transform(Location(0, 0, 7.))))
        loc, ground = _resolve_height(CARLA, road, Location(10, 20, 2.), 'relative_to_ground', 2.)
        self.assertEqual((loc.x, loc.y, loc.z, ground), (10., 20., 9., 7.))
        with self.assertRaises(ValueError):
            _resolve_height(CARLA, NS(get_waypoint=lambda *a, **k: None), Location(), 'relative_to_ground', 2.)

    def test_road_route_keeps_ordered_vias_and_does_not_cut_chords(self):
        nodes = [Waypoint(0, 0, 0), Waypoint(10, 0, 10), Waypoint(10, 10, 20), Waypoint(20, 10, 30)]
        for a, b in zip(nodes, nodes[1:]):
            a.transform.rotation.yaw = math.degrees(math.atan2(b.transform.location.y-a.transform.location.y, b.transform.location.x-a.transform.location.x))
            connect(a, b)
        result = planner(nodes).plan(nodes[0].transform.location, nodes[-1].transform.location,
                                     [nodes[1].transform.location, nodes[2].transform.location], step_m=2)
        self.assertEqual(result, nodes)
        self.assertEqual(_route_distance(Location(10, 5), [wp.transform.location for wp in result]), 0.)

    def test_topology_unreachable_is_an_error(self):
        nodes = [Waypoint(0, 0, 0), Waypoint(20, 0, 20)]
        with self.assertRaises(RuntimeError):
            planner(nodes).plan(nodes[0].transform.location, nodes[-1].transform.location, step_m=2)

    def test_backward_only_route_cannot_be_assigned_to_official_traffic(self):
        nodes = [Waypoint(0, 0, 0), Waypoint(20, 0, 20)]
        connect(nodes[1], nodes[0])
        with self.assertRaises(RuntimeError):
            planner(nodes).plan(nodes[0].transform.location, nodes[-1].transform.location, step_m=2)

    def test_fov_camera_axes_and_behind_rejection(self):
        fov = FieldOfView(CARLA)
        project = fov._project_location_with_camera_matrix
        self.assertEqual(project(np.eye(4), 100, 100, 90, Location(10, 0, 0)), (50., 50.))
        self.assertIsNone(project(np.eye(4), 100, 100, 90, Location(-10, 0, 0)))
        self.assertAlmostEqual(project(np.eye(4), 100, 100, 90, Location(10, 5, 0))[0], 75.)

    def test_all_modalities_save_exact_frame_payloads_with_shared_ids(self):
        image = NS(frame=42, timestamp=2.1, width=2, height=1,
                   raw_data=bytes([0, 0, 0, 255, 255, 255, 255, 255]))
        clouds = NS(frame=42, timestamp=2.1, raw_data=np.array([[1., 2., 3., .5]], dtype=np.float32).tobytes())
        gps = NS(frame=42, timestamp=2.1, latitude=33.4, longitude=-111.9, altitude=5.)
        imu = NS(frame=42, timestamp=2.1, accelerometer=Location(0, 0, 9.81), gyroscope=Location(0, 0, .1), compass=.2)
        payloads = {}
        with tempfile.TemporaryDirectory() as tmp:
            for modality, data in [('rgb', image), ('depth', image), ('semantic', image),
                                   ('lidar_2d', clouds), ('lidar_3d', clouds), ('gps', gps), ('imu', imu)]:
                sensor = CarlaSensor.__new__(CarlaSensor)
                sensor.modality = sensor.name = modality
                sensor.node_label = 'cav_001'
                sensor.blueprint = 'test_fixture'
                result = sensor.save(data, tmp, 5, 42, site_id='site_a', trajectory_id=2)
                payloads[modality] = result
                self.assertTrue(Path(tmp, result['file']).is_file())
                self.assertEqual((result['site_id'], result['trajectory_id'], result['frame_idx'], result['world_frame']),
                                 ('site_a', 2, 5, 42))
                self.assertEqual(result['sensor_timestamp'], 2.1)
            depth = np.load(Path(tmp, payloads['depth']['file']), allow_pickle=False)
            np.testing.assert_allclose(depth, [[0., 1000.]])
            self.assertEqual(np.load(Path(tmp, payloads['lidar_3d']['file'])).shape, (1, 4))
            self.assertEqual(yaml.safe_load(Path(tmp, payloads['gps']['file']).read_text())['measurement_source'], 'carla_gnss_sensor')
            validate_sensor_payloads([('cav_001', payloads)], 42)
            with self.assertRaises(FrameSyncError):
                sensor.save(imu, tmp, 5, 43, site_id='site_a', trajectory_id=2)

    def test_ris_rgb_bbox_uses_sensor_matrix_and_preserves_object_geometry(self):
        fov = FieldOfView(CARLA)
        sensor = NS(cfg={'image_size_x': 100, 'image_size_y': 100, 'fov': 90},
                    sensor=NS(get_transform=lambda: Transform()))
        vertices = [Location(x, y, z) for x in (9, 11) for y in (-1, 1) for z in (-1, 1)]
        actor = NS(get_transform=lambda: Transform(Location(10, 0, 0)),
                   bounding_box=NS(get_world_vertices=lambda _: vertices))
        bbox = fov.bbox(sensor, actor)
        self.assertEqual(bbox['bbox_xyxy_px'], [44, 44, 56, 56])
        self.assertEqual(bbox['visibility'], 1.)
        np.testing.assert_allclose(bbox['bbox_yolo'][:2], [.5, .5])

    def test_virtual_radar_schema_is_consumable_by_stage3(self):
        from Radar_channel_storage.scene_inputs import radar_sensor_pose
        item = {'name': 'radar_front', 'type': 'radar', 'radar_backend': 'sionna_rcs',
                'transform': {'location': [1., 0., .5], 'rotation': [0., 0., 0.]}}
        parent = NS(bounding_box=NS(location=Location(0, 0, .2), extent=Location(1, 1, .8)))
        entry = virtual_radar_calibration(CARLA, item, Transform(Location(10, 0, 0)), parent=parent)
        calibration = {'node_id': 'cav_001', 'sensors': {'radar_front': entry}}
        state = {'nodes': {'cav': {'cav_001': {'location': [10., 0., 0.], 'rotation': {'pitch': 0., 'yaw': 0., 'roll': 0.}}}}}
        position, orientation = radar_sensor_pose(state, calibration, 'radar_front')
        np.testing.assert_allclose(position, [11., 0., 1.5])
        np.testing.assert_allclose(orientation, [0., 0., 0.])
        self.assertEqual(item['transform']['location'], [1., 0., .5])
        from Radar_channel_storage.sionna_rcs_dataset_generator import _radar_entries
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp, 'calibration')
            folder.mkdir()
            Path(folder, 'cav_001.yaml').write_text(yaml.safe_dump(calibration))
            discovered = _radar_entries(Path(tmp))
            self.assertEqual([(x[0], x[1]) for x in discovered], [('cav_001', 'radar_front')])


if __name__ == '__main__':
    unittest.main()
