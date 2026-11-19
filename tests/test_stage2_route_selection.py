"""Seeded map-spawn route selection and explicit route integration contracts."""
import json
import random
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import pytest

from sigma.sensing.config import validate_stage2_config
from sigma.sensing.routes import RoadRoutePlanner, select_seeded_spawn_route
from sigma.sensing.runner import _plan_route, run_stage2
from test_stage2 import CARLA, Location, Transform, Waypoint, config, connect


def road_fixture():
    nodes = [Waypoint(10 * i, station=10 * i) for i in range(4)]
    for start, destination in zip(nodes, nodes[1:]):
        connect(start, destination)
    carla_map = NS(
        get_spawn_points=lambda: [node.transform for node in nodes],
        get_waypoint=lambda location, **kwargs: min(nodes, key=lambda node: node.transform.location.distance(location)),
        generate_waypoints=lambda step: nodes,
        to_opendrive=lambda: '<OpenDRIVE/>')
    return nodes, carla_map, NS(get_map=lambda: carla_map)


def test_seeded_order_is_deterministic_and_keeps_global_rng_state():
    nodes, carla_map, world = road_fixture()
    global_state = random.getstate()
    first, first_metadata = select_seeded_spawn_route(carla_map, CARLA, RoadRoutePlanner(world, CARLA), 42)
    second, second_metadata = select_seeded_spawn_route(carla_map, CARLA, RoadRoutePlanner(world, CARLA), 42)
    assert random.getstate() == global_state
    assert first == second == nodes[2:]
    assert first_metadata == second_metadata
    assert first_metadata == {
        'selection': 'seeded_spawn_points', 'selection_seed': 42,
        'start_spawn_index': 2, 'destination_spawn_index': 3,
        'start_carla_m': [20., 0., 0.], 'destination_carla_m': [30., 0., 0.],
        'attempted_route_candidates': 2,
    }


def test_unreachable_forward_candidate_advances_to_reachable_pair():
    nodes, carla_map, world = road_fixture()
    planner = RoadRoutePlanner(world, CARLA)
    with patch.object(planner, 'plan', wraps=planner.plan) as plan:
        waypoints, metadata = select_seeded_spawn_route(carla_map, CARLA, planner, 42)
    assert waypoints == nodes[2:]
    assert plan.call_count == metadata['attempted_route_candidates'] == 2
    assert plan.call_args_list[0].args == (nodes[2].transform.location, nodes[1].transform.location)
    assert plan.call_args_list[1].args == (nodes[2].transform.location, nodes[3].transform.location)


def test_candidate_limit_bounds_planner_calls():
    _, carla_map, _ = road_fixture()
    planner = NS(plan=Mock(side_effect=RuntimeError('Disconnected driving lanes')))
    with pytest.raises(RuntimeError, match='exhausted 3 candidate pairs'):
        select_seeded_spawn_route(carla_map, CARLA, planner, 42, max_route_candidates=3)
    assert planner.plan.call_count == 3


def test_spawn_filter_requires_driving_lane_and_projection_tolerance():
    spawns = [Transform(Location(x, 0)) for x in [0, 10, 20, 30]]
    responses = [None, NS(lane_type=2, transform=spawns[1]),
                 NS(lane_type=1, transform=Transform(Location(25, 0))),
                 NS(lane_type=1, transform=spawns[3])]
    carla_map = NS(get_spawn_points=lambda: spawns,
                   get_waypoint=lambda location, **kwargs: responses[int(location.x / 10)])
    planner = Mock()
    with pytest.raises(RuntimeError, match='at least two valid Driving spawn points'):
        select_seeded_spawn_route(carla_map, CARLA, planner, 42)
    planner.plan.assert_not_called()


def test_seeded_schema_accepts_map_selected_endpoints_and_explicit_default():
    cfg = config()
    assert validate_stage2_config(cfg) == []
    cfg['stage2']['route'] = {'selection': 'seeded_spawn_points', 'max_route_candidates': 64}
    assert validate_stage2_config(cfg) == []


@pytest.mark.parametrize('key', ['start', 'destination', 'via'])
def test_seeded_schema_requires_endpoint_selection_from_map(key):
    cfg = config()
    cfg['stage2']['route'] = {'selection': 'seeded_spawn_points', key: [] if key == 'via' else {'x': 0, 'y': 0}}
    assert any('selection alone' in error for error in validate_stage2_config(cfg))


@pytest.mark.parametrize('value', [0, -1, True, 1.5, '64'])
def test_candidate_limit_requires_positive_integer(value):
    cfg = config()
    cfg['stage2']['route'] = {'selection': 'seeded_spawn_points', 'max_route_candidates': value}
    assert 'stage2.route.max_route_candidates must be a positive integer' in validate_stage2_config(cfg)


def test_route_selection_requires_implemented_mode():
    cfg = config()
    cfg['stage2']['route']['selection'] = 'random_coordinates'
    assert 'stage2.route.selection must be explicit or seeded_spawn_points' in validate_stage2_config(cfg)


def test_runner_route_dispatch_preserves_explicit_coordinates_and_ordered_vias():
    nodes, _, world = road_fixture()
    route_cfg = {'start': {'x': 0, 'y': 0}, 'destination': {'x': 30, 'y': 0},
                 'via': [{'x': 10, 'y': 0}, {'x': 20, 'y': 0}]}
    waypoints, via, metadata = _plan_route(CARLA, world, route_cfg, None, 42)
    assert waypoints == nodes
    assert [point.x for point in via] == [10., 20.]
    assert metadata == {'selection': 'explicit', 'start_carla_m': [0., 0., 0.],
                        'destination_carla_m': [30., 0., 0.]}


def test_runner_writes_selected_route_metadata_before_vehicle_spawn(tmp_path):
    nodes, _, road_world = road_fixture()
    world = Mock()
    world.get_map.side_effect = road_world.get_map
    world.get_blueprint_library.side_effect = RuntimeError('Fixture vehicle spawn boundary')
    client = Mock()
    fake_carla = NS(**vars(CARLA), Client=Mock(return_value=client))
    cfg = {'site_id': 'town03', 'stage2': {'map': {'name': 'Town03'},
           'route': {'selection': 'seeded_spawn_points', 'max_route_candidates': 2}}}
    with patch.dict('sys.modules', {'carla': fake_carla}), \
         patch('sigma.sensing.runner.check_versions', return_value={'client': '0.9.16', 'server': '0.9.16'}), \
         patch('sigma.sensing.runner.load_world', return_value=world), \
         patch('sigma.sensing.runner.synchronous_settings', return_value=NS()):
        with pytest.raises(RuntimeError, match='Fixture vehicle spawn boundary'):
            run_stage2(cfg, tmp_path, seed=42)
    record = json.loads((tmp_path / 'stage1_multimodal/000001/route.json').read_text())
    assert record['site_id'] == 'town03'
    assert record['trajectory_id'] == 1
    assert record['selection'] == 'seeded_spawn_points'
    assert record['selection_seed'] == 42
    assert record['start_spawn_index'] == 2
    assert record['destination_spawn_index'] == 3
    assert record['start_carla_m'] == [20., 0., 0.]
    assert record['destination_carla_m'] == [30., 0., 0.]
    assert record['waypoints_carla_m'] == [[20., 0., 0.], [30., 0., 0.]]
    assert record['ordered_via_carla_m'] == []
    assert len(record['lane_stations']) == len(nodes[2:])
    world.apply_settings.assert_called_once()
