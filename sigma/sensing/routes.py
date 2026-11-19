"""Road-constrained A* and ordered-via routing from the source generator.

Run through ``sigma stage2 --config configs/town03.yaml``. The planner walks
CARLA driving-lane topology and preserves exact ordered joins. Only forward
successors are accepted because the official Traffic Manager drives in lane
travel direction; a backwards-only route fails the reachability check.
Navigation uses the driving-lane topology directly.
"""
import heapq
import math
import random


def select_seeded_spawn_route(carla_map, carla_module, planner, seed, *,
                              max_route_candidates=64,
                              max_projection_distance_m=3.0, step_m=2.0):
    """Select a reachable route from seeded official-map Driving spawn points.

    Preserve the source traffic generator's map-spawn filtering and random
    ordering. Candidate pairs follow that seeded order; each complete route is
    checked by ``RoadRoutePlanner.plan`` against forward driving-lane topology.
    Returned metadata records the original map spawn indices and coordinates.
    """
    if type(max_route_candidates) is not int or max_route_candidates < 1:
        raise ValueError('max_route_candidates must be a positive integer')
    if not math.isfinite(max_projection_distance_m) or max_projection_distance_m <= 0:
        raise ValueError('max_projection_distance_m must be finite and positive')
    valid = []
    for index, transform in enumerate(carla_map.get_spawn_points()):
        location = transform.location
        waypoint = carla_map.get_waypoint(
            location, project_to_road=True, lane_type=carla_module.LaneType.Driving)
        if waypoint is None or waypoint.lane_type != carla_module.LaneType.Driving:
            continue
        projected = waypoint.transform.location
        if math.hypot(projected.x - location.x, projected.y - location.y) <= max_projection_distance_m:
            valid.append((index, transform))
    if len(valid) < 2:
        raise RuntimeError('Seeded route selection requires at least two valid Driving spawn points')
    random.Random(int(seed)).shuffle(valid)
    attempts = 0
    last_failure = ''
    for start_index, start_transform in valid:
        for destination_index, destination_transform in valid:
            if start_index == destination_index:
                continue
            if attempts >= max_route_candidates:
                break
            attempts += 1
            start, destination = start_transform.location, destination_transform.location
            try:
                waypoints = planner.plan(start, destination, step_m=step_m)
                if len(waypoints or []) < 2:
                    raise RuntimeError('Driving route requires at least two waypoints')
            except RuntimeError as exc:
                last_failure = str(exc)
                continue
            metadata = {
                'selection': 'seeded_spawn_points', 'selection_seed': int(seed),
                'start_spawn_index': start_index, 'destination_spawn_index': destination_index,
                'start_carla_m': [float(start.x), float(start.y), float(start.z)],
                'destination_carla_m': [float(destination.x), float(destination.y), float(destination.z)],
                'attempted_route_candidates': attempts,
            }
            return waypoints, metadata
        if attempts >= max_route_candidates:
            break
    detail = f'; last candidate: {last_failure}' if last_failure else ''
    raise RuntimeError(f'Seeded route selection exhausted {attempts} candidate pairs{detail}')


class RoadRoutePlanner:
    def __init__(self, world, carla_module, route_config=None):
        global carla
        carla = carla_module
        self.world = world
        self.veh_cfg = {'selected_vehicle_route': route_config or {}}
        self._route_dir_2d = None
        self._route_via_locs = []
        self._last_computed_route_waypoints = None

    def plan(self, start, destination, via=(), step_m=2.0, max_steps=10000):
        self._route_via_locs = list(via)
        first = self._route_via_locs[0] if self._route_via_locs else destination
        last = self._route_via_locs[-1] if self._route_via_locs else start
        initial_hint = (first.x-start.x, first.y-start.y)
        final_hint = (destination.x-last.x, destination.y-last.y)
        self._route_dir_2d = initial_hint
        carla_map = self.world.get_map()
        starts = self._directional_route_endpoint_candidates(carla_map, start, initial_hint)
        ends = self._directional_route_endpoint_candidates(carla_map, destination, final_hint)
        alternatives = []
        failures = []
        for start_wp in starts:
            for end_wp in ends:
                try:
                    result = self._compute_ordered_waypoint_route(
                        start, destination, self._route_via_locs, step_m=step_m,
                        max_steps=max_steps, precise_endpoint=True, emit_log=False,
                        start_waypoint=start_wp, end_waypoint=end_wp,
                        direction_hint_2d=initial_hint)
                    if result:
                        points = list(self._last_computed_route_waypoints)
                        length = sum(a.transform.location.distance(b.transform.location)
                                     for a, b in zip(points, points[1:]))
                        alternatives.append((length, points))
                except RuntimeError as exc:
                    failures.append(str(exc))
        if not alternatives:
            detail = ': ' + '; '.join(failures) if failures else ''
            raise RuntimeError('No reachable driving-lane route connects the configured endpoints' + detail)
        points = min(alternatives, key=lambda row: row[0])[1]
        self._last_computed_route_waypoints = points
        return points

    @staticmethod
    def _copy_route_waypoints(route_wps):
        out = []
        for wp in list(route_wps or []):
            try:
                _ = wp.transform.location
                out.append(wp)
            except Exception:
                continue
        return out

    @staticmethod
    def _waypoint_location(wp):
        return wp.transform.location

    def _preflight_route(self, route_wps, route_name):
        """Reject obvious reverse/U-turn connector geometry before route control use."""
        route = self._copy_route_waypoints(route_wps)
        if len(route) < 2:
            raise RuntimeError(
                f"selected route {route_name} has no authoritative waypoint plan")
        route_hint = getattr(self, '_route_dir_2d', None)
        # A multi-leg route can legitimately turn away from its initial heading.
        # Check direction locally to follow curved and returning roads.
        if getattr(self, '_route_via_locs', None):
            route_hint = None
        hint = None
        if route_hint is not None:
            try:
                hx, hy = float(route_hint[0]), float(route_hint[1])
                hn = math.hypot(hx, hy)
                if hn > 1e-6:
                    hint = (hx / hn, hy / hn)
            except Exception:
                hint = None
        prev_vec = None
        reversals = []
        reverse_connectors = []
        for i in range(len(route) - 1):
            a = self._waypoint_location(route[i])
            b = self._waypoint_location(route[i + 1])
            dx = float(b.x) - float(a.x)
            dy = float(b.y) - float(a.y)
            seg_len = math.hypot(dx, dy)
            if seg_len < 0.5:
                continue
            vec = (dx / seg_len, dy / seg_len)
            in_junction = False
            for wp in (route[i], route[i + 1]):
                try:
                    if bool(getattr(wp, 'is_junction', False)):
                        in_junction = True
                        break
                except Exception:
                    pass
            # With a configured start->end direction, the hint-based check below
            # is authoritative. Converted OpenDRIVE junctions can contain one
            # sharp local kink while still progressing toward the destination;
            # treating that kink alone as a U-turn rejects valid straight routes.
            if in_junction and prev_vec is not None and hint is None:
                dot_prev = prev_vec[0] * vec[0] + prev_vec[1] * vec[1]
                if dot_prev < -0.35:
                    reversals.append((i, dot_prev))
            if in_junction and hint is not None:
                dot_hint = hint[0] * vec[0] + hint[1] * vec[1]
                if dot_hint < -0.20:
                    reverse_connectors.append((i, dot_hint))
            prev_vec = vec
        if reversals or reverse_connectors:
            detail = []
            if reversals:
                detail.append(
                    "u_turn_segments=" + ",".join(
                        f"{idx}:{dot:.2f}" for idx, dot in reversals[:4]))
            if reverse_connectors:
                detail.append(
                    "reverse_connector_segments=" + ",".join(
                        f"{idx}:{dot:.2f}" for idx, dot in reverse_connectors[:4]))
            raise RuntimeError(
                f"selected route {route_name} preflight rejected: "
                + " ".join(detail))
        return route

    def _directional_route_endpoint_candidates(
            self, carla_map, location, direction_hint_2d,
            max_candidates=3, max_adjacent_hops=2):
        """Return nearby same-direction driving lanes in deterministic order.

        Adjacent-lane traversal is used only to enumerate alternative starting
        lanes. The route graph itself uses forward lane-following edges, so
        the selected route follows its chosen lanes.
        """
        nearest = carla_map.get_waypoint(
            location, project_to_road=True,
            lane_type=carla.LaneType.Driving)
        if nearest is None:
            return []
        if direction_hint_2d is None:
            return [nearest]

        hx, hy = float(direction_hint_2d[0]), float(direction_hint_2d[1])
        hint_norm = math.hypot(hx, hy)
        if hint_norm <= 1e-6:
            return [nearest]
        hx, hy = hx / hint_norm, hy / hint_norm
        route_cfg = self.veh_cfg.get('selected_vehicle_route', {}) or {}
        lane_cfg = route_cfg.get('lane_selection', {}) or {}
        min_heading_dot = float(lane_cfg.get('same_direction_min_dot', 0.5))
        max_distance_m = float(lane_cfg.get('endpoint_search_radius_m', 12.0))

        candidates = []
        queue = [(nearest, 0)]
        seen = set()
        while queue and len(seen) < 16:
            waypoint, hops = queue.pop(0)
            if waypoint is None:
                continue
            key = (
                int(waypoint.road_id), int(waypoint.section_id),
                int(waypoint.lane_id), round(float(waypoint.s), 2),
            )
            if key in seen:
                continue
            seen.add(key)
            try:
                is_driving = waypoint.lane_type == carla.LaneType.Driving
            except Exception:
                is_driving = True
            distance = waypoint.transform.location.distance(location)
            if is_driving and distance <= max_distance_m:
                yaw = math.radians(float(waypoint.transform.rotation.yaw))
                heading_dot = math.cos(yaw) * hx + math.sin(yaw) * hy
                if heading_dot >= min_heading_dot:
                    candidates.append((
                        float(distance), -float(heading_dot), int(hops),
                        abs(int(waypoint.lane_id)), int(waypoint.lane_id), waypoint))
            if hops < max(0, int(max_adjacent_hops)):
                try:
                    queue.extend([
                        (waypoint.get_left_lane(), hops + 1),
                        (waypoint.get_right_lane(), hops + 1),
                    ])
                except Exception:
                    pass

        if not candidates:
            # A divided road may encode the other carriageway as a different
            # OpenDRIVE road, unreachable via get_left/right_lane. Search the
            # same bounded spatial radius and preserve the driving direction.
            spatial = getattr(self, '_route_endpoint_spatial_waypoints', None)
            if spatial is None:
                spatial = list(carla_map.generate_waypoints(2.0))
                self._route_endpoint_spatial_waypoints = spatial
            by_lane = {}
            for waypoint in spatial:
                distance = waypoint.transform.location.distance(location)
                if distance > max_distance_m:
                    continue
                yaw = math.radians(float(waypoint.transform.rotation.yaw))
                heading_dot = math.cos(yaw) * hx + math.sin(yaw) * hy
                if heading_dot < min_heading_dot:
                    continue
                key = (int(waypoint.road_id), int(waypoint.section_id), int(waypoint.lane_id))
                row = (float(distance), -float(heading_dot), 0,
                       abs(int(waypoint.lane_id)), int(waypoint.lane_id), waypoint)
                if key not in by_lane or row[:5] < by_lane[key][:5]:
                    by_lane[key] = row
            candidates.extend(by_lane.values())
            if candidates:
                print('ROUTE_ENDPOINT_SPATIAL_SNAP=1 radius_m=%.1f aligned_lanes=%d' %
                      (max_distance_m, len(candidates)), flush=True)
        candidates.sort(key=lambda row: row[:5])
        return [row[5] for row in candidates[:max(1, int(max_candidates))]]

    def _directional_route_endpoint_waypoint(
            self, carla_map, location, direction_hint_2d):
        """Prefer the adjacent driving lane whose heading matches the route.

        ``get_waypoint(project_to_road=True)`` returns only the nearest lane.
        A WGS84 endpoint close to a center line can therefore resolve to the
        opposite carriageway. Only the endpoint projection is corrected here;
        the route search itself remains lane locked.
        """
        nearest = carla_map.get_waypoint(
            location, project_to_road=True,
            lane_type=carla.LaneType.Driving)
        if nearest is None or direction_hint_2d is None:
            return nearest

        hx, hy = float(direction_hint_2d[0]), float(direction_hint_2d[1])
        hint_norm = math.hypot(hx, hy)
        if hint_norm <= 1e-6:
            return nearest
        hx, hy = hx / hint_norm, hy / hint_norm

        candidates = []
        queue = [nearest]
        seen = set()
        while queue and len(seen) < 12:
            waypoint = queue.pop(0)
            if waypoint is None:
                continue
            key = (
                int(waypoint.road_id), int(waypoint.section_id),
                int(waypoint.lane_id), round(float(waypoint.s), 2),
            )
            if key in seen:
                continue
            seen.add(key)
            try:
                is_driving = waypoint.lane_type == carla.LaneType.Driving
            except Exception:
                is_driving = True
            distance = waypoint.transform.location.distance(location)
            if is_driving and distance <= 12.0:
                yaw = math.radians(float(waypoint.transform.rotation.yaw))
                heading_dot = math.cos(yaw) * hx + math.sin(yaw) * hy
                candidates.append((float(distance), float(heading_dot), waypoint))
            try:
                queue.extend([waypoint.get_left_lane(), waypoint.get_right_lane()])
            except Exception:
                continue

        aligned = [row for row in candidates if row[1] > 0.0]
        if not aligned:
            return nearest
        chosen = min(aligned, key=lambda row: (row[0], -row[1]))
        if chosen[2] is not nearest:
            print(
                "SELECTED_ROUTE_ENDPOINT_LANE_CORRECTED=1 "
                f"distance_m={chosen[0]:.2f} heading_dot={chosen[1]:.3f} "
                f"road_id={int(chosen[2].road_id)} "
                f"lane_id={int(chosen[2].lane_id)}",
                flush=True,
            )
        return chosen[2]

    def _compute_ordered_waypoint_route(
            self, start_loc, end_loc, via_locs, **kwargs):
        """Join every ordered via leg using the existing A* planner.

        Intermediate locations are projected once to Driving waypoints. The
        identical waypoint terminates one leg and starts the next, so neither
        lane changes nor straight-line shortcuts are introduced at joins.
        """
        if not via_locs:
            return self._compute_waypoint_route(start_loc, end_loc, **kwargs)
        self._last_computed_route_waypoints = None
        points = [start_loc, *via_locs, end_loc]
        carla_map = self.world.get_map()
        anchors = [kwargs.get('start_waypoint')]
        anchors.extend(carla_map.get_waypoint(
            point, project_to_road=True, lane_type=carla.LaneType.Driving)
            for point in via_locs)
        anchors.append(kwargs.get('end_waypoint'))
        if any(wp is None for wp in anchors[1:-1]):
            raise RuntimeError('Ordered route has a via point without a Driving waypoint')
        # Consecutive requests for the same snapped lane station are already
        # satisfied. Retain nonconsecutive points to support valid revisits.
        for index in range(1, len(anchors)):
            a, b = anchors[index - 1], anchors[index]
            if a is not None and b is not None and (
                    a.road_id, a.section_id, a.lane_id, round(float(a.s), 4)) == (
                    b.road_id, b.section_id, b.lane_id, round(float(b.s), 4)):
                anchors[index] = a
        combined_wps = []
        try:
            for index in range(len(points) - 1):
                if anchors[index] is not None and anchors[index] is anchors[index + 1]:
                    continue
                options = dict(kwargs)
                options.update(start_waypoint=anchors[index],
                               end_waypoint=anchors[index + 1],
                               precise_endpoint=True)
                # Derive heading from the chosen start lane to follow curved
                # and returning roads.
                wp = anchors[index]
                if wp is not None:
                    yaw = math.radians(float(wp.transform.rotation.yaw))
                    options['direction_hint_2d'] = (math.cos(yaw), math.sin(yaw))
                route = self._compute_waypoint_route(
                    points[index], points[index + 1], **options)
                leg = list(self._last_computed_route_waypoints or [])
                if not route or not leg:
                    raise RuntimeError(f'Ordered route leg {index + 1} is unreachable')
                if combined_wps:
                    if combined_wps[-1] is not leg[0]:
                        raise RuntimeError(f'Ordered route leg {index + 1} has a discontinuous join')
                    leg = leg[1:]
                combined_wps.extend(leg)
            self._preflight_route(combined_wps, 'ordered-via')
        except Exception:
            self._last_computed_route_waypoints = None
            raise
        self._last_computed_route_waypoints = combined_wps
        return [carla.Location(x=wp.transform.location.x,
                               y=wp.transform.location.y,
                               z=wp.transform.location.z + 0.5)
                for wp in combined_wps]

    def _compute_waypoint_route(
            self, start_loc, end_loc, step_m=5.0, max_steps=2000,
            direction_hint_2d=None, start_waypoint=None, end_waypoint=None,
            emit_log=True, precise_endpoint=False):
        """Compute a whole-route shortest waypoint path between two locations.

        The previous implementation greedily picked the next waypoint that was
        closest to the destination. At junctions, that local decision can enter
        a turning connector even when the overall straight route is much
        shorter. This uses a bounded A* search over the CARLA waypoint graph,
        stores the matching ``carla.Waypoint`` sequence for post-warmup
        route control handoff, and returns ``carla.Location`` points for warmup TM
        path assignment and route adherence.
        """
        self._last_computed_route_waypoints = None
        carla_map = self.world.get_map()
        start_wp = start_waypoint
        if start_wp is None:
            start_wp = self._directional_route_endpoint_waypoint(
                carla_map, start_loc, direction_hint_2d)
        end_wp = end_waypoint
        if end_wp is None:
            end_wp = self._directional_route_endpoint_waypoint(
                carla_map, end_loc, direction_hint_2d)
        if start_wp is None or end_wp is None:
            return None

        step_m = max(1.0, float(step_m))
        max_steps = max(8, int(max_steps))
        end_location = end_wp.transform.location
        end_tolerance_m = max(step_m * 2.0, float(
            self.veh_cfg.get('route_end_tolerance_m', 15.0)))
        selected_route_lane_locked = True
        selected_route_end_snap_max_m = 2.5

        def _wp_key(wp):
            loc = wp.transform.location
            return (
                int(wp.road_id), int(wp.section_id), int(wp.lane_id),
                round(float(wp.s), 2), round(float(loc.x), 2), round(float(loc.y), 2),
            )

        def _loc(wp):
            return wp.transform.location

        def _search(direction):
            def _advance(wp):
                if direction == 'backward':
                    return list(wp.previous(step_m) or [])
                return list(wp.next(step_m) or [])

            start_key = _wp_key(start_wp)
            heap = [(start_wp.transform.location.distance(end_location), 0.0, 0, start_key, start_wp)]
            best_cost = {start_key: 0.0}
            parent = {start_key: None}
            nodes = {start_key: start_wp}
            best_goal = None
            best_goal_dist = float('inf')
            counter = 1

            while heap and len(best_cost) <= max_steps:
                _priority, cost, _seq, key, wp = heapq.heappop(heap)
                if cost > best_cost.get(key, float('inf')) + 1e-6:
                    continue
                dist_to_end = _loc(wp).distance(end_location)
                if dist_to_end < best_goal_dist:
                    best_goal_dist = dist_to_end
                reached = dist_to_end <= end_tolerance_m
                if precise_endpoint:
                    # Reach the requested lane station using its lane identity
                    # and longitudinal coordinate within one routing step.
                    delta = end_location - _loc(wp)
                    yaw = math.radians(float(wp.transform.rotation.yaw))
                    along = delta.x * math.cos(yaw) + delta.y * math.sin(yaw)
                    reached = (
                        dist_to_end <= step_m + 0.01
                        and (wp.road_id, wp.section_id, wp.lane_id)
                        == (end_wp.road_id, end_wp.section_id, end_wp.lane_id)
                        and abs(float(wp.s) - float(end_wp.s)) <= step_m + 0.01
                        and (along >= -0.01 if direction == 'forward' else along <= 0.01))
                if reached:
                    best_goal = key
                    break

                candidates = []
                for nxt in _advance(wp):
                    candidates.append((nxt, _loc(wp).distance(_loc(nxt))))
                for nxt, edge_cost in candidates:
                    nkey = _wp_key(nxt)
                    new_cost = cost + max(0.01, float(edge_cost))
                    if new_cost >= best_cost.get(nkey, float('inf')):
                        continue
                    best_cost[nkey] = new_cost
                    parent[nkey] = key
                    nodes[nkey] = nxt
                    priority = new_cost + _loc(nxt).distance(end_location)
                    heapq.heappush(heap, (priority, new_cost, counter, nkey, nxt))
                    counter += 1

            if best_goal is None:
                return None, float('inf'), best_goal_dist, None

            route_wps = []
            key = best_goal
            while key is not None:
                route_wps.append(nodes[key])
                key = parent[key]
            route_wps.reverse()
            if precise_endpoint:
                if route_wps[-1].transform.location.distance(end_location) < 1e-6:
                    route_wps[-1] = end_wp
                else:
                    route_wps.append(end_wp)
            route_locs = [
                carla.Location(
                    x=wp.transform.location.x,
                    y=wp.transform.location.y,
                    z=wp.transform.location.z + 0.5,
                )
                for wp in route_wps
            ]
            end_snap_dist = (
                route_locs[-1].distance(end_location)
                if route_locs else float('inf'))
            if not precise_endpoint and end_snap_dist <= selected_route_end_snap_max_m:
                route_locs.append(carla.Location(
                    x=end_location.x, y=end_location.y, z=end_location.z + 0.5))
            elif not precise_endpoint and selected_route_lane_locked and emit_log:
                print(
                    "SELECTED_ROUTE_END_SNAP_SKIPPED=1 "
                    f"distance_m={end_snap_dist:.2f} "
                    f"max_m={selected_route_end_snap_max_m:.2f} "
                    "reason=selected_route_lane_locked",
                    flush=True,
                )
            total_len = sum(
                route_locs[i].distance(route_locs[i + 1])
                for i in range(len(route_locs) - 1)
            )
            return route_locs, total_len, route_locs[-1].distance(end_location), route_wps

        alternatives = []
        for direction in ('forward',):
            route_locs, route_len, end_dist, route_wps = _search(direction)
            if route_locs and len(route_locs) >= 2:
                heading_dot = 1.0
                if direction_hint_2d is not None:
                    try:
                        hx, hy = float(direction_hint_2d[0]), float(direction_hint_2d[1])
                        seg_dx = float(route_locs[1].x) - float(route_locs[0].x)
                        seg_dy = float(route_locs[1].y) - float(route_locs[0].y)
                        seg_n = math.hypot(seg_dx, seg_dy)
                        heading_dot = ((seg_dx / seg_n) * hx + (seg_dy / seg_n) * hy) if seg_n > 1e-6 else -1.0
                    except Exception:
                        heading_dot = -1.0
                alternatives.append((route_len, direction, route_locs, end_dist, heading_dot, route_wps))
        if not alternatives:
            if not emit_log:
                return None
            print(
                "WARNING: Whole-route waypoint search failed; selected vehicle "
                f"route start=({start_loc.x:.1f},{start_loc.y:.1f}) "
                f"end=({end_loc.x:.1f},{end_loc.y:.1f})",
                flush=True,
            )
            return None

        aligned = alternatives
        if direction_hint_2d is not None:
            aligned = [item for item in alternatives if float(item[4]) > 0.0]
            if not aligned:
                if not emit_log:
                    return None
                print(
                    "WARNING: selected-route waypoint search found no path whose "
                    "initial direction matches configured start->end; rejecting route",
                    flush=True,
                )
                return None
        preflight_aligned = []
        preflight_errors = []
        for item in aligned:
            try:
                self._preflight_route(
                    item[5], 'candidate')
            except RuntimeError as exc:
                preflight_errors.append(str(exc))
                continue
            preflight_aligned.append(item)
        if not preflight_aligned:
            raise RuntimeError(
                "selected route search produced only preflight-invalid "
                "alternatives: " + "; ".join(preflight_errors))
        aligned = preflight_aligned
        route_len, direction, route_locs, end_dist, heading_dot, route_wps = min(aligned, key=lambda x: x[0])
        straight_dist = start_wp.transform.location.distance(end_location)
        self._last_computed_route_waypoints = list(route_wps or [])
        if emit_log:
            print(
                f"Waypoint route computed (whole-route {direction}): "
                f"{len(route_locs)} points, polyline={route_len:.1f}m, "
                f"straight={straight_dist:.1f}m, end_dist={end_dist:.1f}m, "
                f"heading_dot={heading_dot:.3f}, "
                f"selected_route_lane_locked={selected_route_lane_locked}",
                flush=True,
            )
        return route_locs
