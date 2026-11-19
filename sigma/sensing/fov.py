"""Deterministic RSU camera FoV geometry from the source generator.

BBox top, center and corners are projected with CARLA camera axes. Optional
entry-edge gating uses future image motion and supports simulator-independent tests.
"""
import math
import numpy as np


def actor_is_alive(actor):
    return actor is not None and bool(getattr(actor, 'is_alive', False))


class FieldOfView:
    def __init__(self, carla_module, config=None):
        global carla
        carla = carla_module
        self.config = dict(config or {})

    def _selected_rsu_camera_view_cfg(self):
        return self.config

    def _vehicle_bbox_top_location(self, actor):
        bb = actor.bounding_box
        return actor.get_transform().transform(carla.Location(
            x=bb.location.x, y=bb.location.y,
            z=bb.location.z + bb.extent.z))

    def bbox(self, sensor, actor):
        return self._camera_bbox2d_for_actor(sensor, actor)

    def hit(self, actor, camera_specs, margin_px=0.0, require_entry_edge=False):
        return self._selected_actor_rsu_rgb_view_hit(
            actor, camera_specs, margin_px, require_entry_edge)

    def _project_location_with_camera_matrix(self, world_to_camera, width, height, fov, loc):
            point = np.array([float(loc.x), float(loc.y), float(loc.z), 1.0], dtype=np.float64)
            point_camera = np.asarray(world_to_camera, dtype=np.float64) @ point
            point_camera = np.array([point_camera[1], -point_camera[2], point_camera[0]], dtype=np.float64)
            if point_camera[2] <= 1e-3:
                return None
            focal = float(width) / (2.0 * math.tan(math.radians(float(fov)) / 2.0))
            u = focal * (point_camera[0] / point_camera[2]) + float(width) / 2.0
            v = focal * (point_camera[1] / point_camera[2]) + float(height) / 2.0
            return float(u), float(v)

    def _selected_actor_rsu_rgb_view_hit(self, actor, specs, margin_px=0.0, require_entry_edge=False):
            if not actor_is_alive(actor):
                return False, None
            try:
                loc = self._vehicle_bbox_top_location(actor)
            except Exception:
                return False, None
            hit_locations = [('bbox_top', loc)]
            try:
                tf = actor.get_transform()
                bb = getattr(actor, 'bounding_box', None)
                if bb is not None:
                    center = carla.Location(
                        x=float(bb.location.x),
                        y=float(bb.location.y),
                        z=float(bb.location.z),
                    )
                    transformed_center = tf.transform(center)
                    hit_locations.append(('bbox_center', transformed_center or center))
                    for corner in bb.get_world_vertices(tf):
                        hit_locations.append(('bbox_corner', corner))
            except Exception:
                pass
            view_cfg = self._selected_rsu_camera_view_cfg()
            horizon_s = float(view_cfg.get('crossing_horizon_seconds', 3.0))
            margin = float(margin_px)
            entry_edge_required = bool(require_entry_edge)
            entry_band = float(view_cfg.get('entry_edge_band', 0.25))
            edge_min_abs = view_cfg.get('entry_edge_min_abs_u_norm', None)
            if edge_min_abs is None:
                edge_min_abs = 1.0 - 2.0 * max(0.0, min(0.49, entry_band))
            edge_min_abs = max(0.0, min(0.98, float(edge_min_abs)))
            min_delta = abs(float(view_cfg.get('entry_edge_min_delta_u_norm', 0.02)))
            try:
                vel = actor.get_velocity()
            except Exception:
                vel = carla.Vector3D(0.0, 0.0, 0.0)

            best = None
            best_score = -1.0
            for hit_kind, hit_loc in hit_locations:
                future_loc = carla.Location(
                    x=float(hit_loc.x) + float(vel.x) * horizon_s,
                    y=float(hit_loc.y) + float(vel.y) * horizon_s,
                    z=float(hit_loc.z) + float(vel.z) * horizon_s,
                )
                for spec in specs:
                    uv = self._project_location_with_camera_matrix(
                        spec['world_to_camera'], spec['width'], spec['height'], spec['fov'], hit_loc)
                    if uv is None:
                        continue
                    u, v = uv
                    width = float(spec['width'])
                    height = float(spec['height'])
                    if not (-margin <= u <= width - 1.0 + margin and -margin <= v <= height - 1.0 + margin):
                        continue

                    u_norm = (float(u) - width * 0.5) / max(width * 0.5, 1.0)
                    v_norm = (float(v) - height * 0.5) / max(height * 0.5, 1.0)
                    future_u_norm = None
                    delta_u_norm = 0.0
                    lateral_motion = 0.0
                    toward_center = 0.0
                    crosses_center = False
                    motion_direction = 'unknown'
                    entry_edge_ok = not entry_edge_required
                    entry_side = 'none'
                    uv_future = self._project_location_with_camera_matrix(
                        spec['world_to_camera'], spec['width'], spec['height'], spec['fov'], future_loc)
                    if uv_future is not None:
                        fu, fv = uv_future
                        future_u_norm = (float(fu) - width * 0.5) / max(width * 0.5, 1.0)
                        delta_u_norm = float(future_u_norm) - float(u_norm)
                        lateral_motion = abs(delta_u_norm)
                        toward_center = max(0.0, -float(u_norm) * delta_u_norm)
                        crosses_center = bool(float(u_norm) * float(future_u_norm) <= 0.0)
                        if delta_u_norm >= min_delta:
                            motion_direction = 'left_to_right'
                            entry_side = 'left'
                            entry_edge_ok = float(u_norm) <= -edge_min_abs
                        elif delta_u_norm <= -min_delta:
                            motion_direction = 'right_to_left'
                            entry_side = 'right'
                            entry_edge_ok = float(u_norm) >= edge_min_abs

                    if entry_edge_required and not entry_edge_ok:
                        continue

                    score = abs(float(u_norm)) + lateral_motion + 2.0 * toward_center
                    if hit_kind == 'antenna':
                        score += 0.25
                    if entry_edge_required:
                        score += 2.0 * max(0.0, abs(float(u_norm)) - edge_min_abs)
                    if crosses_center:
                        score += 1.0
                    if score > best_score:
                        best_score = float(score)
                        best = {
                            **spec,
                            'uv': [float(u), float(v)],
                            'uv_norm': [float(u_norm), float(v_norm)],
                            'future_u_norm': future_u_norm,
                            'delta_u_norm': float(delta_u_norm),
                            'lateral_motion_norm': float(lateral_motion),
                            'toward_center_score': float(toward_center),
                            'crosses_center': bool(crosses_center),
                            'motion_direction': motion_direction,
                            'entry_side': entry_side,
                            'entry_edge_required': bool(entry_edge_required),
                            'entry_edge_ok': bool(entry_edge_ok),
                            'entry_edge_min_abs_u_norm': float(edge_min_abs),
                            'hit_kind': str(hit_kind),
                            'crossing_score': float(score),
                        }
            return best is not None, best

    def _camera_project_location(self, camera_tf, width, height, fov, loc):
            try:
                world_2_camera = np.array(camera_tf.get_inverse_matrix())
                point = np.array([loc.x, loc.y, loc.z, 1.0], dtype=np.float64)
                point_camera = world_2_camera @ point
                point_camera = np.array([point_camera[1], -point_camera[2], point_camera[0]], dtype=np.float64)
                if point_camera[2] <= 1e-3:
                    return None
                focal = float(width) / (2.0 * math.tan(math.radians(float(fov)) / 2.0))
                u = focal * (point_camera[0] / point_camera[2]) + float(width) / 2.0
                v = focal * (point_camera[1] / point_camera[2]) + float(height) / 2.0
                return float(u), float(v)
            except Exception:
                return None

    def _camera_bbox2d_for_actor(self, manager, actor):
            try:
                width = int(manager.cfg.get('image_size_x', 1280))
                height = int(manager.cfg.get('image_size_y', 720))
                fov = float(manager.cfg.get('fov', 90.0))
                camera_tf = (manager.world_transform_for_geometry()
                             if hasattr(manager, 'world_transform_for_geometry')
                             else manager.sensor.get_transform())
                if camera_tf is None:
                    return None
                vertices = actor.bounding_box.get_world_vertices(actor.get_transform())
                pts = [self._camera_project_location(camera_tf, width, height, fov, v) for v in vertices]
                pts = [p for p in pts if p is not None]
                if not pts:
                    return None
                xs = [p[0] for p in pts]
                ys = [p[1] for p in pts]
                x1, y1, x2, y2 = min(xs), min(ys), max(xs), max(ys)
                raw_area = max(1e-6, (x2 - x1) * (y2 - y1))
                cx1, cy1 = max(0.0, x1), max(0.0, y1)
                cx2, cy2 = min(float(width - 1), x2), min(float(height - 1), y2)
                if cx2 <= cx1 or cy2 <= cy1:
                    return None
                visibility = max(0.0, min(1.0, ((cx2 - cx1) * (cy2 - cy1)) / raw_area))
                return {
                    'bbox_xyxy_px': [int(round(cx1)), int(round(cy1)), int(round(cx2)), int(round(cy2))],
                    'bbox_yolo': [
                        round(float((cx1 + cx2) / (2.0 * width)), 8),
                        round(float((cy1 + cy2) / (2.0 * height)), 8),
                        round(float((cx2 - cx1) / width), 8),
                        round(float((cy2 - cy1) / height), 8),
                    ],
                    'visibility': round(float(visibility), 6),
                }
            except Exception:
                return None
