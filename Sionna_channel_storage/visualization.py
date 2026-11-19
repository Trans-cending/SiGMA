# -*- coding: utf-8 -*-
from .sionna_config import *

from .raytracing import *
from .channel_processor import bartlett_spectrum_from_vector
from .radio_writer import stage3_root

def scene_top_z(scene):
    """Return the highest loaded scene point, or NaN when unavailable."""
    if scene is None:
        return float("nan")
    try:
        bbox_max = scene.mi_scene.bbox().max
        value = float(bbox_max[2])
        return value if np.isfinite(value) else float("nan")
    except (AttributeError, IndexError, TypeError, ValueError):
        return float("nan")


def make_link_topdown_camera_pose(tx_pos, rx_pos, scene=None):
    tx = np.asarray(tx_pos, dtype=float)
    rx = np.asarray(rx_pos, dtype=float)
    mid = 0.5 * (tx + rx)
    clearance = float(VIS_TOPDOWN_HEIGHT_M)
    reference_top = max(float(tx[2]), float(rx[2]))
    loaded_scene_top = scene_top_z(scene)
    if np.isfinite(loaded_scene_top):
        reference_top = max(reference_top, loaded_scene_top)
    pos = np.array([mid[0], mid[1], reference_top + clearance], dtype=float)
    look_at = np.array([mid[0], mid[1], mid[2]], dtype=float)
    return pos, look_at

def unit_xy_direction(src, dst):
    d = np.array([dst[0] - src[0], dst[1] - src[1], 0.0], dtype=float)
    n = np.linalg.norm(d[:2])
    return d / n if n > 1e-9 else np.array([1.0, 0.0, 0.0], dtype=float)

def make_tx_to_rx_camera_pose(tx_pos, rx_pos):
    direction = unit_xy_direction(tx_pos, rx_pos)
    pos = np.asarray(tx_pos, dtype=float) - direction * VIS_TX_TO_RX_BACKOFF_M + np.array([0.0, 0.0, VIS_TX_TO_RX_HEIGHT_M])
    look_at = np.asarray(rx_pos, dtype=float) + np.array([0.0, 0.0, 0.5])
    return pos, look_at

class _NumpyTensorView:
    """Minimal tensor adapter consumed by Sionna's paths_to_segments()."""

    def __init__(self, value):
        self._value = value

    def numpy(self):
        return self._value


class _RenderPathsView:
    """Read-only Paths proxy with a visualization-only valid-path mask."""

    def __init__(self, paths, valid):
        self._paths = paths
        self.valid = _NumpyTensorView(valid)

    def __getattr__(self, name):
        return getattr(self._paths, name)


def limit_render_paths(paths, max_paths: int):
    """Return a renderer view containing at most the strongest max_paths.

    The computed Sionna Paths object remains intact. Only the valid mask
    read by Sionna's renderer is replaced, so saved raw paths and reconstructed
    channels retain their complete propagation data.
    """
    if paths is None:
        return None, 0, 0
    valid = np.asarray(paths.valid.numpy(), dtype=bool)
    valid_flat = np.flatnonzero(valid)
    total = int(valid_flat.size)
    limit = max(0, int(max_paths))
    if total <= limit:
        return paths, total, total
    if limit == 0:
        return _RenderPathsView(paths, np.zeros_like(valid)), 0, total

    try:
        a_real, a_imag = paths.a
        magnitude = np.hypot(
            np.asarray(a_real.numpy(), dtype=np.float64),
            np.asarray(a_imag.numpy(), dtype=np.float64),
        )
        if magnitude.shape != valid.shape:
            # Synthetic arrays retain singleton antenna axes in Paths.a while
            # Paths.valid uses [rx, tx, path]. Their elements still correspond
            # one-to-one and can be reshaped while preserving path ordering.
            if magnitude.size != valid.size:
                raise ValueError(
                    f"coefficient/valid shape mismatch: {magnitude.shape} != {valid.shape}"
                )
            magnitude = magnitude.reshape(valid.shape)
        score = magnitude.ravel()[valid_flat]
        selected = valid_flat[np.lexsort((valid_flat, -score))[:limit]]
    except Exception as exc:
        warning_line(f"Renderer path ranking unavailable; using first {limit}: {exc}")
        selected = valid_flat[:limit]

    render_valid = np.zeros_like(valid)
    render_valid.ravel()[selected] = True
    return _RenderPathsView(paths, render_valid), int(selected.size), total


def render_to_file_rt201(scene, camera, filename: Path, paths=None):
    """Render through the documented Sionna RT 2.0.1 Scene.render_to_file API."""
    filename = Path(filename)
    filename.parent.mkdir(parents=True, exist_ok=True)
    render_paths, selected_count, total_count = limit_render_paths(
        paths, VIS_RENDER_MAX_PATHS
    )
    if paths is not None:
        status_line(
            f"3D renderer path overlay: selected={selected_count}/{total_count} "
            f"limit={VIS_RENDER_MAX_PATHS}"
        )
    scene.render_to_file(
        camera=camera,
        filename=str(filename),
        resolution=VIS_RESOLUTION,
        num_samples=VIS_NUM_SAMPLES,
        lighting_scale=VIS_LIGHTING_SCALE,
        paths=render_paths if VIS_RENDER_RAYS else None,
        show_devices=True,
        show_orientations=True,
    )
    return filename

def save_topview_plan_plot(out_png: Path, pose_info: dict):
    import matplotlib.pyplot as plt

    tx = pose_info["tx_pos"].astype(float)
    rx = pose_info["rx_pos"].astype(float)
    mid = 0.5 * (tx + rx)
    dist_xy = float(np.linalg.norm((rx - tx)[:2]))
    half = max(dist_xy * 0.65 + 25.0, 30.0)

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot([tx[0], rx[0]], [tx[1], rx[1]], linewidth=2, label="Tx-Rx link")
    ax.scatter([tx[0]], [tx[1]], s=110, marker="^", label="Tx/RSU")
    ax.scatter([rx[0]], [rx[1]], s=110, marker="o", label="Rx/CAV")
    for origin, fwd, label in [
        (tx, pose_info["tx_forward"], "Tx heading"),
        (rx, pose_info["rx_forward"], "Rx heading"),
    ]:
        f = np.asarray(fwd, dtype=float)
        if np.linalg.norm(f[:2]) > 1e-9:
            L = max(min(dist_xy * 0.12, 8.0), 3.0)
            ax.arrow(
                origin[0], origin[1], f[0]*L, f[1]*L,
                head_width=max(L*0.18, 0.6), head_length=max(L*0.25, 0.8),
                length_includes_head=True, linewidth=1.5, label=label,
            )
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(mid[0] - half, mid[0] + half)
    ax.set_ylim(mid[1] - half, mid[1] + half)
    ax.set_xlabel("Sionna X / CARLA X (m)")
    ax.set_ylabel("Sionna Y = -CARLA Y (m)")
    ax.grid(True, linewidth=0.4)
    ax.legend(loc="best")
    fig.tight_layout()
    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png, dpi=160)
    plt.close(fig)
    return out_png

def save_bartlett_png_npz(out_prefix: Path, P_db: np.ndarray, title: str):
    import matplotlib.pyplot as plt

    out_prefix.parent.mkdir(parents=True, exist_ok=True)
    png = out_prefix.with_suffix(".png")
    npz = out_prefix.with_suffix(".npz")

    peak = np.unravel_index(np.nanargmax(P_db), P_db.shape)
    lon = -180.0 + (peak[1] + 0.5) * 360.0 / P_db.shape[1]
    lat = -90.0 + (peak[0] + 0.5) * 180.0 / P_db.shape[0]

    fig, ax = plt.subplots(figsize=(10, 4.5))
    im = ax.imshow(
        P_db, origin="lower", aspect="auto",
        extent=[-180, 180, -90, 90],
        vmin=BARTLETT_DB_FLOOR, vmax=0.0,
    )
    ax.scatter([lon], [lat], marker="x", s=80)
    if P_db.ndim == 2 and P_db.shape[1] > 1 and (
        not np.isfinite(P_db[:, 0]).any() or not np.isfinite(P_db[:, -1]).any()
    ):
        ax.set_xlim(-90.0, 90.0)
    ax.set_xlabel("Local longitude lon (deg, +right)")
    ax.set_ylabel("Local latitude lat (deg, +down)")
    ax.set_title(f"{title}\npeak lon/lat=({lon:.1f}, {lat:.1f}) deg")
    fig.colorbar(im, ax=ax, label="Normalized power (dB)")
    fig.tight_layout()
    fig.savefig(png, dpi=180)
    plt.close(fig)
    np.savez_compressed(npz, P_db=P_db, peak_lon=lon, peak_lat=lat)
    return png, npz

def save_visualizations(traj_dir: Path, frame_name: str, scene, Camera, paths,
                        pose_info: dict, h_by_config: dict, force: bool = False):
    """
    Save one guaranteed non-white 2x2 visualization figure using matplotlib only.

    Layout:
      [0,0] Top-view Tx/Rx geometry plan
      [0,1] Link / path / channel summary
      [1,0] Rx Bartlett, or placeholder if Rx is 1x1
      [1,1] Tx Bartlett

    The first two subplots are drawn directly with matplotlib to provide
    deterministic geometry and channel summaries across rendering variants.
    """
    if not VIS_ENABLE:
        return
    if pose_info is None:
        warning_line(f"{traj_dir.name}/{frame_name}: skip visualization because pose_info is None")
        return

    def _render_scene_topview(traj_dir, frame_name, scene, Camera, paths):
        """Sionna 3D top-down render with optional ray overlay."""
        try:
            vis_dir = stage3_root(traj_dir) / "vis_imgs"
            vis_dir.mkdir(parents=True, exist_ok=True)
            top_pos, top_look = make_link_topdown_camera_pose(
                pose_info["tx_pos"], pose_info["rx_pos"], scene=scene
            )
            loaded_scene_top = scene_top_z(scene)
            status_line(
                "3D top-down camera: "
                f"scene_top_z={loaded_scene_top:.2f} camera_z={top_pos[2]:.2f} "
                f"clearance={VIS_TOPDOWN_HEIGHT_M:.2f}"
            )
            top_cam = make_camera(Camera, "top_90", top_pos, top_look)
            top_render_png = vis_dir / f"{frame_name}_topview_3d.png"
            render_to_file_rt201(
                scene, top_cam, top_render_png,
                paths=paths if VIS_RENDER_RAYS else None,
            )
            status_line(f"Saved 3D top-down render: {top_render_png}")
        except Exception as exc:
            warning_line(f"{traj_dir.name}/{frame_name}: 3D top-down render failed: {exc}")

    if not force:
        try:
            frame_int = int(frame_name)
        except ValueError:
            frame_int = 0
        vis2d_due = VIS_EVERY_N > 0 and frame_int % VIS_EVERY_N == 0
        render_every = max(1, int(VIS_RENDER_SCENE_EVERY_N))
        render3d_due = (
            VIS_RENDER_SCENE and scene is not None and Camera is not None
            and frame_int % render_every == 0
        )
        if not vis2d_due and not render3d_due:
            return
        if not vis2d_due and render3d_due:
            # Only the 3D render is due at this frame; the 2x2 figure stays
            # on its own VIS_EVERY_N cadence.
            _render_scene_topview(traj_dir, frame_name, scene, Camera, paths)
            return

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    vis_dir = stage3_root(traj_dir) / "vis_imgs"
    vis_dir.mkdir(parents=True, exist_ok=True)

    def _is_valid_png(path: Path) -> bool:
        try:
            path = Path(path)
            if not path.is_file() or path.stat().st_size < 16:
                return False
            with open(path, "rb") as f:
                return f.read(8) == b"\x89PNG\r\n\x1a\n"
        except Exception:
            return False

    def _atomic_savefig_png(fig, final_path: Path, dpi: int = 150, bbox_inches="tight"):
        final_path = Path(final_path)
        final_path.parent.mkdir(parents=True, exist_ok=True)
        tmp_path = final_path.with_name(
            f".{final_path.stem}.tmp_{os.getpid()}_{int(time.time() * 1e6)}.png"
        )
        fig.savefig(str(tmp_path), dpi=dpi, bbox_inches=bbox_inches, format="png")
        if not _is_valid_png(tmp_path):
            raise RuntimeError(f"Final visualization is not a valid PNG: {tmp_path}")
        os.replace(str(tmp_path), str(final_path))
        return final_path

    def _draw_placeholder(ax, title: str, lines=None):
        ax.axis("off")
        ax.set_title(title, fontsize=10, fontweight="bold")
        lines = [] if lines is None else lines
        if isinstance(lines, str):
            lines = [lines]
        ax.text(
            0.5, 0.5,
            "\n".join(str(x) for x in lines),
            ha="center",
            va="center",
            fontsize=10,
            transform=ax.transAxes,
            wrap=True,
        )

    def _fmt_vec(v):
        v = np.asarray(v, dtype=float).reshape(3)
        return f"[{v[0]:.2f}, {v[1]:.2f}, {v[2]:.2f}]"

    def _draw_topview_plan(ax, path_rays=None):
        tx = np.asarray(pose_info["tx_pos"], dtype=float).reshape(3)
        rx = np.asarray(pose_info["rx_pos"], dtype=float).reshape(3)
        tx_f = np.asarray(pose_info["tx_forward"], dtype=float).reshape(3)
        rx_f = np.asarray(pose_info["rx_forward"], dtype=float).reshape(3)

        mid = 0.5 * (tx + rx)
        dist_xy = float(np.linalg.norm((rx - tx)[:2]))
        half = max(dist_xy * 0.65 + 25.0, 30.0)

        ax.plot([tx[0], rx[0]], [tx[1], rx[1]], linewidth=2, label="Tx-Rx link")
        ax.scatter([tx[0]], [tx[1]], s=110, marker="^", label="Tx / RSU")
        ax.scatter([rx[0]], [rx[1]], s=110, marker="o", label="Rx / CAV")
        ax.text(tx[0], tx[1], f"  TX\n  z={tx[2]:.1f}", fontsize=9, va="bottom")
        ax.text(rx[0], rx[1], f"  RX\n  z={rx[2]:.1f}", fontsize=9, va="bottom")

        for origin, fwd, label in [
            (tx, tx_f, "Tx heading"),
            (rx, rx_f, "Rx heading"),
        ]:
            f = np.asarray(fwd, dtype=float)
            n_xy = float(np.linalg.norm(f[:2]))
            if n_xy > 1e-9:
                L = max(min(dist_xy * 0.12, 8.0), 3.0)
                f = f / max(float(np.linalg.norm(f)), 1e-12)
                ax.arrow(
                    origin[0], origin[1],
                    f[0] * L, f[1] * L,
                    head_width=max(L * 0.18, 0.6),
                    head_length=max(L * 0.25, 0.8),
                    length_includes_head=True,
                    linewidth=1.5,
                    label=label,
                )

        # Top-3 strongest representative paths. Reflection vertices are drawn
        # when available; LOS falls back to the direct Tx-Rx segment.
        if path_rays:
            for k, ray in enumerate(path_rays):
                points = np.asarray(ray.get("points", []), dtype=float)
                d = np.asarray(ray["dir"], dtype=float).reshape(3)
                n = float(np.linalg.norm(d[:2]))
                if points.ndim == 2 and points.shape[0] >= 2 and points.shape[1] == 3:
                    color = "#d62728" if ray.get("los") else ("#ff7f0e" if k == 0 else "#888888")
                    lbl = (f"path#{k + 1} az={ray['az_deg']:+.0f} deg "
                           f"{ray['db']:+.0f}dB" + (" LOS" if ray.get("los") else ""))
                    ax.plot(
                        points[:, 0], points[:, 1], color=color, linewidth=1.6,
                        alpha=0.9, marker="." if points.shape[0] > 2 else None,
                        label=lbl,
                    )
                    continue
                if n < 1e-9:
                    continue
                L = max(dist_xy * 0.55, 10.0)
                if ray.get("los"):
                    color = "#d62728"
                elif k == 0:
                    color = "#ff7f0e"
                else:
                    color = "#888888"
                lbl = (f"path#{k + 1} az={ray['az_deg']:+.0f}° "
                       f"{ray['db']:+.0f}dB" + (" LOS" if ray.get("los") else ""))
                ax.arrow(
                    tx[0], tx[1],
                    d[0] / n * L, d[1] / n * L,
                    head_width=max(L * 0.05, 0.5),
                    head_length=max(L * 0.09, 0.7),
                    length_includes_head=True,
                    linewidth=1.6, color=color, alpha=0.9,
                    label=lbl,
                )

        ax.set_aspect("equal", adjustable="box")
        ax.set_xlim(mid[0] - half, mid[0] + half)
        ax.set_ylim(mid[1] - half, mid[1] + half)
        ax.set_xlabel("Sionna X / CARLA X (m)")
        ax.set_ylabel("Sionna Y = -CARLA Y (m)")
        ax.grid(True, linewidth=0.4)
        ax.legend(loc="best", fontsize=8)
        ax.set_title("Top-view Tx/Rx geometry", fontsize=10, fontweight="bold")

    def _top3_path_rays():
        """Top-3 strongest representative paths with interaction geometry."""
        if paths is None:
            return []
        try:
            from .path_extractor import _extract_single_link_paths
            from .raytracing import device_basis_from_forward, angles_to_unit_vector
            link_specs = list(pose_info.get("link_specs") or [{"tx_idx": 0, "rx_idx": 0}])
            spec = link_specs[0]
            rx_data = _extract_single_link_paths(
                paths, int(spec.get("tx_idx", 0)), int(spec.get("rx_idx", 0)), pose_info)
            a = np.abs(np.asarray(rx_data.get("a", []), dtype=np.complex64))
            if a.size == 0:
                return []
            theta_t = np.asarray(rx_data["theta_t"], dtype=float)
            phi_t = np.asarray(rx_data["phi_t"], dtype=float)
            los_flags = np.asarray(rx_data.get("los", np.zeros(a.size, dtype=bool)), dtype=bool)
            dedup_keep = np.asarray(
                rx_data.get("dedup_keep", np.ones(a.size, dtype=bool)), dtype=bool)
            vertices = np.asarray(
                rx_data.get("interaction_vertices_m", np.zeros((a.size, 0, 3))),
                dtype=float,
            )
            fwd = np.asarray(
                pose_info.get("tx_forward", pose_info["tx_forward_all"][0]),
                dtype=float).reshape(3)
            B = device_basis_from_forward(fwd)
            rays = []
            candidates = (
                np.flatnonzero(dedup_keep)
                if dedup_keep.shape == a.shape else np.arange(a.size)
            )
            ranked = candidates[np.argsort(a[candidates])[-3:][::-1]]
            tx = np.asarray(pose_info["tx_pos"], dtype=float).reshape(3)
            rx = np.asarray(pose_info["rx_pos"], dtype=float).reshape(3)
            for i in ranked:
                u = angles_to_unit_vector(theta_t[i], phi_t[i])
                interaction_points = []
                if vertices.ndim == 3 and i < vertices.shape[0]:
                    interaction_points = [
                        v for v in vertices[i] if np.isfinite(v).all()
                    ]
                rays.append({
                    "dir": (B @ u).astype(float),
                    "points": np.asarray([tx, *interaction_points, rx], dtype=float),
                    "az_deg": float(np.degrees(phi_t[i])),
                    "db": float(20 * np.log10(a[i] + 1e-12)),
                    "los": bool(los_flags[i]) if i < los_flags.size else False,
                })
            return rays
        except Exception:
            return []

    def _draw_link_summary(ax):
        ax.axis("off")

        tx = np.asarray(pose_info["tx_pos"], dtype=float).reshape(3)
        rx = np.asarray(pose_info["rx_pos"], dtype=float).reshape(3)
        tx_f = np.asarray(pose_info["tx_forward"], dtype=float).reshape(3)
        rx_f = np.asarray(pose_info["rx_forward"], dtype=float).reshape(3)

        d3 = float(np.linalg.norm(rx - tx))
        dxy = float(np.linalg.norm((rx - tx)[:2]))
        dz = float(rx[2] - tx[2])

        # Extract path statistics from the paths object.
        path_stats = []
        if paths is not None:
            try:
                a_raw, tau_raw = get_cir(paths)
                a_np = np.asarray(tensor_to_numpy(a_raw)).ravel()
                tau_np = np.asarray(tensor_to_numpy(tau_raw)).ravel()
                if a_np.size > 0:
                    gains_dB = 20.0 * np.log10(np.maximum(np.abs(a_np), 1e-30))
                    n_paths_val = int(a_np.size)
                    path_stats = [
                        f"n_paths: {n_paths_val}",
                        f"gain max: {np.max(gains_dB):.1f} dB",
                        f"gain min: {np.min(gains_dB):.1f} dB",
                        f"gain mean: {np.mean(gains_dB):.1f} dB",
                        f"tau max: {np.max(tau_np)*1e9:.1f} ns",
                        f"tau min: {np.min(tau_np)*1e9:.1f} ns",
                    ]
                else:
                    path_stats = ["n_paths: 0"]
            except Exception:
                path_stats = ["path stats: unavailable"]

        valid_configs = []
        for cfg in ARRAY_CONFIG_LIST:
            H = h_by_config.get(cfg["name"])
            if H is None:
                continue
            H = np.asarray(H)
            if H.ndim == 3:
                valid_configs.append(f"{cfg['rx_h']}x{cfg['rx_v']} Rx, {cfg['tx_h']}x{cfg['tx_v']} Tx: H={tuple(H.shape)}")

        lines = [
            f"Trajectory: {traj_dir.name}",
            f"Frame: {frame_name}",
            "",
            f"Tx pos: {_fmt_vec(tx)}",
            f"Rx pos: {_fmt_vec(rx)}",
            f"3D distance: {d3:.2f} m",
            f"XY distance: {dxy:.2f} m",
            f"Rx height - Tx height: {dz:.2f} m",
            "",
            f"Tx forward: {_fmt_vec(tx_f)}",
            f"Rx forward: {_fmt_vec(rx_f)}",
            "",
            f"Tx source: {pose_info.get('tx_source', '')}",
            f"Rx source: {pose_info.get('rx_source', '')}",
            "",
            "Path statistics:",
        ]
        lines.extend(path_stats if path_stats else ["n/a"])
        lines.append("")
        lines.append("Generated channel configs:")
        if valid_configs:
            lines.extend(valid_configs[:4])
        else:
            lines.append("No channel matrix available for visualization.")

        ax.set_title("Link / channel summary", fontsize=10, fontweight="bold")
        ax.text(
            0.02, 0.98,
            "\n".join(lines),
            ha="left",
            va="top",
            fontsize=9,
            family="monospace",
            transform=ax.transAxes,
            wrap=True,
        )

    def _draw_bartlett(ax, P_db: np.ndarray, title: str, *, front_only: bool = False):
        P_db = np.asarray(P_db, dtype=np.float32)
        if P_db.size == 0 or not np.isfinite(P_db).any():
            _draw_placeholder(ax, title, ["Invalid Bartlett spectrum."])
            return None

        peak = np.unravel_index(np.nanargmax(P_db), P_db.shape)
        lon = -180.0 + (peak[1] + 0.5) * 360.0 / P_db.shape[1]
        lat = -90.0 + (peak[0] + 0.5) * 180.0 / P_db.shape[0]

        im = ax.imshow(
            P_db,
            origin="lower",
            aspect="auto",
            extent=[-180, 180, -90, 90],
            vmin=BARTLETT_DB_FLOOR,
            vmax=0.0,
        )
        ax.scatter([lon], [lat], marker="x", s=70)
        if front_only:
            ax.set_xlim(-90.0, 90.0)
        ax.set_xlabel("Local longitude lon (deg, +right)")
        ax.set_ylabel("Local latitude lat (deg, +down)")
        scope = " · front hemisphere" if front_only else ""
        ax.set_title(
            f"{title}{scope}\npeak lon/lat=({lon:.1f}, {lat:.1f}) deg",
            fontsize=10,
            fontweight="bold",
        )
        return im

    def _choose_first_valid_channel():
        for cfg in ARRAY_CONFIG_LIST:
            H = h_by_config.get(cfg["name"])
            if H is None:
                continue
            H = np.asarray(H)
            if H.ndim == 3 and H.shape[0] >= 1 and H.shape[1] >= 1 and H.shape[2] >= 1:
                return cfg, H
        return None, None

    try:
        fig, axes = plt.subplots(2, 2, figsize=(16, 10))

        # [0,0] deterministic top-view plan
        _draw_topview_plan(axes[0, 0], path_rays=_top3_path_rays())

        # [0,1] deterministic summary panel
        _draw_link_summary(axes[0, 1])

        chosen_cfg, chosen_H = _choose_first_valid_channel()
        mid_sc = int(chosen_H.shape[-1] // 2) if chosen_H is not None and chosen_H.ndim >= 3 else 0

        # [1,0] Rx Bartlett — show first user, or multi-user grid if N>1
        ax = axes[1, 0]
        if chosen_cfg is not None and chosen_H is not None and chosen_H.shape[0] > 1 and chosen_H.shape[1] > 1:
            # Single-user Bartlett for the 2x2 figure (user 0)
            rx_vec_u0 = chosen_H[0, :, 0, mid_sc] if chosen_H.ndim == 4 else chosen_H[:, 0, mid_sc]
            rx_front_only = bool(ARRAY_IF_FILTER and int(chosen_cfg["rx_rows"]) * int(chosen_cfg["rx_cols"]) > 1)
            P_rx_u0 = bartlett_spectrum_from_vector(
                rx_vec_u0, chosen_cfg["rx_rows"], chosen_cfg["rx_cols"],
                if_filter=rx_front_only,
            )
            im_rx = _draw_bartlett(
                ax, P_rx_u0,
                f"Rx Bartlett {chosen_cfg['rx_h']}x{chosen_cfg['rx_v']} (user 0)",
                front_only=rx_front_only,
            )
            if im_rx is not None:
                fig.colorbar(im_rx, ax=ax, fraction=0.046, pad=0.04, label="dB")

            # Multi-user Rx Bartlett: separate figure
            if NUM_CAVS > 1 and chosen_H.ndim == 4 and chosen_H.shape[0] > 1:
                nrows = int(math.ceil(math.sqrt(NUM_CAVS)))
                ncols = int(math.ceil(NUM_CAVS / nrows))
                fig_mu, axes_mu = plt.subplots(nrows, ncols, figsize=(4*ncols, 3*nrows))
                axes_mu = np.atleast_1d(axes_mu).flatten()
                for u in range(NUM_CAVS):
                    ax_mu = axes_mu[u]
                    rx_vec_mu = chosen_H[u, :, 0, mid_sc]
                    P_mu = bartlett_spectrum_from_vector(
                        rx_vec_mu, chosen_cfg["rx_rows"], chosen_cfg["rx_cols"],
                        if_filter=rx_front_only,
                    )
                    peak_mu = np.unravel_index(np.nanargmax(P_mu), P_mu.shape)
                    lon_mu = -180.0 + (peak_mu[1] + 0.5) * 360.0 / P_mu.shape[1]
                    lat_mu = -90.0 + (peak_mu[0] + 0.5) * 180.0 / P_mu.shape[0]
                    ax_mu.imshow(P_mu, origin="lower", aspect="auto", extent=[-180, 180, -90, 90],
                                 vmin=BARTLETT_DB_FLOOR, vmax=0.0)
                    ax_mu.scatter([lon_mu], [lat_mu], marker="x", s=40)
                    if rx_front_only:
                        ax_mu.set_xlim(-90.0, 90.0)
                    ax_mu.set_xlabel("Lon (deg)")
                    ax_mu.set_ylabel("Lat (deg)")
                    ax_mu.set_title(f"User {u} peak=({lon_mu:.1f},{lat_mu:.1f})", fontsize=9)
                for u in range(NUM_CAVS, len(axes_mu)):
                    axes_mu[u].axis("off")
                fig_mu.suptitle(f"{traj_dir.name} frame {frame_name} — Rx Bartlett all users", fontsize=11)
                fig_mu.tight_layout()
                mu_png = vis_dir / f"{frame_name}_bartlett_all_users.png"
                _atomic_savefig_png(fig_mu, mu_png, dpi=120)
                plt.close(fig_mu)
        else:
            _draw_placeholder(ax, "Rx Bartlett AoA spectrum",
                ["Skipped: Rx array has only one antenna."])

        # [1,1] Tx Bartlett (user 0)
        ax = axes[1, 1]
        if chosen_cfg is not None and chosen_H is not None:
            tx_ant_dim = 1 if chosen_H.ndim == 3 else 2
            if chosen_H.shape[tx_ant_dim] > 1:
                if chosen_H.ndim == 4:
                    tx_vec = np.conj(chosen_H[0, 0, :, mid_sc])
                else:
                    tx_vec = np.conj(chosen_H[0, :, mid_sc])
                tx_front_only = bool(
                    ARRAY_IF_FILTER
                    and int(chosen_cfg["tx_rows"]) * int(chosen_cfg["tx_cols"]) > 1
                )
                P_tx = bartlett_spectrum_from_vector(
                    tx_vec,
                    chosen_cfg["tx_rows"],
                    chosen_cfg["tx_cols"],
                    if_filter=tx_front_only,
                )
                im_tx = _draw_bartlett(
                    ax,
                    P_tx,
                    f"Tx Bartlett AoD {chosen_cfg['tx_h']}x{chosen_cfg['tx_v']}",
                    front_only=tx_front_only,
                )
                if im_tx is not None:
                    fig.colorbar(im_tx, ax=ax, fraction=0.046, pad=0.04, label="dB")
            else:
                _draw_placeholder(
                    ax,
                    "Tx Bartlett AoD spectrum",
                    ["Skipped: Tx array has only one antenna."],
                )
        else:
            _draw_placeholder(
                ax,
                "Tx Bartlett AoD spectrum",
                ["Skipped: no channel matrix is available."],
            )

        suffix = "_warn" if force else ""
        final_png = vis_dir / f"{frame_name}_vis{suffix}.png"

        fig.suptitle(f"{traj_dir.name}  frame {frame_name}", fontsize=12, fontweight="bold")
        fig.tight_layout()
        _atomic_savefig_png(fig, final_png, dpi=150, bbox_inches="tight")
        plt.close(fig)

        status_line(f"Saved visualization: {final_png}")

        # The optional 3D renderer receives only VIS_RENDER_MAX_PATHS paths.
        # This prevents visualization from blocking production after complete
        # raw paths and channels have already been saved.
        frame_int_val = 0
        try:
            frame_int_val = int(frame_name)
        except ValueError:
            pass
        render_every = max(1, int(VIS_RENDER_SCENE_EVERY_N))
        if (VIS_RENDER_SCENE and scene is not None and Camera is not None
                and frame_int_val % render_every == 0):
            _render_scene_topview(traj_dir, frame_name, scene, Camera, paths)

    except Exception as exc:
        warning_line(f"{traj_dir.name}/{frame_name}: save_visualizations failed: {exc}")
        try:
            traceback.print_exc()
        except Exception:
            pass
    finally:
        try:
            plt.close("all")
        except Exception:
            pass


__all__=[k for k in globals() if not k.startswith("__")]
