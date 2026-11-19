# -*- coding: utf-8 -*-
from __future__ import annotations

from importlib import metadata

from .sionna_config import *
from .scene_manager import set_scene_frequency_with_material_report

SIONNA_RT_REQUIRED_VERSION = "2.0.1"


def require_sionna_rt_201() -> str:
    """Fail fast unless the pinned Sionna RT runtime is available."""
    try:
        installed = metadata.version("sionna-rt")
    except metadata.PackageNotFoundError as exc:
        raise RuntimeError(
            "SiGMA Stage3 requires sionna-rt==2.0.1, but sionna-rt is not installed"
        ) from exc
    if installed != SIONNA_RT_REQUIRED_VERSION:
        raise RuntimeError(
            f"SiGMA Stage3 requires sionna-rt=={SIONNA_RT_REQUIRED_VERSION}; "
            f"found {installed}"
        )
    return installed


def angles_to_unit_vector(theta: np.ndarray, phi: np.ndarray) -> np.ndarray:
    theta = np.asarray(theta, dtype=float)
    phi = np.asarray(phi, dtype=float)
    return np.stack([
        np.sin(theta) * np.cos(phi),
        np.sin(theta) * np.sin(phi),
        np.cos(theta),
    ], axis=0)


def unit_vector_to_angles(u: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    u = np.asarray(u, dtype=float)
    n = np.linalg.norm(u, axis=0, keepdims=True)
    u = u / np.maximum(n, 1e-12)
    theta = np.arccos(np.clip(u[2, :], -1.0, 1.0))
    phi = np.arctan2(u[1, :], u[0, :])
    return theta.astype(np.float32), phi.astype(np.float32)


def device_basis_from_forward(forward: np.ndarray) -> np.ndarray:
    """Return device LCS axes expressed in Sionna GCS as matrix columns."""
    f = np.asarray(forward, dtype=float).reshape(3)
    f = f / max(float(np.linalg.norm(f)), 1e-12)
    world_up = np.array([0.0, 0.0, 1.0], dtype=float)
    right = np.cross(f, world_up)
    if np.linalg.norm(right) < 1e-9:
        right = np.array([1.0, 0.0, 0.0], dtype=float)
    right = right / max(float(np.linalg.norm(right)), 1e-12)
    down = np.cross(f, right)
    down = down / max(float(np.linalg.norm(down)), 1e-12)
    return np.stack([right, down, f], axis=1)


def global_angles_to_local_angles(glob_theta, glob_phi, forward: np.ndarray):
    u_gcs = angles_to_unit_vector(glob_theta, glob_phi)
    u_lcs = device_basis_from_forward(forward).T @ u_gcs
    return unit_vector_to_angles(u_lcs)


def tensor_to_numpy(x):
    """Convert NumPy/Dr.Jit/framework tensors to a NumPy view for dataset I/O."""
    if x is None:
        return None
    if isinstance(x, np.ndarray):
        return x
    if hasattr(x, "detach") and hasattr(x, "cpu"):
        try:
            return np.asarray(x.detach().cpu().numpy())
        except Exception:
            pass
    if hasattr(x, "numpy"):
        try:
            return np.asarray(x.numpy())
        except Exception:
            pass
    try:
        import drjit as dr
        return np.asarray(dr.detach(x))
    except Exception:
        return np.asarray(x)


def import_sionna_objects():
    """Import the single supported Sionna RT 2.0.1 API surface."""
    require_sionna_rt_201()
    import mitsuba as mi

    variant = os.environ.get("MITSUBA_VARIANT", "").strip()
    if variant and mi.variant() != variant:
        mi.set_variant(variant)
    if variant.startswith("llvm_"):
        import drjit as dr

        thread_count = max(
            1,
            int(os.environ.get(
                "SIONNA_CPU_THREADS",
                os.environ.get("SLURM_CPUS_PER_TASK", "1"),
            )),
        )
        dr.set_thread_count(thread_count)

    from sionna.rt import (
        Camera,
        PathSolver,
        PlanarArray,
        Receiver,
        Transmitter,
        load_scene,
    )
    return load_scene, PlanarArray, Transmitter, Receiver, Camera, PathSolver


def make_camera(CameraClass, name: str, position, look_at):
    """Create a Sionna RT 2.0.1 camera. ``name`` is retained for call-site API stability."""
    del name
    pos = [float(x) for x in np.asarray(position).reshape(3)]
    target = [float(x) for x in np.asarray(look_at).reshape(3)]
    return CameraClass(position=pos, look_at=target)


def set_device_look_at(device, pos: np.ndarray, forward: np.ndarray, length: float = 6.0):
    pos = np.asarray(pos, dtype=float).reshape(3)
    f = np.asarray(forward, dtype=float).reshape(3)
    f = f / max(float(np.linalg.norm(f)), 1e-12)
    target = pos + f * length
    device.look_at(target.tolist())
    return target


def create_scene_for_ray_tracing(safe_xml: Path, pose_info: dict):
    """Create the 1x1 tracing scene used for later multi-array reconstruction."""
    load_scene, PlanarArray, Transmitter, Receiver, Camera, PathSolver = import_sionna_objects()

    scene = load_scene(
        str(safe_xml),
        merge_shapes=MERGE_SHAPES,
        remove_duplicate_vertices=REMOVE_DUPLICATE_VERTICES,
    )
    set_scene_frequency_with_material_report(scene, FREQUENCY_HZ)

    # Sionna RT 2.0.1 models diffuse scattering through diffuse_reflection plus
    # each material's scattering_coefficient. The configured coefficient is applied only when diffuse reflection is
    # enabled; otherwise the effective value is zero to preserve specular power.
    for rm in scene.radio_materials.values():
        rm.scattering_coefficient = EFFECTIVE_SCATTERING_COEFFICIENT

    scene.tx_array = PlanarArray(
        num_rows=1,
        num_cols=1,
        vertical_spacing=ARRAY_SPACING_WL,
        horizontal_spacing=ARRAY_SPACING_WL,
        pattern=ANTENNA_PATTERN,
        polarization=POLARIZATION,
    )
    scene.rx_array = PlanarArray(
        num_rows=1,
        num_cols=1,
        vertical_spacing=ARRAY_SPACING_WL,
        horizontal_spacing=ARRAY_SPACING_WL,
        pattern=ANTENNA_PATTERN,
        polarization=POLARIZATION,
    )

    tx_pos_arr = np.asarray(pose_info.get("tx_pos_all", pose_info["tx_pos"]), dtype=np.float32).reshape(-1, 3)
    tx_fwd_arr = np.asarray(pose_info.get("tx_forward_all", pose_info["tx_forward"]), dtype=np.float32).reshape(-1, 3)
    num_tx = int(pose_info.get("num_tx", tx_pos_arr.shape[0]))
    if tx_pos_arr.shape[0] < num_tx or tx_fwd_arr.shape[0] < num_tx:
        raise ValueError(
            f"pose_info has {tx_pos_arr.shape[0]} Tx positions and {tx_fwd_arr.shape[0]} "
            f"Tx forwards, expected {num_tx}"
        )
    for i in range(num_tx):
        tx_name = "tx" if num_tx == 1 else f"tx_{i}"
        tx = Transmitter(
            name=tx_name,
            position=tx_pos_arr[i].tolist(),
            display_radius=DEVICE_DISPLAY_RADIUS_M,
        )
        scene.add(tx)
        set_device_look_at(tx, tx_pos_arr[i], tx_fwd_arr[i])

    rx_pos_arr = np.asarray(pose_info.get("rx_pos_all", pose_info["rx_pos"]), dtype=np.float32).reshape(-1, 3)
    rx_fwd_arr = np.asarray(pose_info.get("rx_forward_all", pose_info["rx_forward"]), dtype=np.float32).reshape(-1, 3)
    num_users = int(pose_info.get("num_users", rx_pos_arr.shape[0]))
    if rx_pos_arr.shape[0] < num_users or rx_fwd_arr.shape[0] < num_users:
        raise ValueError(
            f"pose_info has {rx_pos_arr.shape[0]} Rx positions and {rx_fwd_arr.shape[0]} "
            f"Rx forwards, expected {num_users}"
        )
    for i in range(num_users):
        rx = Receiver(
            name=f"rx_{i}",
            position=rx_pos_arr[i].tolist(),
            display_radius=DEVICE_DISPLAY_RADIUS_M,
        )
        scene.add(rx)
        set_device_look_at(rx, rx_pos_arr[i], rx_fwd_arr[i])

    return scene, Camera, PathSolver


def compute_paths(scene, PathSolver):
    """Compute paths with the pinned Sionna RT 2.0.1 PathSolver interface."""
    solver = PathSolver()
    return solver(
        scene=scene,
        max_depth=int(RT_MAX_DEPTH),
        max_num_paths_per_src=int(RT_MAX_NUM_PATHS_PER_SRC),
        samples_per_src=int(RT_SAMPLES_PER_SRC),
        synthetic_array=bool(RT_SYNTHETIC_ARRAY),
        los=bool(RT_LOS),
        specular_reflection=bool(RT_SPECULAR_REFLECTION),
        diffuse_reflection=bool(RT_DIFFUSE_REFLECTION),
        refraction=bool(RT_REFRACTION),
        diffraction=bool(RT_DIFFRACTION),
        edge_diffraction=bool(RT_EDGE_DIFFRACTION),
        seed=int(RT_SEED),
    )


def get_cir(paths):
    """Return the Sionna RT 2.0.1 baseband CIR and unnormalized delays as NumPy."""
    a, tau = paths.cir(normalize_delays=False, out_type="numpy")
    return np.asarray(a), np.asarray(tau)


__all__ = [k for k in globals() if not k.startswith("__")]
