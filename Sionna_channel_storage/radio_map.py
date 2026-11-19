"""RadioMapSolver adapter extracted from validation/xml_radio_map_snapshot.py.

Stage III saves numerical receiver-plane coverage for the same frozen frame
as CSI through a numerical RadioMapSolver interface.
"""
from pathlib import Path
import numpy as np
from .raytracing import create_scene_for_ray_tracing
from .sionna_config import (RT_SEED, RT_LOS, RT_SPECULAR_REFLECTION,
                           RT_DIFFUSE_REFLECTION, RT_REFRACTION, RT_DIFFRACTION)


def save_radio_map(xml: Path, pose: dict, settings: dict, output: Path):
    from sionna.rt import RadioMapSolver
    scene, _, _ = create_scene_for_ray_tracing(xml, pose)
    result = RadioMapSolver()(
        scene, center=settings["center"], orientation=settings.get("orientation", [0, 0, 0]),
        size=settings["size"], cell_size=settings.get("cell_size", [1, 1]),
        samples_per_tx=int(settings.get("samples_per_tx", 100000)),
        max_depth=int(settings.get("max_depth", 3)),
        los=RT_LOS, specular_reflection=RT_SPECULAR_REFLECTION,
        diffuse_reflection=RT_DIFFUSE_REFLECTION,
        refraction=RT_REFRACTION, diffraction=RT_DIFFRACTION, seed=int(RT_SEED))
    output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(output, path_gain=np.asarray(result.path_gain),
                        cell_centers=np.asarray(result.cell_centers),
                        tx_position=np.asarray(pose["tx_pos"]), seed=RT_SEED,
                        center=np.asarray(settings["center"]), size=np.asarray(settings["size"]),
                        cell_size=np.asarray(settings.get("cell_size", [1, 1])),
                        orientation=np.asarray(settings.get("orientation", [0, 0, 0])),
                        los=RT_LOS, specular_reflection=RT_SPECULAR_REFLECTION,
                        diffuse_reflection=RT_DIFFUSE_REFLECTION,
                        refraction=RT_REFRACTION, diffraction=RT_DIFFRACTION)
    return output
