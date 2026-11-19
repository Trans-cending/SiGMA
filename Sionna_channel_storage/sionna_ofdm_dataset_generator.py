# -*- coding: utf-8 -*-
"""SiGMA Stage 3 OFDM dataset-generation entry point.

Run through validated YAML:
    python -m sigma stage3 -c configs/town03.yaml

nearfield traces physical arrays; per-frame errors preserve outputs and fail the stage.
"""

from .sionna_config import *
from .scene_manager import *
from .pose_loader import *
from .raytracing import *
from .path_extractor import *
from .link_manager import *
from .channel_processor import *
from .radio_writer import *
from .visualization import *
from .ris_pipeline import *
from .explicit_array import trace_explicit_channel
from .radio_map import save_radio_map

def list_trajectories() -> list[Path]:
    """List canonical trajectories with enumeration independent of scheduler variables."""
    root = DATASET_ROOT / "stage1_multimodal"
    if not root.is_dir():
        raise FileNotFoundError(f"Missing Stage II trajectory root: {root}")
    return [path for path in sorted(root.iterdir())
            if path.is_dir() and re.fullmatch(r"\d{6}", path.name)
            and dynamic_xml_dir(path).is_dir()
            and stage1_world_state_dir(path).is_dir()]

def list_frames(traj_dir: Path) -> list[str]:
    frames = []
    for p in sorted(dynamic_xml_dir(traj_dir).glob("*.xml")):
        if p.name.endswith("_sionna_safe.xml") or p.name.endswith("_merged.xml"):
            continue
        frame = p.stem
        # Validate that the matching 100 Hz stage1 world_state exists. Dynamic
        # XML names may be traj_000004_000125.xml or 000125.xml; stage3 output
        # always stores only the six-digit frame index.
        stage1_world_state_path(traj_dir, frame)
        frames.append(frame)
    return frames

def process_frame(traj_dir: Path, frame_name: str):
    frame_idx = frame_idx_from_name(frame_name)
    world_state = read_yaml(stage1_world_state_path(traj_dir, frame_name))
    selected_nodes, node_aliases = _select_stage1_nodes(world_state)
    links = build_radio_links(node_aliases)
    if not links:
        raise RuntimeError(f"{traj_dir.name}/{frame_name}: no configured radio links available from stage1 nodes")
    write_radio_index(node_aliases=node_aliases, links=links)

    ris_cfg, ris_metadata = load_ris_stage3_config(
        traj_dir, stage3_frequency_hz=FREQUENCY_HZ)

    radio_map_cfg = yaml.safe_load(os.environ.get("SIGMA_RADIO_MAP_YAML", "{}")) or {}
    maps_complete = (not radio_map_cfg.get("enabled") or all(
        (stage3_root(traj_dir) / "radio_maps" / tx_alias / f"rm_{frame_idx:06d}.npz").is_file()
        for _, tx_alias, _ in group_links_by_tx(links)))
    if (SKIP_EXISTING_FRAMES and maps_complete
            and all_outputs_complete(traj_dir, frame_idx, links)
            and all_ris_outputs_complete(
                traj_dir, frame_idx, links, ARRAY_CONFIG_LIST, ris_cfg)):
        return "skip all"

    work_dir = stage3_root(traj_dir) / WORK_SUBDIR / f"{frame_idx:06d}"
    safe_xml = make_safe_merged_xml(traj_dir, frame_name, work_dir)

    total_paths = []
    saved_channels = 0
    traced_batches = 0
    vis_context = None
    vis_h_by_config = {}

    for link_type, tx_alias, link_list in group_links_by_tx(links):
        cached_paths = {}
        missing_path_links = []
        for link in link_list:
            if SKIP_EXISTING_FRAMES and path_frame_complete(traj_dir, frame_idx, link_type, link):
                cached_paths[link["link_id"]], _cached_pose = load_path_frame(traj_dir, frame_idx, link_type, link)
            else:
                missing_path_links.append(link)

        computed_paths_by_link = {}
        if missing_path_links:
            group_pose = load_link_group_pose_from_stage1(traj_dir, frame_name, missing_path_links, selected_nodes)
            scene, Camera, PathSolver = create_scene_for_ray_tracing(safe_xml, group_pose)
            n_tx = len({str(link["tx"]) for link in missing_path_links})
            n_rx = len({str(link["rx"]) for link in missing_path_links})
            progress_line(
                f"  {traj_dir.name} | frame {frame_idx:06d} | {link_type}/{tx_alias} "
                f"| computing paths for {len(missing_path_links)} links ({n_tx} tx, {n_rx} rx) ...")
            group_paths = compute_paths(scene, PathSolver)
            group_paths_data = extract_paths_data(group_paths, group_pose)
            traced_batches += 1
            for link_i, link in enumerate(missing_path_links):
                single_paths = paths_data_for_link(group_paths_data, link_i)
                rx0 = single_paths.get("rx_users", [_empty_rx_path_data()])[0]
                keep = np.asarray(rx0.get("dedup_keep", []), dtype=bool)
                raw_n = int(np.asarray(rx0.get("a", [])).shape[0])
                duplicate_n = int(raw_n - np.count_nonzero(keep)) if keep.shape == (raw_n,) else 0
                if duplicate_n > 0:
                    progress_line(
                        f"  [PATH_DEDUP] {traj_dir.name} | frame {frame_idx:06d} | "
                        f"{link_type}/{link['link_id']} raw={raw_n} "
                        f"channel_unique={raw_n - duplicate_n} duplicates={duplicate_n}"
                    )
                save_path_frame(traj_dir, frame_idx, link_type, link, single_paths)
                computed_paths_by_link[link["link_id"]] = single_paths
            vis_context = (scene, Camera, group_paths, group_pose)

        for rx_i, link in enumerate(link_list):
            paths_data = computed_paths_by_link.get(link["link_id"]) or cached_paths.get(link["link_id"])
            if paths_data is None:
                paths_data, _cached_pose = load_path_frame(traj_dir, frame_idx, link_type, link)
            pose_info = load_link_pose_from_stage1(traj_dir, frame_name, link, selected_nodes)

            n_paths = int(paths_data.get("rx_users", [_empty_rx_path_data()])[0]["a"].shape[0])
            total_paths.append(n_paths)

            for cfg in ARRAY_CONFIG_LIST:
                if link_type not in cfg.get("enabled_link_types", []):
                    continue
                if (SKIP_EXISTING_FRAMES
                        and channel_frame_complete(
                            traj_dir, frame_idx, cfg, link_type, link)):
                    h_freq = load_channel_frame(
                        traj_dir, frame_idx, cfg, link_type, link)
                else:
                    if CHANNEL_RECONSTRUCTION_MODE == "nearfield":
                        explicit_path = (stage3_root(traj_dir) / "explicit_paths" / cfg["id"]
                                         / link_type / link["link_id"] / f"p_{frame_idx:06d}.npz")
                        h_freq = trace_explicit_channel(safe_xml, pose_info, cfg, explicit_path)
                    else:
                        h_freq = synthesize_ofdm_channel(paths_data, cfg, pose_info)
                    save_channel_frame(traj_dir, frame_idx, cfg, link_type, link, h_freq)
                    saved_channels += 1
                if ris_metadata is not None and link_type == "rc":
                    process_ris_for_direct_link(
                        traj_dir=traj_dir,
                        frame_idx=frame_idx,
                        safe_xml=safe_xml,
                        link=link,
                        direct_pose=pose_info,
                        direct_channel=h_freq,
                        array_cfg=cfg,
                        ris_cfg=ris_cfg,
                        ris_metadata=ris_metadata,
                        skip_existing=SKIP_EXISTING_FRAMES,
                    )
                vis_h_by_config.setdefault(cfg["id"], h_freq)

    radio_map_cfg = yaml.safe_load(os.environ.get("SIGMA_RADIO_MAP_YAML", "{}")) or {}
    if radio_map_cfg.get("enabled"):
        for link_type, tx_alias, link_list in group_links_by_tx(links):
            map_pose = load_link_pose_from_stage1(traj_dir, frame_name, link_list[0], selected_nodes)
            map_output = stage3_root(traj_dir) / "radio_maps" / tx_alias / f"rm_{frame_idx:06d}.npz"
            if not (SKIP_EXISTING_FRAMES and map_output.is_file()):
                save_radio_map(safe_xml, map_pose, radio_map_cfg, map_output)

    if VIS_ENABLE and vis_context is not None:
        try:
            scene, Camera, group_paths, group_pose = vis_context
            save_visualizations(
                traj_dir, f"{frame_idx:06d}", scene, Camera, group_paths,
                group_pose, vis_h_by_config, force=False)
        except Exception as exc:
            warning_line(f"{traj_dir.name}/{frame_name}: visualization failed: {exc}")

    for product in stage3_root(traj_dir).rglob(f"*_{frame_idx:06d}.npz"):
        with np.load(product, allow_pickle=False) as saved:
            payload = {key: saved[key] for key in saved.files}
        payload.update(frame_identity(traj_dir, frame_idx))
        temporary = product.with_suffix(".npz.tmp")
        with temporary.open("wb") as handle:
            np.savez_compressed(handle, **payload)
        temporary.replace(product)

    if not KEEP_WORK_XML:
        shutil.rmtree(work_dir, ignore_errors=True)


    if any(n == 0 for n in total_paths):
        return f"done links={sum(len(v) for v in links.values())} traced_batches={traced_batches} channels={saved_channels} n_paths={total_paths} n_paths=0"
    return f"done links={sum(len(v) for v in links.values())} traced_batches={traced_batches} channels={saved_channels} n_paths_min={min(total_paths) if total_paths else 0}"


def process_trajectory(traj_dir: Path, traj_idx: int, traj_total: int):
    frames = list_frames(traj_dir)
    total = len(frames)
    if not total:
        raise RuntimeError(f"No dynamic XML frames for {traj_dir.name}")


    out_root = stage3_root(traj_dir)
    status_line(
        f"Stage3 output paths: trajectory={traj_dir.name} "
        f"stage3_root={out_root} paths_root={out_root / 'paths'} "
        f"channels_root={out_root / 'channels'} radio_index={radio_index_path()}"
    )
    status_line(f"START traj {traj_idx}/{traj_total}: {traj_dir.name}, frames={total}")
    t0 = time.perf_counter()

    zero_span_start = None
    prev_frame = None
    failures = []

    for idx, frame in enumerate(frames, start=1):
        try:
            result = process_frame(traj_dir, frame)
        except Exception as e:
            failures.append((frame, str(e)))
            record_frame_error(traj_dir, frame, e)
            error_line(f"{traj_dir.name}/{frame}: {e}")
            traceback.print_exc()
            prev_frame = frame
            continue

        clear_frame_error(traj_dir, frame)
        is_zero = "n_paths=0" in result
        if is_zero:
            if zero_span_start is None:
                zero_span_start = frame
                warning_line(f"{traj_dir.name}/{frame}: n_paths=0 (start of zero-path block)")
        else:
            if zero_span_start is not None:
                warning_line(f"{traj_dir.name}/{frame}: zero-path block ended (frames {zero_span_start}–{prev_frame})")
                zero_span_start = None

        prev_frame = frame
        elapsed = time.perf_counter() - t0
        eta = elapsed / idx * (total - idx) if idx > 0 else 0.0
        bar_len = 28
        fill = int(round(bar_len * idx / total))
        bar = "[" + "#" * fill + "-" * (bar_len - fill) + "]"
        progress_line(
            f"  {traj_dir.name} | frame {frame} | {bar} {idx}/{total} "
            f"{100.0*idx/total:5.1f}% | ETA {eta/60:.1f}m | {result}"
        )

    if zero_span_start is not None:
        warning_line(f"{traj_dir.name}: zero-path block reached end (frames {zero_span_start}–{prev_frame})")

    _finish_progress_line()
    if failures:
        raise RuntimeError(f"{traj_dir.name}: {len(failures)} frame(s) failed; see stage3_radio/{traj_dir.name}/_errors: {failures[:3]}")
    status_line(f"DONE  traj {traj_idx}/{traj_total}: {traj_dir.name}")

def print_config():
    status_line("===== Sionna multi-array OFDM dataset generator config =====")
    pairs = [
        ("DATASET_ROOT", DATASET_ROOT),
        ("STAGE3_SUBDIR", STAGE3_SUBDIR),
        ("RADIO_RSU_IDS", radio_selector_label(RADIO_RSU_IDS)),
        ("RADIO_CAV_IDS", radio_selector_label(RADIO_CAV_IDS)),
        ("RADIO_UAV_IDS", radio_selector_label(RADIO_UAV_IDS)),
        ("FREQUENCY_HZ", FREQUENCY_HZ),
        ("BANDWIDTH_HZ", BANDWIDTH_HZ),
        ("N_SUBCARRIERS", N_SUBCARRIERS),
        ("ARRAY_CONFIGS", ARRAY_CONFIGS or "<stage3 defaults h001/h002/h003>"),
        ("CHANNEL_RECONSTRUCTION_MODE", CHANNEL_RECONSTRUCTION_MODE),
        ("ARRAY_IF_FILTER", ARRAY_IF_FILTER),
        ("SIONNA_MATERIAL_EXTRAPOLATE", SIONNA_MATERIAL_EXTRAPOLATE),
        ("parsed configs", ", ".join(cfg["id"] for cfg in ARRAY_CONFIG_LIST)),
        ("POSE_SOURCE", "stage1_world_state (single canonical pose source)"),
        ("DYNAMIC_SUBDIR", DYNAMIC_SUBDIR),
        ("WORLD_STATE_VIEW", "stage1_multimodal/world_state"),
        ("ANTENNA_PATTERN", ANTENNA_PATTERN),
        ("POLARIZATION", POLARIZATION),
        ("ARRAY_SPACING_WL", ARRAY_SPACING_WL),
        ("RT_MAX_DEPTH", RT_MAX_DEPTH),
        ("RT_SAMPLES_PER_SRC", RT_SAMPLES_PER_SRC),
        ("RT_MAX_NUM_PATHS_PER_SRC", RT_MAX_NUM_PATHS_PER_SRC),
        ("RT_SYNTHETIC_ARRAY", RT_SYNTHETIC_ARRAY),
        ("RT_LOS", RT_LOS),
        ("RT_SPECULAR_REFLECTION", RT_SPECULAR_REFLECTION),
        ("RT_DIFFUSE_REFLECTION", RT_DIFFUSE_REFLECTION),
        ("VIS_ENABLE", VIS_ENABLE),
        ("VIS_EVERY_N", VIS_EVERY_N),
        ("VIS_RENDER_SCENE", VIS_RENDER_SCENE),
        ("VIS_RENDER_RAYS", VIS_RENDER_RAYS),
        ("VIS_RENDER_MAX_PATHS", VIS_RENDER_MAX_PATHS),
    ]
    for k, v in pairs:
        status_line(f"{k:28s} = {v}")
    status_line("============================================================")

def main():
    validate_stage3_dataset_root()
    print_config()
    trajs = list_trajectories()
    status_line(f"Selected trajectories: {len(trajs)}")
    if not trajs:
        raise RuntimeError("No trajectories have matching world_state and dynamic XML inputs")
    for i, traj in enumerate(trajs, start=1):
        process_trajectory(traj, i, len(trajs))
    status_line("ALL DONE")


if __name__ == '__main__':
    main()
