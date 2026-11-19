"""Numerical and association checks for paper experiment data adapters."""
import csv
import json

import numpy as np
import pytest

from experiments.metrics import beam_subset, ranking_metrics
from experiments.ris_labels import candidate_powers, export_labels, grouped_folds
from Sionna_channel_storage.ris_channel import cascade_channel


def channels():
    rng = np.random.default_rng(23)
    h_br = rng.normal(size=(1, 1024, 16)) + 1j * rng.normal(size=(1, 1024, 16))
    h_ru = rng.normal(size=(1, 1, 1024)) + 1j * rng.normal(size=(1, 1, 1024))
    phases = np.exp(1j * rng.uniform(-np.pi, np.pi, (256, 1024))) * 0.8
    beam = np.full(16, 0.25, dtype=complex)
    return h_br, h_ru, phases, beam


def test_ris_only_labels_match_original_cascade_algebra():
    h_br, h_ru, phases, beam = channels()
    powers = candidate_powers(h_br, h_ru, phases, beam, field_coupling=0.5)
    for q in (0, 17, 255):
        response = cascade_channel(h_br, h_ru, phases[q])[0, 0] @ beam
        assert powers[q] == pytest.approx(float(abs(response * 0.5) ** 2), rel=1e-5)
    np.testing.assert_array_equal(candidate_powers(h_br, h_ru, phases, beam, field_coupling=0), np.zeros(256))


def test_ris_enforces_paper_dimensions_and_fixed_unit_beam():
    h_br, h_ru, phases, beam = channels()
    with pytest.raises(ValueError, match="unit norm"):
        candidate_powers(h_br, h_ru, phases, beam * 2)
    with pytest.raises(ValueError, match="1 subcarrier"):
        candidate_powers(h_br.repeat(2, axis=0), h_ru.repeat(2, axis=0), phases, beam)
    with pytest.raises(ValueError, match="amplitude"):
        candidate_powers(h_br, h_ru, phases * 2, beam)
    with pytest.raises(ValueError, match="finite"):
        damaged = h_ru.copy()
        damaged[0, 0, 0] = np.nan
        candidate_powers(h_br, damaged, phases, beam)


def test_1024_candidate_codebook_is_supported():
    h_br, h_ru, phases, beam = channels()
    phases = np.tile(phases, (4, 1))
    powers = candidate_powers(h_br, h_ru, phases, beam)
    assert powers.shape == (1024,)
    np.testing.assert_allclose(powers[:256], powers[768:], rtol=1e-12)


def test_three_fold_grouping_prevents_route_leakage():
    rows = [dict(site_id="s", trajectory_id=str(t), group_id=str(t // 2))
            for t in range(8) for _ in range(t + 1)]
    folds = grouped_folds(rows, 7)
    assert folds == grouped_folds(rows, 7)
    assert set(folds) == {0, 1, 2}
    for group in {r["group_id"] for r in rows}:
        assert len({fold for row, fold in zip(rows, folds) if row["group_id"] == group}) == 1
    inconsistent = [dict(site_id="s", trajectory_id="0", group_id=str(i)) for i in range(3)]
    with pytest.raises(ValueError, match="inconsistent"):
        grouped_folds(inconsistent, 1)
    with pytest.raises(ValueError, match="at least three"):
        grouped_folds(rows[:2], 1)


def test_32_beam_labels_are_recomputed_over_subset():
    power = np.zeros((1, 64))
    power[0, 3], power[0, 20] = 10, 9
    subset, labels = beam_subset(power, 32)
    assert subset.shape == (1, 32)
    assert labels.tolist() == [10]
    assert beam_subset(power, 64)[1].tolist() == [3]


def test_metrics_count_failed_association_in_denominator():
    metrics = ranking_metrics(np.array([[1, 4], [1, 4], [0, 0]]), [[1, 0], [1, 0], []], [1, 2], valid=[True, False, True])
    assert metrics["scan_budgets"]["1"]["normalized_gain"] == pytest.approx(1 / 3)
    assert metrics["scan_budgets"]["1"]["top_k_accuracy"] == pytest.approx(1 / 3)
    assert metrics["zero_reference_frames"] == 1
    assert metrics["failed_frames"] == 2
    with pytest.raises(ValueError, match="repeat"):
        ranking_metrics(np.ones((1, 2)), [[1, 1]], [1])


def test_export_preserves_ids_groups_and_excludes_direct_channel(tmp_path):
    h_br, h_ru, phases, beam = channels()
    image = tmp_path / "rgb.png"
    image.write_bytes(b"fixture-presence-only")
    codebook = tmp_path / "phase.npz"
    np.savez(codebook, phi=phases, metadata_json=np.asarray(json.dumps({"fixed_incident_compensation": True})))
    beam_path = tmp_path / "beam.npy"
    np.save(beam_path, beam)
    manifest = tmp_path / "input.csv"
    rows = []
    for i in range(3):
        component = tmp_path / f"ris_{i}.npz"
        metadata = {"site_id": "site", "trajectory_id": str(i), "frame_id": "25", "timestamp_s": 1.25,
                    "panel_projected_aperture_coupling": {"field_amplitude_factor": 0.5}}
        np.savez(component, h_br=h_br, h_ru=h_ru, h_direct=np.full((1, 1, 16), 1e9),
                 site_id="site", trajectory_id=f"{i:06d}", frame_idx=25, time_s=1.25,
                 metadata_json=np.asarray(json.dumps(metadata)))
        rows.append(dict(site_id="site", trajectory_id=str(i), frame_id="25", timestamp_s="1.25",
                         rgb_path=image.name, components_path=component.name, group_id=f"route{i}"))
    with manifest.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    output = tmp_path / "labels"
    result = export_labels(manifest, codebook, beam_path, output, 7)
    assert result["training_implemented"] is False
    with np.load(output / "powers.npz") as data:
        np.testing.assert_allclose(data["powers"][0], candidate_powers(h_br, h_ru, phases, beam, field_coupling=0.5))
        assert set(data["test_fold"]) == {0, 1, 2}
    with (output / "samples.csv").open() as handle:
        exported = list(csv.DictReader(handle))
    assert [r["trajectory_id"] for r in exported] == ["0", "1", "2"]
    assert all(r["frame_id"] == "25" for r in exported)
    with pytest.raises(FileExistsError):
        export_labels(manifest, codebook, beam_path, output, 7)


def test_export_rejects_mismatched_component_state(tmp_path):
    h_br, h_ru, phases, beam = channels()
    np.savez(tmp_path / "phases.npz", phi=phases, metadata_json=np.asarray(json.dumps({"fixed_incident_compensation": True})))
    np.save(tmp_path / "beam.npy", beam)
    (tmp_path / "rgb.png").write_bytes(b"fixture")
    np.savez(tmp_path / "components.npz", h_br=h_br, h_ru=h_ru,
             metadata_json=np.asarray(json.dumps({"frame_id": "wrong"})))
    with (tmp_path / "input.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=["site_id", "trajectory_id", "frame_id", "timestamp_s", "rgb_path", "components_path", "group_id"])
        writer.writeheader()
        writer.writerows(dict(site_id="s", trajectory_id=str(i), frame_id="25", timestamp_s="0.2",
                              rgb_path="rgb.png", components_path="components.npz", group_id=str(i)) for i in range(3))
    with pytest.raises(ValueError, match="frame_id disagrees"):
        export_labels(tmp_path / "input.csv", tmp_path / "phases.npz", tmp_path / "beam.npy", tmp_path / "labels", 1)
    assert not (tmp_path / "labels").exists()
