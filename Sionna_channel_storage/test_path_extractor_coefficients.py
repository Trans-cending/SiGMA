# -*- coding: utf-8 -*-
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]

from Sionna_channel_storage import path_extractor as pe


class FakePaths:
    def __init__(self):
        shape = (1, 1, 1, 1, 3)
        self.theta_r = np.full(shape, np.pi / 2, dtype=np.float32)
        self.phi_r = np.array([[[[[0.1, 0.2, 0.3]]]]], dtype=np.float32)
        self.theta_t = np.full(shape, np.pi / 2, dtype=np.float32)
        self.phi_t = np.array([[[[[0.4, 0.5, 0.6]]]]], dtype=np.float32)
        self.tau = np.array([[[[[1.0e-9, 2.0e-9, 3.0e-9]]]]], dtype=np.float32)
        real = np.array([[[[[1.0e-3, 2.0e-3, 3.0e-3]]]]], dtype=np.float32)
        imag = np.array([[[[[0.0, 1.0e-3, -1.0e-3]]]]], dtype=np.float32)
        self.a = (real, imag)
        self.valid = np.array([[[[[True, False, True]]]]], dtype=bool)
        self.interactions = np.array([[[[[[0, 1, 0]]]]]], dtype=np.uint8)
        self.objects = np.array([[[[[[0, 7, 0]]]]]], dtype=np.uint32)
        self.primitives = np.array([[[[[[0, 3, 0]]]]]], dtype=np.uint32)
        self.vertices = np.zeros((1, 1, 1, 1, 1, 3, 3), dtype=np.float32)

    def cir(self, **_kwargs):
        raise AssertionError("extractor must use Paths.a/Paths.tau, not Paths.cir()")


def _pose_info():
    return {
        "num_users": 1,
        "num_tx": 1,
        "tx_forward": np.array([1.0, 0.0, 0.0], dtype=np.float32),
        "rx_forward": np.array([1.0, 0.0, 0.0], dtype=np.float32),
    }


def test_extractor_uses_aligned_paths_a_tau_and_valid_mask():
    out = pe.extract_paths_data(FakePaths(), _pose_info())
    rx = out["rx_users"][0]

    assert rx["tau"].tolist() == [np.float32(1.0e-9), np.float32(3.0e-9)]
    assert rx["los"].tolist() == [True, True]
    assert rx["path_type"].tolist() == [0, 0]

    expected_passband = np.array([1.0e-3 + 0.0j, 3.0e-3 - 1.0e-3j], dtype=np.complex64)
    expected_tau = np.array([1.0e-9, 3.0e-9], dtype=np.float64)
    expected = expected_passband * np.exp(-1j * 2.0 * np.pi * float(pe.FREQUENCY_HZ) * expected_tau)
    assert np.allclose(rx["a"], expected.astype(np.complex64), rtol=1e-5, atol=1e-8)


if __name__ == "__main__":
    test_extractor_uses_aligned_paths_a_tau_and_valid_mask()
    print("PATH_EXTRACTOR_COEFFICIENT_TESTS=PASS")
