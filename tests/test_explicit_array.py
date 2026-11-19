"""NumPy numerical invariants of physical-array CIR conversion."""
import numpy as np
import pytest
from Sionna_channel_storage.explicit_array import cir_to_ofdm


def test_pair_specific_delays_are_preserved_and_invalid_paths_masked():
    a = np.ones((2, 3, 2), np.complex64)
    tau = np.zeros((2, 3, 2))
    tau[..., 0] = np.arange(6).reshape(2, 3) * 1e-9
    tau[..., 1] = -1
    offsets = np.array([-10e6, 0, 10e6])
    channel = cir_to_ofdm(a, tau, offsets)
    reference = np.exp(-2j * np.pi * tau[..., 0, None] * offsets)
    assert channel.shape == (1, 2, 3, 3)
    assert channel.dtype == np.complex64
    np.testing.assert_allclose(channel[0], reference, atol=1e-6)


def test_carrier_phase_is_not_applied_twice():
    a = np.full((1, 1, 1), 0.5j, np.complex64)
    result = cir_to_ofdm(a, np.full(a.shape, 1e-6), [0])
    assert result[0, 0, 0, 0] == 0.5j
    with pytest.raises(ValueError, match="matching"):
        cir_to_ofdm(a, np.zeros((1, 1)), [0])
