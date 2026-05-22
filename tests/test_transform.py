import numpy as np
from lidar.transform import polar_to_xy, filter_scan


def test_zero_angle_is_up():
    x, y = polar_to_xy(np.array([0.0]), np.array([3.0]))
    assert np.isclose(x[0], 0.0, atol=1e-6)
    assert np.isclose(y[0], 3.0, atol=1e-6)


def test_quarter_turn_is_right():
    x, y = polar_to_xy(np.array([np.pi / 2]), np.array([2.0]))
    assert np.isclose(x[0], 2.0, atol=1e-6)
    assert np.isclose(y[0], 0.0, atol=1e-6)


def test_half_turn_is_down():
    x, y = polar_to_xy(np.array([np.pi]), np.array([1.0]))
    assert np.isclose(x[0], 0.0, atol=1e-6)
    assert np.isclose(y[0], -1.0, atol=1e-6)


def test_three_quarter_is_left():
    x, y = polar_to_xy(np.array([3 * np.pi / 2]), np.array([1.0]))
    assert np.isclose(x[0], -1.0, atol=1e-6)
    assert np.isclose(y[0], 0.0, atol=1e-6)


def test_filter_drops_too_close():
    a = np.array([0.0, 1.0, 2.0])
    r = np.array([0.01, 5.0, 8.0])
    q = np.array([50, 50, 50], dtype=np.uint8)
    a2, r2, q2 = filter_scan(a, r, q)
    assert len(a2) == 2
    assert np.allclose(r2, [5.0, 8.0])


def test_filter_drops_too_far():
    a = np.array([0.0, 1.0])
    r = np.array([5.0, 15.0])
    q = np.array([50, 50], dtype=np.uint8)
    a2, r2, q2 = filter_scan(a, r, q)
    assert len(r2) == 1
    assert r2[0] == 5.0


def test_filter_drops_zero_quality():
    a = np.array([0.0, 1.0])
    r = np.array([5.0, 5.0])
    q = np.array([0, 50], dtype=np.uint8)
    a2, r2, q2 = filter_scan(a, r, q)
    assert len(q2) == 1
    assert q2[0] == 50
