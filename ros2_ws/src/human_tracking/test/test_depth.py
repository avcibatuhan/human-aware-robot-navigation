import numpy as np
import pytest

from human_tracking.depth import is_valid_depth, sample_depth


def image(value=3.0, shape=(480, 640)):
    return np.full(shape, value, dtype=np.float32)


@pytest.mark.parametrize(
    ("z", "valid"),
    [
        (0.49, False),
        (0.5, True),
        (5.0, True),
        (10.0, True),
        (10.01, False),
        (float("nan"), False),
        (float("inf"), False),
        (float("-inf"), False),
    ],
)
def test_band_pass(z, valid):
    assert is_valid_depth(z) is valid


def test_uniform_image_returns_its_value():
    assert sample_depth(image(3.0), 320, 240) == pytest.approx(3.0)


def test_median_ignores_a_minority_of_background_pixels():
    depth = image(2.0)
    depth[236:241, 315:326] = 9.0  # 55 of 121 window pixels are background
    assert sample_depth(depth, 320, 240, half_window=5) == pytest.approx(2.0)


def test_invalid_pixels_are_excluded_from_the_median():
    depth = image(4.0)
    depth[235:246, 315:320] = np.nan
    depth[235:246, 320:323] = np.inf
    depth[235, 323] = 0.1  # closer than the band-pass
    depth[236, 323] = 50.0  # farther than the band-pass
    assert sample_depth(depth, 320, 240, half_window=5) == pytest.approx(4.0)


def test_window_without_valid_pixels_returns_none():
    assert sample_depth(image(np.inf), 320, 240) is None
    assert sample_depth(image(0.2), 320, 240) is None
    assert sample_depth(image(12.0), 320, 240) is None


def test_window_is_clipped_at_the_image_border():
    depth = image(6.0)
    assert sample_depth(depth, 0, 0) == pytest.approx(6.0)
    assert sample_depth(depth, 639, 479) == pytest.approx(6.0)


def test_centre_outside_the_image_returns_none():
    assert sample_depth(image(), -1, 240) is None
    assert sample_depth(image(), 640, 240) is None
    assert sample_depth(image(), 320, 480) is None


def test_integer_depth_is_read_as_millimetres():
    depth = np.full((480, 640), 2500, dtype=np.uint16)
    assert sample_depth(depth, 320, 240) == pytest.approx(2.5)


def test_custom_band():
    assert sample_depth(image(0.3), 320, 240, min_depth=0.2) == pytest.approx(0.3)
