import pytest

from human_tracking.filters import JumpFilter, VelocityEstimator


class TestJumpFilter:
    def test_first_position_is_accepted(self):
        assert JumpFilter().accept(1, (0.0, 0.0, 0.0), time=0.0)

    def test_small_move_is_accepted_and_becomes_the_reference(self):
        f = JumpFilter(max_jump=1.0)
        f.accept(1, (0.0, 0.0, 0.0), 0.0)
        assert f.accept(1, (0.8, 0.0, 0.0), 0.1)
        assert f.accept(1, (1.6, 0.0, 0.0), 0.2)  # 0.8 m from the new reference

    def test_implausible_jump_is_rejected_and_reference_is_kept(self):
        f = JumpFilter(max_jump=1.0)
        f.accept(1, (0.0, 0.0, 0.0), 0.0)
        assert not f.accept(1, (3.0, 0.0, 0.0), 0.1)
        assert f.accept(1, (0.5, 0.0, 0.0), 0.2)  # still measured from the origin

    def test_tracks_are_independent(self):
        f = JumpFilter(max_jump=1.0)
        f.accept(1, (0.0, 0.0, 0.0), 0.0)
        assert f.accept(2, (5.0, 5.0, 0.0), 0.0)

    def test_stale_reference_is_replaced(self):
        f = JumpFilter(max_jump=1.0, reset_after=1.0)
        f.accept(1, (0.0, 0.0, 0.0), 0.0)
        assert not f.accept(1, (3.0, 0.0, 0.0), 0.9)
        assert f.accept(1, (3.0, 0.0, 0.0), 1.1)

    def test_forget_older_than(self):
        f = JumpFilter(max_jump=1.0, reset_after=100.0)
        f.accept(1, (0.0, 0.0, 0.0), 0.0)
        f.forget_older_than(5.0)
        assert f.accept(1, (9.0, 0.0, 0.0), 6.0)


class TestVelocityEstimator:
    def test_first_sample_has_zero_velocity(self):
        assert VelocityEstimator().update(1, (1.0, 2.0, 0.0), 0.0) == (0.0, 0.0, 0.0)

    def test_finite_difference(self):
        v = VelocityEstimator(smoothing=1.0)
        v.update(1, (0.0, 0.0, 0.0), 0.0)
        assert v.update(1, (0.1, -0.05, 0.0), 0.1) == pytest.approx((1.0, -0.5, 0.0))

    def test_constant_motion_converges_to_the_true_velocity(self):
        v = VelocityEstimator(smoothing=0.4)
        velocity = None
        for i in range(30):
            velocity = v.update(1, (0.7 * i * 0.1, 0.0, 0.0), i * 0.1)
        assert velocity == pytest.approx((0.7, 0.0, 0.0))

    def test_smoothing_damps_a_noisy_sample(self):
        v = VelocityEstimator(smoothing=0.4)
        v.update(1, (0.0, 0.0, 0.0), 0.0)
        v.update(1, (0.1, 0.0, 0.0), 0.1)  # 1.0 m/s
        vx, _, _ = v.update(1, (0.4, 0.0, 0.0), 0.2)  # raw 3.0 m/s
        assert vx == pytest.approx(0.4 * 3.0 + 0.6 * 1.0)

    def test_long_gap_restarts_the_estimate(self):
        v = VelocityEstimator(smoothing=1.0, max_gap=1.0)
        v.update(1, (0.0, 0.0, 0.0), 0.0)
        v.update(1, (0.1, 0.0, 0.0), 0.1)
        assert v.update(1, (5.0, 0.0, 0.0), 3.0) == (0.0, 0.0, 0.0)
        assert v.update(1, (5.1, 0.0, 0.0), 3.1) == pytest.approx((1.0, 0.0, 0.0))

    def test_duplicate_timestamp_keeps_the_estimate(self):
        v = VelocityEstimator(smoothing=1.0)
        v.update(1, (0.0, 0.0, 0.0), 0.0)
        first = v.update(1, (0.1, 0.0, 0.0), 0.1)
        assert v.update(1, (0.9, 0.0, 0.0), 0.1) == first

    def test_tracks_are_independent(self):
        v = VelocityEstimator(smoothing=1.0)
        v.update(1, (0.0, 0.0, 0.0), 0.0)
        v.update(2, (0.0, 0.0, 0.0), 0.0)
        v.update(1, (0.1, 0.0, 0.0), 0.1)
        assert v.update(2, (0.0, 0.2, 0.0), 0.1) == pytest.approx((0.0, 2.0, 0.0))

    def test_invalid_smoothing_is_rejected(self):
        with pytest.raises(ValueError):
            VelocityEstimator(smoothing=0.0)
