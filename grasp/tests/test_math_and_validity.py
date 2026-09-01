import numpy as np

from rim_grasp_perception.math3d import contact_orientation, pose_matrix
from rim_grasp_perception.validation import is_stale


def test_contact_frame_axes_are_closing_and_opposite_approach():
    closing = np.array([1.0, 0.0, 0.0])
    approach = np.array([0.0, 0.0, -1.0])
    q = contact_orientation(closing, approach)
    r = pose_matrix([0, 0, 0], q)[:3, :3]
    np.testing.assert_allclose(r[:, 0], closing, atol=1e-7)
    np.testing.assert_allclose(-r[:, 2], approach, atol=1e-7)


def test_stale_timeout_boundary():
    assert not is_stale(10.0, 10.75, 0.75)
    assert is_stale(10.0, 10.751, 0.75)
    assert not is_stale(0.0, 100.0, 0.75)
