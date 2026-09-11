import pytest

from sawyer_control import JointVector


def test_joint_vector_requires_seven_finite_values():
    assert JointVector(range(7)).values == tuple(float(value) for value in range(7))
    with pytest.raises(ValueError):
        JointVector(range(6))
    with pytest.raises(ValueError):
        JointVector([0.0] * 6 + [float("nan")])
