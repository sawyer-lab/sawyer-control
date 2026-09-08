import pytest

from sawyer_control import CommandSequence, JointCommandSample, JointVector


def test_joint_vector_requires_seven_finite_values():
    assert JointVector(range(7)).values == tuple(float(value) for value in range(7))
    with pytest.raises(ValueError):
        JointVector(range(6))
    with pytest.raises(ValueError):
        JointVector([0.0] * 6 + [float("nan")])


def test_sequence_requires_a_sample():
    with pytest.raises(ValueError):
        CommandSequence([])
    assert len(CommandSequence([JointCommandSample(position=[0.0] * 7)]).samples) == 1
