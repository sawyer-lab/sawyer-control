import json
import struct

import pytest

cv2 = pytest.importorskip("cv2", reason="the camera driver needs OpenCV")
numpy = pytest.importorskip("numpy")

from sawyer_control import camera as camera_module
from sawyer_control.camera import (
    CameraError,
    _normalise,
    _patch_avi_rate,
    _Recorder,
    _RECORDING_NAME,
    find_device,
)


def test_normalise_turns_video_nodes_into_indices():
    assert _normalise("/dev/video8") == 8
    assert _normalise("4") == 4
    assert _normalise(2) == 2


def test_normalise_leaves_stable_by_id_paths_alone():
    path = "/dev/v4l/by-id/usb-046d_Logitech_BRIO_7C86342B-video-index0"
    assert _normalise(path) == path


def test_find_device_prefers_the_first_matching_pattern(monkeypatch):
    brio = "/dev/v4l/by-id/usb-046d_Logitech_BRIO_1234-video-index0"
    other = "/dev/v4l/by-id/usb-046d_0825_5678-video-index0"
    monkeypatch.setattr(
        camera_module.glob, "glob", lambda pattern: [brio] if "BRIO" in pattern else [other]
    )
    assert find_device() == brio


def test_find_device_explains_itself_when_nothing_is_attached(monkeypatch):
    monkeypatch.setattr(camera_module.glob, "glob", lambda pattern: [])
    with pytest.raises(CameraError, match="No Logitech camera found"):
        find_device()


def test_recording_names_reject_path_separators():
    assert _RECORDING_NAME.fullmatch("toss_001")
    assert _RECORDING_NAME.fullmatch("trial-2.take3")
    assert not _RECORDING_NAME.fullmatch("../escape")
    assert not _RECORDING_NAME.fullmatch("has space")
    assert not _RECORDING_NAME.fullmatch("")


def _fake_avi(micros_per_frame: int, scale: int, rate: int) -> bytes:
    avih = b"avih" + struct.pack("<I", 56) + struct.pack("<I", micros_per_frame) + bytes(52)
    strh = (
        b"strh"
        + struct.pack("<I", 56)
        + b"vids"
        + b"MJPG"
        + bytes(12)
        + struct.pack("<II", scale, rate)
        + bytes(28)
    )
    body = b"AVI " + avih + strh
    return b"RIFF" + struct.pack("<I", len(body)) + body


def _read_rates(data: bytes) -> tuple[int, int, int]:
    avih = data.find(b"avih")
    strh = data.find(b"strh")
    micros = struct.unpack_from("<I", data, avih + 8)[0]
    scale, rate = struct.unpack_from("<II", data, strh + 8 + 20)
    return micros, scale, rate


def test_patch_avi_rate_rewrites_both_headers(tmp_path):
    path = tmp_path / "clip.avi"
    path.write_bytes(_fake_avi(micros_per_frame=33_333, scale=1, rate=30))

    assert _patch_avi_rate(path, 50.0) is True

    micros, scale, rate = _read_rates(path.read_bytes())
    assert micros == 20_000
    assert rate / scale == pytest.approx(50.0)


def test_patch_avi_rate_leaves_other_files_untouched(tmp_path):
    path = tmp_path / "clip.mp4"
    original = b"not a riff file at all"
    path.write_bytes(original)

    assert _patch_avi_rate(path, 30.0) is False
    assert path.read_bytes() == original


def test_patch_avi_rate_ignores_a_nonsense_rate(tmp_path):
    path = tmp_path / "clip.avi"
    original = _fake_avi(micros_per_frame=33_333, scale=1, rate=30)
    path.write_bytes(original)

    assert _patch_avi_rate(path, 0.0) is False
    assert path.read_bytes() == original


def _frame(value: int):
    return numpy.full((48, 64, 3), value, dtype=numpy.uint8)


def test_recorder_writes_a_video_and_a_timestamp_manifest(tmp_path):
    path = tmp_path / "trial.avi"
    recorder = _Recorder(path, fps=10.0)
    for index in range(10):
        recorder.submit(_frame(index * 10), stamp=1000.0 + index * 0.1,
                        monotonic=index * 0.1, sequence=index)
    clip = recorder.finish()

    assert clip.frame_count == 10
    assert clip.dropped_frames == 0
    assert (clip.width, clip.height) == (64, 48)
    assert clip.duration_s == pytest.approx(0.9)
    assert clip.effective_fps == pytest.approx(10.0)
    assert path.exists() and path.stat().st_size > 0

    manifest = json.loads(clip.manifest_path.read_text())
    assert manifest["video_file"] == "trial.avi"
    assert manifest["frame_count"] == 10
    assert [record["sequence"] for record in manifest["frames"]] == list(range(10))
    assert manifest["frames"][3]["wall_time"] == pytest.approx(1000.3)


def test_recorder_reports_a_recording_that_never_got_a_frame(tmp_path):
    recorder = _Recorder(tmp_path / "empty.avi", fps=10.0)
    with pytest.raises(CameraError, match="no frames"):
        recorder.finish()


def test_recorder_refuses_a_frame_size_change(tmp_path):
    recorder = _Recorder(tmp_path / "resized.avi", fps=10.0)
    recorder.submit(_frame(10), stamp=1.0, monotonic=0.0, sequence=0)
    recorder.submit(numpy.zeros((32, 32, 3), dtype=numpy.uint8),
                    stamp=1.1, monotonic=0.1, sequence=1)
    with pytest.raises(CameraError, match="frame size changed"):
        recorder.finish()


def test_recorded_video_reads_back_with_the_patched_rate(tmp_path):
    path = tmp_path / "playback.avi"
    recorder = _Recorder(path, fps=10.0)
    for index in range(20):
        recorder.submit(_frame(index * 5), stamp=float(index) / 25.0,
                        monotonic=index / 25.0, sequence=index)
    clip = recorder.finish()

    # Submitted at 25 fps while the writer was told 10, so the file should
    # describe the rate the frames actually arrived at.
    assert clip.effective_fps == pytest.approx(25.0)
    capture = cv2.VideoCapture(str(path))
    try:
        assert capture.get(cv2.CAP_PROP_FPS) == pytest.approx(25.0, abs=0.1)
        assert int(capture.get(cv2.CAP_PROP_FRAME_COUNT)) == 20
    finally:
        capture.release()
