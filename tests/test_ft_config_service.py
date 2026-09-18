"""End-to-end tests for the force/torque config plane over real gRPC.

A real server and a real client, with only the device's HTTP transport faked,
so the proto, handlers, status mapping and client wrappers are all exercised.
"""

import sys
from concurrent import futures
from pathlib import Path

import grpc
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "bridge"))

from sawyer_control import ForceTorqueConfigClient  # noqa: E402
from sawyer_control.v1 import control_pb2_grpc  # noqa: E402
from sawyer_control_bridge.service import ForceTorqueConfigService  # noqa: E402
from test_ft_config import FakeBox  # noqa: E402


class StubRuntime:
    def __init__(self, box):
        self._box = box

    def get_force_torque_config(self):
        return self._box


@pytest.fixture
def box():
    return FakeBox()


@pytest.fixture
def client(box):
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=2))
    control_pb2_grpc.add_ForceTorqueConfigServicer_to_server(
        ForceTorqueConfigService(StubRuntime(box)), server)
    port = server.add_insecure_port("127.0.0.1:0")
    server.start()
    connection = ForceTorqueConfigClient.connect(f"127.0.0.1:{port}")
    yield connection
    connection.close()
    server.stop(None)


@pytest.fixture
def unavailable_client():
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=2))
    control_pb2_grpc.add_ForceTorqueConfigServicer_to_server(
        ForceTorqueConfigService(StubRuntime(None)), server)
    port = server.add_insecure_port("127.0.0.1:0")
    server.start()
    connection = ForceTorqueConfigClient.connect(f"127.0.0.1:{port}")
    yield connection
    connection.close()
    server.stop(None)


# ── reads ────────────────────────────────────────────────────────────────────

def test_identity_crosses_the_wire(client):
    identity = client.identity()
    assert identity.mac == "00:16:BD:00:23:56"
    assert identity.firmware == "2.2.59"
    assert identity.internal_rate_hz == 7000


def test_scaling_reports_counts_and_units(client):
    scaling = client.scaling()
    assert scaling.counts_per_force == 1000000
    assert scaling.force_unit == "N" and scaling.torque_unit == "Nm"


def test_configuration_carries_tool_transform_and_nested_scaling(client):
    config = client.configuration()
    assert list(config.tool_transform) == [0.0, 0.0, 132.710, 0.0, 0.0, 0.0]
    assert config.tool_distance_unit == "mm"
    assert config.scaling.counts_per_force == 1000000
    assert list(config.sensing_ranges) == [330.0, 330.0, 990.0, 30.0, 30.0, 30.0]


def test_list_configurations_returns_every_slot(client):
    assert len(client.configurations()) == 16


def test_active_slot_is_requested_with_minus_one(client, box):
    client.configuration()
    assert all(params.get("index") != -1 for _page, params in box.calls)
    client.configuration(15)
    assert any(params.get("index") == 15 for _page, params in box.calls)


def test_settings_distinguishes_no_filter_from_a_cutoff(client):
    settings = client.settings()
    assert settings.filter_code == 0
    assert settings.filter_enabled is False and settings.filter_cutoff_hz == 0


def test_communications_reports_rate_and_buffer(client):
    comms = client.communications()
    assert comms.rate_hz == 100 and comms.buffer_records == 1
    assert comms.internal_rate_hz == 7000


def test_monitor_conditions_cross_the_wire(client):
    conditions = client.monitor_conditions()
    assert conditions.enabled is False
    assert len(conditions.conditions) == 16
    assert conditions.conditions[0].axis == "tx"
    assert conditions.conditions[0].comparison == "<"
    assert conditions.conditions[2].axis == ""          # disabled statement


def test_peaks_cross_the_wire(client):
    peaks = client.peaks()
    assert peaks.max_counts[2] == 20557075
    assert peaks.enabled is True


def test_device_status_is_decoded_not_raw(client):
    status = client.device_status()
    assert status.status == 0
    assert status.healthy is True and status.saturated is False
    assert list(status.faults) == []


# ── writes ───────────────────────────────────────────────────────────────────

def test_set_rate_reaches_the_device(client, box):
    client.set_rate_hz(7000)
    assert ("comm.cgi", {"comrdtrate": 7000}) in box.calls


def test_set_filter_reaches_the_device(client, box):
    client.set_filter(5)
    assert ("setting.cgi", {"setuserfilter": 5}) in box.calls


def test_write_configuration_sends_only_the_given_fields(client, box):
    client.write_configuration(15, name="throwing", force_unit="N")
    params = next(p for page, p in box.calls if page == "config.cgi")
    assert params["cfgid"] == 15 and params["cfgnam"] == "throwing"
    assert params["cfgfu"] == 2
    assert "cfgtu" not in params and "cfgusra" not in params


def test_units_accept_names_and_numeric_codes(client, box):
    client.write_configuration(15, force_unit="N", torque_unit=3)
    params = next(p for page, p in box.calls if page == "config.cgi")
    assert params["cfgfu"] == 2 and params["cfgtu"] == 3


def test_tool_transform_sends_units_with_the_values(client, box):
    client.set_tool_transform(15, dz=132.71, distance_unit="mm", angle_unit="degrees")
    params = next(p for page, p in box.calls if page == "config.cgi")
    assert params["cfgtdu"] == 3 and params["cfgtau"] == 1
    assert params["cfgtfx2"] == pytest.approx(132.71)


def test_bias_round_trip(client, box):
    client.clear_bias()
    params = next(p for page, p in box.calls if "setbias0" in p)
    assert params == {f"setbias{i}": 0 for i in range(6)}


def test_monitor_condition_write(client, box):
    client.set_monitor_condition(4, "fz", ">", 5_000_000, 0x02)
    params = next(p for page, p in box.calls if "mcx4" in p)
    assert params["mcx4"] == 2 and params["mcc4"] == 1
    assert params["mcv4"] == 5_000_000 and params["mco4"] == 2


def test_monitor_condition_disables_with_no_axis(client, box):
    client.set_monitor_condition(4, None, ">", 0, 0, enabled=False)
    params = next(p for page, p in box.calls if "mcx4" in p)
    assert params["mcx4"] == -1 and params["mce4"] == 0


# ── error mapping ────────────────────────────────────────────────────────────

@pytest.mark.parametrize("call", [
    lambda c: c.set_rate_hz(99999),
    lambda c: c.set_filter(99),
    lambda c: c.set_buffer_records(0),
    lambda c: c.select_configuration(99),
    lambda c: c.set_monitor_condition(0, "bogus", ">", 0, 0),
    lambda c: c.set_bias_vector([1, 2, 3]),
])
def test_invalid_arguments_become_invalid_argument(client, call):
    with pytest.raises(grpc.RpcError) as error:
        call(client)
    assert error.value.code() == grpc.StatusCode.INVALID_ARGUMENT


def test_validation_failures_never_reach_the_device(client, box):
    with pytest.raises(grpc.RpcError):
        client.set_filter(99)
    assert not any(page.endswith(".cgi") for page, _ in box.calls)


def test_missing_sensor_reports_unavailable(unavailable_client):
    for call in (lambda c: c.identity(), lambda c: c.settings(),
                 lambda c: c.set_rate_hz(1000)):
        with pytest.raises(grpc.RpcError) as error:
            call(unavailable_client)
        assert error.value.code() == grpc.StatusCode.UNAVAILABLE


def test_device_errors_become_unavailable(client, box):
    box._xml = "<not-xml"
    with pytest.raises(grpc.RpcError) as error:
        client.settings()
    assert error.value.code() == grpc.StatusCode.UNAVAILABLE


def test_network_settings_are_not_in_the_service_surface():
    names = [name for name in dir(ForceTorqueConfigClient) if not name.startswith("_")]
    assert not any("net" in name.lower() or "ip" in name.lower().split("_")
                   for name in names if name != "set_ethernet_ip_enabled")
