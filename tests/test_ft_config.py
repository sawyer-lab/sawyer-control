import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "bridge"))

from robot_api.hardware.ft_config import (  # noqa: E402
    AXES, CONFIGURATION_SLOTS, FILTER_CUTOFF_HZ, INTERNAL_SAMPLE_RATE_HZ,
    NetBoxConfig, NetBoxError, achievable_rates_hz, effective_rate_hz,
)

# Trimmed from a live netftapi2.xml (Gamma SI-330-30, firmware 2.2.59).
SAMPLE_XML = """<?xml version="1.0"?><netft>
<runstat>0x00000000</runstat><runrate>7000</runrate>
<runpkmx>15186840;15450145;20557075;1367250;2491043;1854244</runpkmx>
<runpkmn>-15529289;-16465881;-25931018;-3018564;-1881652;-729991</runpkmn>
<runmcb>0x00000000</runmcb><runmco>0x00</runmco><runmcl>0</runmcl>
<cfgnam>empty</cfgnam><cfgcalsel>0</cfgcalsel><cfgcalsn>FT27462</cfgcalsn>
<scfgfu>N</scfgfu><scfgtu>Nm</scfgtu>
<cfgtfx>0;0;132.710;0;0;0</cfgtfx><scfgtdu>mm</scfgtdu><scfgtau>degrees</scfgtau>
<cfgcpf>1000000</cfgcpf><cfgcpt>1000000</cfgcpt>
<cfgmr>330;330;990;30;30;30</cfgmr><cfgusra>empty</cfgusra><cfgusrb>empty</cfgusrb>
<setcfgsel>0</setcfgsel><setpke>1</setpke><setmce>0</setmce>
<setbias>745;17;447;184;-596;-253</setbias>
<mce>1;1;0;0;0;0;0;0;0;0;0;0;0;0;0;0</mce>
<mcx>3;3;-1;-1;-1;-1;-1;-1;-1;-1;-1;-1;-1;-1;-1;-1</mcx>
<mcc>-1;1;1;1;1;1;1;1;1;1;1;1;1;1;1;1</mcc>
<mcv>100000;-10000;0;0;0;0;0;0;0;0;0;0;0;0;0;0</mcv>
<mco>0x01;0x00;0x00;0x00;0x00;0x00;0x00;0x00;0x00;0x00;0x00;0x00;0x00;0x00;0x00;0x00</mco>
<setuserfilter>0</setuserfilter>
<nethwaddr>00:16:BD:00:23:56</nethwaddr><netip>192.168.1.11</netip>
<comnetip>192.168.1.11</comnetip><comnetdhcp>Enabled</comnetdhcp>
<comrdte>Enabled</comrdte><comrdtrate>100</comrdtrate><comrdtbsiz>1</comrdtbsiz>
<mfgdigsn>LOT2821</mfgdigsn><mfgdigver>2.2.59</mfgdigver>
</netft>"""


class FakeBox(NetBoxConfig):
    """Records every request instead of performing HTTP."""

    def __init__(self, xml=SAMPLE_XML):
        super().__init__("fake")
        self.calls = []
        self._xml = xml

    def _request(self, page, params=None):
        for key in params or {}:
            if key.lower().startswith("comnet"):
                raise NetBoxError(f"{key}: network settings are out of scope")
        self.calls.append((page, dict(params or {})))
        return self._xml


# ── rate quantization ────────────────────────────────────────────────────────

@pytest.mark.parametrize("requested,expected", [
    (7000, 7000), (3500, 3500), (2333, 2333), (1000, 1000),
    (100, 100), (99, 100), (150, 152), (1, 1),
])
def test_effective_rate_rounds_up_to_achievable(requested, expected):
    assert effective_rate_hz(requested) == expected


def test_effective_rate_is_always_an_integer_fraction_of_the_internal_rate():
    for requested in (1, 7, 37, 250, 999, 4000, 7000):
        rate = effective_rate_hz(requested)
        assert INTERNAL_SAMPLE_RATE_HZ // (INTERNAL_SAMPLE_RATE_HZ // rate) == rate
        assert rate >= requested


@pytest.mark.parametrize("bad", [0, -1, 7001])
def test_effective_rate_rejects_out_of_range(bad):
    with pytest.raises(ValueError):
        effective_rate_hz(bad)


def test_achievable_rates_are_descending_and_include_the_extremes():
    rates = achievable_rates_hz()
    assert rates[0] == INTERNAL_SAMPLE_RATE_HZ and rates[-1] == 1
    assert rates == sorted(set(rates), reverse=True)


# ── network settings are unreachable ─────────────────────────────────────────

def test_no_public_method_writes_network_settings():
    box = FakeBox()
    box.set_rate_hz(1000)
    box.set_filter(2)
    box.set_peak_logging(True)
    box.select_configuration(15)
    box.set_tool_transform(15, dz=10.0)
    box.set_monitor_condition(0, "fz", ">", 1000, 0x01)
    written = {key for _, params in box.calls for key in params}
    assert not any(key.lower().startswith("comnet") for key in written)


def test_transport_refuses_network_parameters_directly():
    box = FakeBox()
    for key in ("comnetip", "comnetmsk", "comnetgw", "comnetdhcp", "COMNETIP"):
        with pytest.raises(NetBoxError):
            box._request("comm.cgi", {key: "192.168.1.99"})


def test_construction_performs_no_io():
    box = FakeBox()
    assert box.calls == []


# ── parsing ──────────────────────────────────────────────────────────────────

def test_scaling_reads_calibration_factors_and_units():
    assert FakeBox().scaling() == {
        "counts_per_force": 1000000, "counts_per_torque": 1000000,
        "force_unit": "N", "torque_unit": "Nm",
    }


def test_configuration_parses_tool_transform_and_ranges():
    config = FakeBox().configuration()
    assert config["tool_transform"] == [0.0, 0.0, 132.710, 0.0, 0.0, 0.0]
    assert config["sensing_ranges"] == [330.0, 330.0, 990.0, 30.0, 30.0, 30.0]
    assert config["calibration_serial"] == "FT27462"
    assert config["tool_distance_unit"] == "mm"


def test_settings_decodes_filter_and_flags():
    settings = FakeBox().settings()
    assert settings["filter_code"] == 0 and settings["filter_cutoff_hz"] is None
    assert settings["peak_logging"] is True
    assert settings["monitor_conditions_enabled"] is False
    assert settings["bias_vector"] == [745, 17, 447, 184, -596, -253]


def test_communications_reports_rdt_only():
    comms = FakeBox().communications()
    assert comms == {"rdt_enabled": True, "rate_hz": 100,
                     "buffer_records": 1, "internal_rate_hz": 7000}
    assert not any("net" in key for key in comms)


def test_monitor_conditions_decode_axis_and_comparison():
    conditions = FakeBox().monitor_conditions()
    assert len(conditions) == 16
    assert conditions[0]["enabled"] and conditions[0]["axis"] == "tx"
    assert conditions[0]["comparison"] == "<" and conditions[0]["counts"] == 100000
    assert conditions[0]["output_code"] == 0x01
    assert conditions[1]["comparison"] == ">" and conditions[1]["counts"] == -10000
    assert conditions[2]["axis"] is None


def test_status_parses_hex_words():
    assert FakeBox().status()["status"] == 0
    assert FakeBox().status()["condition_latched"] is False


def test_peaks_are_reported_in_counts():
    peaks = FakeBox().peaks()
    assert peaks["max_counts"][2] == 20557075
    assert peaks["min_counts"][2] == -25931018


def test_malformed_xml_raises_netbox_error():
    with pytest.raises(NetBoxError):
        FakeBox(xml="<not-xml").settings()


# ── setter validation ────────────────────────────────────────────────────────

def test_set_rate_sends_comrdtrate_and_reads_back():
    box = FakeBox()
    box.set_rate_hz(7000)
    assert box.calls[0] == ("comm.cgi", {"comrdtrate": 7000})


@pytest.mark.parametrize("bad", [0, 7001, -5])
def test_set_rate_validates_before_writing(bad):
    box = FakeBox()
    with pytest.raises(ValueError):
        box.set_rate_hz(bad)
    assert box.calls == []


@pytest.mark.parametrize("bad", [0, 41, -1])
def test_set_buffer_records_validates(bad):
    box = FakeBox()
    with pytest.raises(ValueError):
        box.set_buffer_records(bad)
    assert box.calls == []


@pytest.mark.parametrize("code", sorted(FILTER_CUTOFF_HZ))
def test_set_filter_accepts_every_documented_code(code):
    box = FakeBox()
    box.set_filter(code)
    assert box.calls[0] == ("setting.cgi", {"setuserfilter": code})


@pytest.mark.parametrize("bad", [-1, 13, 99])
def test_set_filter_rejects_undocumented_codes(bad):
    box = FakeBox()
    with pytest.raises(ValueError):
        box.set_filter(bad)
    assert box.calls == []


@pytest.mark.parametrize("bad", [-1, CONFIGURATION_SLOTS, 99])
def test_slot_bounds_are_enforced(bad):
    box = FakeBox()
    for call in (lambda: box.select_configuration(bad),
                 lambda: box.write_configuration(bad, name="x"),
                 lambda: box.set_tool_transform(bad, dz=1.0)):
        with pytest.raises(ValueError):
            call()
    assert box.calls == []


def test_write_configuration_always_targets_an_explicit_slot():
    box = FakeBox()
    box.write_configuration(15, name="throwing")
    page, params = box.calls[0]
    assert page == "config.cgi" and params["cfgid"] == 15 and params["cfgnam"] == "throwing"


def test_write_configuration_refuses_an_empty_write():
    box = FakeBox()
    with pytest.raises(ValueError):
        box.write_configuration(15)
    assert box.calls == []


def test_tool_transform_writes_all_six_components():
    box = FakeBox()
    box.set_tool_transform(15, dz=132.71)
    _, params = box.calls[0]
    assert params["cfgid"] == 15
    assert [params[f"cfgtfx{i}"] for i in range(6)] == [0.0, 0.0, 132.71, 0.0, 0.0, 0.0]


def test_bias_vector_requires_six_int16_values():
    box = FakeBox()
    with pytest.raises(ValueError):
        box.set_bias_vector([0, 0, 0])
    with pytest.raises(ValueError):
        box.set_bias_vector([0, 0, 0, 0, 0, 40000])
    assert box.calls == []
    box.clear_bias()
    assert box.calls[0][1] == {f"setbias{i}": 0 for i in range(6)}


@pytest.mark.parametrize("axis", AXES)
def test_monitor_condition_maps_every_axis(axis):
    box = FakeBox()
    box.set_monitor_condition(0, axis, ">", 1000, 0x01)
    assert box.calls[0][1]["mcx0"] == AXES.index(axis)


def test_monitor_condition_none_axis_disables_statement():
    box = FakeBox()
    box.set_monitor_condition(3, None, ">", 0, 0)
    assert box.calls[0][1]["mcx3"] == -1


def test_monitor_condition_validates_arguments():
    box = FakeBox()
    for bad in (lambda: box.set_monitor_condition(16, "fz", ">", 0, 0),
                lambda: box.set_monitor_condition(0, "bogus", ">", 0, 0),
                lambda: box.set_monitor_condition(0, "fz", "!=", 0, 0),
                lambda: box.set_monitor_condition(0, "fz", ">", 2**31, 0),
                lambda: box.set_monitor_condition(0, "fz", ">", 0, 256)):
        with pytest.raises(ValueError):
            bad()
    assert box.calls == []


def test_comparison_encoding_matches_the_manual():
    box = FakeBox()
    box.set_monitor_condition(0, "fz", ">", 10, 1)
    box.set_monitor_condition(1, "fz", "<", 10, 1)
    writes = [params for page, params in box.calls if page == "moncon.cgi"]
    assert writes[0]["mcc0"] == 1
    assert writes[1]["mcc1"] == -1


# ── unit handling ────────────────────────────────────────────────────────────

from robot_api.hardware.ft_config import (  # noqa: E402
    ANGLE_UNITS, CALIBRATION_SLOTS, DISTANCE_UNITS, FORCE_UNITS, TORQUE_UNITS,
)


def test_units_match_the_manual_tables():
    assert FORCE_UNITS == {1: "lbf", 2: "N", 3: "klbf", 4: "kN", 5: "kgf", 6: "gf"}
    assert TORQUE_UNITS == {1: "lbf-in", 2: "lbf-ft", 3: "Nm", 4: "Nmm", 5: "kgf-cm", 6: "kNm"}
    assert DISTANCE_UNITS == {1: "in", 2: "ft", 3: "mm", 4: "cm", 5: "m"}
    assert ANGLE_UNITS == {1: "degrees", 2: "radians"}


@pytest.mark.parametrize("given,expected", [("N", 2), ("n", 2), (2, 2), ("kgf", 5), (6, 6)])
def test_force_unit_accepts_name_or_code(given, expected):
    box = FakeBox()
    box.write_configuration(15, force_unit=given)
    assert box.calls[0][1]["cfgfu"] == expected


@pytest.mark.parametrize("field,value,param", [
    ("torque_unit", "Nm", "cfgtu"),
    ("distance_unit", "mm", "cfgtdu"),
    ("angle_unit", "radians", "cfgtau"),
])
def test_every_unit_field_is_writable(field, value, param):
    box = FakeBox()
    box.write_configuration(15, **{field: value})
    assert param in box.calls[0][1]


@pytest.mark.parametrize("field,bad", [
    ("force_unit", "newtons"), ("force_unit", 7), ("torque_unit", 0),
    ("distance_unit", "yd"), ("angle_unit", 3),
])
def test_unknown_units_are_rejected_before_writing(field, bad):
    box = FakeBox()
    with pytest.raises(ValueError):
        box.write_configuration(15, **{field: bad})
    assert box.calls == []


def test_calibration_selection_is_bounded():
    box = FakeBox()
    for bad in (-1, CALIBRATION_SLOTS):
        with pytest.raises(ValueError):
            box.write_configuration(15, calibration=bad)
    assert box.calls == []


def test_tool_transform_can_set_its_own_units_atomically():
    box = FakeBox()
    box.set_tool_transform(15, dz=132.71, distance_unit="mm", angle_unit="degrees")
    page, params = box.calls[0]
    assert page == "config.cgi"
    assert params["cfgtdu"] == 3 and params["cfgtau"] == 1
    assert params["cfgtfx2"] == 132.71


def test_ethernet_ip_toggle_writes_comeipe():
    box = FakeBox()
    box.set_ethernet_ip_enabled(False)
    assert box.calls[0] == ("comm.cgi", {"comeipe": 0})


def test_configuration_reports_unit_codes():
    config = FakeBox().configuration()
    assert config["force_unit_code"] == 0      # absent from the trimmed fixture
    assert "tool_distance_unit_code" in config


# ── coverage audit against the manual's CGI tables ───────────────────────────

def test_every_documented_writable_parameter_is_reachable():
    """Manual Tables 7.1-7.4, minus the network settings that are out of scope
    and the fieldbus CAN protocol selector."""
    box = FakeBox()
    box.set_rate_hz(1000)
    box.set_buffer_records(40)
    box.set_rdt_enabled(True)
    box.set_ethernet_ip_enabled(True)
    box.set_filter(0)
    box.set_peak_logging(True)
    box.select_configuration(0)
    box.clear_bias()
    box.write_configuration(15, name="x", calibration=0, force_unit="N", torque_unit="Nm",
                            distance_unit="mm", angle_unit="degrees",
                            user_field_a="a", user_field_b="b")
    box.set_tool_transform(15, dx=1, dy=2, dz=3, rx=4, ry=5, rz=6)
    box.set_monitor_conditions_enabled(True)
    box.set_monitor_condition(0, "fz", ">", 1, 1)

    written = {key for page, params in box.calls if page.endswith(".cgi") for key in params}
    expected = {
        "comrdtrate", "comrdtbsiz", "comrdte", "comeipe",                    # 7.4
        "setuserfilter", "setpke", "setcfgsel", *[f"setbias{i}" for i in range(6)],  # 7.1
        "cfgid", "cfgnam", "cfgcalsel", "cfgfu", "cfgtu", "cfgtdu", "cfgtau",
        "cfgusra", "cfgusrb", *[f"cfgtfx{i}" for i in range(6)],             # 7.3
        "setmce", "mce0", "mcx0", "mcc0", "mcv0", "mco0",                    # 7.2
    }
    assert expected <= written, f"unreachable parameters: {sorted(expected - written)}"
