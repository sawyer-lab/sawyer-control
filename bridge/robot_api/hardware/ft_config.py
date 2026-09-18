"""Config-plane client for the ATI Net F/T Net Box (manual #9620-05-NET FT-22).

The Net Box has two independent interfaces. The *data plane* is RDT over UDP
49152 (samples, software bias, start/stop) and lives in `ft_sensor.py`. This
module is the *config plane*: the box's HTTP server, which reads settings from
`netftapi2.xml` and writes them through four CGI endpoints.

Scope rules this module enforces:

* Mechanism only. Nothing is written unless a caller explicitly asks; there are
  no defaults, no startup writes, and no opinions about which slot, rate or
  filter is appropriate. Those are the calling application's decisions.
* Network settings (`comnetip`, `comnetmsk`, `comnetgw`, `comnetdhcp`) are out
  of scope and unreachable through this API. They only take effect on power-up,
  so a bad value fails silently until a reboot, and recovery is physical
  (DIP switch 9). Change them from the box's own web pages if ever needed.
* Setters validate against the protocol's limits and then read back what the
  box actually accepted, because the box quantizes and clamps silently.

Standalone by design: stdlib only, no imports from this package, so it can be
lifted out into its own library unchanged.
"""

from __future__ import annotations

import urllib.parse
import urllib.request
import xml.etree.ElementTree as ET
from typing import Dict, Iterable, List, Optional

INTERNAL_SAMPLE_RATE_HZ = 7000   # fixed strain-gage sample rate (manual §9.1)
MAX_BUFFER_RECORDS = 40          # comrdtbsiz upper bound (manual §7.4)
CONFIGURATION_SLOTS = 16         # setcfgsel range (manual §7.1)
MONITOR_CONDITIONS = 16          # moncon statement count (manual §7.2)

# setuserfilter code -> cutoff in Hz; 0 disables filtering (manual §7.1).
FILTER_CUTOFF_HZ = {
    0: None, 1: 838, 2: 326, 3: 152, 4: 73, 5: 35, 6: 18,
    7: 8, 8: 5, 9: 1500, 10: 2000, 11: 2500, 12: 3000,
}

# Unit codes for the active configuration (manual §7.3, Table 7.3).
FORCE_UNITS = {1: "lbf", 2: "N", 3: "klbf", 4: "kN", 5: "kgf", 6: "gf"}
TORQUE_UNITS = {1: "lbf-in", 2: "lbf-ft", 3: "Nm", 4: "Nmm", 5: "kgf-cm", 6: "kNm"}
DISTANCE_UNITS = {1: "in", 2: "ft", 3: "mm", 4: "cm", 5: "m"}
ANGLE_UNITS = {1: "degrees", 2: "radians"}

CALIBRATION_SLOTS = 16          # cfgcalsel range (manual §7.3)

AXES = ("fx", "fy", "fz", "tx", "ty", "tz")     # moncon mcx values 0..5; -1 disables
GREATER_THAN = 1                                 # moncon mcc values (manual §10.8)
LESS_THAN = -1

# Refused everywhere as defense in depth; no public method can produce them.
_FORBIDDEN_PREFIX = "comnet"


class NetBoxError(RuntimeError):
    pass


def effective_rate_hz(requested_hz: int) -> int:
    """The rate the box will actually run given `requested_hz`.

    RDT output rate is an integer fraction of the 7 kHz internal rate; the box
    rounds a non-achievable request up to the next possible rate (manual §4.7).
    """
    if not 1 <= requested_hz <= INTERNAL_SAMPLE_RATE_HZ:
        raise ValueError(f"rate must be 1..{INTERNAL_SAMPLE_RATE_HZ} Hz, got {requested_hz}")
    divisor = max(1, INTERNAL_SAMPLE_RATE_HZ // requested_hz)
    return INTERNAL_SAMPLE_RATE_HZ // divisor


def achievable_rates_hz() -> List[int]:
    """Every distinct RDT output rate the box can produce, descending."""
    return sorted({INTERNAL_SAMPLE_RATE_HZ // n for n in range(1, INTERNAL_SAMPLE_RATE_HZ + 1)},
                  reverse=True)


def _int(text, default=0):
    try:
        return int(str(text), 0)   # base 0 so "0x80010000" parses
    except (TypeError, ValueError):
        return default


def _floats(text) -> List[float]:
    return [float(v) for v in str(text).replace(",", ";").replace(" ", ";").split(";") if v != ""]


def _ints(text) -> List[int]:
    return [_int(v) for v in str(text).replace(",", ";").replace(" ", ";").split(";") if v != ""]


def _enabled(text) -> bool:
    return str(text).strip().lower() in ("1", "enabled", "true")


class NetBoxConfig:
    """HTTP config plane of one Net Box. Construction performs no I/O."""

    def __init__(self, host: str, timeout: float = 2.0):
        self._host = host
        self._timeout = timeout

    # ── transport ────────────────────────────────────────────────────────────

    def _request(self, page: str, params: Optional[Dict[str, object]] = None) -> str:
        for key in params or {}:
            if key.lower().startswith(_FORBIDDEN_PREFIX):
                raise NetBoxError(
                    f"{key}: network settings are out of scope for this client; "
                    "change them from the Net Box web pages")
        url = f"http://{self._host}/{page}"
        if params:
            url += "?" + urllib.parse.urlencode(params)
        try:
            with urllib.request.urlopen(url, timeout=self._timeout) as response:
                return response.read().decode("utf-8", "replace")
        except Exception as exc:                      # urllib raises a wide family
            raise NetBoxError(f"{url}: {exc}") from exc

    def _elements(self, index: Optional[int] = None) -> Dict[str, str]:
        """Raw `netftapi2.xml` elements as text, for `index` or the active slot."""
        params = {"index": index} if index is not None else None
        text = self._request("netftapi2.xml", params)
        try:
            root = ET.fromstring(text)
        except ET.ParseError as exc:
            raise NetBoxError(f"malformed netftapi2.xml from {self._host}: {exc}") from exc
        return {child.tag: (child.text or "") for child in root}

    # ── reads ────────────────────────────────────────────────────────────────

    def raw(self, index: Optional[int] = None) -> Dict[str, str]:
        """Every XML element verbatim. Use for backups and for fields without
        a typed accessor."""
        return self._elements(index)

    def identity(self) -> Dict[str, object]:
        e = self._elements()
        return {
            "host": self._host,
            "ip": e.get("netip", ""),
            "mac": e.get("nethwaddr", ""),
            "firmware": e.get("mfgdigver", ""),
            "serial": e.get("mfgdigsn", ""),
            "internal_rate_hz": _int(e.get("runrate"), INTERNAL_SAMPLE_RATE_HZ),
        }

    def scaling(self, index: Optional[int] = None) -> Dict[str, object]:
        """Counts-per-unit factors and units for a slot. The data plane needs
        these to convert RDT counts, and must re-read them after any slot or
        unit change."""
        e = self._elements(index)
        return {
            "counts_per_force": _int(e.get("cfgcpf"), 1),
            "counts_per_torque": _int(e.get("cfgcpt"), 1),
            "force_unit": e.get("scfgfu", ""),
            "torque_unit": e.get("scfgtu", ""),
        }

    def configuration(self, index: Optional[int] = None) -> Dict[str, object]:
        e = self._elements(index)
        config = {
            "slot": _int(e.get("setcfgsel")) if index is None else index,
            "active_slot": _int(e.get("setcfgsel")),
            "name": e.get("cfgnam", ""),
            "calibration": _int(e.get("cfgcalsel")),
            "calibration_serial": e.get("cfgcalsn", ""),
            "tool_transform": _floats(e.get("cfgtfx", "0;0;0;0;0;0")),
            "tool_distance_unit": e.get("scfgtdu", ""),
            "tool_angle_unit": e.get("scfgtau", ""),
            "force_unit_code": _int(e.get("cfgfu")),
            "torque_unit_code": _int(e.get("cfgtu")),
            "tool_distance_unit_code": _int(e.get("cfgtdu")),
            "tool_angle_unit_code": _int(e.get("cfgtau")),
            "sensing_ranges": _floats(e.get("cfgmr", "")),
            "user_field_a": e.get("cfgusra", ""),
            "user_field_b": e.get("cfgusrb", ""),
        }
        config.update(self.scaling(index))
        return config

    def configurations(self) -> List[Dict[str, object]]:
        """All 16 slots. One HTTP round trip each."""
        return [self.configuration(slot) for slot in range(CONFIGURATION_SLOTS)]

    def settings(self) -> Dict[str, object]:
        e = self._elements()
        code = _int(e.get("setuserfilter"))
        return {
            "active_slot": _int(e.get("setcfgsel")),
            "filter_code": code,
            "filter_cutoff_hz": FILTER_CUTOFF_HZ.get(code),
            "peak_logging": _enabled(e.get("setpke")),
            "monitor_conditions_enabled": _enabled(e.get("setmce")),
            "bias_vector": _ints(e.get("setbias", "")),
        }

    def communications(self) -> Dict[str, object]:
        """RDT transport settings only; network addressing is out of scope."""
        e = self._elements()
        return {
            "rdt_enabled": _enabled(e.get("comrdte")),
            "rate_hz": _int(e.get("comrdtrate")),
            "buffer_records": _int(e.get("comrdtbsiz")),
            "internal_rate_hz": _int(e.get("runrate"), INTERNAL_SAMPLE_RATE_HZ),
        }

    def monitor_conditions(self) -> List[Dict[str, object]]:
        """The 16 hardware threshold statements. These are evaluated inside the
        box at the internal 7 kHz rate regardless of the RDT output rate, and
        report through status bit 16."""
        e = self._elements()
        enabled = _ints(e.get("mce", ""))
        axes = _ints(e.get("mcx", ""))
        comparisons = _ints(e.get("mcc", ""))
        values = _ints(e.get("mcv", ""))
        outputs = _ints(e.get("mco", ""))

        def at(values_, i, default=0):
            return values_[i] if i < len(values_) else default

        conditions = []
        for i in range(MONITOR_CONDITIONS):
            axis = at(axes, i, -1)
            conditions.append({
                "index": i,
                "enabled": bool(at(enabled, i)),
                "axis": AXES[axis] if 0 <= axis < len(AXES) else None,
                "axis_code": axis,
                "comparison": ">" if at(comparisons, i) == GREATER_THAN else "<",
                "counts": at(values, i),
                "output_code": at(outputs, i),
            })
        return conditions

    def peaks(self) -> Dict[str, object]:
        """Min/max per axis captured in hardware since the last reset, in counts.
        Only meaningful while peak logging is enabled."""
        e = self._elements()
        return {
            "min_counts": _ints(e.get("runpkmn", "")),
            "max_counts": _ints(e.get("runpkmx", "")),
            "enabled": _enabled(e.get("setpke")),
        }

    def status(self) -> Dict[str, object]:
        """Live run-time status word and Monitor Condition latch state."""
        e = self._elements()
        return {
            "status": _int(e.get("runstat")),
            "conditions_breached": _int(e.get("runmcb")),
            "condition_output": _int(e.get("runmco")),
            "condition_latched": bool(_int(e.get("runmcl"))),
        }

    # ── writes: RDT transport (comm.cgi) ─────────────────────────────────────

    def set_rate_hz(self, rate_hz: int) -> int:
        """Set the RDT output rate. Returns the rate the box actually adopted,
        which may be higher than requested because the box quantizes to integer
        fractions of 7 kHz.

        Changing this disturbs any running RDT stream; the data plane is
        responsible for restarting it.
        """
        effective_rate_hz(rate_hz)                  # validates the range
        self._request("comm.cgi", {"comrdtrate": int(rate_hz)})
        return self.communications()["rate_hz"]

    def set_buffer_records(self, records: int) -> int:
        """Records per UDP packet in buffered streaming mode (1..40)."""
        if not 1 <= records <= MAX_BUFFER_RECORDS:
            raise ValueError(f"buffer size must be 1..{MAX_BUFFER_RECORDS}, got {records}")
        self._request("comm.cgi", {"comrdtbsiz": int(records)})
        return self.communications()["buffer_records"]

    def set_rdt_enabled(self, enabled: bool) -> bool:
        """Enable/disable the RDT interface itself. Disabling it stops the data
        plane from receiving anything."""
        self._request("comm.cgi", {"comrdte": 1 if enabled else 0})
        return self.communications()["rdt_enabled"]

    def set_ethernet_ip_enabled(self, enabled: bool) -> bool:
        """Enable/disable the EtherNet/IP protocol (manual §7.4). Independent of
        RDT and of the box's web server, so it does not affect this client or
        the data plane."""
        self._request("comm.cgi", {"comeipe": 1 if enabled else 0})
        return _enabled(self._elements().get("comeipe"))

    # ── writes: global settings (setting.cgi) ────────────────────────────────

    def set_filter(self, code: int) -> int:
        """Set the low-pass filter by code (0 = off, 1..12 per FILTER_CUTOFF_HZ).

        The cutoff should be chosen against the RDT output rate: the box always
        samples at 7 kHz internally and decimates without filtering, so a low
        output rate with no filter aliases. This client does not choose for you.
        """
        if code not in FILTER_CUTOFF_HZ:
            raise ValueError(f"filter code must be one of {sorted(FILTER_CUTOFF_HZ)}, got {code}")
        self._request("setting.cgi", {"setuserfilter": int(code)})
        return self.settings()["filter_code"]

    def set_peak_logging(self, enabled: bool) -> bool:
        self._request("setting.cgi", {"setpke": 1 if enabled else 0})
        return self.settings()["peak_logging"]

    def select_configuration(self, slot: int) -> int:
        """Activate a configuration slot.

        Note this can change units and calibration, and therefore the
        counts-per-unit factors the data plane scales with; re-read `scaling()`
        afterwards.
        """
        _check_slot(slot)
        self._request("setting.cgi", {"setcfgsel": int(slot)})
        return self.settings()["active_slot"]

    def set_bias_vector(self, gages: Iterable[int]) -> List[int]:
        """Set the six per-strain-gage software bias offsets. All zeros removes
        the bias (manual §4.4). Distinct from the data plane's RDT bias command,
        which computes a bias from the current load."""
        values = list(gages)
        if len(values) != 6:
            raise ValueError(f"bias vector needs 6 values, got {len(values)}")
        params = {}
        for index, value in enumerate(values):
            if not -32768 <= value <= 32767:
                raise ValueError(f"bias {index} out of int16 range: {value}")
            params[f"setbias{index}"] = int(value)
        self._request("setting.cgi", params)
        return self.settings()["bias_vector"]

    def clear_bias(self) -> List[int]:
        return self.set_bias_vector([0] * 6)

    # ── writes: configuration slots (config.cgi) ─────────────────────────────

    def write_configuration(self, slot: int, name: Optional[str] = None,
                            calibration: Optional[int] = None,
                            force_unit=None, torque_unit=None,
                            distance_unit=None, angle_unit=None,
                            user_field_a: Optional[str] = None,
                            user_field_b: Optional[str] = None) -> Dict[str, object]:
        """Write fields of configuration slot `slot`, leaving the rest alone.

        `cfgid` is required on every config.cgi call and identifies the target
        slot, so a write never touches the active slot implicitly.

        Units may be given as a code or as the manual's menu name, e.g.
        `force_unit="N"` or `force_unit=2`. Changing force or torque units
        changes the slot's counts-per-unit factors, so the data plane must
        re-read `scaling()` afterwards. `distance_unit` and `angle_unit` are
        the units the slot's tool transform is expressed in.
        """
        _check_slot(slot)
        params: Dict[str, object] = {"cfgid": int(slot)}
        if name is not None:
            params["cfgnam"] = name[:32]
        if calibration is not None:
            if not 0 <= calibration < CALIBRATION_SLOTS:
                raise ValueError(
                    f"calibration must be 0..{CALIBRATION_SLOTS - 1}, got {calibration}")
            params["cfgcalsel"] = int(calibration)
        if force_unit is not None:
            params["cfgfu"] = _unit_code(force_unit, FORCE_UNITS, "force")
        if torque_unit is not None:
            params["cfgtu"] = _unit_code(torque_unit, TORQUE_UNITS, "torque")
        if distance_unit is not None:
            params["cfgtdu"] = _unit_code(distance_unit, DISTANCE_UNITS, "distance")
        if angle_unit is not None:
            params["cfgtau"] = _unit_code(angle_unit, ANGLE_UNITS, "angle")
        if user_field_a is not None:
            params["cfgusra"] = user_field_a[:16]
        if user_field_b is not None:
            params["cfgusrb"] = user_field_b[:16]
        if len(params) == 1:
            raise ValueError("write_configuration called with nothing to write")
        self._request("config.cgi", params)
        return self.configuration(slot)

    def set_tool_transform(self, slot: int, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0,
                           rx: float = 0.0, ry: float = 0.0, rz: float = 0.0,
                           distance_unit=None, angle_unit=None) -> List[float]:
        """Set the tool transform of `slot`. All six components are written
        together.

        Distances are interpreted in the slot's distance unit and rotations in
        its angle unit. Pass `distance_unit` / `angle_unit` to set those in the
        same request, so the values are never briefly interpreted in the unit
        the slot happened to hold before; otherwise read `configuration()` to
        see which units apply.
        """
        _check_slot(slot)
        components = (dx, dy, dz, rx, ry, rz)
        params: Dict[str, object] = {"cfgid": int(slot)}
        if distance_unit is not None:
            params["cfgtdu"] = _unit_code(distance_unit, DISTANCE_UNITS, "distance")
        if angle_unit is not None:
            params["cfgtau"] = _unit_code(angle_unit, ANGLE_UNITS, "angle")
        params.update({f"cfgtfx{i}": float(v) for i, v in enumerate(components)})
        self._request("config.cgi", params)
        return self.configuration(slot)["tool_transform"]

    # ── writes: monitor conditions (moncon.cgi) ──────────────────────────────

    def set_monitor_conditions_enabled(self, enabled: bool) -> bool:
        """Master enable for all Condition statement processing."""
        self._request("moncon.cgi", {"setmce": 1 if enabled else 0})
        return self.settings()["monitor_conditions_enabled"]

    def set_monitor_condition(self, index: int, axis: Optional[str], comparison: str,
                              counts: int, output_code: int,
                              enabled: bool = True) -> Dict[str, object]:
        """Define threshold statement `index`.

        `axis` is one of AXES, or None to disable the statement. `comparison` is
        ">" or "<". `counts` is in raw counts, not user units - convert with the
        slot's counts_per_force / counts_per_torque.
        """
        if not 0 <= index < MONITOR_CONDITIONS:
            raise ValueError(f"condition index must be 0..{MONITOR_CONDITIONS - 1}, got {index}")
        if axis is None:
            axis_code = -1
        elif axis.lower() in AXES:
            axis_code = AXES.index(axis.lower())
        else:
            raise ValueError(f"axis must be one of {AXES} or None, got {axis!r}")
        if comparison not in (">", "<"):
            raise ValueError(f"comparison must be '>' or '<', got {comparison!r}")
        if not -2147483648 <= counts <= 2147483647:
            raise ValueError(f"counts out of int32 range: {counts}")
        if not 0 <= output_code <= 0xFF:
            raise ValueError(f"output code must be 0x00..0xFF, got {output_code}")
        self._request("moncon.cgi", {
            f"mce{index}": 1 if enabled else 0,
            f"mcx{index}": axis_code,
            f"mcc{index}": GREATER_THAN if comparison == ">" else LESS_THAN,
            f"mcv{index}": int(counts),
            f"mco{index}": int(output_code),
        })
        return self.monitor_conditions()[index]


def _unit_code(value, table: Dict[int, str], kind: str) -> int:
    """Coerce a unit given as a code or as its menu name into its code."""
    if isinstance(value, str):
        for code, name in table.items():
            if name.lower() == value.strip().lower():
                return code
        raise ValueError(f"{kind} unit must be one of {sorted(table.values())}, got {value!r}")
    if value not in table:
        raise ValueError(f"{kind} unit code must be one of {sorted(table)}, got {value}")
    return int(value)


def _check_slot(slot: int) -> None:
    if not 0 <= slot < CONFIGURATION_SLOTS:
        raise ValueError(f"slot must be 0..{CONFIGURATION_SLOTS - 1}, got {slot}")
