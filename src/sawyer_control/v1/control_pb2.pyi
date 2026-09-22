from typing import ClassVar as _ClassVar
from typing import Iterable as _Iterable
from typing import Mapping as _Mapping
from typing import Optional as _Optional
from typing import Union as _Union

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper

DESCRIPTOR: _descriptor.FileDescriptor

class ControlMode(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    CONTROL_MODE_UNSPECIFIED: _ClassVar[ControlMode]
    POSITION: _ClassVar[ControlMode]
    VELOCITY: _ClassVar[ControlMode]
    TORQUE: _ClassVar[ControlMode]
    TRAJECTORY: _ClassVar[ControlMode]

class CameraId(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    CAMERA_ID_UNSPECIFIED: _ClassVar[CameraId]
    HEAD: _ClassVar[CameraId]
    HAND: _ClassVar[CameraId]
    BRIO: _ClassVar[CameraId]

CONTROL_MODE_UNSPECIFIED: ControlMode
POSITION: ControlMode
VELOCITY: ControlMode
TORQUE: ControlMode
TRAJECTORY: ControlMode
CAMERA_ID_UNSPECIFIED: CameraId
HEAD: CameraId
HAND: CameraId
BRIO: CameraId

class Empty(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class HealthRequest(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class HealthReply(_message.Message):
    __slots__ = ("bridge_version", "protocol_version")
    PROTOCOL_VERSION_FIELD_NUMBER: _ClassVar[int]
    BRIDGE_VERSION_FIELD_NUMBER: _ClassVar[int]
    protocol_version: str
    bridge_version: str
    def __init__(
        self, protocol_version: _Optional[str] = ..., bridge_version: _Optional[str] = ...
    ) -> None: ...

class JointVector(_message.Message):
    __slots__ = ("values",)
    VALUES_FIELD_NUMBER: _ClassVar[int]
    values: _containers.RepeatedScalarFieldContainer[float]
    def __init__(self, values: _Optional[_Iterable[float]] = ...) -> None: ...

class Position(_message.Message):
    __slots__ = ("x", "y", "z")
    X_FIELD_NUMBER: _ClassVar[int]
    Y_FIELD_NUMBER: _ClassVar[int]
    Z_FIELD_NUMBER: _ClassVar[int]
    x: float
    y: float
    z: float
    def __init__(
        self, x: _Optional[float] = ..., y: _Optional[float] = ..., z: _Optional[float] = ...
    ) -> None: ...

class Quaternion(_message.Message):
    __slots__ = ("w", "x", "y", "z")
    X_FIELD_NUMBER: _ClassVar[int]
    Y_FIELD_NUMBER: _ClassVar[int]
    Z_FIELD_NUMBER: _ClassVar[int]
    W_FIELD_NUMBER: _ClassVar[int]
    x: float
    y: float
    z: float
    w: float
    def __init__(
        self,
        x: _Optional[float] = ...,
        y: _Optional[float] = ...,
        z: _Optional[float] = ...,
        w: _Optional[float] = ...,
    ) -> None: ...

class Pose(_message.Message):
    __slots__ = ("orientation", "position")
    POSITION_FIELD_NUMBER: _ClassVar[int]
    ORIENTATION_FIELD_NUMBER: _ClassVar[int]
    position: Position
    orientation: Quaternion
    def __init__(
        self,
        position: _Optional[_Union[Position, _Mapping]] = ...,
        orientation: _Optional[_Union[Quaternion, _Mapping]] = ...,
    ) -> None: ...

class RobotState(_message.Message):
    __slots__ = (
        "clock_id",
        "efforts",
        "enabled",
        "pose",
        "positions",
        "received_ns",
        "sample_id",
        "session_id",
        "source_stamp_s",
        "stopped",
        "timestamp_s",
        "velocities",
        "velocity_valid",
    )
    TIMESTAMP_S_FIELD_NUMBER: _ClassVar[int]
    POSITIONS_FIELD_NUMBER: _ClassVar[int]
    VELOCITIES_FIELD_NUMBER: _ClassVar[int]
    EFFORTS_FIELD_NUMBER: _ClassVar[int]
    POSE_FIELD_NUMBER: _ClassVar[int]
    ENABLED_FIELD_NUMBER: _ClassVar[int]
    STOPPED_FIELD_NUMBER: _ClassVar[int]
    RECEIVED_NS_FIELD_NUMBER: _ClassVar[int]
    SAMPLE_ID_FIELD_NUMBER: _ClassVar[int]
    CLOCK_ID_FIELD_NUMBER: _ClassVar[int]
    SOURCE_STAMP_S_FIELD_NUMBER: _ClassVar[int]
    VELOCITY_VALID_FIELD_NUMBER: _ClassVar[int]
    SESSION_ID_FIELD_NUMBER: _ClassVar[int]
    timestamp_s: float
    positions: JointVector
    velocities: JointVector
    efforts: JointVector
    pose: Pose
    enabled: bool
    stopped: bool
    received_ns: int
    sample_id: int
    clock_id: str
    source_stamp_s: float
    velocity_valid: bool
    session_id: str
    def __init__(
        self,
        timestamp_s: _Optional[float] = ...,
        positions: _Optional[_Union[JointVector, _Mapping]] = ...,
        velocities: _Optional[_Union[JointVector, _Mapping]] = ...,
        efforts: _Optional[_Union[JointVector, _Mapping]] = ...,
        pose: _Optional[_Union[Pose, _Mapping]] = ...,
        enabled: bool = ...,
        stopped: bool = ...,
        received_ns: _Optional[int] = ...,
        sample_id: _Optional[int] = ...,
        clock_id: _Optional[str] = ...,
        source_stamp_s: _Optional[float] = ...,
        velocity_valid: bool = ...,
        session_id: _Optional[str] = ...,
    ) -> None: ...

class JointCommandSample(_message.Message):
    __slots__ = ("acceleration", "effort", "position", "velocity")
    POSITION_FIELD_NUMBER: _ClassVar[int]
    VELOCITY_FIELD_NUMBER: _ClassVar[int]
    EFFORT_FIELD_NUMBER: _ClassVar[int]
    ACCELERATION_FIELD_NUMBER: _ClassVar[int]
    position: JointVector
    velocity: JointVector
    effort: JointVector
    acceleration: JointVector
    def __init__(
        self,
        position: _Optional[_Union[JointVector, _Mapping]] = ...,
        velocity: _Optional[_Union[JointVector, _Mapping]] = ...,
        effort: _Optional[_Union[JointVector, _Mapping]] = ...,
        acceleration: _Optional[_Union[JointVector, _Mapping]] = ...,
    ) -> None: ...

class MoveToRequest(_message.Message):
    __slots__ = ("target", "timeout_s")
    TARGET_FIELD_NUMBER: _ClassVar[int]
    TIMEOUT_S_FIELD_NUMBER: _ClassVar[int]
    target: JointVector
    timeout_s: float
    def __init__(
        self,
        target: _Optional[_Union[JointVector, _Mapping]] = ...,
        timeout_s: _Optional[float] = ...,
    ) -> None: ...

class JointCommandRequest(_message.Message):
    __slots__ = ("cycle_id", "mode", "sample")
    MODE_FIELD_NUMBER: _ClassVar[int]
    SAMPLE_FIELD_NUMBER: _ClassVar[int]
    CYCLE_ID_FIELD_NUMBER: _ClassVar[int]
    mode: ControlMode
    sample: JointCommandSample
    cycle_id: str
    def __init__(
        self,
        mode: _Optional[_Union[ControlMode, str]] = ...,
        sample: _Optional[_Union[JointCommandSample, _Mapping]] = ...,
        cycle_id: _Optional[str] = ...,
    ) -> None: ...

class CommandResult(_message.Message):
    __slots__ = ("clock_id", "cycle_id", "message", "published_ns", "received_ns", "success")
    SUCCESS_FIELD_NUMBER: _ClassVar[int]
    MESSAGE_FIELD_NUMBER: _ClassVar[int]
    CYCLE_ID_FIELD_NUMBER: _ClassVar[int]
    CLOCK_ID_FIELD_NUMBER: _ClassVar[int]
    RECEIVED_NS_FIELD_NUMBER: _ClassVar[int]
    PUBLISHED_NS_FIELD_NUMBER: _ClassVar[int]
    success: bool
    message: str
    cycle_id: str
    clock_id: str
    received_ns: int
    published_ns: int
    def __init__(
        self,
        success: bool = ...,
        message: _Optional[str] = ...,
        cycle_id: _Optional[str] = ...,
        clock_id: _Optional[str] = ...,
        received_ns: _Optional[int] = ...,
        published_ns: _Optional[int] = ...,
    ) -> None: ...

class StopRequest(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class StreamStateRequest(_message.Message):
    __slots__ = ("rate_hz",)
    RATE_HZ_FIELD_NUMBER: _ClassVar[int]
    rate_hz: float
    def __init__(self, rate_hz: _Optional[float] = ...) -> None: ...

class StreamReadingsRequest(_message.Message):
    __slots__ = ("rate_hz",)
    RATE_HZ_FIELD_NUMBER: _ClassVar[int]
    rate_hz: float
    def __init__(self, rate_hz: _Optional[float] = ...) -> None: ...

class GripperState(_message.Message):
    __slots__ = ("grasping", "position", "state")
    POSITION_FIELD_NUMBER: _ClassVar[int]
    GRASPING_FIELD_NUMBER: _ClassVar[int]
    STATE_FIELD_NUMBER: _ClassVar[int]
    position: float
    grasping: bool
    state: str
    def __init__(
        self, position: _Optional[float] = ..., grasping: bool = ..., state: _Optional[str] = ...
    ) -> None: ...

class ForceTorqueReading(_message.Message):
    __slots__ = (
        "clock_id",
        "config_version",
        "faults",
        "force_unit",
        "fx",
        "fy",
        "fz",
        "healthy",
        "received_ns",
        "sample_id",
        "sequence",
        "status",
        "timestamp_s",
        "torque_unit",
        "tx",
        "ty",
        "tz",
    )
    FX_FIELD_NUMBER: _ClassVar[int]
    FY_FIELD_NUMBER: _ClassVar[int]
    FZ_FIELD_NUMBER: _ClassVar[int]
    TX_FIELD_NUMBER: _ClassVar[int]
    TY_FIELD_NUMBER: _ClassVar[int]
    TZ_FIELD_NUMBER: _ClassVar[int]
    SEQUENCE_FIELD_NUMBER: _ClassVar[int]
    STATUS_FIELD_NUMBER: _ClassVar[int]
    TIMESTAMP_S_FIELD_NUMBER: _ClassVar[int]
    FAULTS_FIELD_NUMBER: _ClassVar[int]
    HEALTHY_FIELD_NUMBER: _ClassVar[int]
    RECEIVED_NS_FIELD_NUMBER: _ClassVar[int]
    SAMPLE_ID_FIELD_NUMBER: _ClassVar[int]
    CONFIG_VERSION_FIELD_NUMBER: _ClassVar[int]
    CLOCK_ID_FIELD_NUMBER: _ClassVar[int]
    FORCE_UNIT_FIELD_NUMBER: _ClassVar[int]
    TORQUE_UNIT_FIELD_NUMBER: _ClassVar[int]
    fx: float
    fy: float
    fz: float
    tx: float
    ty: float
    tz: float
    sequence: int
    status: int
    timestamp_s: float
    faults: _containers.RepeatedScalarFieldContainer[str]
    healthy: bool
    received_ns: int
    sample_id: int
    config_version: int
    clock_id: str
    force_unit: str
    torque_unit: str
    def __init__(
        self,
        fx: _Optional[float] = ...,
        fy: _Optional[float] = ...,
        fz: _Optional[float] = ...,
        tx: _Optional[float] = ...,
        ty: _Optional[float] = ...,
        tz: _Optional[float] = ...,
        sequence: _Optional[int] = ...,
        status: _Optional[int] = ...,
        timestamp_s: _Optional[float] = ...,
        faults: _Optional[_Iterable[str]] = ...,
        healthy: bool = ...,
        received_ns: _Optional[int] = ...,
        sample_id: _Optional[int] = ...,
        config_version: _Optional[int] = ...,
        clock_id: _Optional[str] = ...,
        force_unit: _Optional[str] = ...,
        torque_unit: _Optional[str] = ...,
    ) -> None: ...

class FtSlotRequest(_message.Message):
    __slots__ = ("slot",)
    SLOT_FIELD_NUMBER: _ClassVar[int]
    slot: int
    def __init__(self, slot: _Optional[int] = ...) -> None: ...

class FtIdentity(_message.Message):
    __slots__ = ("firmware", "host", "internal_rate_hz", "ip", "mac", "serial")
    HOST_FIELD_NUMBER: _ClassVar[int]
    IP_FIELD_NUMBER: _ClassVar[int]
    MAC_FIELD_NUMBER: _ClassVar[int]
    FIRMWARE_FIELD_NUMBER: _ClassVar[int]
    SERIAL_FIELD_NUMBER: _ClassVar[int]
    INTERNAL_RATE_HZ_FIELD_NUMBER: _ClassVar[int]
    host: str
    ip: str
    mac: str
    firmware: str
    serial: str
    internal_rate_hz: int
    def __init__(
        self,
        host: _Optional[str] = ...,
        ip: _Optional[str] = ...,
        mac: _Optional[str] = ...,
        firmware: _Optional[str] = ...,
        serial: _Optional[str] = ...,
        internal_rate_hz: _Optional[int] = ...,
    ) -> None: ...

class FtScaling(_message.Message):
    __slots__ = ("counts_per_force", "counts_per_torque", "force_unit", "torque_unit")
    COUNTS_PER_FORCE_FIELD_NUMBER: _ClassVar[int]
    COUNTS_PER_TORQUE_FIELD_NUMBER: _ClassVar[int]
    FORCE_UNIT_FIELD_NUMBER: _ClassVar[int]
    TORQUE_UNIT_FIELD_NUMBER: _ClassVar[int]
    counts_per_force: int
    counts_per_torque: int
    force_unit: str
    torque_unit: str
    def __init__(
        self,
        counts_per_force: _Optional[int] = ...,
        counts_per_torque: _Optional[int] = ...,
        force_unit: _Optional[str] = ...,
        torque_unit: _Optional[str] = ...,
    ) -> None: ...

class FtConfiguration(_message.Message):
    __slots__ = (
        "active_slot",
        "calibration",
        "calibration_serial",
        "force_unit_code",
        "name",
        "scaling",
        "sensing_ranges",
        "slot",
        "tool_angle_unit",
        "tool_angle_unit_code",
        "tool_distance_unit",
        "tool_distance_unit_code",
        "tool_transform",
        "torque_unit_code",
        "user_field_a",
        "user_field_b",
    )
    SLOT_FIELD_NUMBER: _ClassVar[int]
    ACTIVE_SLOT_FIELD_NUMBER: _ClassVar[int]
    NAME_FIELD_NUMBER: _ClassVar[int]
    CALIBRATION_FIELD_NUMBER: _ClassVar[int]
    CALIBRATION_SERIAL_FIELD_NUMBER: _ClassVar[int]
    TOOL_TRANSFORM_FIELD_NUMBER: _ClassVar[int]
    TOOL_DISTANCE_UNIT_FIELD_NUMBER: _ClassVar[int]
    TOOL_ANGLE_UNIT_FIELD_NUMBER: _ClassVar[int]
    FORCE_UNIT_CODE_FIELD_NUMBER: _ClassVar[int]
    TORQUE_UNIT_CODE_FIELD_NUMBER: _ClassVar[int]
    TOOL_DISTANCE_UNIT_CODE_FIELD_NUMBER: _ClassVar[int]
    TOOL_ANGLE_UNIT_CODE_FIELD_NUMBER: _ClassVar[int]
    SENSING_RANGES_FIELD_NUMBER: _ClassVar[int]
    USER_FIELD_A_FIELD_NUMBER: _ClassVar[int]
    USER_FIELD_B_FIELD_NUMBER: _ClassVar[int]
    SCALING_FIELD_NUMBER: _ClassVar[int]
    slot: int
    active_slot: int
    name: str
    calibration: int
    calibration_serial: str
    tool_transform: _containers.RepeatedScalarFieldContainer[float]
    tool_distance_unit: str
    tool_angle_unit: str
    force_unit_code: int
    torque_unit_code: int
    tool_distance_unit_code: int
    tool_angle_unit_code: int
    sensing_ranges: _containers.RepeatedScalarFieldContainer[float]
    user_field_a: str
    user_field_b: str
    scaling: FtScaling
    def __init__(
        self,
        slot: _Optional[int] = ...,
        active_slot: _Optional[int] = ...,
        name: _Optional[str] = ...,
        calibration: _Optional[int] = ...,
        calibration_serial: _Optional[str] = ...,
        tool_transform: _Optional[_Iterable[float]] = ...,
        tool_distance_unit: _Optional[str] = ...,
        tool_angle_unit: _Optional[str] = ...,
        force_unit_code: _Optional[int] = ...,
        torque_unit_code: _Optional[int] = ...,
        tool_distance_unit_code: _Optional[int] = ...,
        tool_angle_unit_code: _Optional[int] = ...,
        sensing_ranges: _Optional[_Iterable[float]] = ...,
        user_field_a: _Optional[str] = ...,
        user_field_b: _Optional[str] = ...,
        scaling: _Optional[_Union[FtScaling, _Mapping]] = ...,
    ) -> None: ...

class FtConfigurationList(_message.Message):
    __slots__ = ("configurations",)
    CONFIGURATIONS_FIELD_NUMBER: _ClassVar[int]
    configurations: _containers.RepeatedCompositeFieldContainer[FtConfiguration]
    def __init__(
        self, configurations: _Optional[_Iterable[_Union[FtConfiguration, _Mapping]]] = ...
    ) -> None: ...

class FtWriteConfigurationRequest(_message.Message):
    __slots__ = (
        "angle_unit",
        "calibration",
        "distance_unit",
        "force_unit",
        "name",
        "slot",
        "torque_unit",
        "user_field_a",
        "user_field_b",
    )
    SLOT_FIELD_NUMBER: _ClassVar[int]
    NAME_FIELD_NUMBER: _ClassVar[int]
    CALIBRATION_FIELD_NUMBER: _ClassVar[int]
    FORCE_UNIT_FIELD_NUMBER: _ClassVar[int]
    TORQUE_UNIT_FIELD_NUMBER: _ClassVar[int]
    DISTANCE_UNIT_FIELD_NUMBER: _ClassVar[int]
    ANGLE_UNIT_FIELD_NUMBER: _ClassVar[int]
    USER_FIELD_A_FIELD_NUMBER: _ClassVar[int]
    USER_FIELD_B_FIELD_NUMBER: _ClassVar[int]
    slot: int
    name: str
    calibration: int
    force_unit: str
    torque_unit: str
    distance_unit: str
    angle_unit: str
    user_field_a: str
    user_field_b: str
    def __init__(
        self,
        slot: _Optional[int] = ...,
        name: _Optional[str] = ...,
        calibration: _Optional[int] = ...,
        force_unit: _Optional[str] = ...,
        torque_unit: _Optional[str] = ...,
        distance_unit: _Optional[str] = ...,
        angle_unit: _Optional[str] = ...,
        user_field_a: _Optional[str] = ...,
        user_field_b: _Optional[str] = ...,
    ) -> None: ...

class FtToolTransformRequest(_message.Message):
    __slots__ = ("angle_unit", "distance_unit", "dx", "dy", "dz", "rx", "ry", "rz", "slot")
    SLOT_FIELD_NUMBER: _ClassVar[int]
    DX_FIELD_NUMBER: _ClassVar[int]
    DY_FIELD_NUMBER: _ClassVar[int]
    DZ_FIELD_NUMBER: _ClassVar[int]
    RX_FIELD_NUMBER: _ClassVar[int]
    RY_FIELD_NUMBER: _ClassVar[int]
    RZ_FIELD_NUMBER: _ClassVar[int]
    DISTANCE_UNIT_FIELD_NUMBER: _ClassVar[int]
    ANGLE_UNIT_FIELD_NUMBER: _ClassVar[int]
    slot: int
    dx: float
    dy: float
    dz: float
    rx: float
    ry: float
    rz: float
    distance_unit: str
    angle_unit: str
    def __init__(
        self,
        slot: _Optional[int] = ...,
        dx: _Optional[float] = ...,
        dy: _Optional[float] = ...,
        dz: _Optional[float] = ...,
        rx: _Optional[float] = ...,
        ry: _Optional[float] = ...,
        rz: _Optional[float] = ...,
        distance_unit: _Optional[str] = ...,
        angle_unit: _Optional[str] = ...,
    ) -> None: ...

class FtSettings(_message.Message):
    __slots__ = (
        "active_slot",
        "bias_vector",
        "filter_code",
        "filter_cutoff_hz",
        "filter_enabled",
        "monitor_conditions_enabled",
        "peak_logging",
    )
    ACTIVE_SLOT_FIELD_NUMBER: _ClassVar[int]
    FILTER_CODE_FIELD_NUMBER: _ClassVar[int]
    FILTER_CUTOFF_HZ_FIELD_NUMBER: _ClassVar[int]
    FILTER_ENABLED_FIELD_NUMBER: _ClassVar[int]
    PEAK_LOGGING_FIELD_NUMBER: _ClassVar[int]
    MONITOR_CONDITIONS_ENABLED_FIELD_NUMBER: _ClassVar[int]
    BIAS_VECTOR_FIELD_NUMBER: _ClassVar[int]
    active_slot: int
    filter_code: int
    filter_cutoff_hz: int
    filter_enabled: bool
    peak_logging: bool
    monitor_conditions_enabled: bool
    bias_vector: _containers.RepeatedScalarFieldContainer[int]
    def __init__(
        self,
        active_slot: _Optional[int] = ...,
        filter_code: _Optional[int] = ...,
        filter_cutoff_hz: _Optional[int] = ...,
        filter_enabled: bool = ...,
        peak_logging: bool = ...,
        monitor_conditions_enabled: bool = ...,
        bias_vector: _Optional[_Iterable[int]] = ...,
    ) -> None: ...

class FtCommunications(_message.Message):
    __slots__ = ("buffer_records", "internal_rate_hz", "rate_hz", "rdt_enabled")
    RDT_ENABLED_FIELD_NUMBER: _ClassVar[int]
    RATE_HZ_FIELD_NUMBER: _ClassVar[int]
    BUFFER_RECORDS_FIELD_NUMBER: _ClassVar[int]
    INTERNAL_RATE_HZ_FIELD_NUMBER: _ClassVar[int]
    rdt_enabled: bool
    rate_hz: int
    buffer_records: int
    internal_rate_hz: int
    def __init__(
        self,
        rdt_enabled: bool = ...,
        rate_hz: _Optional[int] = ...,
        buffer_records: _Optional[int] = ...,
        internal_rate_hz: _Optional[int] = ...,
    ) -> None: ...

class FtFilterRequest(_message.Message):
    __slots__ = ("code",)
    CODE_FIELD_NUMBER: _ClassVar[int]
    code: int
    def __init__(self, code: _Optional[int] = ...) -> None: ...

class FtEnableRequest(_message.Message):
    __slots__ = ("enabled",)
    ENABLED_FIELD_NUMBER: _ClassVar[int]
    enabled: bool
    def __init__(self, enabled: bool = ...) -> None: ...

class FtBiasRequest(_message.Message):
    __slots__ = ("gages",)
    GAGES_FIELD_NUMBER: _ClassVar[int]
    gages: _containers.RepeatedScalarFieldContainer[int]
    def __init__(self, gages: _Optional[_Iterable[int]] = ...) -> None: ...

class FtRateRequest(_message.Message):
    __slots__ = ("rate_hz",)
    RATE_HZ_FIELD_NUMBER: _ClassVar[int]
    rate_hz: int
    def __init__(self, rate_hz: _Optional[int] = ...) -> None: ...

class FtBufferRequest(_message.Message):
    __slots__ = ("records",)
    RECORDS_FIELD_NUMBER: _ClassVar[int]
    records: int
    def __init__(self, records: _Optional[int] = ...) -> None: ...

class FtMonitorCondition(_message.Message):
    __slots__ = ("axis", "axis_code", "comparison", "counts", "enabled", "index", "output_code")
    INDEX_FIELD_NUMBER: _ClassVar[int]
    ENABLED_FIELD_NUMBER: _ClassVar[int]
    AXIS_FIELD_NUMBER: _ClassVar[int]
    AXIS_CODE_FIELD_NUMBER: _ClassVar[int]
    COMPARISON_FIELD_NUMBER: _ClassVar[int]
    COUNTS_FIELD_NUMBER: _ClassVar[int]
    OUTPUT_CODE_FIELD_NUMBER: _ClassVar[int]
    index: int
    enabled: bool
    axis: str
    axis_code: int
    comparison: str
    counts: int
    output_code: int
    def __init__(
        self,
        index: _Optional[int] = ...,
        enabled: bool = ...,
        axis: _Optional[str] = ...,
        axis_code: _Optional[int] = ...,
        comparison: _Optional[str] = ...,
        counts: _Optional[int] = ...,
        output_code: _Optional[int] = ...,
    ) -> None: ...

class FtMonitorConditions(_message.Message):
    __slots__ = ("conditions", "enabled")
    CONDITIONS_FIELD_NUMBER: _ClassVar[int]
    ENABLED_FIELD_NUMBER: _ClassVar[int]
    conditions: _containers.RepeatedCompositeFieldContainer[FtMonitorCondition]
    enabled: bool
    def __init__(
        self,
        conditions: _Optional[_Iterable[_Union[FtMonitorCondition, _Mapping]]] = ...,
        enabled: bool = ...,
    ) -> None: ...

class FtMonitorConditionRequest(_message.Message):
    __slots__ = ("axis", "comparison", "counts", "enabled", "index", "output_code")
    INDEX_FIELD_NUMBER: _ClassVar[int]
    AXIS_FIELD_NUMBER: _ClassVar[int]
    COMPARISON_FIELD_NUMBER: _ClassVar[int]
    COUNTS_FIELD_NUMBER: _ClassVar[int]
    OUTPUT_CODE_FIELD_NUMBER: _ClassVar[int]
    ENABLED_FIELD_NUMBER: _ClassVar[int]
    index: int
    axis: str
    comparison: str
    counts: int
    output_code: int
    enabled: bool
    def __init__(
        self,
        index: _Optional[int] = ...,
        axis: _Optional[str] = ...,
        comparison: _Optional[str] = ...,
        counts: _Optional[int] = ...,
        output_code: _Optional[int] = ...,
        enabled: bool = ...,
    ) -> None: ...

class FtPeaks(_message.Message):
    __slots__ = ("enabled", "max_counts", "min_counts")
    MIN_COUNTS_FIELD_NUMBER: _ClassVar[int]
    MAX_COUNTS_FIELD_NUMBER: _ClassVar[int]
    ENABLED_FIELD_NUMBER: _ClassVar[int]
    min_counts: _containers.RepeatedScalarFieldContainer[int]
    max_counts: _containers.RepeatedScalarFieldContainer[int]
    enabled: bool
    def __init__(
        self,
        min_counts: _Optional[_Iterable[int]] = ...,
        max_counts: _Optional[_Iterable[int]] = ...,
        enabled: bool = ...,
    ) -> None: ...

class FtDeviceStatus(_message.Message):
    __slots__ = (
        "condition_latched",
        "condition_output",
        "conditions_breached",
        "faults",
        "healthy",
        "saturated",
        "status",
    )
    STATUS_FIELD_NUMBER: _ClassVar[int]
    CONDITIONS_BREACHED_FIELD_NUMBER: _ClassVar[int]
    CONDITION_OUTPUT_FIELD_NUMBER: _ClassVar[int]
    CONDITION_LATCHED_FIELD_NUMBER: _ClassVar[int]
    FAULTS_FIELD_NUMBER: _ClassVar[int]
    HEALTHY_FIELD_NUMBER: _ClassVar[int]
    SATURATED_FIELD_NUMBER: _ClassVar[int]
    status: int
    conditions_breached: int
    condition_output: int
    condition_latched: bool
    faults: _containers.RepeatedScalarFieldContainer[str]
    healthy: bool
    saturated: bool
    def __init__(
        self,
        status: _Optional[int] = ...,
        conditions_breached: _Optional[int] = ...,
        condition_output: _Optional[int] = ...,
        condition_latched: bool = ...,
        faults: _Optional[_Iterable[str]] = ...,
        healthy: bool = ...,
        saturated: bool = ...,
    ) -> None: ...

class CameraRequest(_message.Message):
    __slots__ = ("camera",)
    CAMERA_FIELD_NUMBER: _ClassVar[int]
    camera: CameraId
    def __init__(self, camera: _Optional[_Union[CameraId, str]] = ...) -> None: ...

class CameraStreamRequest(_message.Message):
    __slots__ = ("camera", "rate_hz")
    CAMERA_FIELD_NUMBER: _ClassVar[int]
    RATE_HZ_FIELD_NUMBER: _ClassVar[int]
    camera: CameraId
    rate_hz: float
    def __init__(
        self, camera: _Optional[_Union[CameraId, str]] = ..., rate_hz: _Optional[float] = ...
    ) -> None: ...

class ImageFrame(_message.Message):
    __slots__ = ("data", "encoding", "height", "timestamp_s", "width")
    DATA_FIELD_NUMBER: _ClassVar[int]
    ENCODING_FIELD_NUMBER: _ClassVar[int]
    WIDTH_FIELD_NUMBER: _ClassVar[int]
    HEIGHT_FIELD_NUMBER: _ClassVar[int]
    TIMESTAMP_S_FIELD_NUMBER: _ClassVar[int]
    data: bytes
    encoding: str
    width: int
    height: int
    timestamp_s: float
    def __init__(
        self,
        data: _Optional[bytes] = ...,
        encoding: _Optional[str] = ...,
        width: _Optional[int] = ...,
        height: _Optional[int] = ...,
        timestamp_s: _Optional[float] = ...,
    ) -> None: ...

class HistoryRequest(_message.Message):
    __slots__ = ("after_id", "clock_id", "end_ns", "limit", "session_id", "start_ns")
    SESSION_ID_FIELD_NUMBER: _ClassVar[int]
    AFTER_ID_FIELD_NUMBER: _ClassVar[int]
    START_NS_FIELD_NUMBER: _ClassVar[int]
    END_NS_FIELD_NUMBER: _ClassVar[int]
    LIMIT_FIELD_NUMBER: _ClassVar[int]
    CLOCK_ID_FIELD_NUMBER: _ClassVar[int]
    session_id: str
    after_id: int
    start_ns: int
    end_ns: int
    limit: int
    clock_id: str
    def __init__(
        self,
        session_id: _Optional[str] = ...,
        after_id: _Optional[int] = ...,
        start_ns: _Optional[int] = ...,
        end_ns: _Optional[int] = ...,
        limit: _Optional[int] = ...,
        clock_id: _Optional[str] = ...,
    ) -> None: ...

class FtWindowRequest(_message.Message):
    __slots__ = ("history", "include_samples", "max_age_s", "window_ns")
    HISTORY_FIELD_NUMBER: _ClassVar[int]
    MAX_AGE_S_FIELD_NUMBER: _ClassVar[int]
    INCLUDE_SAMPLES_FIELD_NUMBER: _ClassVar[int]
    WINDOW_NS_FIELD_NUMBER: _ClassVar[int]
    history: HistoryRequest
    max_age_s: float
    include_samples: bool
    window_ns: int
    def __init__(
        self,
        history: _Optional[_Union[HistoryRequest, _Mapping]] = ...,
        max_age_s: _Optional[float] = ...,
        include_samples: bool = ...,
        window_ns: _Optional[int] = ...,
    ) -> None: ...

class HistoryInfo(_message.Message):
    __slots__ = (
        "clock_id",
        "cursor_ahead",
        "end_ns",
        "incomplete_window",
        "newest_id",
        "next_id",
        "oldest_id",
        "overflow",
        "server_ns",
        "session_changed",
        "session_id",
        "start_ns",
        "truncated",
    )
    CLOCK_ID_FIELD_NUMBER: _ClassVar[int]
    SESSION_ID_FIELD_NUMBER: _ClassVar[int]
    SERVER_NS_FIELD_NUMBER: _ClassVar[int]
    START_NS_FIELD_NUMBER: _ClassVar[int]
    END_NS_FIELD_NUMBER: _ClassVar[int]
    OLDEST_ID_FIELD_NUMBER: _ClassVar[int]
    NEWEST_ID_FIELD_NUMBER: _ClassVar[int]
    NEXT_ID_FIELD_NUMBER: _ClassVar[int]
    OVERFLOW_FIELD_NUMBER: _ClassVar[int]
    SESSION_CHANGED_FIELD_NUMBER: _ClassVar[int]
    TRUNCATED_FIELD_NUMBER: _ClassVar[int]
    CURSOR_AHEAD_FIELD_NUMBER: _ClassVar[int]
    INCOMPLETE_WINDOW_FIELD_NUMBER: _ClassVar[int]
    clock_id: str
    session_id: str
    server_ns: int
    start_ns: int
    end_ns: int
    oldest_id: int
    newest_id: int
    next_id: int
    overflow: bool
    session_changed: bool
    truncated: bool
    cursor_ahead: bool
    incomplete_window: bool
    def __init__(
        self,
        clock_id: _Optional[str] = ...,
        session_id: _Optional[str] = ...,
        server_ns: _Optional[int] = ...,
        start_ns: _Optional[int] = ...,
        end_ns: _Optional[int] = ...,
        oldest_id: _Optional[int] = ...,
        newest_id: _Optional[int] = ...,
        next_id: _Optional[int] = ...,
        overflow: bool = ...,
        session_changed: bool = ...,
        truncated: bool = ...,
        cursor_ahead: bool = ...,
        incomplete_window: bool = ...,
    ) -> None: ...

class FtSample(_message.Message):
    __slots__ = (
        "config_version",
        "dropped_before",
        "force_unit",
        "ft_sequence",
        "rdt_sequence",
        "received_ns",
        "sample_id",
        "status",
        "torque_unit",
        "values",
        "wall_s",
    )
    SAMPLE_ID_FIELD_NUMBER: _ClassVar[int]
    RECEIVED_NS_FIELD_NUMBER: _ClassVar[int]
    RDT_SEQUENCE_FIELD_NUMBER: _ClassVar[int]
    FT_SEQUENCE_FIELD_NUMBER: _ClassVar[int]
    STATUS_FIELD_NUMBER: _ClassVar[int]
    VALUES_FIELD_NUMBER: _ClassVar[int]
    CONFIG_VERSION_FIELD_NUMBER: _ClassVar[int]
    FORCE_UNIT_FIELD_NUMBER: _ClassVar[int]
    TORQUE_UNIT_FIELD_NUMBER: _ClassVar[int]
    DROPPED_BEFORE_FIELD_NUMBER: _ClassVar[int]
    WALL_S_FIELD_NUMBER: _ClassVar[int]
    sample_id: int
    received_ns: int
    rdt_sequence: int
    ft_sequence: int
    status: int
    values: _containers.RepeatedScalarFieldContainer[float]
    config_version: int
    force_unit: str
    torque_unit: str
    dropped_before: int
    wall_s: float
    def __init__(
        self,
        sample_id: _Optional[int] = ...,
        received_ns: _Optional[int] = ...,
        rdt_sequence: _Optional[int] = ...,
        ft_sequence: _Optional[int] = ...,
        status: _Optional[int] = ...,
        values: _Optional[_Iterable[float]] = ...,
        config_version: _Optional[int] = ...,
        force_unit: _Optional[str] = ...,
        torque_unit: _Optional[str] = ...,
        dropped_before: _Optional[int] = ...,
        wall_s: _Optional[float] = ...,
    ) -> None: ...

class FtWindow(_message.Message):
    __slots__ = (
        "age_s",
        "config_version",
        "count",
        "dropped",
        "first_ns",
        "force_unit",
        "history",
        "last_ns",
        "mean",
        "reader_error",
        "reasons",
        "samples",
        "status",
        "torque_unit",
        "valid",
    )
    HISTORY_FIELD_NUMBER: _ClassVar[int]
    VALID_FIELD_NUMBER: _ClassVar[int]
    REASONS_FIELD_NUMBER: _ClassVar[int]
    COUNT_FIELD_NUMBER: _ClassVar[int]
    MEAN_FIELD_NUMBER: _ClassVar[int]
    AGE_S_FIELD_NUMBER: _ClassVar[int]
    FIRST_NS_FIELD_NUMBER: _ClassVar[int]
    LAST_NS_FIELD_NUMBER: _ClassVar[int]
    STATUS_FIELD_NUMBER: _ClassVar[int]
    DROPPED_FIELD_NUMBER: _ClassVar[int]
    CONFIG_VERSION_FIELD_NUMBER: _ClassVar[int]
    FORCE_UNIT_FIELD_NUMBER: _ClassVar[int]
    TORQUE_UNIT_FIELD_NUMBER: _ClassVar[int]
    SAMPLES_FIELD_NUMBER: _ClassVar[int]
    READER_ERROR_FIELD_NUMBER: _ClassVar[int]
    history: HistoryInfo
    valid: bool
    reasons: _containers.RepeatedScalarFieldContainer[str]
    count: int
    mean: _containers.RepeatedScalarFieldContainer[float]
    age_s: float
    first_ns: int
    last_ns: int
    status: int
    dropped: int
    config_version: int
    force_unit: str
    torque_unit: str
    samples: _containers.RepeatedCompositeFieldContainer[FtSample]
    reader_error: str
    def __init__(
        self,
        history: _Optional[_Union[HistoryInfo, _Mapping]] = ...,
        valid: bool = ...,
        reasons: _Optional[_Iterable[str]] = ...,
        count: _Optional[int] = ...,
        mean: _Optional[_Iterable[float]] = ...,
        age_s: _Optional[float] = ...,
        first_ns: _Optional[int] = ...,
        last_ns: _Optional[int] = ...,
        status: _Optional[int] = ...,
        dropped: _Optional[int] = ...,
        config_version: _Optional[int] = ...,
        force_unit: _Optional[str] = ...,
        torque_unit: _Optional[str] = ...,
        samples: _Optional[_Iterable[_Union[FtSample, _Mapping]]] = ...,
        reader_error: _Optional[str] = ...,
    ) -> None: ...

class RobotHistory(_message.Message):
    __slots__ = ("history", "samples")
    HISTORY_FIELD_NUMBER: _ClassVar[int]
    SAMPLES_FIELD_NUMBER: _ClassVar[int]
    history: HistoryInfo
    samples: _containers.RepeatedCompositeFieldContainer[RobotState]
    def __init__(
        self,
        history: _Optional[_Union[HistoryInfo, _Mapping]] = ...,
        samples: _Optional[_Iterable[_Union[RobotState, _Mapping]]] = ...,
    ) -> None: ...
