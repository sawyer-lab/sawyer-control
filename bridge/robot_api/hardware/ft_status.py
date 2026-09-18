"""Decoding for the Net F/T 32-bit system status word (manual Table 17.1).

Every RDT record carries this word. Bit 31 is set whenever any error bit is
set; bit 16 is a latched Monitor Condition and is not an error.
"""

_BITS = {
    31: "error",
    30: "cpu_or_ram_error",
    29: "digital_board_error",
    28: "analog_board_error",
    27: "serial_link_error",
    26: "program_memory_verification_error",
    25: "halted_configuration_errors",
    24: "settings_validation_error",
    23: "configuration_incompatible_with_calibration",
    22: "network_communication_failure",
    21: "can_communication_error",
    20: "rdt_communication_error",
    19: "ethernet_ip_protocol_failure",
    18: "devicenet_protocol_failure",
    17: "transducer_saturation",
    16: "monitor_condition_latched",
    14: "watchdog_timeout",
    13: "stack_check_error",
    12: "eeprom_i2c_failure",
    11: "flash_spi_failure",
    10: "analog_watchdog_timeout",
    9: "excessive_gage_excitation",
    8: "insufficient_gage_excitation",
    7: "analog_ground_out_of_range",
    6: "supply_too_high",
    5: "supply_too_low",
    4: "serial_link_data_unavailable",
    3: "reference_voltage_error",
    2: "internal_temperature_error",
}

# Codes the manual calls healthy: no faults, and no-faults-with-condition-latched.
_HEALTHY = (0x00000000, 0x80010000)

# Bit 16 is informational; bit 31 is a roll-up of the others.
_NOT_A_FAULT = (1 << 16) | (1 << 31)


def decode(status: int) -> list:
    """Names of every set status bit, most significant first."""
    return [name for bit, name in sorted(_BITS.items(), reverse=True) if status & (1 << bit)]


def faults(status: int) -> list:
    """Names of set bits that represent real errors (excludes the roll-up and latch bits)."""
    return [name for bit, name in sorted(_BITS.items(), reverse=True)
            if status & (1 << bit) and not (1 << bit) & _NOT_A_FAULT]


def is_healthy(status: int) -> bool:
    return status in _HEALTHY


def latched(status: int) -> bool:
    """True if a Monitor Condition has tripped since the last latch reset."""
    return bool(status & (1 << 16))


def saturated(status: int) -> bool:
    return bool(status & (1 << 17))


def describe(status: int) -> str:
    if is_healthy(status):
        return "ok (condition latched)" if latched(status) else "ok"
    return ", ".join(decode(status)) or "unknown"
