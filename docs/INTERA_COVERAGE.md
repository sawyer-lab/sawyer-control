# Intera review

The bridge is based on the official Intera SDK API surface. Version 1 exposes
the hardware needed for the control platform: robot lifecycle and state,
endpoint pose, joint commands and generic command sequences, ClickSmart gripper
open/close/state, head and hand image capture, and raw ATI force/torque reads.

The authoritative references used for the review are the [Intera module
index](https://rethinkrobotics.github.io/intera_sdk_docs/5.1.0/intera_interface/html/intera_interface-module.html),
the [gripper API](https://rethinkrobotics.github.io/intera_sdk_docs/5.1.0/intera_interface/html/intera_interface.gripper.Gripper-class.html),
and the [robot-enable API](https://rethinkrobotics.github.io/intera_sdk_docs/5.0.4/intera_interface/html/intera_interface.robot_enable.RobotEnable-class.html).

The public surface is defined exclusively in
[`control.proto`](../proto/sawyer_control/v1/control.proto).
