.. Copyright (c) 2026 Freedom Veiculos Eletricos
.. SPDX-License-Identifier: Apache-2.0

.. _zephyr_motor_control_module:

Zephyr Motor Control Module
###########################

Overview
********

This is an external `Zephyr RTOS <https://www.zephyrproject.org/>`_ module
implementing a motor control subsystem. Developed by Freedom Veiculos Eletricos
as a proof-of-concept implementation of the motor control architecture proposed
in `zephyrproject-rtos/zephyr#102158
<https://github.com/zephyrproject-rtos/zephyr/issues/102158>`_.

The goal is to validate the architecture against real electric vehicle use cases
at Freedom Veiculos Eletricos before contributing the design upstream to Zephyr.

Background
**********

Zephyr currently lacks native driver support for hard real-time motor control
applications. Existing PWM and ADC APIs have critical limitations: sequential
PWM updates cause phase shifts, software-triggered ADC introduces sampling
jitter, and software-based fault paths exceed the Short Circuit Withstand Time
(SCWT) of power switches.

Issue `#102158 <https://github.com/zephyrproject-rtos/zephyr/issues/102158>`_
proposed two solutions: modular drivers (extending existing PWM/ADC APIs) or a
unified driver encapsulating PWM + ADC + fault + hardware linking. The Zephyr
Architecture Working Group selected the unified approach (Solution 2) for its
optimal real-time performance and simpler coordination model.

This module implements a comprehensive proposal by Gerson Fernando Budke
(`@nandojve <https://github.com/nandojve>`_) that extends the unified driver
concept into a full 5-layer motor control stack, covering everything from
application-level commands down to hardware abstraction. The full proposal
document is available as an attachment to `issue #102158
<https://github.com/zephyrproject-rtos/zephyr/issues/102158>`_ and serves as
the baseline reference for this module's design.

Architecture
************

Shared type definitions (SI units, enums, state codes) used across all layers
are defined in ``include/zephyr/motor_types.h``.

The module is organized into five layers with three well-defined interfaces:

.. code-block:: none

   +----------------------------------------------+
   |         Application Code                     |
   +----------------------------------------------+
                      |
   +----------------------------------------------+
   |  Interface A: motor.h                        |
   |  Motor-type agnostic, opaque handle,         |
   |  state machine, STO                          |
   +----------------------------------------------+
                      |
   +----------------------------------------------+
   |  Controller: motor_controller.h              |
   |  Multi-rate scheduling, algorithm dispatch   |
   +---------------------+------------------------+
             |                        |
   +--------------------+  +------------------------+
   | Interface B:       |  | Interface C:           |
   | motor_sensor.h     |  | motor_actuator.h       |
   | Sensor backends    |  | Power stage backends   |
   +--------------------+  +------------------------+
                      |
   +----------------------------------------------+
   |  Subsystem: motor_subsys.h                   |
   |  Multi-instance mgmt, motor groups           |
   +----------------------------------------------+

Application API -- Interface A
==============================

Defined in ``include/zephyr/motor.h``. Provides a motor-type agnostic API
with an opaque ``motor_t`` handle.

- **State machine**: UNINIT, IDLE, ALIGN, RUN, STOP, STO, FAULT with
  well-defined transitions.
- **Commands**: ``motor_set_torque()``, ``motor_set_speed()``,
  ``motor_set_position()``, ``motor_set_drive_mode()``.
- **Lifecycle**: ``motor_init()``, ``motor_enable()``, ``motor_disable()``,
  ``motor_estop()``.
- **Safe Torque Off (STO)**: Dual-channel monitoring per IEC 61508 / ISO 13849.
- **Parameter persistence**: Save/load via Zephyr settings subsystem.
- **Callbacks**: State change and fault notifications.

Controller Layer
================

Defined in ``include/zephyr/motor_controller.h``. Implements multi-rate
control loop scheduling with four rates:

- **Rate 0** (ISR, 20--100 kHz): Current loop -- Clarke/Park transforms,
  PI regulators, SVPWM or 6-step commutation.
- **Rate 1** (Thread, ~1 kHz): Speed loop -- PI control with anti-windup
  and acceleration profiling (trapezoidal, S-curve).
- **Rate 2** (Thread, ~100--500 Hz): Position loop -- P/PD control with
  trajectory generation.
- **Rate 3** (Thread, ~10--100 Hz): Supervision -- fault policy, thermal
  derating, watchdog, state transitions.

Algorithm vtable (``motor_algo_ops``) supports: FOC, 6-step, open-loop,
V/f scalar, and step/dir.

Sensor Backend -- Interface B
=============================

Defined in ``include/zephyr/motor_sensor.h``. Provides a vtable-based
abstraction for feedback sensors with ISR-safe data paths.

- **Position/speed backends**: Quadrature encoder, Hall sensors, SPI absolute
  encoder, resolver (SIN/COS), BEMF observer, HFI observer, step counter.
- **Current sense backends**: 3-shunt, 2-shunt, single DC-link shunt,
  StallGuard, none.
- **Data layout**: HOT region (~24 bytes, one cache line) for ISR-copied
  theta/omega/phase currents; COLD region for supervision-only data
  (bus voltage, temperatures, faults).

Power Stage Backend -- Interface C
==================================

Defined in ``include/zephyr/motor_actuator.h``. Abstracts the power
electronics hardware.

- **Topologies**: Half-bridge, full H-bridge, dual H-bridge (stepper),
  3-phase inverter.
- **Actuation**: Voltage vector (Valpha/Vbeta for SVPWM) or direct per-phase
  duty cycles.
- **Drive modes**: Normal, coast, brake, regenerative braking.
- **Safety**: Hardware fault monitoring (nFAULT GPIO), STO arm/release with
  dual-channel self-test.
- **Control callback**: Fired from PWM period ISR for current-loop heartbeat.

Subsystem Layer
===============

Defined in ``include/zephyr/motor_subsys.h``. Manages multiple motor
instances and coordinated operation.

- **Auto-init**: All DT-declared instances initialized via ``SYS_INIT()``.
- **Discovery**: By DT label (``motor_subsys_get_by_label()``) or index.
- **Motor groups**: Named sets of motors commanded atomically with zero
  inter-motor skew.
- **Group enable barrier**: Parallel alignment with all-RUN gate.
- **Fault propagation**: Configurable per group -- estop-all, disable-all,
  or isolate.

Supported Motor Types
*********************

.. list-table::
   :header-rows: 1
   :widths: 25 20 30 20

   * - Motor Type
     - Algorithm
     - Sensor
     - Power Stage
   * - DC Brushed PM
     - ``openloop`` or ``foc``
     - Encoder (optional) + 1-shunt
     - H-bridge
   * - BLDC 6-step (Hall)
     - ``6step``
     - Hall x3 + 1/2-shunt
     - 3-phase inverter
   * - BLDC sensorless
     - ``6step`` + sensorless
     - BEMF observer + 2-shunt
     - 3-phase inverter
   * - PMSM -- encoder
     - ``foc``
     - Encoder/SPI ABS/resolver + 3-shunt
     - 3-phase inverter
   * - PMSM -- sensorless
     - ``foc`` (sensorless)
     - BEMF + HFI observer + 3-shunt
     - 3-phase inverter
   * - Stepper open-loop
     - ``stepdir``
     - Step counter + StallGuard
     - Dual H-bridge
   * - Stepper closed-loop
     - ``stepdir`` + position PID
     - Encoder + 1-shunt
     - Dual H-bridge
   * - AC Induction -- V/f
     - ``vf-scalar``
     - Tach/encoder (optional) + 3-shunt
     - 3-phase inverter
   * - AC Induction -- FOC
     - ``foc`` (rotor flux)
     - Encoder or MRAS observer + 3-shunt
     - 3-phase inverter

Complete devicetree overlay examples are provided in
``samples/motor-control/examples.overlay``.

Devicetree Bindings
********************

The following bindings are defined in ``dts/bindings/motor-control/``:

- ``zephyr,motor-controller`` -- Top-level composition node linking sensor,
  actuator, algorithm, motor parameters, and loop rates.
- ``zephyr,motor-sensor-encoder`` -- Quadrature encoder with phase current ADC.
- ``zephyr,motor-sensor-hall`` -- Hall effect sensors (3-wire) with current ADC.
- ``zephyr,motor-sensor-resolver`` -- SIN/COS resolver with current ADC.
- ``zephyr,motor-sensor-sensorless`` -- BEMF/HFI observer with optional ADC.
- ``zephyr,motor-stage-3ph-inverter`` -- 3-phase 6-switch inverter with
  complementary PWM, dead-time, and fault/STO GPIOs.
- ``zephyr,motor-stage-hbridge`` -- H-bridge (single or dual for stepper).

Getting Started
***************

Prerequisites
=============

- Zephyr RTOS v4.4.0-rc3 or later.
- `Nix <https://nixos.org/>`_ package manager (recommended, for reproducible
  builds) **or** a manually configured Zephyr SDK toolchain.

Setting Up the Workspace
========================

Using Nix (recommended):

.. code-block:: bash

   git clone https://github.com/FreedomVeiculosEletricos/zephyr-motion-control.git
   cd zephyr-motion-control
   nix develop   # or: direnv allow
   west init -l .
   west update

Manual setup:

.. code-block:: bash

   git clone https://github.com/FreedomVeiculosEletricos/zephyr-motion-control.git
   cd zephyr-motion-control
   west init -l .
   west update

Using as an External Module
===========================

To reference this module from another west workspace, add it to your
``west.yml`` manifest:

.. code-block:: yaml

   manifest:
     projects:
       - name: zephyr-motion-control
         url: https://github.com/FreedomVeiculosEletricos/zephyr-motion-control.git
         revision: main
         import: true

The module's ``zephyr/module.yml`` declares the Kconfig entry point, CMake
root, DTS root, and board root automatically.

Building and Testing
====================

.. code-block:: bash

   # Build for a specific board with an overlay
   west build -b <board> samples/motor-control -- \
     -DDTC_OVERLAY_FILE=samples/motor-control/examples.overlay

   # Run the test suite via Twister
   west twister -G --testsuite-root . --tag motor_control

Project Status
**************

This module is a **proof of concept**. Current state:

- API headers defining the full 5-layer architecture are **complete**.
- Devicetree bindings for all sensor and power stage types are **defined**.
- Sample DTS overlays for 4 motor configurations are **provided**.
- CI/CD pipeline (GitHub Actions + Zephyr Twister) is **operational**.
- Implementation code (``subsys/*.c``, drivers) is **not yet written**.
- The API **may change** as the design is validated on real hardware.

License
*******

This project is licensed under the Apache License, Version 2.0.
See the SPDX headers in each file for details.
