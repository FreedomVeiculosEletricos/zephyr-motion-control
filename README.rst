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
concept into a full motor control stack, covering everything from
application-level commands down to hardware abstraction. The full proposal
document is available as an attachment to `issue #102158
<https://github.com/zephyrproject-rtos/zephyr/issues/102158>`_ and serves as
the baseline reference for this module's design.

Architecture
************

Shared type definitions (state codes, drive modes, fault flags, the opaque
``motor_t`` handle) used across all layers live in
``include/zephyr/subsys/motor/motor_types.h``.

The module is organized into layers with well-defined interfaces:

.. code-block:: none

   +------------------------------------------------------------+
   |                  Application Code                          |
   +------------------------------------------------------------+
                              |
   +-----------------------------+  +---------------------------+
   | Interface A:                |  | Algorithm public APIs:    |
   | motor.h                     |  | motor/algorithms/<algo>/  |
   | Generic state + lifecycle:  |  | <algo>.h                  |
   |  enable / disable / estop / |  | Per-algorithm setpoints,  |
   |  drive mode / fault clear / |  | gains, limits, runtime    |
   |  get_status                 |  | state (e.g. dc-current:   |
   |                             |  | set_current, get_state,   |
   |                             |  | set_pi_gains, …)          |
   +--------------+--------------+  +-------------+-------------+
                  |                               |
   +------------------------------------------------------------+
   | Controller + Pipeline (internal):                           |
   | motor_controller.h, motor_pipeline.h, motor_block.h         |
   | Composable blocks scheduled per pipeline stage.             |
   | Hot ISR (PWM period, ZLI on STM32) runs the inner block.    |
   | Slow timer (PWM / slow-sample-div, default /10) wakes a     |
   | per-instance thread for non-ISR work (e.g. angle fetch).    |
   +-------------------+-----------------+----------------------+
                       |                 |
   +----------------------------+  +----------------------------+
   | Interface B:               |  | Interface C:               |
   | motor_sensor.h             |  | motor_actuator.h           |
   | Hot ADC + optional Zephyr  |  | H-bridge family power      |
   | Sensor angle channel       |  | stage (HS + LS, complemen- |
   |                            |  | tary, dead-time, en-gpios) |
   +----------------------------+  +----------------------------+
                              |
   +------------------------------------------------------------+
   | Subsystem: motor_subsys.h                                   |
   | DT auto-instantiation, instance discovery, motor groups,    |
   | fault propagation policy                                    |
   +------------------------------------------------------------+

Application API -- Interface A
==============================

Defined in ``include/zephyr/subsys/motor/motor.h``. Motor-type agnostic API
with an opaque ``motor_t`` handle.

- **State machine**: ``UNINIT`` → ``IDLE`` → ``RUN`` (and ``FAULT`` from any
  state). Planned: ``ALIGN``, ``STOP``, ``STO`` for safety-rated workflows.
- **Lifecycle**: ``motor_self_test()``, ``motor_enable()``, ``motor_disable()``,
  ``motor_estop()``, ``motor_clear_fault()``.
- **Status / drive mode**: ``motor_get_status()``,
  ``motor_set_drive_mode()`` (normal / coast; brake / regen reserved).
- **Callbacks**: ``motor_register_callbacks()`` for state changes and fault
  notifications.

Setpoints (current, speed, position, …) are **not** exposed on this header.
They live in each algorithm's public header so the application picks the
right command shape for the pipeline it instantiates.

Algorithm Public APIs
=====================

Each composable algorithm module exposes its own public header alongside its
DT binding. Today the module ships one algorithm:

- **DC current** -- ``include/zephyr/subsys/motor/algorithms/dc_current/motor_algo_dc_current.h``,
  binding ``zephyr,motor-algorithm-dc-current``.
  PI current regulator with anti-windup. Maps the signed PI output to
  bidirectional full-bridge PWM (``duty[0] = (u + 1)/2``,
  ``duty[1] = 1 - duty[0]``) so the actuator only needs to apply duty per leg.

  Public API: ``motor_algo_dc_current_set_current``,
  ``motor_algo_dc_current_get_state``, ``motor_algo_dc_current_set_pi_gains``,
  ``motor_algo_dc_current_get_pi_gains``, ``motor_algo_dc_current_set_limits``,
  ``motor_algo_dc_current_get_limits``.

Future algorithms (FOC, 6-step, V/f scalar, step/dir) plug into the same
pipeline infrastructure and ship their own public headers.

Controller and Pipeline (internal)
==================================

Defined in ``include/zephyr/subsys/motor/motor_controller.h``,
``motor_pipeline.h``, ``motor_block.h``. Surface mainly for algorithm/driver
authors; application code does not touch it directly.

- **Pipeline**: ordered list of ``motor_block`` callbacks. Each block declares
  its target stage (today ``MOTOR_STAGE_INNER_ISR``; more stages reserved for
  outer / supervisory work). Blocks expose ``entry`` (hot path),
  ``set_params`` (precompute derived constants at init or after public API
  parameter changes), and ``reset`` (clear integrators/state).
- **Hot ISR (inner stage)**: runs from the actuator PWM-period interrupt with
  an N-1 sampling cadence -- read the previous sample, advance the algorithm,
  push new duty cycles, start the next ADC conversion. ZLI on STM32 (see
  ``.cursor/rules/motor-hot-path.mdc`` for what is forbidden in this context).
- **Slow timer**: each ``motor_actuator`` owns a Zephyr ``counter`` alarm
  programmed to fire every ``slow-sample-div`` PWM periods (default 10). The
  alarm signals the controller's slow thread, which performs work not allowed
  in the ZLI ISR (e.g. fetching a Zephyr Sensor for angle). The pipeline
  picks up the result on subsequent hot ticks.
- **Sense bundle**: hot ISR fills a ``motor_sense_bundle`` (current array,
  angle) and passes it as the block input. Algorithm authors only consume
  the bundle, never raw devices.

Sensor Backend -- Interface B
=============================

Defined in ``include/zephyr/drivers/motor/motor_sensor.h``. Vtable-based, with
two logical channels today: ``CHAN_CURRENT`` (composite ADC array) and
``CHAN_ANGLE`` (optional Zephyr Sensor feedback). Extension slots are reserved
for future composites.

Built-in drivers:

- ``zephyr,motor-sensor-stm32`` -- STM32G4 LL, injected ADC sequence,
  software-triggered, JEOC ISR latches all ranks. ZLI-direct ISR.
- ``zephyr,motor-sensor-esp32`` -- ESP32 family SAR ADC + digital DMA (GDMA
  on S3/C3/H2, I2S- or SPI-linked DMA on ESP32 / ESP32-S2). EOF latches the
  last block sample.

ADC scaling (``vref / counts × amps-per-volt``) is **precomputed at init**
from three DT properties on the sensor node and applied as a single
multiplication per channel in the hot path:

- ``adc-vref-mv`` -- full-scale reference voltage in mV (default 3300; on
  ESP32 it follows the chosen ``adc-channel-attenuation-db``: ~1100 mV at
  0 dB, ~1500 mV at 2.5 dB, ~2200 mV at 6 dB, ~3300 mV at 12 dB).
- ``adc-resolution-bits`` -- 6/8/10/12 on STM32 (driver honors via LL),
  fixed 12 on ESP32.
- ``amps-per-volt-milli`` -- shunt + gain conversion.

Calibration zeroes per-channel offsets so the channel can carry signed
currents (e.g. unipolar shunt + bias network).

Power Stage Backend -- Interface C
==================================

Defined in ``include/zephyr/drivers/motor/motor_actuator.h``. Duty array
per leg in ``[0.0, 1.0]``; the algorithm is responsible for mapping its
signed output to that range. No vector / sign-magnitude API -- each leg
gets its own duty value.

Built-in drivers:

- ``zephyr,motor-stage-hbridge-stm32`` -- STM32G4 LL, 1..3 complementary
  half-bridges (CHx + CHxN) on an advanced timer (TIM1/TIM8). Build-time
  asserts reject non-advanced timers. Hardware dead-time via BDTR DTG.
- ``zephyr,motor-stage-hbridge-espressif-mcpwm`` -- one MCPWM operator,
  single half-bridge (gen_high + gen_low complementary). Multi-leg
  full-bridge is planned (needs binding extension to multiple operators).

Shared properties (``zephyr,motor-stage-hbridge-common``):

- ``pwm-channels`` -- one timer channel per leg.
- ``single-ended`` -- optional flag. When false (default), driver applies
  the ``pinctrl-0`` ("complementary", 2·N pins). When true, driver applies
  ``pinctrl-1`` ("single-ended", N HS pins only) -- the LS pin stays at
  its hardware reset state, free for other use.
- ``en-gpios`` -- optional gate driver ``EN`` pin, driven on enable and
  released on disable.
- ``nfault-gpios``, ``nsleep-gpios`` -- standard gate driver lines.
- ``slow-timer``, ``slow-sample-div`` -- Zephyr counter wired for the
  slow ISR (see Controller section).

Drive modes: ``MOTOR_DRIVE_NORMAL`` and ``MOTOR_DRIVE_COAST`` today;
``BRAKE`` / ``REGEN`` reserved.

Subsystem Layer
===============

Defined in ``include/zephyr/subsys/motor/motor_subsys.h``. Top of the stack.

- **DT auto-instantiation**: every ``zephyr,motor-controller`` node in the
  devicetree is materialised by ``motor_subsys.c`` (no ``MOTOR_SUBSYS_DEFINE_DT``
  in application code). The subsystem inspects the controller's
  ``algorithm`` phandle compatible and instantiates the matching pipeline.
- **Auto-init**: ``SYS_INIT()`` calls ``motor_subsys_init()`` after
  POST_KERNEL device init when ``CONFIG_MOTOR_SUBSYS_AUTO_INIT=y``.
- **Discovery**: ``motor_subsys_get_by_label()`` (DT node label),
  ``motor_subsys_get_by_index()``, ``motor_subsys_count()``,
  ``motor_subsys_label_get()``.
- **Motor groups**: named sets of ``motor_t`` commanded as a unit;
  ``motor_group_enable()`` arms each member and resolves when all reach
  ``RUN`` (or any reaches ``FAULT``).
- **Fault propagation**: ``MOTOR_GROUP_FAULT_ESTOP_ALL``,
  ``MOTOR_GROUP_FAULT_DISABLE_ALL``, ``MOTOR_GROUP_FAULT_ISOLATE``.

Supported Motor Types
*********************

.. list-table::
   :header-rows: 1
   :widths: 25 20 30 15 10

   * - Motor Type
     - Algorithm
     - Sensor
     - Power Stage
     - Status
   * - DC Brushed PM
     - ``dc-current``
     - 1- or 2-shunt + optional encoder
     - Full-bridge (TIM1/TIM8)
     - Implemented
   * - BLDC 6-step (Hall)
     - ``6step`` (planned)
     - Hall × 3 + shunt
     - 3-phase inverter
     - Planned
   * - BLDC sensorless
     - ``6step`` + BEMF observer
     - BEMF observer + 2-shunt
     - 3-phase inverter
     - Planned
   * - PMSM -- encoder
     - ``foc`` (planned)
     - Encoder/SPI ABS/resolver + 3-shunt
     - 3-phase inverter
     - Planned
   * - PMSM -- sensorless
     - ``foc`` + observer
     - BEMF + HFI observer + 3-shunt
     - 3-phase inverter
     - Planned
   * - Stepper
     - ``stepdir`` (planned)
     - Step counter / encoder
     - Dual H-bridge
     - Planned
   * - AC Induction -- V/f
     - ``vf-scalar`` (planned)
     - Tach/encoder (optional) + 3-shunt
     - 3-phase inverter
     - Planned
   * - AC Induction -- FOC
     - ``foc`` (rotor flux, planned)
     - Encoder or MRAS observer + 3-shunt
     - 3-phase inverter
     - Planned

Planned types use the same composable pipeline + per-algorithm public API
pattern that ``dc-current`` follows today. Reference devicetree shapes are
collected in ``samples/motor-control/examples.overlay`` for review.

Devicetree Bindings
*******************

In-tree bindings live under ``dts/bindings/motor-control/``:

- ``zephyr,motor-controller`` -- composition node: ``sensor``, ``actuator``,
  and ``algorithm`` phandles. Materialised by the subsystem (no application
  glue).
- ``zephyr,motor-sensor-base`` -- abstract base: ``adc``, ``adc-channels``,
  ``sync-actuator``, ``adc-vref-mv``, ``adc-resolution-bits``,
  ``amps-per-volt-milli``, optional ``feedback-sensor``.
- ``zephyr,motor-sensor-stm32`` -- STM32 LL injected ADC.
- ``zephyr,motor-sensor-esp32`` -- ESP32 SAR ADC digital + DMA.
- ``zephyr,motor-stage-hbridge-common`` -- abstract base: ``pwm-channels``,
  ``pwm-frequency``, ``slow-timer``, ``slow-sample-div``, optional
  ``single-ended``, ``en-gpios``, ``nfault-gpios``, ``nsleep-gpios``,
  ``sto-gpios``, ``deadtime-ns``.
- ``zephyr,motor-stage-hbridge-stm32`` -- STM32 advanced timer
  (``st,pwm-timer``, ``trgo-source``).
- ``zephyr,motor-stage-hbridge-espressif-mcpwm`` -- ESP32 MCPWM operator.
- ``zephyr,motor-algorithm-dc-current`` -- PI tuning + limits
  (``dc-current-kp-milli``, ``dc-current-ki-milli``, ``dc-current-out-min-milli``,
  ``dc-current-out-max-milli``, ``i-max-ma``).

Samples
*******

- ``samples/motor_shell`` -- interactive shell on the console UART for bench
  bring-up: list instances, select, self-test, enable, set current, read
  status, retune PI / limits at runtime. STM32 LL on NUCLEO-G474RE (TIM1
  complementary, ADC1 injected). See ``samples/motor_shell/README.rst``.
- ``samples/motor_trapezoid`` -- shell-started trapezoidal current trajectory
  (ramp / hold / ramp-down / dwell, then mirrored negative half) using the
  algorithm's ``i-max-ma`` as peak. Same hardware as ``motor_shell``. See
  ``samples/motor_trapezoid/README.rst``.
- ``samples/motor-control/examples.overlay`` -- reference DTS overlays for
  future motor configurations (PMSM/FOC, BLDC/6-step, stepper, …). No
  application code yet -- documents the binding shapes.

Hot Path Performance Rule
*************************

Real-time correctness depends on keeping the inner ISR cheap. The rule in
``.cursor/rules/motor-hot-path.mdc`` is enforced across this module:

- No division, transcendentals, or kernel API in the hot path.
- Quantities derived only from DT or ``set_params`` input are **precomputed**
  in the algorithm/driver data struct at init and re-read in the ISR.
  Examples:

  - ``motor_algo_dc_current_data::ki_dt`` (= ``pi.ki × control_loop_dt_s``)
    cached so the hot path only does one multiplication for the integral
    term.
  - ``motor_actuator_stm32_data::max_duty`` (= ``ARR + 1``) cached at init
    so ``set_duty`` skips an MMIO read of ``TIMx_ARR`` every ISR.
  - ``motor_sensor_*_data::amps_per_count`` (= ``vref/counts × amps_per_volt``)
    cached so the ADC ISR/EOF path produces amps with a single multiply per
    channel, no division, no nested scaling.

Algorithm and driver authors are expected to follow the same pattern.

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

   export ZEPHYR_BASE="$PWD/deps/zephyr"
   export ZEPHYR_EXTRA_MODULES="$PWD"

   # Build a sample for NUCLEO-G474RE
   west build -b nucleo_g474re samples/motor_shell
   west build -b nucleo_g474re samples/motor_trapezoid

   # Run the test suite via Twister on native_sim (algorithm + pipeline +
   # subsystem unit tests run blackbox against fake sensor/actuator devices).
   ./tests/run_twister.sh -T "$PWD/tests" -p native_sim/native/64

Project Status
**************

This module is a **proof of concept**. Current state:

- Architecture, layered headers, and DT bindings -- **complete** (subject to
  API tweaks while validating on hardware).
- ``dc-current`` algorithm with PI + anti-windup, complementary full-bridge
  output -- **implemented**.
- In-tree sensor drivers: STM32 LL injected ADC, ESP32 SAR + DMA --
  **implemented**.
- In-tree power stage drivers: STM32 advanced-timer half/full-bridge,
  ESP32 MCPWM half-bridge -- **implemented**. ESP32 multi-operator
  full-bridge is **planned**.
- Subsystem with DT auto-instantiation, motor groups, fault propagation
  policy -- **implemented**.
- Samples (``motor_shell``, ``motor_trapezoid``) build for NUCLEO-G474RE --
  **green**.
- Test suite (algorithm, pipeline, subsystem, API) on ``native_sim`` --
  **green**.
- STO arm/release, ``ALIGN``/``STOP`` states, multi-rate outer loops
  (speed / position / supervision), and additional motor type algorithms --
  **planned**.

License
*******

This project is licensed under the Apache License, Version 2.0.
See the SPDX headers in each file for details.
