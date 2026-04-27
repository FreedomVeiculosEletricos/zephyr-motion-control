..
   Copyright (c) 2026 Zephyr Motor Control Contributors
   SPDX-License-Identifier: Apache-2.0

DC brushed current loop (scalar PI)
====================================

Overview
--------

This sample wires the out-of-tree motor control module for a **DC brushed** motor
with a **power stage** (Interface C) and **shunt current sense** (Interface B), using
the **dc-current** algorithm (scalar PI on armature current).

Integration uses the **STM32 LL synchronous chain** — timer TRGO triggers injected ADC
conversions; the **ADC JEOC ISR** latches the sample and invokes the registered
``motor_control_cb_t`` (no Zephyr PWM/ADC driver on the critical path). Sensor and
stage drivers are SoC-specific (e.g. ``zephyr,motor-sensor-encoder-stm32``,
``zephyr,motor-stage-hbridge-stm32``).

The application uses the **motor subsystem** API: devicetree declares a
``zephyr,motor-controller`` node (label ``motor_brushed``) with phandles to the
sensor, actuator, and a ``zephyr,motor-algorithm-dc-current`` child for PI gains
and limits; the sample uses ``MOTOR_SUBSYS_DEFINE_DT()`` and
``motor_subsys_get_by_label("motor_brushed")``. The app commands **current (A)**
with ``motor_set_current()``.

For a **dedicated full-bridge** sample (two half-bridge legs, same algorithm), see
:file:`../dc_current_full_bridge/README.rst`.

STM32 LL: half-bridge count (same sample, different overlay)
--------------------------------------------------------------

On STM32G4, ``zephyr,motor-stage-hbridge-stm32`` drives **one timer** with one to
three entries in ``pwm-channels`` (each entry is one half-bridge leg). The same
application source is used; pick the overlay for your hardware:

* **One half-bridge (unidirectional)** — single timer channel; torque magnitude
  only (no bridge reversal). Overlay:
  :file:`boards/nucleo_g474re_stm32_ll_1hb.overlay` (``pwm-channels = <1>;``).
* **Full bridge / bidirectional** — two legs on the same timer (sign-magnitude).
  Overlay: :file:`boards/nucleo_g474re.overlay` (``pwm-channels = <1 2>;``), merged
  automatically when building for ``nucleo_g474re``, or use the
  :file:`../dc_current_full_bridge` sample which uses the same overlay.

Optional: set ``stm32,complementary-pwm`` on the stage node for CHx/CHxN with
dead-time on TIM1/TIM8 (not used on the default TIM3 NUCLEO overlays).

Hardware (NUCLEO-G474RE)
------------------------

* **PWM / timer**: Default STM32 LL overlays use ``TIM3``. ``&pwm3`` is **disabled**
  so only the motor driver owns ``TIM3``.
* **Current**: ``ADC1`` channel 1 (adjust in devicetree to match your shunt input and
  op-amp gain). Set effective ``amps_per_volt`` in the sensor driver for your front-end.
* **IRQ priority**: On the STM32 LL path, set ``zephyr,adc-irq-priority`` on the sensor
  node so the **ADC ISR** is higher priority than SysTick and lower than non-preemptible
  code, per your policy.

Synchronous chain (STM32 LL)
----------------------------

.. code-block:: text

   TIMx TRGO (update) → ADC injected start → JEOC → ISR: latch → motor_control_cb_t
   → motor_sensor_update/get → current PI → motor_actuator_set_vector (LL compare)

Sequence (Mermaid — render in editors / docs that support it):

.. code-block:: mermaid

   sequenceDiagram
     participant TIM as Timer_PWM_CC
     participant ADC as ADC_HW
     participant ISR as CurrentISR_HAL
     participant CB as motor_control_cb_t
     participant ALG as algo_inner_step
     TIM->>ADC: hardware_trigger_sample_start
     ADC->>ISR: ADC_conversion_complete_IRQ
     ISR->>CB: invoke_registered_callback
     CB->>ALG: sensor_update_get_PI_set_vector

Build (STM32 LL — full bridge, two half-bridge legs)
----------------------------------------------------

.. code-block:: shell

   west build -b nucleo_g474re path/to/zephyr-motion-control/samples/dc_current -- \
     -DZEPHYR_EXTRA_MODULES=path/to/zephyr-motion-control \
     -DCONF_FILE=prj_stm32_ll.conf

Build (STM32 LL — single half-bridge, unidirectional)
-----------------------------------------------------

.. code-block:: shell

   west build -b nucleo_g474re path/to/zephyr-motion-control/samples/dc_current -- \
     -DZEPHYR_EXTRA_MODULES=path/to/zephyr-motion-control \
     -DCONF_FILE=prj_stm32_ll.conf \
     -DDTC_OVERLAY_FILE=boards/nucleo_g474re_stm32_ll_1hb.overlay

Use a **Nix devshell** (``nix develop``) if the host Python is too old; the flake bundles
Python ≥3.12 and Zephyr SDK toolchains. Prefer a clean shell, e.g.
``nix develop -c bash --noprofile --norc -c '...'``, so host ``~/.local`` tools do not
override Nix-provided CMake/west.

Adjust ``ZEPHYR_EXTRA_MODULES`` so the Zephyr build picks up this repository as a module.

Porting to STM32H7
------------------

Use the same module layout: add a board overlay pointing ``st,pwm-timer`` / ``st,adc``
and channels to the timers and ADC banks available on that SoC. **Revalidate** the
clock tree, ADC multi-bank behaviour, and **cache** coherency if DMA is added later.
Align ``adc_ext_trigger_inj`` / ``trgo-source`` with the reference manual for that
device; re-tune ``control_loop_dt_s`` and PI gains after migration.

Safety
------

Use a current-limited supply, freewheel / brake as required by your bridge, and verify
nFAULT / interlocks before applying current. This sample is for **laboratory** use.
