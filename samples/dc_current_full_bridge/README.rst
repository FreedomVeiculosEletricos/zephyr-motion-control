..
   Copyright (c) 2026 Zephyr Motor Control Contributors
   SPDX-License-Identifier: Apache-2.0

DC brushed current loop — full bridge (STM32 LL)
=================================================

This sample is the **full-bridge** variant of the DC current demo: ``pwm-channels``
lists **two** half-bridge legs on one STM32 LL timer (sign-magnitude / bidirectional
drive). Like :file:`../dc_current`, it uses ``zephyr,motor-controller`` +
``MOTOR_SUBSYS_DEFINE_DT()`` + ``motor_subsys_get_by_label("motor_brushed")``. Only
the devicetree overlay differs from the **single half-bridge** (unidirectional) overlay
in :file:`../dc_current`.

See :file:`../dc_current/README.rst` for the synchronous chain diagram, NUCLEO-G474RE
wiring notes, safety, and the **1 HB** build command.

Build (NUCLEO-G474RE, STM32 LL)
-------------------------------

.. code-block:: shell

   west build -b nucleo_g474re path/to/zephyr-motion-control/samples/dc_current_full_bridge -- \
     -DZEPHYR_EXTRA_MODULES=path/to/zephyr-motion-control \
     -DCONF_FILE=prj_stm32_ll.conf
