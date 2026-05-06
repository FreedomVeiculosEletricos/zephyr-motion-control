..
   Copyright (c) 2026 Freedom Veiculos Eletricos
   SPDX-License-Identifier: UNLICENSED

Motor control shell
====================

Purpose
--------

Bring up a **shell on the console UART** so you can exercise the motor
subsystem and **tune the DC current loop** on the bench: list DT instances,
select, self-test, enable, set current reference, read status, and adjust
PI and ``i_max`` as exposed by the dc-current algorithm API.

The sample does almost nothing in ``main`` — interaction is through shell
commands registered when ``CONFIG_MOTOR_CONTROL_SHELL`` is enabled
(see ``subsys/motor_shell.c``).

Build (NUCLEO-G474RE, STM32 LL)
--------------------------------

.. code-block:: shell

   west build -b nucleo_g474re zephyr-motion-control/samples/motor_shell -- \
     -DZEPHYR_EXTRA_MODULES=zephyr-motion-control

Use a **Nix devshell** (``nix develop``) for Zephyr Python ≥3.12 and toolchains.
See the repo **tests/run_twister.sh** comment for ``PATH`` if the host
``cmake``/``python3`` shadow the Nix environment.

Use ``uart`` on the NUCLEO ST-Link VCP, then e.g.::

  uart:~$ motor list
  uart:~$ motor use 0
  uart:~$ motor selftest
  uart:~$ motor enable
  uart:~$ motor current 0.1
  uart:~$ motor status
  uart:~$ motor pi 0.5 2000 0

Hardware
---------

Same as the previous DC sample: ``TIM3`` for the bridge, ``ADC1`` injected
with TRGO, ``&pwm3`` **disabled** so the motor driver owns the timer. Adjust
shunt/ADC in the **sensor** overlay to match your board.

Safety
-------

Use a current-limited supply and verify interlocks before energising. For
laboratory use.
