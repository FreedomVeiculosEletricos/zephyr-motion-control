Motor Trapezoid Sample
######################

This sample drives the ``dc-current`` algorithm with a shell-started
trapezoidal current trajectory on the NUCLEO-G474RE STM32 LL motor path.

Build:

.. code-block:: bash

   west build -b nucleo_g474re samples/motor_trapezoid

Run from the console:

.. code-block:: text

   trapezoid start
   trapezoid status
   trapezoid stop

The trajectory uses the ``i-max-ma`` value from the algorithm devicetree node as
its peak current. The board overlay sets it to 1000 mA, so the cycle is:

* 0 A to +1 A in 5 s
* +1 A hold for 5 s
* +1 A to 0 A in 5 s
* 0 A hold for 1 s
* 0 A to -1 A in 5 s
* -1 A hold for 5 s
* -1 A to 0 A in 5 s
* 0 A hold for 1 s
