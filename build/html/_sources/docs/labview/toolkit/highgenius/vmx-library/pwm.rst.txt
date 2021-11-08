PWM 
===

Handles the PWM outputs on the VMX.

.. figure:: images/pwm-1.png
    :align: center

.. list-table:: Description of PWM
    :widths: 30 50
    :header-rows: 1
    :align: center
   
    *  - vi
       - Attributes
    *  - PWM
       - PWM initialization
    *  - Write
       - PWM writing

PWM (vi)
^^^^^^^^

.. figure:: images/pwm-2.png
    :align: center

Is a class that contains the code for setting the PWM value on the PWM ports on the VMX. Has only a ``HG_LIB`` output.

Write
^^^^^

.. figure:: images/pwm-3.png
    :align: center

A vi that allows for writing the PWM value to the port specified by the ``Create ID`` vi.

.. list-table:: Inputs and Outputs
    :widths: 30 20 50
    :header-rows: 1
    :align: center
   
    *  - Name
       - I/O
       - Attribute
    *  - PWM in
       - Input
       - The input cluster from Create ID
    *  - PWM
       - Input
       - The duty cycle to set the PWM to
    *  - error in (no error)
       - Input
       - The error input cluster
    *  - PWM out
       - Output
       - The output cluster to go to Delete ID
    *  - error out
       - Output
       - The error output cluster

PWM Example
-----------

This example shows the writing of a PWM value to a Servo on digital port 14 to make it move.

.. figure:: images/pwm-example.png
    :align: center