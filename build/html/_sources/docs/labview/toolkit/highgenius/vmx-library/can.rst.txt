CAN Bus 
=======

Handles the CAN bus communication on the VMX. Specifically used to control the speed of a motor on the Titan. 

.. figure:: images/can-1.png
    :align: center

.. list-table:: Description of CAN
    :widths: 30 50
    :header-rows: 1
    :align: center
   
    *  - vi
       - Attributes
    *  - CAN
       - CAN bus initialization
    *  - Write
       - CAN bus writing

CAN
^^^

.. figure:: images/can-2.png
    :align: center

Is a class that contains the code for setting the motor speed on the Titan using the CAN bus on the VMX. Has only a ``HG_LIB`` output.

Read
^^^^

.. figure:: images/can-3.png
    :align: center

A vi that allows for setting the motor speed from the port specified by the ``Create ID`` vi.

.. list-table:: Inputs and Outputs
    :widths: 30 20 50
    :header-rows: 1
    :align: center
   
    *  - Name
       - I/O
       - Attribute
    *  - CAN in
       - Input
       - The input cluster from Create ID
    *  - PWM
       - Input
       - Motor to control
    *  - Duty Cycle
       - Input
       - Speed to set the motor at
    *  - INA
       - Input
       - One of the Directional registers for motor direction
    *  - INB
       - Input
       - One of the Directional registers for motor direction
    *  - error in (no error)
       - Input
       - The error input cluster
    *  - CAN out
       - Output
       - The output cluster to go to Delete ID
    *  - error out
       - Output
       - The error output cluster

CAN Example
-----------

This example controls M0 on the Titan. 

.. figure:: images/can-write-example.png
    :align: center
