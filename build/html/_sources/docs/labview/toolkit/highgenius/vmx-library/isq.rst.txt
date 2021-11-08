Interupt Status Queue / Ultrasonic
==================================

Handles the echo of the Ultrasonic sensor. 

.. figure:: images/isq-1.png
    :align: center

.. list-table:: Description of ISQ
    :widths: 30 50
    :header-rows: 1
    :align: center
   
    *  - vi
       - Attributes
    *  - ISQ
       - ISQ initialization
    *  - Read
       - ISQ reading

Pulse Init 
^^^^^^^^^^

.. figure:: images/isq-2.png
    :align: center

Is a class that contains the code for the return pulse of the Ultrasonic pulse on the VMX. Has only a ``HG_LIB`` output.

Pulse Output
^^^^^^^^^^^^

.. figure:: images/isq-3.png
    :align: center

A vi that allows for collecting the return pulse of the Ultrasonic pulse on the port specified by the ``Create ID`` vi.

.. list-table:: Inputs and Outputs
    :widths: 30 20 50
    :header-rows: 1
    :align: center
   
    *  - Name
       - I/O
       - Attribute
    *  - ISQ in
       - Input
       - The input cluster from Create ID
    *  - error in (no error)
       - Input
       - The error input cluster
    *  - ISQ out
       - Output
       - The output cluster to go to Delete ID
    *  - PING
       - Output
       - The distance calculated by the ultrasonic sensor in mm
    *  - error out
       - Output
       - The error output cluster

Ultrasonic Example
------------------

.. note:: This example requires Pulse and ISQ.

This example will Pulse the Ultrasonic sensor on digital port 12 and receive the echo on digital port 8 then calulate the distance.

.. figure:: images/ultrasonic-example.png
    :align: center