Digital Input and Output 
========================

Handles the Digital inputs and outputs on the VMX.

.. important:: If using the High-Current Digital I/O Bus, it can only be configured as all ``inputs`` or all ``outputs`` (default).

.. figure:: images/digital-input-output-1.png
    :align: center

.. list-table:: Description of Digital Input and Output
    :widths: 30 50
    :header-rows: 1
    :align: center
   
    *  - vi
       - Attributes
    *  - Digital Input and Output
       - Digital signal initialization
    *  - Read
       - Digital signal reading
    *  - Write
       - Digital signal writing

Digital Input and Output (vi)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. figure:: images/digital-input-output-4.png
    :align: center

Is a class that contains the code for reading and writing to the digital I/O bus. Has only a ``HG_LIB`` output.

Read
^^^^

.. figure:: images/digital-input-output-3.png
    :align: center

A vi that allows for reading the digital state of the input pin specified by the ``Create ID`` vi.

.. list-table:: Inputs and Outputs
    :widths: 30 20 50
    :header-rows: 1
    :align: center
   
    *  - Name
       - I/O
       - Attribute
    *  - Digital Input and Output in
       - Input
       - The input cluster from Create ID
    *  - error in (no error)
       - Input
       - The error input cluster
    *  - Digital Input and Output out
       - Output
       - The output cluster to go to Delete ID
    *  - Boolean
       - Output
       - The boolean value read on the digital pin
    *  - error out
       - Output
       - The error output cluster

Write
^^^^^

.. figure:: images/digital-input-output-2.png
    :align: center

A vi that allows for writing a high or low to the output pin specified by the ``Create ID`` vi.

.. list-table:: Inputs and Outputs
    :widths: 30 20 50
    :header-rows: 1
    :align: center
   
    *  - Name
       - I/O
       - Attribute
    *  - Digital Input and Output in
       - Input
       - The input cluster from Create ID
    *  - Control
       - Input
       - The boolean value to write to the digital pin
    *  - error in (no error)
       - Input
       - The error input cluster
    *  - Digital Input and Output out
       - Output
       - The output cluster to go to Delete ID
    *  - error out
       - Output
       - The error output cluster


Digital Output Example
----------------------

This example will turn on an LED connected to digital port 12 on the VMX. 

.. figure:: images/digital-output-example.png
    :align: center

Digital Input Example
---------------------

This example will read a digital signal from a Pushbutton connected to digital port 11 on the VMX.

.. figure:: images/digital-input-example.png
    :align: center