IMU / NavX
==========

Handles the IMU data from the internal NavX of the VMX.

.. figure:: images/imu-1.png
    :align: center

.. list-table:: Description of IMU
    :widths: 30 50
    :header-rows: 1
    :align: center
   
    *  - vi
       - Attributes
    *  - IMU
       - IMU initialization
    *  - Read
       - IMU reading
    *  - Reset Yaw
       - IMU reset

IMU
^^^

.. figure:: images/imu-2.png
    :align: center

Is a class that contains the code for reading the imu data on the VMX. Has only a ``HG_LIB`` output.

Read
^^^^

.. figure:: images/imu-3.png
    :align: center

A vi that allows for reading of the imu data on the port specified by the ``Create ID`` vi.

.. list-table:: Inputs and Outputs
    :widths: 30 20 50
    :header-rows: 1
    :align: center
   
    *  - Name
       - I/O
       - Attribute
    *  - IMU in
       - Input
       - The input cluster from Create ID
    *  - Initialize? (F)
       - Input
       - Initialize the imu or not, default false
    *  - error in (no error)
       - Input
       - The error input cluster
    *  - Pulse out
       - Output
       - The output cluster to go to Delete ID
    *  - IMU
       - Output
       - The output cluster with YAW, PITCH, ROLL
    *  - YAW(rad)
       - Output
       - The YAW value in radians
    *  - error out
       - Output
       - The error output cluster

Reset Yaw
^^^^^^^^^

.. figure:: images/imu-4.png
    :align: center

A vi that allows for resetting of the imu data on the port specified by the ``Create ID`` vi.

.. list-table:: Inputs and Outputs
    :widths: 30 20 50
    :header-rows: 1
    :align: center
   
    *  - Name
       - I/O
       - Attribute
    *  - IMU in
       - Input
       - The input cluster from Create ID
    *  - error in (no error)
       - Input
       - The error input cluster
    *  - IMU out
       - Output
       - The output cluster to go to Delete ID
    *  - error out
       - Output
       - The error output cluster

IMU Read Example
----------------

This example shows the reading of the imu data.

.. figure:: images/imu-read-example.png
    :align: center

IMU Reset Example
-----------------

This example shows the resetting of the imu data. 

.. figure:: images/imu-reset-example.png
    :align: center