Servo Power Block
=================

The Servo Power Block allows for the proper power to be supplied to the servo motors on a robot. 

.. figure:: images/servo-power-block-1.png
    :align: center

Power Block Specs
-----------------

.. list-table:: Power Block Specs
    :widths: 30 10 10 10
    :header-rows: 1
    :align: center

    * - Function
      - Min
      - Nom
      - Max
    * - Input Voltage
      - ---
      - 12VDC
      - --- 
    * - Output Voltage
      - 5.5VDC
      - 6.0VDC
      - 6.5VDC
    * - Output Current (shared)
      - ---
      - ---
      - 10A

.. note:: The internal regulator has overcurrent protection and will shutoff at 10A output. Test's have shown that it will shutoff just before 10A.

    