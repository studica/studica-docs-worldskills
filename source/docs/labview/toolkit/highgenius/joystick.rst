Joystick
========

The joystick library takes the joystick data from the computer and sends it over UDP(WiFi) to the robot to be driven around using a joystick. 

There is two vis in the joystick library. 

1. joystick
2. Robot-joystick

.. figure:: images/hg-toolkit-2.png
    :align: center

.. list-table:: Description of Joystick
    :widths: 30 50
    :header-rows: 1
    :align: center
   
    *  - vi
       - attributes
    *  - joystick
       - gets placed on a vi that runs on the computer
    *  - Robot-joystick
       - gets placed in the main vi loop for the robot

.. note:: These vis have no inputs or outputs. They should just be placed in the main loops on the computer and robot.
