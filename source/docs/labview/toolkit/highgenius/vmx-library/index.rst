VMX Library 
===========

.. hint:: All example images can be dragged and dropped into LabVIEW.

The VMX Library holds all the classes and underlying functions in the toolkit.

The Library has two simple functions and ten seperate sections contating specifc code for those functions. 

.. list-table:: Description of Simple Functions
    :widths: 30 50
    :header-rows: 1
    :align: center
   
    *  - vi
       - attributes
    *  - Create ID
       - Creates a port
    *  - Delete ID
       - Deletes a port

.. note:: Example use of the Create ID and Delete ID will be shown in the sections below. 

The ten seperate specifc sections are 

.. toctree::
    :maxdepth: 1

    digital-input-and-output
    analog-in
    titanEnc
    pulse
    isq
    encoder
    can 
    pwm
    imu
    iic

.. figure:: images/vmx-library-1.png
    :align: center

LabVIEW VMX-Library Available Pins
----------------------------------

.. list-table:: Inputs and Outputs
    :widths: 30 20 50
    :header-rows: 1
    :align: center
   
    *  - Function
       - LabVIEW Interface
       - VMX Interface
    *  - ENC
       - 0
       - FlexDIO 0,1
    *  - ENC
       - 1
       - FlexDIO 2,3
    *  - ENC
       - 2
       - FlexDIO 4,5
    *  - ENC
       - 3
       - FlexDIO 6,7
    *  - IMU
       - 0
       - 
    *  - AI
       - 22
       - 22
    *  - AI
       - 23
       - 23
    *  - AI
       - 24
       - 24
    *  - AI
       - 25
       - 25
    *  - CAN
       - 2
       - CAN
    *  - PWM
       - 14
       - 14
    *  - PWM
       - 15
       - 15
    *  - PWM
       - 16
       - 16
    *  - PWM
       - 17
       - 17
    *  - PWM
       - 18
       - 18
    *  - Pulse
       - 12
       - 12
    *  - Pulse
       - 13
       - 13
    *  - ISQ
       - 8
       - 8
    *  - ISQ
       - 10
       - 10
    *  - IIC
       - 0
       - IIC
    *  - DO
       - 19
       - 19
    *  - DO
       - 20
       - 20
    *  - DO
       - 21
       - 21
    *  - DI
       - 9
       - 9
    *  - DI
       - 11
       - 11

.. list-table:: Inputs from Titan
    :widths: 30 20
    :header-rows: 1
    :align: center
   
    *  - Function
       - LabVIEW Interface
    *  - M0 Limit Low
       - 0
    *  - M0 Limit High
       - 1
    *  - M1 Limit Low
       - 2
    *  - M1 Limit High
       - 3
    *  - M2 Limit Low
       - 4
    *  - M2 Limit High
       - 5
    *  - M3 Limit Low
       - 6
    *  - M3 Limit High
       - 7
    *  - M0 Encoder
       - 0
    *  - M1 Encoder
       - 1
    *  - M2 Encoder
       - 2
    *  - M3 Encoder
       - 3
