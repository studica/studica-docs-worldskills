Training Platform 
=================

The following code is the LabVIEW project for the `VMX Training Platform. <https://www.studica.co/worldskills-mobile-robotics-workshop-kit-2021>`__

With the project created and the project connected to the VMX, code can now be written. 

Creating the Main vi 
--------------------

Right click on the ``Raspberry Pi 2 B (172.16.0.1)`` target and select ``New`` -> ``VI``

.. figure:: images/training-platform-1.png
    :align: center 

Two new windows will pop up, ``Front Panel`` and ``Block Diagram``. 

.. figure:: images/training-platform-2.png
    :align: center

On the ``Front Panel`` (Grey window), hit ``File`` -> ``Save As`` and save the vi as ``Main.vi``

.. figure:: images/training-platform-3.png
    :align: center

Adding Titan Code 
-----------------

Referring back to the `Titan Code <https://docs.wsr.studica.com/en/latest/docs/labview/toolkit/highgenius/vmx-library/can.html>`__ to move the motor in the toolkit docs page. Adding the code for ``M1`` on the Titan should be very simple.

.. figure:: images/training-platform-4.png
    :align: center

.. note:: In the docs page for the toolkit, we use M0, but here it needs to be changed to M1.

Adding Servo Code 
-----------------

Referring to the `Servo Code <https://docs.wsr.studica.com/en/latest/docs/labview/toolkit/highgenius/vmx-library/pwm.html>`__ to move the servo in the toolkit docs page, the code can be added to our motor code easily. 

.. figure:: images/training-platform-5.png
    :align: center

The servo will be connected to digital port 14.

Adding Sharp IR Code
--------------------

Referring to the `Analog Input Code <https://docs.wsr.studica.com/en/latest/docs/labview/toolkit/highgenius/vmx-library/analog-in.html>`__ to read the Sharp IR Sensor in the toolkit docs page, the code can be added. 

.. figure:: images/training-platform-6.png
    :align: center

The Sharp IR sensor will be on port 22 of the VMX.

Adding Ultrasonic Code
----------------------

Referring back to the `Pulse Code <https://docs.wsr.studica.com/en/latest/docs/labview/toolkit/highgenius/vmx-library/pulse.html>`__ & `ISQ Code <https://docs.wsr.studica.com/en/latest/docs/labview/toolkit/highgenius/vmx-library/isq.html>`__ to read the Ultrasonic Sensor in the toolkit docs page, the code can be added. 

.. figure:: images/training-platform-7.png
    :align: center

The Ultrasonic sensor will use digital port 12 for the pulse and digital port 8 for the echo. 

Add Cobra Code
--------------

Referring back to the `IIC Code <https://docs.wsr.studica.com/en/latest/docs/labview/toolkit/highgenius/vmx-library/iic.html>`__ for the Cobra, the values can be easily read. 

.. figure:: images/training-platform-8.png
    :align: center

The Cobra is plugged into the ADC, which is plugged into the IIC port on the VMX.

Adding NavX Code 
----------------

Referring back to the `IMU Code <https://docs.wsr.studica.com/en/latest/docs/labview/toolkit/highgenius/vmx-library/imu.html>`__ for the internal NavX, the values can be easily read. 

.. figure:: images/training-platform-9.png
    :align: center

The NavX is internal and requires no wire connection. 

Adding an LED Output
--------------------

Referring back to the `Digital Output Code <https://docs.wsr.studica.com/en/latest/docs/labview/toolkit/highgenius/vmx-library/digital-input-and-output.html>`__ an LED can be turned on. 

.. figure:: images/training-platform-10.png
    :align: center

.. figure:: images/training-platform-11.png
    :align: center

For this example, the loop counter will be used to turn the light on digital port 21 on and off every 500 ms. 

Adding a Digital Input
----------------------

Referring back to the `Digital Input Code <https://docs.wsr.studica.com/en/latest/docs/labview/toolkit/highgenius/vmx-library/digital-input-and-output.html>`__ a push button can be read. 

.. figure:: images/training-platform-12.png
    :align: center

Here a pushbutton in a ``Normally Open`` configuration is used to turn an indicator on the front panel on and off. 

.. note:: The VMX has internal pullups for inputs. 


Full Code Example
=================

Below is all the code above in one image that can be dragged into LabVIEW. 

.. note:: The image might have to be saved first then dragged in. 

.. figure:: images/training-platform-full.png
    :align: center