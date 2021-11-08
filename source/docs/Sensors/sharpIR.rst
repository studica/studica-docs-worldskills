Sharp IR Distance Sensor
========================

The Sharp GP2Y0A21YK is one of the most reliable and accurate sensors in the collection. The Sharp IR has many benefits that make it one of the best sensors for a robot for distance tracking. 

.. figure:: images/sharpIR-1.png
    :align: center

.. list-table:: Electrical Characteristics
    :widths: 30 10 10 10
    :header-rows: 1
    :align: center
   
    *  - Function
       - Min
       - Nom
       - Max
    *  - Input Voltage
       - 4.5VDC
       - 5V
       - 7VDC
    *  - Output Voltage
       - -0.3VDC
       - ---
       - VIN + 0.3VDC
    *  - Sensing Range
       - 10cm
       - ---
       - 80cm
    *  - Current
       - ---
       - 30mA
       - 40mA
    *  - Operating Temperature
       - -10°C 
       - ---
       - 60°C
    *  - Storage Temperature
       - -40°C
       - ---
       - 70°C

Programming the Sharp IR Sensor
-------------------------------

.. tabs::
   
    .. tab:: Java

        .. code-block:: java
            :linenos:

            //import the Analog Library
            import edu.wpi.first.wpilibj.AnalogInput;

            //Create the Analog Object
            private AnalogInput sharp;

            //Constuct a new instance
            sharp = new AnalogInput(port);

            //Create an accessor method
            public double getDistance()
            {
                return (Math.pow(sharp.getAverageVoltage(), -1.2045)) * 27.726;
            }
    
        The accessor method will output the range in cm.

        .. note:: The valid Analog ports are ``0-3`` 

    .. tab:: C++

        .. code-block:: c++
            :linenos:

            //Include the Analog and Math Library
            #include "frc/AnalogInput.h"
            #include <cmath>

            //Constructors
            frc::AnalogInput sharp{port};

            //Create an accessor function
            double getDistance(void)
            {
                return (pow(sharp.GetAverageVoltage(), -1.2045)) * 27.726;
            }

        The accessor function will output the range in cm.  

        .. note:: The valid Analog ports are ``0-3`` 

    
