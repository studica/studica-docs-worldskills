Cobra 
=====

The Cobra Line Follower provides an array of line IR reflective sensors to be used for detecting a line. The Cobra uses four QRE1113 sensors for detecting the line. 

.. figure:: images/cobra-1.jpg
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
       - 3.3VDC
       - 5VDC
       - 5VDC
    *  - Current
       - 25mA
       - 70mA
       - 100mA
    *  - Sensing Distance
       - 2mm
       - 3mm
       - ---

Analog Module
-------------

To plug the Cobra into the VMXpi the analog module is required. 

.. figure:: images/cobra-2.png
    :align: center

The Cobra will plug directly into the analog module. The module will then use the provided JST SH to JST GH cable to connect to the ``i2c`` port on the VMXpi. 

Programming the Cobra
---------------------

.. tabs::
   
    .. tab:: Java

        .. code-block:: java
            :linenos:

            import com.studica.frc.Cobra;

            private Cobra cobra;

            cobra = new Cobra();
            // or if sensor is using 3.3V
            cobra = new Cobra(3.3F);

            //Can then use these accssors to get data
            cobra.getVoltage(channel); //returns a float
            cobra.getRawValue(channel); //returns a double
    
        The accessor methods will output either the voltage (0 - 5V) or the raw ADC value (0 - 2047).

    .. tab:: C++

        **Header**

        .. code-block:: c++
            :linenos:

            #include "Cobra.h"

            private:
                Cobra cobra(void);
                //or if sensor is using 3.3V
                Cobra cobra(3.3F);
        
        **CPP**

        .. code-block:: c++
            :linenos:

            //Use these to access data
            cobra.getVoltage(channel); //returns a float
            cobra.getRawValue(channel); //returns a double

        The accessor methods will output either the voltage (0 - 5V) or the raw ADC value (0 - 2047).


