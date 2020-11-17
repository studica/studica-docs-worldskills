Encoders 
========

Encoders are a sensor placed normally on a shaft to provide feedback to controller. This feedback allows for the detection of position, speed and direction of motion control system. There are two types of encoders; absolute and incremental. Absolute encoders report back a location specfic position. Incremental encoders only indicate that there has been a change in postion and what that change was. In robotics we tend to mostly use incremental encoders as they are easier to use and have some more benefical advandages than that of the absolute encoder. 

The encoders in the worldSkills collection are built into the motors already. This makes it easier for designing drive systems as an external encoder does not need to be designed in. 

Math
----

There is a lot of math assosiated with encoders. Before the encoder class can be used the distance per tick has to be calculated. The formula can be given as:

.. math::

      \begin{equation}
      {distancePerTick} =
      \frac{2 \pi r}{ticksPerRev * gearRatio}
      \end{equation}

Where:

- ``r`` = wheel radius
- ``ticksPerRev`` = encoder pulses on the output shaft of the motor
- ``gearRatio`` = an external gear ratio used.

Example
^^^^^^^

Lets look at an example using the Maverick with the 100mm omni wheel attached directly on the shaft of the motor. 

- ``r`` = 51 mm (actual measured value)
- ``ticksPerRev`` = 732 (encoder counts per 1 revolution of the motor output shaft)
- ``gearRatio`` = 1:1

.. math::

      \begin{equation}
      {distancePerTick} = \frac{2 \pi r}{ticksPerRev * gearRatio}
      = \frac{2 \pi 51}{732 * 1}
      = \frac{102 \pi}{732}
      = 0.43776291
      \end{equation}

Therefor we can conculde that the distancePerTick for the Maverick using the 100mm omni wheels is **0.43776291**. 

Application
^^^^^^^^^^^

Now that we have the distancePerTick we can calculate the distance traveled. This is simply formulated by:

.. math::

    distance = {distancePerTick} * {encoderCount}

Where:

- ``distancePerTick`` = 0.43776291
- ``encoderCount`` = is the incremental count from the encoder

Lets look at a few examples:

**One Wheel Rotation**

encoderCount = 732

.. math::

    distance = {0.43776291} * {732} = 320.44 mm

.. note:: The distance measured is in mm as the radius was specficed in mm.

**Ten Wheel Rotations**

encoderCount = 7320

.. math::

    distance = {0.43776291} * {7320} = 3204.43 mm

Code
----

Now that we know the math behind it, let's look at how to program the encoder.

.. tabs::
   
    .. tab:: Java

        **Constants.java**

        .. code-block:: java
            :linenos:

            /**
             * Motor Constants
             */
            public static final int TITAN_ID                = 42; 
            public static final int MOTOR                   = 2;

            /**
             * Encoder Constants
             */

            //Radius of drive wheel in mm
            public static final int wheelRadius             = 51;

            //Encoder pulses per rotation of motor shaft    
            public static final int pulsePerRotation        = 732;

            //Gear ratio between motor shaft and output shaft
            public static final double gearRatio            = 1/1;
           
            //Pulse per rotation combined with gear ratio
            public static final double encoderPulseRatio    = pulsePerRotation * gearRatio;

            //Distance per tick
            public static final double distancePerTick      = (Math.PI * 2 * wheelRadius) / encoderPulseRatio;
        
        **Subsystem**

        .. code-block:: java   
            :linenos:

            import com.studica.frc.TitanQuad;
            import com.studica.frc.TitanQuadEncoder;

            public class Subsystem
            {
                /**
                 * Motors
                 */
                private TitanQuad motor;

                /**
                 * Sensors
                 */
                private TitanQuadEncoder encoder;

                public Subsystem()
                {
                    //Motors
                    motor = new TitanQuad(Constants.TITAN_ID, Constants.MOTOR);

                    //Sensors
                    encoder = new TitanQuadEncoder(motor, Constants.MOTOR, Constants.distancePerTick);
                }

                /**
                 * Gets the distance traveled of the motor
                 * <p>
                 * @return the distance traveled
                 */
                public double getEncoderDistance()
                {
                    return encoder.getEncoderDistance();
                }
            }

    .. tab:: C++ (Header)

        .. code-block:: c++
            :linenos:

            #include <studica/TitanQuad.h>
            #include <studica/TitanQuadEncoder.h>

            #include <cmath>

            class Subsystem : public frc2::SubsystemBase
            {
                public:
                    Subsystem();

                    double GetEncoderDistance (void);

                private:
                    /**
                     * Motor Constants
                     */
                    #define TITAN_ID                42
                    #define MOTOR_N                   2

                    /**
                     * Encoder Constants
                     */

                    //Radius of drive wheel in mm
                    #define wheelRadius             51

                    //Encoder pulses per rotation of motor shaft    
                    #define pulsePerRotation        732

                    //Gear ratio between motor shaft and output shaft
                    #define gearRatio               1/1
                
                    //Pulse per rotation combined with gear ratio
                    #define encoderPulseRatio       pulsePerRotation * gearRatio

                    //Distance per tick
                    #define distancePerTick         (M_PI * 2 * wheelRadius) / encoderPulseRatio

                    /**
                     * Objects
                     */
                    studica::TitanQuad motor{TITAN_ID, MOTOR_N};
                    studica::TitanQuadEncoder encoder{motor, MOTOR_N, distancePerTick};
            }
    
    .. tab:: C++ (Source)

        .. code-block:: c++
            :linenos:

            #include "subsystems/Subsystem.h"

            Subsystem::Subsystem(){};

            /**
             * Gets the distance traveled of the motor
             * <p>
             * @return the distance traveled
             */
            double GetEncoderDistance (void)
            {
                return encoder.GetEncoderDistance();
            }