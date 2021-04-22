Programming the Titan
=====================


Motor Setup
-----------

.. tabs::
   
    .. tab:: Java

        .. code-block:: java
            :linenos:

            //import the TitanQuad Library
            import com.studica.frc.TitanQuad;

            //Create the TitanQuad Object
            private TitanQuad motor;

            //Constuct a new instance
            motor = new TitanQuad(TITAN_CAN_ID, TITAN_MOTOR_NUMBER);
    
        .. note:: ``TITAN_CAN_ID`` is the CAN id for the Titan, by defualt it is 42. ``TITAN_MOTOR_NUMBER`` is the motor port to be used. Valid range is ``0 - 3``, this corresponds to the M0 - M3 on the Titan. 

    .. tab:: C++ (Header)

        .. code-block:: c++
            :linenos:

            //Include the TitanQuad Library
            #include <studica/TitanQuad.h>

            //Constuct a new instance
            private:
                studica::TitanQuad motor{TITAN_CAN_ID, TITAN_MOTOR_NUMBER};
        
        .. note:: ``TITAN_CAN_ID`` is the CAN id for the Titan, by defualt it is 42. ``TITAN_MOTOR_NUMBER`` is the motor port to be used. Valid range is ``0 - 3``, this corresponds to the M0 - M3 on the Titan. 


Setting Motor Speed
-------------------

.. tabs::
   
    .. tab:: Java

        .. code-block:: java
            :linenos:

            /**
             * Sets the speed of a motor
             * <p>
             * @param speed range -1 to 1 (0 stop)
             */
            public void setMotorSpeed(double speed)
            {
                motor.set(speed);
            }

    .. tab:: C++ (Source)

        .. code-block:: c++
            :linenos:

            /**
             * Sets the speed of a motor
             * <p>
             * @param speed range -1 to 1 (0 stop)
             */
            void ClassName::SetMotorSpeed(double speed)
            {
                motor.Set(speed);
            }

Full Example
------------

.. tabs::
   
    .. tab:: Java

        .. code-block:: java
            :linenos:

            package frc.robot.subsystems;

            //Subsystem Base import
            import edu.wpi.first.wpilibj2.command.SubsystemBase;

            //Titan import
            import com.studica.frc.TitanQuad;

            public class Example extends SubsystemBase
            {
                /**
                 * Motors
                 */
                private TitanQuad motor;

                public Example()
                {
                    //Motors
                    motor = new TitanQuad(TITAN_CAN_ID, TITAN_MOTOR_NUMBER);
                }

                /**
                 * Sets the speed of a motor
                 * <p>
                 * @param speed range -1 to 1 (0 stop)
                 */
                public void setMotorSpeed(double speed)
                {
                    motor.set(speed);
                }
            }
            

    .. tab:: C++ (Header)

        .. code-block:: c++
            :linenos:

            #pragma once

            //Include SubsystemBase
            #include <frc2/command/SubsystemBase.h>

            //Include Titan Library
            #include "studica/TitanQuad.h"

            class Example : public frc2::SubsystemBase
            {
                public:
                    Example();
                    void SetMotorSpeed(double speed);

                private:
                    studica::TitanQuad motor(TITAN_CAN_ID, TITAN_MOTOR_NUMBER);
            };
    
    .. tab:: C++ (Source)

            .. code-block:: c++
                :linenos:
    
                //Include Header
                #include "subsystems/Example.h"

                //Constructor
                Example::Example(){}

                /**
                 * Sets the speed of a motor
                 * <p>
                 * @param speed range -1 to 1 (0 stop)
                 */
                void Example::SetMotorSpeed(double speed)
                {
                    motor.Set(speed);
                }        


