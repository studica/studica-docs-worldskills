DriveTrain
==========

For the autonomous to work a subsystem needs to be defined and implemented. For this example only a single motor needs to be created and a way to set the motor speed.

.. tabs::
   
    .. tab:: Java

        .. code-block:: java
            :linenos:

            package frc.robot.subsystems;

            //Vendor imports
            import com.studica.frc.TitanQuad;

            //WPI imports
            import edu.wpi.first.wpilibj2.command.SubsystemBase;
            import frc.robot.Constants;

            /**
             * DriveTrain class 
             * <p>
             * This class creates the instance of the Titan and enables and sets the speed of the defined motor. 
             */
            public class DriveTrain extends SubsystemBase
            {
                /**
                 * Motors
                 */
                private TitanQuad motor;

                /**
                 * Constructor
                 */
                public DriveTrain()
                {
                    //Motors
                    motor = new TitanQuad(Constants.TITAN_ID, Constants.MOTOR);
                }

                /**
                 * Sets the speed of the motor
                 * <p>
                 * @param speed range -1 to 1 (0 stop)
                 */
                public void setMotorSpeed(double speed)
                {
                    motor.set(speed);
                }
            }


- Lines ``4`` - ``8`` are the imports required. The TitanQuad library for the motor, SubsystemBase for the subsystem, and Constants for the motor parameters. 
- Line ``20`` is the creation of the motor object.
- Lines ``25`` - ``29`` is the constructor required for creating an instance of the subsystem.
- Line ``28`` creates a new instance of the TitanQuad and assigns that instance to the M2 port. 
- Line ``36`` - ``39`` is the mutator method to set the speed of the motor. 