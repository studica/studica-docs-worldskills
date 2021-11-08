SimpleDrive
===========

``SimpleDrive`` is the command that controls the subsystem output. Commands can be called again and again which makes them perfect for autonomous routines. 

.. tabs::
   
    .. tab:: Java

        .. code-block:: java
            :linenos:

            package frc.robot.commands.driveCommands;

            //WPI imports
            import edu.wpi.first.wpilibj2.command.CommandBase;

            //RobotContainer import
            import frc.robot.RobotContainer;

            //Subsystem imports
            import frc.robot.subsystems.DriveTrain;

            /**
             * SimpleDrive class
             * <p>
             * This class drives a motor at 50% speed until the command is ended
             */
            public class SimpleDrive extends CommandBase
            {
                //Grab the subsystem instance from RobotContainer
                private static final DriveTrain drive = RobotContainer.drive;

                /**
                 * Constructor
                 */
                public SimpleDrive()
                {
                    addRequirements(drive); // Adds the subsystem to the command
                }

                /**
                 * Runs before execute
                 */
                @Override
                public void initialize()
                {

                }

                /**
                 * Called continously until command is ended
                 */
                @Override
                public void execute()
                {
                    drive.setMotorSpeed(0.5); // Set motor speed to 50%
                }

                /**
                 * Called when the command is told to end or is interrupted
                 */
                @Override
                public void end(boolean interrupted)
                {
                    drive.setMotorSpeed(0.0); // Stop motor
                }

                /**
                 * Creates an isFinished condition if needed
                 */
                @Override
                public boolean isFinished()
                {
                    return false;
                }

            }

- Lines ``4`` - ``10`` are the imports required. 
- Line ``20`` grabs the instance of the ``DriveTrain`` subsystem defined and instantiated in ``RobotContainer``.
- Lines ``25`` - ``28`` are the constructor.
- Line ``27`` says that this command requires the subsystem ``drive`` which is the handle for the subsystem ``DriveTrain``.
- Lines ``33`` - ``37`` is the initialize section of the command. In this case there is nothing to initialize so it is left blank.
- Lines ``42`` - ``46`` is the execute section of the command. As long as the command is active anything in here will run every robot packet (20ms).
- Line ``45`` is setting the motor speed to ``0.5`` which is equal to ``50%`` speed. 
- Lines ``51`` - ``55`` is the end section of the command. When the command is scheduled to end or is interrupted this method is called. 
- Line ``54`` sets the motor speed to ``0.0`` this will stop the motor. **It is a good idea to always add a stop motor instruction here unless its not required**.
- Lines ``60`` - ``64`` is the isFinished section of the command. This method can be called to check if the command is finished or not. Useful if you wanted to put a stop condition based on sensor feedback here. For example using the sharp sensor to sence distance and it hits the threshold. 