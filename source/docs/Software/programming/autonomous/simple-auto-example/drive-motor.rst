DriveMotor
==========

The ``DriveMotor`` class is the class used to create and send the inline auto command to ``AutoCommand``. The ``DriveMotor`` class is also the command that will be called by the auto scheduler. 

.. tabs::
   
    .. tab:: Java

        .. code-block:: java
            :linenos:

            package frc.robot.commands.auto;

            // import the SimpleDrive command
            import frc.robot.commands.driveCommands.SimpleDrive;

            /**
             * DriveMotor class
             * <p>
             * This class creates the inline auto command to drive the motor
             */
            public class DriveMotor extends AutoCommand
            {
                /**
                 * Constructor
                 */
                public DriveMotor()
                {
                    /**
                     * Calls the SimpleDrive command and adds a 5 second timeout
                     * When the timeout is complete it will call the end() method in the SimpleDrive command
                     */
                    super(new SimpleDrive().withTimeout(5)); 
                }
            }

- Line ``4`` is the only import required for this auto command.
- Lines ``16`` - ``23`` is the constructor and auto command.
- Line ``22`` is the inline auto command. In this case we are running an instance of ``SimpleDrive`` and giving it a ``5`` second timeout. For this example the timeout gives us the 5 second runtime we are looking for in running the motor at 50% for 5 seconds. 

.. note:: A command can end before a timeout. Sometimes it's a good idea to add timeouts in case a sensor gives bad data or there was a unhandled error. 