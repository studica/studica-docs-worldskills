AutoCommand
===========

The AutoCommand class is used to create the inline command stackup for autonomous routines. To learn more about inline command stackups have a look at the Auto Basics section. 

.. tabs::
   
    .. tab:: Java

        .. code-block:: java
            :linenos:

            package frc.robot.commands.auto;

            //WPI imports
            import edu.wpi.first.wpilibj2.command.Command;
            import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

            /**
             * AutoCommand Class
             * <p>
             * This class is used to create the inline command stackup for autonomous routines
             */
            public abstract class AutoCommand extends SequentialCommandGroup
            {
                /**
                 * Base Constructor
                 */
                public AutoCommand()
                {
                    super();
                }

                /**
                 * Overloaded Constructor to create inline commands
                 * <p>
                 * @param cmd The cmd to be executed
                 */
                public AutoCommand(Command ... cmd)
                {
                    super(cmd);
                }
            }

- Lines ``4`` & ``5`` are the required imports for Command and SequentialCommandGroup.
- Lines ``17`` - ``20`` are the base constructor with no parameters. 
- Lines ``27`` - ``30`` is the constructor that will be used for most if not all autonomous routines. The parameter will be the inline string of commands to be run. 