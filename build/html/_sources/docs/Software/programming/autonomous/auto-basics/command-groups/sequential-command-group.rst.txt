Sequential Command Group
========================

The ``SequentialCommandGroup`` is the most popular command group. Works by running a list of commands in sequential order. Starts with the first command in the list then the second and so on.

.. warning:: The ``SequentialCommandGroup`` will not finish unless all the commands finish. Also if a command in the list does not finish the next command in line will not start.

Example code
------------

.. tabs::
   
    .. tab:: Java

        .. code-block:: java
            :linenos:

            import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

            public class Example extends SequentialCommandGroup
            {
                public Example()
                {
                    addCommands(
                        //Drive Forward
                        new DriveForward(),

                        //Drive Reverse
                        new DriveReverse());
                }
            }
    
    .. tab:: C++ (Header)

        .. code-block:: c++
            :linenos:

            #prama once

            #include <frc2/command/CommandHelper.h>
            #include <frc2/command/SequentialCommandGroup.h>

            class Example
                : public frc2::CommandHelper<frc2::SequentialCommandGroup, Example>
            {
                public:

                    Example(void);
            };
    
    .. tab:: C++ (Source)

        .. code-block:: c++
            :linenos:

            #include "commands/Example.h"

            Example::Example(void)
            {
                AddCommands(

                    //Drive Forward
                    DriveForward(),

                    //Drive Backwards
                    DriveReverse());
            }

In the above example using ``SequentialCommandGroup`` the command ``DriveForward()`` will be exeuted first and when complete the command ``DriveReverse()`` will be executed. 

.. note:: If ``DriveForward()`` does not end then ``DriveReverse()`` will never start.