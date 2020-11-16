Parallel Race Group
===================

The ``ParallelRaceGroup`` is similar to the ``ParallelCommandGroup`` except that a race condition is being created. All commands start at the same time but when one command is finished it interrupts all the other commands running and ends the command group. 

Example code
------------

.. tabs::
   
    .. tab:: Java

        .. code-block:: java
            :linenos:

            import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

            public class Example extends ParallelRaceGroup
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
            #include <frc2/command/ParallelRaceGroup.h>

            class Example
                : public frc2::CommandHelper<frc2::ParallelRaceGroup, Example>
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

In the above example using ``ParallelRaceGroup`` the commands ``DriveForward()`` and ``DriveReverse()`` will be executed at the same time. 

.. note:: If ``DriveForward()`` or ``DriveReverse()`` complete before the other than the other will be interrupted and stop running. 