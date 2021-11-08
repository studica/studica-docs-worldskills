Parallel Command Group
======================

The ``ParallelCommandGroup`` is just like the ``SequentialCommandGroup`` except that all the commands run at the same time. The command group will only finish when all commands are finished.

Example code
------------

.. tabs::
   
    .. tab:: Java

        .. code-block:: java
            :linenos:

            import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

            public class Example extends ParallelCommandGroup
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
            #include <frc2/command/ParallelCommandGroup.h>

            class Example
                : public frc2::CommandHelper<frc2::ParallelCommandGroup, Example>
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

In the above example using ``ParallelCommandGroup`` the commands ``DriveForward()``and ``DriveReverse()`` will be executed at the same time. 

.. note:: If ``DriveForward()`` and ``DriveReverse()`` do not complete then whatever calls Example() will never move on. 