Java Only Benefits
==================

Java has some unique features in it's language base and one of those features is Static Factory Methods. This allows a simplar way to declare command groups. 

sequence()
^^^^^^^^^^

The ``sequence()`` static method allows for a sequential command group.

parallel()
^^^^^^^^^^

The ``parallel()`` static method allows for a parallel command group

race()
^^^^^^

The ``race()`` static method allows for a parallel race group.

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
                        race(new DriveForward(), new ElevatorUP()),
                        parallel(new ShootObject(), new LineupGoal()),
                        sequence(new DriveReverse(), new StrafeRight()));
                }
            }

If we analyze this command group we can break down what is happening. 

1. We have ``race(new DriveForward(), new ElevatorUP())`` this will create a ``ParallelRaceGroup`` that has the two commands ``DriveForward()`` and ``ElevatorUP()`` run at the same time in a race. When **one** finishes it will stop the other.
2. As the main command group is the ``SequentialCommandGroup`` we then move on to the next command which is a ``ParallelCommandGroup``.
3. The ``ParallelCommandGroup`` of ``parallel(new ShootObject(), new LineupGoal())`` tells us that ``ShootObject()`` and ``LineupGoal()`` happen at the same time. When **both** are complete it will pass to the next command group. 
4. The last command group here is the ``sequence(new DriveReverse(), new StrafeRight())`` which is also a ``SequentialCommandGroup``. This group is telling the robot to ``DriveReverse()`` and when that is done to ``StrafeRight()``. 