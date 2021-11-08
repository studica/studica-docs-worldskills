Robot Container Part 2
======================

In part 2, the error is going to be fixed. The error occurs as the **VisionSubsystem** and **VisionCommand** are linked in the command. However, they are not linked on the subsystem level. 

Imports
-------

In the imports section, one more import will be added.

.. code-block:: java
    :linenos:
    :lineno-start: 8

    package frc.robot;

    import frc.robot.subsystems.VisionSubsystem;
    import frc.robot.commands.VisionCommand;

- Line ``11`` is the addition as **VisionCommand** needs to be imported. 

Constructor
-----------

The last change is in the constructor, where the default command for the **VisionSubsystem** will be set.

.. code-block:: java
    :linenos:
    :lineno-start: 27

        public RobotContainer()
        {
            vision = new VisionSubsystem();

            vision.setDefaultCommand(new VisionCommand());
        }

- Line ``31`` assigns a default command for the **VisionSubsystem** and that default command is **VisionCommand**

Full Code Part 2
----------------

.. code-block:: java
    :linenos:

    /*----------------------------------------------------------------------------*/
    /* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
    /* Open Source Software - may be modified and shared by FRC teams. The code   */
    /* must be accompanied by the FIRST BSD license file in the root directory of */
    /* the project.                                                               */
    /*----------------------------------------------------------------------------*/

    package frc.robot;

    import frc.robot.subsystems.VisionSubsystem;
    import frc.robot.commands.VisionCommand;

    /**
    * This class is where the bulk of the robot should be declared.  Since Command-based is a
    * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
    * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
    * (including subsystems, commands, and button mappings) should be declared here.
    */
    public class RobotContainer 
    {
        // The robot's subsystems and commands are defined here...
        public static VisionSubsystem vision;

        /**
        * The container for the robot.  Contains subsystems, OI devices, and commands.
        */
        public RobotContainer()
        {
            vision = new VisionSubsystem();

            vision.setDefaultCommand(new VisionCommand());
        }
    }


