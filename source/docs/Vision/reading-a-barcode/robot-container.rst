Robot Container Part 1
======================

The Robot Container is used to create an instance of subsystems that can be shared across commands. This saves resources and speeds up the processes. In part 1, only the subsystem will be declared and instantiated. 

Imports
-------

.. code-block:: java
    :linenos:
    :lineno-start: 8

    package frc.robot;

    import frc.robot.subsystems.VisionSubsystem;

- Line ``8`` adds the robot container to the robot package.
- Line ``10`` imports the **VisionSubsystem**.

Class
-----

.. code-block:: java
    :linenos:
    :lineno-start: 18

    public class RobotContainer

- Line ``18`` creates the class.

Objects
-------

.. code-block:: java
    :linenos:
    :lineno-start: 21

    public static VisionSubsystem vision;

- Line ``21`` creates the **VisionSubsystem** object.

Constructor
-----------

.. code-block:: java
    :linenos:
    :lineno-start: 26

    public RobotContainer()

- Line ``26`` is the constructor for the **RobotContainer** class. 

Instantiation
-------------

.. code-block:: java
    :linenos:
    :lineno-start: 28

    vision = new VisionSubsystem();

- Line ``28`` creates the instance of **VisionSubsystem** and assigns it to **vision**. 

Full Code Part 1
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
        }
    }
