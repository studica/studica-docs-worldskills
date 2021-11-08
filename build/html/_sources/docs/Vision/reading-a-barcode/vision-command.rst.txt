Vision Command
==============

The Vision Command is used to tell the **VisionSubsystem** code what to do and when to do it. 

VisionCommand.java
------------------

Imports
^^^^^^^

.. code-block:: java
    :linenos:

    package frc.robot.commands;

    import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
    import edu.wpi.first.wpilibj2.command.CommandBase;
    import frc.robot.RobotContainer;
    import frc.robot.subsystems.VisionSubsystem;

- Line ``1`` adds **VisionCommand** to the correct package.
- Line ``3`` imports the **SmartDashboard**, used to display data and get a button input.
- Line ``4`` imports the **CommandBase**, used to make this class part of the command framework.
- Line ``5`` imports the **RobotContainer** so that instances of subsystems can be shared.
- Line ``6`` imports the **VisionSubsystem** which is needed so the command can operate.

Class
^^^^^

.. code-block:: java
    :linenos:
    :lineno-start: 8

    public class VisionCommand extends CommandBase

- Line ``8`` creates the class for **VisionCommand** and extends the **CommandBase** framework with it. 

Objects, Instances, and Variables
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: java
    :linenos:
    :lineno-start: 10

        private static final VisionSubsystem vision = RobotContainer.vision;

        boolean getNewBarcode;

- Line ``10`` creates the **VisionSubsystem** object and assigns the instance of **VisionSubsystem** from the one created in **RobotContainer**.
- Line ``12`` is a simple boolean flag used to see if the user wants to get a new barcode reading. 

Constructor
^^^^^^^^^^^

.. note:: After adding this step, there will be an error shown. Ignore this error as it will be fixed in RobotContainer part 2.

.. code-block:: java
    :linenos:
    :lineno-start: 14

    public VisionCommand ()
    {
        addRequirements(vision);
    }

- Line ``14`` is the constructor for the **VisionCommand** class.
- Line ``16`` tells the **CommandBase** that **VisionCommand** requires a **VisionSubsystem** instance to run. 

Initialize
^^^^^^^^^^

If there is any code that needs to be initialized, it will go in here. But as that is not required, this is an empty method. 

.. code-block:: java
    :linenos:
    :lineno-start: 19

    @Override
    public void initialize(){}

Execute
^^^^^^^

The execute method is what is called every time the command is called. Meaning that the code to be run continuously should be in here. 

.. code-block:: java
    :linenos:
    :lineno-start: 22

    @Override
    public void execute()
    {
        getNewBarcode = SmartDashboard.getBoolean("Get New Barcode", false);

        if (getNewBarcode)
        {
            vision.readBarcode();
            SmartDashboard.putBoolean("Get New Barcode", false);
        }
    }

- Line ``23`` is the execute method.
- Line ``25`` assigns the boolean value taken from ``Get New Barcode`` on the dashboard to **getNewBarcode**. The second parameter in the **getBoolean** call is the default value if nothing can be found. 
- Line ``27`` is a conditional statement that checks if **getNewBarcode** is true. (Button was pushed)
- Line ``29`` will call the **readBarcode** method from **VisionSubsystem**, which in turn will call the scripts on the VMX.
- Line ``30`` sets the **getNewBarcode** button on the dashboard to ``false`` to prevent a continuous call to the scripts when only one call is required. 

End 
^^^

The end method is called when the command is interrupted or is finished as there are no motors or anything safety-related called by this command. The end method can remain blank.

.. code-block:: java
    :linenos:
    :lineno-start: 34

    @Override
    public void end(boolean interrupted){}

isFinished
^^^^^^^^^^

Is finished is a method that is used to create an end condition for the command. As this command should run all the time and never end, a false statement will be returned. 

.. code-block:: java
    :linenos:
    :lineno-start: 37

    @Override
    public boolean isFinished()
    {
        return false;
    }

Full VisionCommand Code 
-----------------------

.. code-block:: java
    :linenos:

    package frc.robot.commands;

    import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
    import edu.wpi.first.wpilibj2.command.CommandBase;
    import frc.robot.RobotContainer;
    import frc.robot.subsystems.VisionSubsystem;

    public class VisionCommand extends CommandBase
    {
        private static final VisionSubsystem vision = RobotContainer.vision;

        boolean getNewBarcode;

        public VisionCommand ()
        {
            addRequirements(vision);
        }

        @Override
        public void initialize(){}
        
        @Override
        public void execute()
        {
            getNewBarcode = SmartDashboard.getBoolean("Get New Barcode", false);

            if (getNewBarcode)
            {
                vision.readBarcode();
                SmartDashboard.putBoolean("Get New Barcode", false);
            }
        }

        @Override
        public void end(boolean interrupted){}

        @Override
        public boolean isFinished()
        {
            return false;
        }
    }