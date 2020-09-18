Updating Robot
==============

Before we can create the command to use the driveTrain subsystem, we need to create an instance of the subsystem in ``Robot.java``. 

All we need is three lines of codes to be added to the Robot class.

.. code-block:: java
   :linenos:
   :emphasize-lines: 1,6,11
   
   import frc.robot.subsystems.DriveTrain;
   
   public class Robot extends TimedRobot {
   
      public static OI oi;
      public static DriveTrain drive;
      
      @Override
      public void robotInit(){
         oi = new OI();
         drive = new DriveTrain();
      }

- On line 1, we import the driveTrain subsystem.
- On line 6, we create the reference to DriveTrain and assign it a local identifier.
- On line 11, we create the instance of DriveTrain. 

That's all that needs to be added, and the command scheduler will handle the rest. 