ExampleSubsystem
================

Subsystems are independently controlled parts of a system. Each subsystem is an expanded instance of the subsystem class. Some examples of a subsystem on a robot would be the drive system, elevator, shooter and an intake.  

.. tabs::

   .. tab:: Java
   
      The ``ExampleSubsystem.java`` class is simple to setup and understand. 
      
      .. code-block:: java
         :linenos:
         :emphasize-lines: 15, 20, 21
         
         /*----------------------------------------------------------------------------*/
         /* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
         /* Open Source Software - may be modified and shared by FRC teams. The code   */
         /* must be accompanied by the FIRST BSD license file in the root directory of */
         /* the project.                                                               */
         /*----------------------------------------------------------------------------*/

         package frc.robot.subsystems;

         import edu.wpi.first.wpilibj.command.Subsystem;

         /**
          * An example subsystem. You can replace with me with your own subsystem.
          */
         public class ExampleSubsystem extends Subsystem {
           // Put methods for controlling this subsystem
           // here. Call these from Commands.


           @Override
           protected void initDefaultCommand() {
             // Set the default command for a subsystem here.
             // setDefaultCommand(new MySpecialCommand());
           }
         }
         
      The highlighted lines are explained below. 
      
      Line 15: The class declaration is including the extension of subsystem. This allows the ExampleSubsystem class to access methods inside the subsystem class and add onto of them.  
      
      Line 20: @Override is telling the compiler that we want to use this method and not the method of the same name in the subsystem class. 
      
      Line 21: This is the only **required** method of the subsystem class. This method will attach the ExampleSubsystem to a command of choice. For example a drive subsystem might want to be defaulted to a teleop command.
