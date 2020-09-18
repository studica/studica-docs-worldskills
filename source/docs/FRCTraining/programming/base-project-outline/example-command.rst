ExampleCommand
==============

Commands are one of the most useful features when programing a robot. They allow robot code to be broken off in small pieces and not one giant mess. Commands are easily reusable, interruptible and allow for task to be run in parallel. 

.. tabs::

   .. tab:: Java
   
      The ``ExampleCommand.java`` class is simple framework that just needs to be populated with code. Below each method is explained.
      
      .. code-block:: java
         :linenos:
         
         /*----------------------------------------------------------------------------*/
         /* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
         /* Open Source Software - may be modified and shared by FRC teams. The code   */
         /* must be accompanied by the FIRST BSD license file in the root directory of */
         /* the project.                                                               */
         /*----------------------------------------------------------------------------*/

         package frc.robot.commands;

         import edu.wpi.first.wpilibj.command.Command;
         import frc.robot.Robot;

         /**
          * An example command.  You can replace me with your own command.
          */
         public class ExampleCommand extends Command {
         
      This is the import and class declaration section. Notice how ExampleCommand is an extension of Command. 
      
      .. code-block:: java
         :linenos:
         :lineno-start: 17
         
         public ExampleCommand() {
            // Use requires() here to declare subsystem dependencies
            requires(Robot.m_subsystem);
         }
         
      This is the ExampleCommand constructor and indicates to the Command class that the ``Robot.m_subsystem`` is required for this command. 
      
      .. note:: If you look back at the ``Robot.java`` class you can see the ExampleSubsystem defined on line 26.
      
      .. code-block:: java
         :linenos:
         :lineno-start: 22
         
         // Called just before this Command runs the first time
         @Override
         protected void initialize() {
         }
         
      Code placed in the initialize method will be run once when the command is called for the first time. 
         
      .. hint:: This is a good place to reset variables for example Encoder distance.
      
      .. code-block:: java
         :linenos:
         :lineno-start: 27
         
         // Called repeatedly when this Command is scheduled to run
         @Override
         protected void execute() {
         }
         
      Code here will be run repeatedly until the command is no longer called to run 
      
      .. code-block:: java
         :linenos:
         :lineno-start: 32
         
         // Make this return true when this Command no longer needs to run execute()
         @Override
         protected boolean isFinished() {
            return false;
         }
         
      This method is used by the scheduler to see if the command needs to be run again. Currently isFinished() returns false so the scheduler assumes the command is still running and will keep calling the execute().
      
      .. code-block:: java
         :linenos:
         :lineno-start: 38
         
         // Called once after isFinished returns true
         @Override
         protected void end() {
         }
         
      If isFinished() returns true this method is called once before exiting.
      
      .. hint:: This is a good place to stop motors for example.
      
      .. code-block:: java
         :linenos:
         :lineno-start: 43
         
         // Called when another command which requires one or more of the same
         // subsystems is scheduled to run
         @Override
         protected void interrupted() {
         }
         
      This method is called when ever the command is interrupted by a higher priority call. 
      
      .. hint:: Placing a call to end() is all that is required here in most cases. 
      