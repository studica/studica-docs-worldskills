Robot    
=====

.. tabs::
   
   .. tab:: Java
      
      There are multiple sections of ``Robot.java``, below discusses each one and the purpose
      
      **Title Block**
      
      .. code-block:: java
         :linenos:
         
         /*----------------------------------------------------------------------------*/
         /* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
         /* Open Source Software - may be modified and shared by FRC teams. The code   */
         /* must be accompanied by the FIRST BSD license file in the root directory of */
         /* the project.                                                               */
         /*----------------------------------------------------------------------------*/
         
      
      .. hint:: The title block is important as it displays licenses, authors, last edited dates, etc...
      
      Some nice things to have in any title block would be the ``author`` and ``date``. This would provide the next person coming to see who wrote what and when they wrote it.
      
      
      **Imports**
      
      The import section holds all the imports of various libraries that are required for use in the current class.

      .. code-block:: java
         :linenos:
         :lineno-start: 8
      
         package frc.robot;

         import edu.wpi.first.wpilibj.TimedRobot;
         import edu.wpi.first.wpilibj.command.Command;
         import edu.wpi.first.wpilibj.command.Scheduler;
         import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
         import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
         import frc.robot.commands.ExampleCommand;
         import frc.robot.subsystems.ExampleSubsystem;
         
      **Class Declaration**
      
      In every Java program the class needs to be declared. Below is the class declaration for the ``Robot.java`` class.
      
      .. code-block:: java
         :linenos:
         :lineno-start: 18
         
         /**
          * The VM is configured to automatically run this class, and to call the
          * functions corresponding to each mode, as described in the TimedRobot
          * documentation. If you change the name of this class or the package after
          * creating this project, you must also update the build.gradle file in the
          * project.
          */
         public class Robot extends TimedRobot {
         
      **Declaring Objects**

      This section declares objects that are required for use later in the class.
      
      .. code-block:: java
         :linenos:
         :lineno-start: 26
      
         public static ExampleSubsystem m_subsystem = new ExampleSubsystem();
         public static OI m_oi;

         Command m_autonomousCommand;
         SendableChooser<Command> m_chooser = new SendableChooser<>();
         
      **Robot Initialization**
      
      This is where any code is to be run when the robot is first booting up.
      
      .. code-block:: java
         :linenos:
         :lineno-start: 32
      
         /**
          * This function is run when the robot is first started up and should be
          * used for any initialization code.
          */
         @Override
         public void robotInit() {
            m_oi = new OI();
            m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
            // chooser.addOption("My Auto", new MyAutoCommand());
            SmartDashboard.putData("Auto mode", m_chooser);
         }
         
      **Robot Periodic**
      
      .. warning:: Code here is run every robot packet and is not controlled by the **Enable/Disable** buttons. 
      
      Robot periodic is a good section to add code for diagnostics or anything that requires constant polling.
      
      .. code-block:: java
         :linenos:
         :lineno-start: 44
      
         /**
          * This function is called every robot packet, no matter the mode. Use
          * this for items like diagnostics that you want ran during disabled,
          * autonomous, teleoperated and test.
          *
          * <p>This runs after the mode specific periodic functions, but before
          * LiveWindow and SmartDashboard integrated updating.
          */
         @Override
         public void robotPeriodic() {
         }
         
      **Disabled Initialization**
      
      When ever the robot is put into a disabled state it enters here first. 
      
      .. code-block:: java
         :linenos:
         :lineno-start: 56
      
         /**
          * This function is called once each time the robot enters Disabled mode.
          * You can use it to reset any subsystem information you want to clear when
          * the robot is disabled.
          */
         @Override
         public void disabledInit() {
         }
         
      **Disabled Periodic**
      
      Code that will run every robot packet when the robot is disabled.
      
      .. code-block:: java
         :linenos:
         :lineno-start: 65
         
         @Override
         public void disabledPeriodic() {
            Scheduler.getInstance().run();
         }
         
      **Autonomous Initialization**
      
      Code that is run at the start of an autonomous run.
      
      .. code-block::
         :linenos:
         :lineno-start: 70
         
         /**
          * This autonomous (along with the chooser code above) shows how to select
          * between different autonomous modes using the dashboard. The sendable
          * chooser code works with the Java SmartDashboard. If you prefer the
          * LabVIEW Dashboard, remove all of the chooser code and uncomment the
          * getString code to get the auto name from the text box below the Gyro
          *
          * <p>You can add additional auto modes by adding additional commands to the
          * chooser code above (like the commented example) or additional comparisons
          * to the switch structure below with additional strings & commands.
          */
         @Override
         public void autonomousInit() {
            m_autonomousCommand = m_chooser.getSelected();
         
            /*
             * String autoSelected = SmartDashboard.getString("Auto Selector",
             * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
             * = new MyAutoCommand(); break; case "Default Auto": default:
             * autonomousCommand = new ExampleCommand(); break; }
             */

            // schedule the autonomous command (example)
            if (m_autonomousCommand != null) {
               m_autonomousCommand.start();
            }
         }
         
      **Autonomous Periodic**
      
      Code that is run every robot packet during the autonomous run.
      
      .. code-block: java
         :linenos:
         :lineno-start: 98
         
         /**
          * This function is called periodically during autonomous.
          */
         @Override
         public void autonomousPeriodic() {
            Scheduler.getInstance().run();
         }
         
      **Teleop Initialization**
      
      Code that is run at the start of a teleoperated run.
      
      .. code-block:: java
         :linenos:
         :lineno-start: 106
         
         @Override
         public void teleopInit() {
            // This makes sure that the autonomous stops running when
            // teleop starts running. If you want the autonomous to
            // continue until interrupted by another command, remove
            // this line or comment it out.
            if (m_autonomousCommand != null) {
               m_autonomousCommand.cancel();
            }
         }
         
      **Teleop Periodic**
      
      Code that is run every robot packet when in a periodic run.
      
      .. code-block:: java
         :linenos:
         :lineno-start: 117
         
         /**
          * This function is called periodically during operator control.
          */
         @Override
         public void teleopPeriodic() {
            Scheduler.getInstance().run();
         }   
      
      **Test Periodic**
      
      Code that is run every robot packet when in a test run.
      
      .. code-block:: java
         :linenos:
         :lineno-start: 125
         
         /**
           * This function is called periodically during test mode.
           */
         @Override
         public void testPeriodic() {
         }
      