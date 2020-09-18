RobotMap
========

.. tabs::

   .. tab:: Java
   
      The ``RobotMap.java`` class is one of the most used classes in the whole project. Although there is nothing inside the base class, once filled with constants it makes changing something electrical on the robot very easy.
      
      .. code-block:: java
         :linenos:
         
         /*----------------------------------------------------------------------------*/
         /* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
         /* Open Source Software - may be modified and shared by FRC teams. The code   */
         /* must be accompanied by the FIRST BSD license file in the root directory of */
         /* the project.                                                               */
         /*----------------------------------------------------------------------------*/

         package frc.robot;

         /**
          * The RobotMap is a mapping from the ports sensors and actuators are wired into
          * to a variable name. This provides flexibility changing wiring, makes checking
          * the wiring easier and significantly reduces the number of magic numbers
          * floating around.
          */
         public class RobotMap {
           // For example to map the left and right motors, you could define the
           // following variables to use with your drivetrain subsystem.
           // public static int leftMotor = 1;
           // public static int rightMotor = 2;

           // If you are using multiple modules, make sure to define both the port
           // number and the module. For example you with a rangefinder:
           // public static int rangefinderPort = 1;
           // public static int rangefinderModule = 1;
         }
      
      What's nice about the RobotMap class is that we can map out a whole robot here and if a change is ever made electrically, such as putting motor 0 in motor 1's port its as simple as changing the constant here and not having to do it in every class that we have. 
      
      Below is an example of a RobotMap class used in a robot.
      
      .. note:: This example has been renamed to ElectricalConstants
      
      .. code-block:: java
         :linenos:
         
         package frc.robot;


         public class ElectricalConstants {

           /**
            * Drive Constants
            */

             //Right Drive
               public static final int RIGHT_DRIVE_FRONT               = 3;
               public static final int RIGHT_DRIVE_BACK                = 2;

             //Left Drive
               public static final int LEFT_DRIVE_FRONT                = 0;
               public static final int LEFT_DRIVE_BACK                 = 1;

           /**
            * Drive Encoders
            */

             //Right Encoder
               public static final int RIGHT_DRIVE_FRONT_ENCODER_A     = 2;
               public static final int RIGHT_DRIVE_FRONT_ENCODER_B     = 3;
               public static final int RIGHT_DRIVE_BACK_ENCODER_A      = 0;
               public static final int RIGHT_DRIVE_BACK_ENCODER_B      = 1;

             //Left Encoder
               public static final int LEFT_DRIVE_FRONT_ENCODER_A      = 4;
               public static final int LEFT_DRIVE_FRONT_ENCODER_B      = 5;
               public static final int LEFT_DRIVE_BACK_ENCODER_A       = 6;
               public static final int LEFT_DRIVE_BACK_ENCODER_B       = 7;

           /**
            * Encoder Constants
            */

             //Radius of drive wheel in inches
               public static final int wheelRadius                     = 2;

             //Encoder Pulses per rotation
               public static final int pulsePerRotation                = 1120;//280

             //Gear Ratio between encoder and wheel
               public static final double gearRatio                    = 1/1;
             
             //Pulse per Rotation ofthe wheel  
               public static final double encdoerPulseRatio            = pulsePerRotation * gearRatio;

             //Distance per tick
               public static final double encoderDistPerTick           = (Math.PI * 2 * wheelRadius) / encdoerPulseRatio;

             //Encoder Reverse
               public static final boolean rightDriveEncoderReverse    = false;
               public static final boolean leftDriveEncoderReverse     = false;
         }