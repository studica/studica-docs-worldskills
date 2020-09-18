Constants
=========

.. tabs::

   .. tab:: Java
   
      The ``Constants.java`` class is one of the most used classes in the whole project. Although there is nothing inside the base class, once filled with constants it makes changing something electrical on the robot very easy.
      
      .. code-block:: java
         :linenos:
         
         /*----------------------------------------------------------------------------*/
         /* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
         /* Open Source Software - may be modified and shared by FRC teams. The code   */
         /* must be accompanied by the FIRST BSD license file in the root directory of */
         /* the project.                                                               */
         /*----------------------------------------------------------------------------*/

         package frc.robot;

         /**
          * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
          * constants.  This class should not be used for any other purpose.  All constants should be
          * declared globally (i.e. public static).  Do not put anything functional in this class.
          *
          * <p>It is advised to statically import this class (or one of its inner classes) wherever the
          * constants are needed, to reduce verbosity.
          */
         public final class Constants {
         }
      
      Whats nice about the constants class is that we can map out a whole robot here and if a change is ever made electrically, such as putting motor 0 in motor 1's port its as simple as changing the constant here and not having to do it in every class that we have. 
      
      Below is an example of a constants class used in a robot.
      
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