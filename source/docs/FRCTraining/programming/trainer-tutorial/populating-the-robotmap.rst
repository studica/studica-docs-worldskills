Populating the RobotMap
=======================

The first step after creating a project is to create constants that map out the robot's electrical connections. Open ``RobotMap.java``, we will start populating it based on the connections of the training robot. 

If you received the training robot assembled, it would be wired as such:

-  CAN ID = 42
-  Left Rear Motor = 0
-  Left Front Motor = 1
-  Right Rear Motor = 2
-  Right Front Motor = 3

To start, let's look again at the base RobotMap class.

.. code-block:: java
   :linenos:
   
   package frc.robot;

   /**
    * Electrical Map for Training Robot
    */
   public class RobotMap
   {

   }

If you notice the comment block above, ``public class RobotMap`` has been changed to be more specific to the current application. The comment block above the class declaration is fundamental. In competition, the comment block for this class could contain multiple things such as the person who wired the robot, author of the current file, and the last time the file was edited. This is very important for diagnosing if something goes wrong. If the left side motor starts moving when the right side motor should be moving, then something is wrong. For diagnosing, we can look at the comment block and ask the person who wired the robot if they made an electrical change. If not, then we can look for the person who created the RobotMap. 

Based on the wiring description above, five constants are required to be added to the RobotMap.

.. code-block:: java
   :linenos:
   
   public static final int TITAN_CAN_ID      = 42;
   
Above is an example of a constant used in this class. 

-  ``public`` - allows the constants to be used by another class.
-  ``static`` - saves memory (`See more <https://www.javatpoint.com/static-keyword-in-java>`__).
-  ``final`` - tells the compiler that this variable cannot be changed. 
-  ``int`` - tells the compiler that the constant is an integer.
-  ``TITAN_CAN_ID`` - the constants name.
-  ``42`` - the value assigned to the constant. 

Now all five constants will be added to the RobotMap.

.. code-block:: java
   :linenos:
   
   public class RobotMap
   {
   
      /*
       * Drive Motor Constants
       */
       
         //Titan CAN ID
            public static final int TITAN_CAN_ID            = 42;
       
         //Right Drive
            public static final int RIGHT_DRIVE_FRONT       = 3;
            public static final int RIGHT_DRIVE_REAR        = 2;
         
         //Left Drive
            public static final int LEFT_DRIVE_FRONT        = 1;
            public static final int LEFT_DRIVE_REAR         = 0;
            
   }

