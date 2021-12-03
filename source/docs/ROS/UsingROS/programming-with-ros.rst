Programming With ROS
====================

In the ``vmxpi_ros_bringup/src`` directory there is a ``main.cpp`` file, this is the blank project file where the robot code should go. The empty ``main.cpp`` file has been configured to accept and pass messages between the various packages available in Studica's ROS library.

.. figure:: images/vmxpi_ros_bringup-src.jpg
    :align: center
    :width: 50%
    
To access these classes, simply include their respective headers and begin programming.

.. note:: An executable for ``main.cpp`` has been generated and added to the launch file. For more on launch files see the Running the Package section.

Example Code
------------

Opening the ``main.cpp`` file, there is already an implementation of the Ultrasonic Distance Sensor using the header for the Ultrasonic Ros library. From analyzing the code, you can see that the program constructs an ultrasonic sensor object and directly returns the distances in microseconds, centimeters, or inches.

.. code-block:: c++
            :linenos:
            
            //Include the Ultrasonic Library
            #include "Ultrasonic_ros.h"
            
            
            double ultrasonic_cm;
            
            // Returns the distance value reported by the Ultrasonic Distance sensor
            void ultrasonic_cm_callback(const std_msgs::Float32::ConstPtr& msg)
            {
               ultrasonic_cm = msg->data;
            }
            
            int main(int argc, char **argv)
            {
               system("/usr/local/frc/bin/frcKillRobot.sh"); //Terminal call to kill the robot manager used for WPILib before running the executable.
               ros::init(argc, argv, "ultrasonic_node");
               
               /**
                * Constructor
                * Ultrasonic's ros threads (publishers and services) will run asynchronously in the background
                */
                
               ros::NodeHandle nh; //internal reference to the ROS node that the program will use to interact with the ROS system
               VMXPi vmx(true, (uint8_t)50); //realtime bool and the update rate to use for the VMXPi AHRS/IMU interface, default is 50hz within a valid range of 4-200Hz
               
               ros::Subscriber ultrasonicCM_sub;
               
               UltrasonicROS ultrasonic(&nh, &vmx, 8, 9); //channel_index_out(8), channel_index_in(9)
               ultrasonic.Ultrasonic(); //Sends an ultrasonic pulse for the ultrasonic object to read
                              
               // Use these to directly access data
               uint32_t raw_distance = ultrasonic.GetRawValue(); // returns distance in microseconds
               // or can use
               uint32_t cm_distance = ultrasonic.GetDistanceCM(raw_distance); //converts microsecond distance from GetRawValue() to CM
               // or can use
               uint32_t inch_distance = ultrasonic.GetDistanceIN(raw_distance); //converts microsecond distance from GetRawValue() to IN
                    
               // Subscribing to Ultrasonic distance topic to access the distance data
               ultrasonicCM_sub = nh.subscribe("channel/9/ultrasonic/dist/cm", 1, ultrasonic_cm_callback); //This is subscribing to channel 9, which is the input channel set in the constructor
               
               ros::spin(); //ros::spin() will enter a loop, pumping callbacks to obtain the latest sensor data
               
               return 0;
            }
    
Because an executable has already been generated for ``main.cpp``, there is no need to modify its ``CMakeLists.txt`` or the launch file. Refer to previous sections on building and running the code.

.. note:: This is a similar code block to the example shown in the ``Roscpp`` codeblock under the Ulrasonic Distance Sensor section.