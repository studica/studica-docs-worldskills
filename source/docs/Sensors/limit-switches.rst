Limit Switches
==============

Reading a Digital Input
-----------------------

.. tabs::
   
    .. tab:: Java

        .. code-block:: java
            :linenos:

            //import the DigitalInput Library
            import com.wpi.first.wpilibj.DigitalInput;

            //Create the DigitalInput Object
            private DigitalInput input;

            //Constuct a new instance
            input = new DigitalInput(port);

            //Can then use these accssor to get data
            input.get(); //Will return true for a high signal and false for a low signal

    .. tab:: C++

        .. code-block:: c++
            :linenos:

            //Include the DigitalInput Library
            #include "frc/DigitalInput.h"

            //Constructors
            frc::DigitalInput input{port};

            //Use these to access data
            input.Get(); //Will return true for a high signal and false for a low signal
            
    .. tab:: Roscpp
    
         .. code-block:: c++
            :linenos:
            
            //Include the DigitalInput Library
            #include "DI_ros.h"
            
            int main(int argc, char **argv)
            {
               system("/usr/local/frc/bin/frcKillRobot.sh"); //Terminal call to kill the robot manager used for WPILib before running the executable.
               ros::init(argc, argv, "digitalin_node");
               
               /**
                * Constructors
                * Create the DigitalInput object
                * DI ros threads (publishers and services) will run asynchronously in the background
                */
               
               ros::NodeHandle nh; //internal reference to the ROS node that the program will use to interact with the ROS system
               VMXPi vmx(true, (uint8_t)50); //realtime bool and the update rate to use for the VMXPi AHRS/IMU interface, default is 50hz within a valid range of 4-200Hz
               
               DigitalInputROS digital_in(&nh, &vmx, channel);
               
               digital_in.Get(); //Will return true for a high signal and false for a low signal
               
               ros::spin(); //ros::spin() will enter a loop, pumping callbacks to obtain the latest sensor data
               
               return 0;
            }
            
         .. important:: Subscribe to DI topics to access the data being published and write callbacks to pass messages between various processes.