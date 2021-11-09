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
            
            //Include the DigitalInput and the DigitalOuptut Library
            #include "DI_ros.h"
            #include "DI_ros.h"
            
            /**
             * Constructors
             * Create the DigitalOuptut and DigitalInput objects
             * DIO ros threads (publishers and services) will run asynchronously in the background
             */
            
            ros::NodeHandle nh; //internal reference to the ROS node that the program will use to interact with the ROS system
            VMXPi vmx(true, (uint8_t)50); //realtime bool and the update rate to use for the VMXPi AHRS/IMU interface, default is 50hz within a valid range of 4-200Hz
            
            DigitalOuptutROS(&nh, &vmx, channel);
            DigitalInputROS(&nh, &vmx, channel);
            
            ros::ServiceClient sendPing, setDuration;
            
            // Declaring message types
            std_srvs::Empty msg1;
            vmxpi_ros::Int msg2;
            
            // Use these to send a ping signal and set duration, default duration: 10 microseconds
            sendPing.call(msg1);
            
            int duration = 10;
            msg2.request.data = duration; //Configure ping duration for a length of 10 microseconds
            setDuration.call(msg2);
            
         .. important:: Subscribe to DI topics to access the data being published and write callbacks to pass messages between various processes.
            
