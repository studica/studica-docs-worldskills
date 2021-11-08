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
            
            DigitalOuptutROS(&nh, &vmx, channel);
            DigitalInputROS(&nh, &vmx, channel);
            
            ros::ServiceClient sendPing, setDuration;
            
            // Declaring message types
            std_srvs::Empty msg1; //ROS service type for sending a signal to a ROS node
            vmxpi_ros::Int msg2; //VMX-pi service type
            
            //Use these to send a ping signal and set duration
            sendPing.call(msg1); //Send a ping singal with default duration 10 microseconds
            
            int duration = 10;
            msg2.request.data = duration; //Configure ping duration for a length of 10 microseconds
            setDuration.call(msg2);
            
         .. important:: Subscribe to DI_ros topics to access the data being published and write callbacks to pass messages between various processes.
            
