Services and Clients
====================

Writing the Service Node
------------------------

For the purposes of this demonstration we will use the NavX's ROS Library for reference.

.. code-block:: c++
   :linenos:
   
   //Include the NavX Library
   #include "navX_ros_wrapper.h"
   
   
   bool navXROSWrapper::GetUpdateRate(vmxpi_ros::IntRes::Request &req, vmxpi_ros::IntRes::Response &res)
   {
      res.data = ahrs->GetActualUpdateRate();
      return true;
   }
   
   int main(int argc, char **argv)
   {
      system("/usr/local/frc/bin/frcKillRobot.sh"); //Terminal call to kill the robot manager used for WPILib before running the executable.
      ros::init(argc, argv, "update_rate_server");
      
      ros::NodeHandle nh;
   
      ros::ServiceServer update_rate = nh.advertiseService("get_update_rate", &navXROSWrapper::GetUpdateRate);
      ros::spin();
      
      return 0;
   }
   
Explaining the Code
^^^^^^^^^^^^^^^^^^^

Let's go over each section of the code.

.. code-block:: c++
   
   #include "navX_ros_wrapper.h"
   
``navX_ros_wrapper.h`` is the header for Studica's NavX sensor ROS library.

.. code-block:: c++
   
   bool navXROSWrapper::GetUpdateRate(vmxpi_ros::IntRes::Request &req, vmxpi_ros::IntRes::Response &res)

This function provides the service for obtataining the update rate. From the parameters passed, we can observe that the request and respose is of service type ``IntRes`` that is defined in the ``IntRes.srv`` file located in the srv folder of ``vmxpi_ros``.

To declare more services, run:

.. code-block:: rst

   cd /home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv
   
.. code-block:: rst

   cat > [Service Name].srv
   
Enter the request and response types separated by a ``---`` line, for example:

.. code-block:: none

   int32 data
   ---
   int32 response
   
Press ``Ctrl-D`` to save and exit the text file.

Confirm the creation of the ``.srv`` file by running:

.. code-block:: rst

   rossrv show [Service Name]

Also add the newly created ``.srv`` file to the ``add_service_files`` in CMakeLists.txt as such:

.. code-block:: rst

   ## Generate services in the 'srv' folder
   add_service_files(
     FILES
     Int.srv
     IntRes.srv
     Float.srv
     FloatRes.srv
     MotorSpeed.srv
     StopMode.srv
     StringRes.srv
     [Service Name].srv //New service file
   )

Below is an example of running the above commands:

.. figure:: images/custom_srv_file.jpg
    :align: center
    :width: 70%
    
.. note:: For more information on creating ``.srv`` service types, visit the `Creating a ROS Msg and Srv <http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv>`__ tutorial.
   
.. code-block:: c++
   
   {
      res.data = ahrs->GetActualUpdateRate();
      return true;
   }

Here the ``GetActualUpdateRate()`` accessor method included in the ``navX_ros_wrapper.h`` header is stored in the response variable and the service returns true.

.. code-block:: rst
   
   ros::ServiceServer update_rate = nh.advertiseService("get_update_rate", &navXROSWrapper::GetUpdateRate);
   
The service is created and advertised over ROS.

Writing the Client Node
-----------------------

.. code-block:: c++
   :linenos:
   
   //Include the NavX Library
   #include "navX_ros_wrapper.h"   
   
   int main(int argc, char **argv)
   {
      system("/usr/local/frc/bin/frcKillRobot.sh"); //Terminal call to kill the robot manager used for WPILib before running the executable.
      ros::init(argc, argv, "update_rate_client");
      
      ros::NodeHandle nh;
   
      ros::ServiceClient update_rate_client = nh.serviceClient<vmxpi_ros::IntRes>("get_update_rate");
      
      vmxpi_ros::IntRes srv;
      
      if (update_rate_client.call(srv));
      {
         ROS_INFO("Update Rate: %ld", (long int)srv.response.data);
      }
      else
      {
         ROS_ERROR("Failed to call service get_update_rate");
      }
      
      return 0;
   }
   
Explaining the Code
^^^^^^^^^^^^^^^^^^^

Let's go over each section of the code.

.. note:: Lines that have already been explained above will be ignored.

.. code-block:: c++
   
   ros::ServiceClient update_rate_client = nh.serviceClient<vmxpi_ros::IntRes>("get_update_rate");
   
This creates the ``get_update_rate`` client, which will be used to call the service later.

.. code-block:: c++
   
   vmxpi_ros::IntRes srv;
   
Since we are only receiving a response from the service, there is no need to stuff ``srv`` with information in its request member.

.. code-block:: c++
   
   update_rate_client.call(srv);
   
This is where the service is called, if the call succeeds a value of ``true`` is returned and ``srv.response`` will contain a valid value, otherwise ``false`` is returned  meaning the value of ``srv.response`` will be invalid.