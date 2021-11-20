Subscribers and Publishers
==========================

Writing the Subscriber Node
---------------------------

For the purposes of this demonstration we will use the Sharp IR sensor's ROS Library for reference.

.. code-block:: c++
   :linenos:
   
   //Include the Sharp Library
   #include "Sharp_ros.h"
   
   
   double sharp_dist;
   
   // Returns the distance value reported by the Sharp IR sensor
   void sharp_dist_callback(const std_msgs::Float32::ConstPtr& msg)
   {
      sharp_dist = msg->data;
   }
   
   int main(int argc, char **argv)
   {
   
      ros::init(argc, argv, "sharp_sub_node");
       
      ros::NodeHandle nh; //internal reference to the ROS node that the program will use to interact with the ROS system
      
      ros::Subsriber sharpDist_sub;
      
      // Subscribing to Sharp distance topic to access the distance data
      sharpDist_sub = nh.subscribe("channel/22/sharp_ir/dist", 1, sharp_dist_callback);
       
      ros::spin(); //ros::spin() will enter a loop, pumping callbacks to obtain the latest sensor data
      
      return 0;
   }
   
Explaining the Code
^^^^^^^^^^^^^^^^^^^

Let's go over each section of the code.

.. code-block:: c++
   
   #include "Sharp_ros.h"
   
``Sharp_ros.h`` is the header for Studica's Sharp sensor ROS library.

.. code-block:: c++
   
   double sharp_dist;
   
This is the variable that accepts the sensor data from the messages being published on the ``channel/22/sharp_ir/dist`` topic.

.. code-block:: c++
   
   void sharp_dist_callback(const std_msgs::Float32::ConstPtr& msg)
   {
      sharp_dist = msg->data;
   }

This is the callback fuction that will get called when a new message has arrived on the ``channel/22/sharp_ir/dist`` topic. This message in particular is a ``Float32`` type passed on a `boost shared_ptr <https://www.boost.org/doc/libs/1_37_0/libs/smart_ptr/shared_ptr.htm>`__.

.. code-block:: c++
   
   ros::init(argc, argv, "sharp_sub_node");
   
Used to initialize ROS and must be called before using any other parts of the ROS system. Its parameters must be ``argc`` and ``argv`` such that it can perform any ROS arguments or name remappings provided in the command line. The name of the node is also specified here.

.. code-block:: c++
   
   ros::NodeHandle nh;

Internal reference to the ROS node that the program will use to interact with the ROS system.

.. code-block:: c++
   
   ros::Subsriber sharpDist_sub;
   
Constructs a ROS subscriber object called ``sharpDist_sub``.

.. code-block:: c++
   
   sharpDist_sub = nh.subscribe("channel/22/sharp_ir/dist", 1, sharp_dist_callback);
   
This line calls the ``subscribe()`` method, this is used to inform the ROS Master node that we want to accept messages being streamed on a certain topic. The particular topic is declared in the first argument, in this case ``channel/22/sharp_ir/dist``. The second argument is where we set the capacity of the queue, this is important for cases where messages are being sent faster than they are being recieved and processed. ``1`` is the queue size, meaning if the size of the queue is greater than one, old messages will start being discarded as new ones arrive. The final parameter passed is the callback function that gets called whenever a new message arrives on the topic. The ``sharpDist_sub`` object is maintained until all copies of it are destroyed, in this case the ``channel/22/sharp_ir/dist`` topic will be automatically unsubcribed from.

.. note:: ROS Master acts as a registry where nodes establish peer-to-peer connections in order to pass messages, it keeps track of what nodes are publishing and nodes that are subscribing.

.. code-block:: c++
   
   ros::spin();
   
``ros::spin()`` will enter a loop, pumping callbacks to obtain the latest sensor data.

Writing the Publisher Node
--------------------------

.. important:: For using Studica's ROS Library, publishers have already been implemented where relevant sensor information has been organized into topics. This section is for creating a publisher node from scratch.

For the purposes of this demonstration we will use the Sharp IR sensor's ROS Library for reference.

.. code-block:: c++
   :linenos:
   
   //Include the Sharp Library
   #include "Sharp_ros.h"

   int main(int argc, char* argv[])
   {
      ros::init(argc, argv, "sharp_pub_node");
      
      ros::NodeHandle nh;
      
      ros::Publisher sharp_dist_pub;
      
      sharp_dist_pub = nh->advertise<std_msgs::Float32>("channel/22/sharp_ir/dist", 1);

      ros::Rate loop_rate(50);
      while (ros::ok()) {
         std_msgs::Float32 msg;
         
         msg.data = GetIRDistance();
         sharp_dist_pub.publish(msg);
         
         loop_rate.sleep();
      }

Explaining the Code
^^^^^^^^^^^^^^^^^^^

Let's go over each section of the code.

.. note:: Lines that have already been explained above will be ignored.

.. code-block:: c++
   
   ros::Subsriber sharp_dist_pub;
   
Constructs a ROS subscriber object called ``sharp_dist_pub``.

.. code-block:: c++
   
   sharp_dist_pub = nh->advertise<std_msgs::Float32>("channel/22/sharp_ir/dist", 1);
   
This line calls the ``advertise()`` method, this is used to inform the ROS Master node that we are going to be publishing distance messages over a certain topic. The particular topic is declared in the first argument, in this case ``channel/22/sharp_ir/dist``. The second argument is where we set the capacity of the queue, this is important for cases where messages are being sent faster than they are being recieved and processed. ``1`` is the queue size, meaning if the size of the queue is greater than one, old messages will start being discarded as new ones arrive. The ``sharp_dist_pub`` object is maintained until all copies of it are destroyed, in this case the ``channel/22/sharp_ir/dist`` topic will automatically stop advertising messages.

.. code-block:: c++
   
   ros::Rate loop_rate(50);
   
The ``ros::Rate`` class creates an object that allows us to set a frequency that we would like to run the ``while()`` loop at. This is used in conjunction with the ``sleep()`` method by tracking the time in between calls to ``Rate::sleep()`` and sleeping for the appropriate duration to set the loop frequency. For this example, the loop will run at 50Hz.

.. code-block:: c++
   
   while (ros::ok()) {
   
``ros::ok()`` will return false if:

- a SIGINT is received ``(Ctrl-C)``
- we have been kicked off the network by another node with the same name
- ``ros::shutdown()`` is evoked
- all ``ros::NodeHandle(s)`` have been destroyed

.. code-block:: c++
   
   std_msgs::Float32 msg;
   
Message datatype of ``Float32``.

.. code-block:: c++
   
   msg.data = GetIRDistance();
   
The message variable is passed with information from the ``GetIRDistance`` accessor function that is included in ``Sharp_ros.h``
   
.. code-block:: c++

   sharp_dist_pub.publish(msg);
   
Once filled with sensor data, the publish object advertises the message to the ``channel/22/sharp_ir/dist`` distance topic for any nodes connected.

.. code-block:: c++

   loop_rate.sleep();

Like previously mentioned, this is used to sleep for a duration that allows the ``loop_rate`` object to maintain a frequency of ``50`` specified in its declaration.
   
   