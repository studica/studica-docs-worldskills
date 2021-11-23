Running the Package
===================

In a new terminal, run ``roscore``

.. code-block:: rst
   
   roscore

The following should appear:

.. code-block:: rst
   
   ... logging to /home/pi/.ros/log/634b1d0a-4664-11ec-90e3-dca63268e7bc/roslaunch-raspberrypi-18104.log
   Checking log directory for disk usage. This may take a while.
   Press Ctrl-C to interrupt
   Done checking log file disk usage. Usage is <1GB.

   started roslaunch server http://raspberrypi:38727/
   ros_comm version 1.15.11


   SUMMARY
   ========

   PARAMETERS
    * /rosdistro: noetic
    * /rosversion: 1.15.11

   NODES

   auto-starting new master
   process[master]: started with pid [18113]
   ROS_MASTER_URI=http://raspberrypi:11311/

   setting /run_id to 634b1d0a-4664-11ec-90e3-dca63268e7bc
   process[rosout-1]: started with pid [18136]
   started core service [/rosout]
   
``roscore`` is the first command that should be run to allow for ROS nodes to communicate. The command essentially prepares your system by launching the pre-requisite nodes and programs needed for a ROS system. 

.. tip:: Run ``rosclean check`` to check the disk usage of ROS log files. If disk usage >1GB, run ``rosclean purge`` to clear existing ROS log files.

2. In another terminal, run ``sudo su`` to run commands as root.

.. code-block:: rst
   
   sudo su

Running a command with the ``sudo`` prefix is required for commands that require superuser privileges.

.. caution:: Switching to the superuser (root) can be dangerous, it grants access to "super powers" like the ability to modify or delete any file in any directory on the system, hence one should be careful with the commands run under the root account.
   
3. As the root user run:

.. code-block:: rst

   echo "source /opt/ros/noetic/setup.bash" >> ~/.profile
   echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
   echo "source /home/pi/catkin_ws/devel/setup.bash" >> ~/.profile
   echo "source /home/pi/catkin_ws/devel/setup.bash" >> ~/.bashrc

4. Close the root terminal and reopen it. Navigate to ``cd catkin_ws/src`` and run  ``sudo su`` once again.

.. code-block:: rst
   
   sudo su

5. Now, run the following command to start the nodes in the launch file.

.. code-block:: rst
   
   roslaunch vmxpi_ros_bringup wrapper.launch
   
Configuring the Launch File
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Navigate to the launch file directory in the file  explorer and open the ``wrapper.launch`` file.

.. code-block:: rst
   
   cd /home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros_bringup/launch
   nano wrapper.launch
   
.. figure:: images/wrapper_launch.jpg
    :align: center
    :width: 70%
    
From the image above, there are three nodes in the xml launch file. The ``camera_node`` and the ``opencv_node`` are both commented out using the xml syntax ``<!-- Comment -->``. Remove these tags to have these nodes run when the launch file is called, or add them when not in use to save resources.

.. tip :: Observe the resource usage by running ``htop``.

.. figure:: images/htop.jpg
    :align: center
    :width: 70%