Getting Started
===============

Downloading the Base Repo
-------------------------

To get started make sure ROS is installed on your machine:

.. note:: The following instructions can be found in the Studica ROS repository on github: `ROS Package <REPO LINK>`__.

Installing ROS
^^^^^^^^^^^^^^

1. If you haven't already, run ``installNoetic.sh`` to install ROS Noetic. For more information, visit `ROS Noetic <http://wiki.ros.org/noetic/Installation/Debian>`__

2. Download `Catkn Tools <https://catkin-tools.readthedocs.io/en/latest/installing.html>`__, then run ``sudo apt install python3-catkin-tools python3-osrf-pycommon`` to finalize the install.

.. attention:: Running the ``installNoetic.sh`` script will require some time, this process can take approximately 3 hours. 

Creating a ROS Workspace
^^^^^^^^^^^^^^^^^^^^^^^^

1. Create a directory ``catkin_ws`` by running ``mkdir -p ~/catkin_ws/src``, preferrably in ``/home/pi``.

.. code-block:: rst
   
   mkdir -p ~/catkin_ws/src
   
2. Then ``cd catkin_ws`` and run ``catkin init`` to initialize the workspace environment.

.. code-block:: rst
   
   cd catkin_ws
   catkin init
   
3. Next, run ``catkin build``.

.. code-block:: rst
   
   catkin build

This will create two additional ``build`` and ``devel`` folders in the ``catkin_ws`` directory.

.. figure:: images/catkin_ws_img.jpg
    :align: center
    :width: 70%

Now clone the ros-dev repo into the ``src`` folder.

.. code-block:: rst
   
   git clone <REPO LINK>

Configuring the ROS Environment
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1. Permanently source the setup.bash files by running the following:

.. code-block:: rst
   
   echo "source /opt/ros/noetic/setup.bash" >> ~/.profile
   echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
   echo "source /home/pi/catkin_ws/devel/setup.bash" >> ~/.profile
   echo "source /home/pi/catkin_ws/devel/setup.bash" >> ~/.bashrc
   
   echo "/usr/local/frc/bin/frcKillRobot.sh" >> ~/.profile
   echo "/usr/local/frc/bin/frcKillRobot.sh" >> ~/.bashrc
   
2. Close the terminal and open a new one.

3. Navigate to the work space ``cd catkin_ws/src``

4. Change the name of the ``ros-dev`` folder to ``vmxpi_ros``

.. code-block:: rst
   
   mv ros-dev vmxpi_ros

5. To build the packages run ``catkin build -cs``. Note, this may take a while as the command builds all the packages in the catkin workspace.

.. figure:: images/catkin_build_img.jpg
    :align: center
    :width: 70%

With everything built, you can begin running the node. The VMXPi HAL uses the pigpio library, which unfortunately can only be used in one process. Thus, everything that interfaces with the VMXPi must be run on the same executable.

.. important:: Run ``frcKillRobot.sh`` to kill the robot manager used for WPILib.

Running the Package Initially
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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