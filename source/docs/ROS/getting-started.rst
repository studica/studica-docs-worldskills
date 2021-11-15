Getting Started
===============

Downloading the Base Repo
-------------------------

To get started make sure ROS is installed on your machine:

.. note:: The following instructions can be found in the Studica ROS repository on github: ``https://github.com/studica/ros-dev``.

Installing ROS
^^^^^^^^^^^^^^

1. If you haven't already, run ``installNoetic.sh`` to install ROS Noetic. For more information, visit http://wiki.ros.org/noetic/Installation/Debian

2. Download catkin tools from https://catkin-tools.readthedocs.io/en/latest/installing.html, then run ``sudo apt install python3-catkin-tools python3-osrf-pycommon`` to finalize the install.

.. attention:: Running the ``installNoetic.sh`` script will require some time, this process can take approximately 3 hours. 

Creating a ROS Workspace
^^^^^^^^^^^^^^^^^^^^^^^^

1. Create a directory ``catkin_ws`` by running ``mkdir -p ~/catkin_ws/src``, preferrably in ``/home/pi``. Then ``cd catkin_ws`` and run ``catkin init`` to initialize the workspace environment.

2. Next, run ``catkin build``.

This will create two additional ``build`` and ``devel`` folders in the ``catkin_ws`` directory.

.. figure:: images/catkin_ws_img.jpg
    :align: center
    :width: 70%

Now clone the ros-dev repo into the ``src`` folder via ``git clone https://github.com/studica/ros-dev.git``.

Configuring the ROS Environment
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1. Permanently source the setup.bash files by running the following: ``echo "source /opt/ros/noetic/setup.bash" >> ~/.profile``, ``echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc``, ``echo "source /home/pi/catkin_ws/devel/setup.bash" >> ~/.profile`` and ``echo "source /home/pi/catkin_ws/devel/setup.bash" >> ~/.bashrc``.

2. Close the terminal and open a new one.

3. Navigate to the work space ``cd catkin_ws/src``

4. Change the name of the ``ros-dev`` folder to ``vmxpi_ros``

5. To build the packages run ``catkin build -cs``. Note, this may take a while as the command builds all the packages in the catkin workspace.

.. figure:: images/catkin_build_img.jpg
    :align: center
    :width: 70%

With everything built, you can begin running the node. The VMXPi HAL uses the pigpio library, which unfortunately can only be used in one process. Thus, everything that interfaces with the VMXPi must be run on the same executable.

.. important:: Run ``frcKillRobot.sh`` to kill the robot manager used for WPILib.

Running the Package Initially
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1. In a new terminal, run ``roscore`` to allow ROS communication.

2. In another terminal, run ``sudo su`` to run commands as root.

3. In root run ``echo "source /opt/ros/noetic/setup.bash" >> ~/.profile``, ``echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc``, ``echo "source /home/pi/catkin_ws/devel/setup.bash" >> ~/.profile`` and ``echo "source /home/pi/catkin_ws/devel/setup.bash" >> ~/.bashrc``.

4. Close the terminal and reopen it. Navigate to ``cd catkin_ws/src`` and run ``sudo su`` once again.

5. Now, run ``roslaunch vmxpi_ros_bringup wrapper.launch`` to start the nodes in the launch file.